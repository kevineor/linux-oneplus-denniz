// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Copyright (c) 2026 Kevin Andrieux
 *
 * MT6893 connectivity subsystem power-on.
 *
 * The WiFi is a block of the SoC with its own MCU and a separate radio chip.
 * Four stages come up in order: conn_infra (the connectivity bus), the
 * MT6635 A-die behind an SPI master, WFSYS (the WiFi digital block), then the
 * MCU, which boots from code placed in shared memory.
 *
 * The "conn" power domain provides the clocks, the TOPRGU reset, power,
 * isolation and bus protections, and pinctrl muxes the A-die interface.
 * Register sequences follow the vendor conninfra and gen4m soc3_0 drivers.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include "mt6893.h"

/*
 * SPI master setup for the WiFi case, from connsys_spi_master_cfg_mt6893():
 * the A-die addresses and words the sequencer uses to power the radio up and
 * down. A zero mask means a full write.
 */
struct mt6893_wmac_regval {
	u16 off;
	u32 mask;
	u32 val;
};

static const struct mt6893_wmac_regval mt6893_wmac_spi_cfg[] = {
	/* clock and wake-up addresses */
	{ 0x070, 0,	 0x0af40a04 },
	{ 0x074, 0,	 0x00a00090 },
	{ 0x078, 0,	 0x009c008c },
	{ 0x07c, 0xfff,	 0x00000a08 },
	{ 0x080, 0xfff,	 0x00000094 },
	{ 0x084, 0xfff,	 0x0000002c },
	{ 0x088, 0,	 0x0afc0a0c },
	{ 0x08c, 0xfff,	 0x000000f0 },
	{ 0x090, 0xfff,	 0x000000f4 },
	{ 0x094, 0xfff,	 0x000000fc },
	{ 0x098, 0xfff,	 0x000000f8 },
	/* command length, BG block addresses */
	{ 0x004, 0x1f,	 0x00000008 },
	{ 0x010, 0xffff, 0x0000a03c },
	{ 0x014, 0xffff, 0x0000a03c },
	{ 0x018, 0xffff, 0x0000aa18 },
	{ 0x01c, 0xffff, 0x0000aa18 },
	{ 0x020, 0xffff, 0x0000a0c8 },
	{ 0x024, 0xffff, 0x0000aa00 },
	{ 0x028, 0xffff, 0x0000a0b4 },
	{ 0x02c, 0xffff, 0x0000a34c },
	/* BG power-on sequence */
	{ 0x030, 0,	 0x00000000 },
	{ 0x034, 0,	 0x00000000 },
	{ 0x038, 0,	 0x74e0fff5 },
	{ 0x03c, 0,	 0x76e8fff5 },
	{ 0x040, 0,	 0x00000000 },
	{ 0x044, 0,	 0xffffffff },
	{ 0x048, 0,	 0x00000019 },
	{ 0x04c, 0,	 0x00010400 },
	/* BG power-off sequence */
	{ 0x050, 0,	 0x57400000 },
	{ 0x054, 0,	 0x57400000 },
	{ 0x058, 0,	 0x44e0fff5 },
	{ 0x05c, 0,	 0x44e0fff5 },
	{ 0x060, 0,	 0x00000001 },
	{ 0x064, 0,	 0x00000000 },
	{ 0x068, 0,	 0x00040019 },
	{ 0x06c, 0,	 0x00410440 },
};

static void mt6893_wmac_remap(struct mt6893_dev *dev, u32 off, u32 addr)
{
	u32 val = mt76_rr(dev, off);

	val &= ~CONN2AP_REMAP_ADDR;
	val |= addr & CONN2AP_REMAP_ADDR;
	mt76_wr(dev, off, val);
}

/* Must be set before the A-die is brought up. */
static int mt6893_wmac_set_emi(struct mt6893_dev *dev)
{
	struct reserved_mem *rmem;
	struct device_node *np;

	np = of_parse_phandle(dev->mt76.dev->of_node, "memory-region", 0);
	if (!np)
		return dev_err_probe(dev->mt76.dev, -ENODEV,
				     "missing memory-region\n");

	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem)
		return dev_err_probe(dev->mt76.dev, -EINVAL,
				     "reserved memory region not found\n");

	if (rmem->size < MT6893_EMI_SIZE)
		return dev_err_probe(dev->mt76.dev, -EINVAL,
				     "EMI region of %pa, 0x%x needed\n",
				     &rmem->size, MT6893_EMI_SIZE);

	if (rmem->base & ((1UL << CONN2AP_REMAP_SHIFT) - 1))
		return dev_err_probe(dev->mt76.dev, -EINVAL,
				     "EMI region at %pa is not 64 KiB aligned\n",
				     &rmem->base);

	dev->emi_base = rmem->base;
	dev->emi_size = rmem->size;

	mt6893_wmac_remap(dev, CONN2AP_REMAP_MCU_EMI,
			  rmem->base >> CONN2AP_REMAP_SHIFT);
	mt6893_wmac_remap(dev, CONN2AP_REMAP_WF_PERI, CONN2AP_REMAP_PERI);
	mt6893_wmac_remap(dev, CONN2AP_REMAP_BT_PERI, CONN2AP_REMAP_PERI);
	mt6893_wmac_remap(dev, CONN2AP_REMAP_GPS_PERI, CONN2AP_REMAP_PERI);

	dev_info(dev->mt76.dev, "shared memory at %pa\n", &rmem->base);

	return 0;
}

static void mt6893_wmac_rmw(struct mt6893_dev *dev, u32 off, u32 mask, u32 val)
{
	if (mask)
		val = (mt76_rr(dev, off) & ~mask) | (val & mask);
	mt76_wr(dev, off, val);
}

/* consys_spi_read_nolock(): writing WDAT triggers the transfer. */
static int mt6893_wmac_spi_read(struct mt6893_dev *dev, u16 addr, u32 *out)
{
	if (!mt76_poll(dev, CONN_RF_SPI_STA, CONN_RF_SPI_BUSY, 0, 5000))
		return -ETIMEDOUT;

	mt76_wr(dev, CONN_RF_SPI_ADDR, CONN_RF_SPI_READ | addr);
	mt76_wr(dev, CONN_RF_SPI_WDAT, 0);

	if (!mt76_poll(dev, CONN_RF_SPI_STA, CONN_RF_SPI_BUSY, 0, 5000))
		return -ETIMEDOUT;

	*out = mt76_rr(dev, CONN_RF_SPI_RDAT);

	return 0;
}

static int mt6893_wmac_spi_write(struct mt6893_dev *dev, u16 addr, u32 val)
{
	if (!mt76_poll(dev, CONN_RF_SPI_STA, CONN_RF_SPI_BUSY, 0, 5000))
		return -ETIMEDOUT;

	mt76_wr(dev, CONN_RF_SPI_ADDR, CONN_RF_SPI_WRITE | addr);
	mt76_wr(dev, CONN_RF_SPI_WDAT, val);

	if (!mt76_poll(dev, CONN_RF_SPI_STA, CONN_RF_SPI_BUSY, 0, 5000))
		return -ETIMEDOUT;

	return 0;
}

/* Write @size bits of @val, taken at @val_ofs, at @reg_ofs of an A-die register. */
static int mt6893_wmac_spi_field(struct mt6893_dev *dev, u16 addr, u32 val,
				 int reg_ofs, int val_ofs, int size)
{
	u32 mask = GENMASK(reg_ofs + size - 1, reg_ofs);
	u32 cur;
	int ret;

	ret = mt6893_wmac_spi_read(dev, addr, &cur);
	if (ret)
		return ret;

	val = ((val >> val_ofs) << reg_ofs) & mask;

	return mt6893_wmac_spi_write(dev, addr, (cur & ~mask) | val);
}

/*
 * Copy the A-die efuses to sysram for the firmware, and apply the thermal
 * sensor calibration they carry: connsys_a_die_efuse_read() and
 * connsys_a_die_thermal_cal().
 */
static int mt6893_wmac_adie_efuse(struct mt6893_dev *dev)
{
	u32 ctrl, efuse[4];
	int i, ret;

	ret = mt6893_wmac_spi_read(dev, ATOP_EFUSE_CTRL, &ctrl);
	if (ret)
		return ret;
	ret = mt6893_wmac_spi_write(dev, ATOP_EFUSE_CTRL, ctrl & ~ATOP_EFUSE_KICK);
	if (ret)
		return ret;

	ret = mt6893_wmac_spi_read(dev, ATOP_EFUSE_CTRL, &ctrl);
	if (ret)
		return ret;
	ctrl = (ctrl & ~ATOP_EFUSE_CLR) | ATOP_EFUSE_KICK;
	ret = mt6893_wmac_spi_write(dev, ATOP_EFUSE_CTRL, ctrl);
	if (ret)
		return ret;

	for (i = 0; i < 70; i++) {
		ret = mt6893_wmac_spi_read(dev, ATOP_EFUSE_CTRL, &ctrl);
		if (ret)
			return ret;
		if (!(ctrl & ATOP_EFUSE_KICK))
			break;
		udelay(30);
	}

	if (!(ctrl & ATOP_EFUSE_VALID)) {
		dev_warn(dev->mt76.dev, "invalid A-die efuses: 0x%08x\n",
			 ctrl);
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(efuse); i++) {
		ret = mt6893_wmac_spi_read(dev, ATOP_EFUSE_RDATA(i), &efuse[i]);
		if (ret)
			return ret;
		mt76_wr(dev, CONN_SYSRAM_ADIE_EFUSE(i), efuse[i]);
	}

	dev_info(dev->mt76.dev, "A-die efuses: %08x %08x %08x %08x\n",
		 efuse[0], efuse[1], efuse[2], efuse[3]);

	if (efuse[1] & BIT(7)) {
		ret = mt6893_wmac_spi_field(dev, ATOP_RG_TOP_THADC_BG, efuse[1],
					    12, 3, 4);
		if (!ret)
			ret = mt6893_wmac_spi_field(dev, ATOP_RG_TOP_THADC,
						    efuse[1], 23, 0, 3);
		if (ret)
			return ret;
	}

	if (efuse[1] & BIT(15))
		return mt6893_wmac_spi_field(dev, ATOP_RG_TOP_THADC, efuse[1],
					     26, 13, 2);

	return 0;
}

/* End of connsys_a_die_cfg_mt6893(), for a 26 MHz crystal. */
static int mt6893_wmac_adie_cfg(struct mt6893_dev *dev, u32 id)
{
	static const struct {
		u16 addr;
		u32 val;
	} atop[] = {
		{ ATOP_RG_ENCAL_WBTAC_IF_SW,	0x80000000 },	/* WF_PAD high-Z */
		{ ATOP_RG_WF0_TOP_01,		0x000e8002 },	/* cal LDO off */
		{ ATOP_RG_WF1_TOP_01,		0x000e8002 },
		{ ATOP_RG_TOP_XTAL_01,		0xf6e8fff5 },	/* XOBUF voltage */
		{ ATOP_RG_TOP_XTAL_02,		0x55555fff },
		{ ATOP_RG_WF0_BG,		0x00002008 },	/* THADC initial IR */
		{ ATOP_RG_WF1_BG,		0x00002008 },
	};
	int i, ret;

	mt76_wr(dev, CONN_SYSRAM_ADIE_CHIP_ID, id);

	/* SX calibration maps, WiFi sleep in DBDC mode */
	ret = mt6893_wmac_spi_write(dev, ATOP_WRI_CTR2, 0x00007700);
	if (!ret)
		ret = mt6893_wmac_spi_write(dev, ATOP_SMCTK11, 0x00000021);
	if (ret)
		return ret;

	mt6893_wmac_rmw(dev, CONN_RF_SPI_FM_CTRL, CONN_RF_SPI_FM_RD_EXT, 0);
	mt76_wr(dev, CONN_TOP_THERM_AADDR, CONN_TOP_THERM_AADDR_MT6635);

	ret = mt6893_wmac_adie_efuse(dev);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(atop); i++) {
		u32 val = atop[i].val;

		/* E1 revision */
		if (atop[i].addr == ATOP_RG_TOP_XTAL_02 && id == 0x66358a00)
			val = 0xd5555fff;

		ret = mt6893_wmac_spi_write(dev, atop[i].addr, val);
		if (ret)
			return ret;
	}

	return 0;
}

/* connsys_afe_wbg_cal_mt6893() and connsys_subsys_pll_initial_mt6893(). */
static int mt6893_wmac_afe_cal(struct mt6893_dev *dev)
{
	mt76_wr(dev, CONN_AFE_WBG_AFE_01, 0x00000000);
	mt76_wr(dev, CONN_AFE_WBG_RCK_01, 0x144b0160);
	mt76_wr(dev, CONN_AFE_WBG_GL1_01, 0x10990c13);
	mt76_wr(dev, CONN_AFE_WBG_GL5_01, 0x10990c13);
	mt76_wr(dev, CONN_AFE_WBG_BT_TX_03, 0xcd258051);
	mt76_wr(dev, CONN_AFE_WBG_WF0_TX_03, 0xc5258251);
	mt76_wr(dev, CONN_AFE_WBG_WF1_TX_03, 0xc5258251);

	/* RC calibration */
	mt6893_wmac_rmw(dev, CONN_AFE_DIG_EN_01, CONN_AFE_EN_RCK, CONN_AFE_EN_RCK);
	udelay(60);
	mt6893_wmac_rmw(dev, CONN_AFE_DIG_EN_01, CONN_AFE_EN_RCK, 0);

	/* TX calibration, with the PLLs on */
	mt6893_wmac_rmw(dev, CONN_AFE_CTL_PLL, CONN_AFE_CTL_BPLL_EN,
			CONN_AFE_CTL_BPLL_EN);
	udelay(30);
	mt6893_wmac_rmw(dev, CONN_AFE_CTL_PLL, CONN_AFE_CTL_WPLL_EN,
			CONN_AFE_CTL_WPLL_EN);
	udelay(60);
	mt6893_wmac_rmw(dev, CONN_AFE_DIG_EN_01, CONN_AFE_EN_TXCAL,
			CONN_AFE_EN_TXCAL);
	udelay(800);
	mt6893_wmac_rmw(dev, CONN_AFE_DIG_EN_01, CONN_AFE_EN_TXCAL, 0);
	mt6893_wmac_rmw(dev, CONN_AFE_CTL_PLL,
			CONN_AFE_CTL_BPLL_EN | CONN_AFE_CTL_WPLL_EN, 0);

	if (mt6893_wmac_spi_write(dev, ATOP_RG_ENCAL_WBTAC_IF_SW, 0x00000005))
		return -ETIMEDOUT;

	/* PLL settle times, 30 and 50 us, and per-radio PLL enables */
	mt6893_wmac_rmw(dev, CONN_AFE_PLL_STB_TIME, 0x7fff7fff, 0x03140521);
	mt6893_wmac_rmw(dev, CONN_AFE_DIG_EN_02, 0xcf, 0x4f);

	mt76_set(dev, CONN_SYSRAM_RADIO_STATUS, CONN_SYSRAM_RADIO_WIFI);

	return 0;
}

/*
 * connsys_d_die_cfg_mt6893(), connsys_spi_master_cfg_mt6893() and
 * connsys_a_die_cfg_mt6893().
 */
static int mt6893_wmac_adie_on(struct mt6893_dev *dev)
{
	u32 id;
	int ret;

	mt76_wr(dev, CONN_INFRA_RGU_SYSRAM_PDN, 0);
	mt76_wr(dev, CONN_INFRA_RGU_SYSRAM_SLP, 0xff);
	mt76_wr(dev, CONN_INFRA_RGU_COEXT_PDN, 0);
	mt76_wr(dev, CONN_INFRA_RGU_COEXT_SLP, 1);

	for (int i = 0; i < ARRAY_SIZE(mt6893_wmac_spi_cfg); i++) {
		const struct mt6893_wmac_regval *r = &mt6893_wmac_spi_cfg[i];

		mt6893_wmac_rmw(dev, CONN_WT_SLP_CTL + r->off, r->mask, r->val);
	}

	for (int i = 0; i < CONN_INFRA_SYSRAM_SIZE; i += 4)
		mt76_wr(dev, CONN_INFRA_SYSRAM + i, 0);

	/*
	 * The MT6635 samples a strap on CONN_WF_CTRL2 when its reset is
	 * released: the pin has to be a GPIO driven high at that point.
	 */
	mt6893_wmac_rmw(dev, CONN_ADIE_CTL, CONN_ADIE_RSTB, 0);
	usleep_range(1000, 2000);

	ret = pinctrl_select_state(dev->pinctrl, dev->pins_strap);
	if (ret)
		return dev_err_probe(dev->mt76.dev, ret, "A-die strap\n");

	mt6893_wmac_rmw(dev, CONN_ADIE_CTL, CONN_ADIE_RSTB, CONN_ADIE_RSTB);
	usleep_range(1000, 2000);

	ret = mt6893_wmac_spi_read(dev, ATOP_CHIP_ID, &id);
	if (ret)
		return dev_err_probe(dev->mt76.dev, ret,
				     "SPI master busy, no A-die\n");

	if (id >> 16 != ATOP_CHIP_ID_MT6635) {
		dev_err(dev->mt76.dev, "no A-die: ATOP CHIP_ID 0x%08x\n", id);
		return -ENODEV;
	}

	dev_info(dev->mt76.dev, "A-die MT6635, revision 0x%04x\n",
		 id & 0xffff);

	ret = mt6893_wmac_adie_cfg(dev, id);
	if (ret)
		return dev_err_probe(dev->mt76.dev, ret,
				     "A-die configuration\n");

	ret = pinctrl_select_default_state(dev->mt76.dev);
	if (ret)
		return dev_err_probe(dev->mt76.dev, ret,
				     "CONN_WF_CTRL2 back to CONN function\n");

	ret = mt6893_wmac_afe_cal(dev);
	if (ret)
		return dev_err_probe(dev->mt76.dev, ret,
				     "analog front-end calibration\n");

	dev_info(dev->mt76.dev, "A-die configured, front-end calibrated\n");

	return 0;
}

/* wf_pwr_on_consys_mcu() in gen4m soc3_0, and consys_bus_clock_ctrl(). */
static int mt6893_wmac_wfsys_on(struct mt6893_dev *dev)
{
	u32 val;

	mt76_wr(dev, CONN_INFRA_FORCE_ON, 1);
	if (!mt76_poll(dev, CONN_ON2OFF_SLP_PROT_ACK,
		       CONN_ON2OFF_SLP_PROT_ACK_BIT, 0, 10000))
		dev_warn(dev->mt76.dev, "force-on not acknowledged: 0x%08x\n",
			 mt76_rr(dev, CONN_ON2OFF_SLP_PROT_ACK));

	mt6893_wmac_rmw(dev, WFSYS_CPU_SW_RST, WFSYS_CPU_SW_RST_B, 0);

	mt6893_wmac_rmw(dev, CONN_AFE_CTL_PLL, CONN_AFE_CTL_BPLL_EN,
			CONN_AFE_CTL_BPLL_EN);
	usleep_range(30, 60);
	mt6893_wmac_rmw(dev, CONN_AFE_CTL_PLL, CONN_AFE_CTL_WPLL_EN,
			CONN_AFE_CTL_WPLL_EN);
	usleep_range(50, 100);

	val = mt76_rr(dev, WFSYS_ON_TOP_PWR_CTL);
	val = (val & WFSYS_ON_TOP_PWR_KEEP) | WFSYS_ON_TOP_PWR_KEY |
	      WFSYS_ON_TOP_PWR_ON;
	mt76_wr(dev, WFSYS_ON_TOP_PWR_CTL, val);

	if (!mt76_poll(dev, CONNSYS_PWR_STATUS, CONNSYS_PWR_STATUS_WFSYS,
		       CONNSYS_PWR_STATUS_WFSYS, 5000))
		return dev_err_probe(dev->mt76.dev, -ETIMEDOUT,
				     "WFSYS power not acknowledged: 0x%08x\n",
				     mt76_rr(dev, CONNSYS_PWR_STATUS));

	mt6893_wmac_rmw(dev, CONN_WF_SLP_CTRL, CONN_SLP_PROT_EN, 0);
	if (!mt76_poll(dev, CONN_WF_SLP_CTRL, CONN_SLP_PROT_ACK, 0, 50000))
		return dev_err_probe(dev->mt76.dev, -ETIMEDOUT,
				     "conn_infra to WFSYS protection: 0x%08x\n",
				     mt76_rr(dev, CONN_WF_SLP_CTRL));

	mt6893_wmac_rmw(dev, CONN_WFDMA_SLP_CTRL, CONN_SLP_PROT_EN, 0);
	if (!mt76_poll(dev, CONN_WFDMA_SLP_CTRL, CONN_SLP_PROT_ACK, 0, 50000))
		return dev_err_probe(dev->mt76.dev, -ETIMEDOUT,
				     "WFDMA to conn_infra protection: 0x%08x\n",
				     mt76_rr(dev, CONN_WFDMA_SLP_CTRL));

	mt6893_wmac_rmw(dev, CONN_INFRA_BRCM_VDNR, CONN_INFRA_BRCM_WF_VDNR_EN,
			CONN_INFRA_BRCM_WF_VDNR_EN);

	if (!mt76_poll(dev, WFSYS_VERSION_ID, ~0u, WFSYS_VERSION_ID_MT6893,
		       5000)) {
		dev_err(dev->mt76.dev, "no WFSYS: WFSYS_VERSION_ID 0x%08x\n",
			mt76_rr(dev, WFSYS_VERSION_ID));
		return -ETIMEDOUT;
	}

	dev_info(dev->mt76.dev, "WFSYS_VERSION_ID 0x%08x\n",
		 mt76_rr(dev, WFSYS_VERSION_ID));

	return 0;
}

/* wlanDownloadEMISection(): the image body, header stripped, at its offset. */
static int mt6893_wmac_load_emi(struct mt6893_dev *dev, void __iomem *emi,
				const char *name)
{
	const struct mt6893_rom_hdr *hdr;
	const struct firmware *fw;
	size_t size;
	u32 offset;
	int ret;

	ret = request_firmware(&fw, name, dev->mt76.dev);
	if (ret)
		return dev_err_probe(dev->mt76.dev, ret, "firmware %s\n", name);

	if (fw->size <= sizeof(*hdr)) {
		dev_err(dev->mt76.dev, "%s: %zu bytes, shorter than its header\n",
			name, fw->size);
		ret = -EINVAL;
		goto out;
	}

	hdr = (const struct mt6893_rom_hdr *)fw->data;
	size = fw->size - sizeof(*hdr);
	offset = le32_to_cpu(hdr->addr) & MT6893_EMI_OFFSET;

	if (offset > dev->emi_size || size > dev->emi_size - offset) {
		dev_err(dev->mt76.dev,
			"%s: 0x%zx bytes at offset 0x%06x, out of the region\n",
			name, size, offset);
		ret = -EINVAL;
		goto out;
	}

	memcpy_toio(emi + offset, fw->data + sizeof(*hdr), size);

	dev_info(dev->mt76.dev, "%s: 0x%zx bytes at offset 0x%06x\n",
		 name, size, offset);

out:
	release_firmware(fw);

	return ret;
}

/* The region is no-map, hence the explicit mapping. */
static int mt6893_wmac_load_fw(struct mt6893_dev *dev)
{
	void __iomem *emi;
	int ret;

	emi = ioremap_wc(dev->emi_base, dev->emi_size);
	if (!emi)
		return -ENOMEM;

	ret = mt6893_wmac_load_emi(dev, emi, MT6893_FW_WMMCU);
	if (!ret)
		ret = mt6893_wmac_load_emi(dev, emi, MT6893_FW_WIFI);

	iounmap(emi);

	return ret;
}

/*
 * End of wf_pwr_on_consys_mcu(). Once released, the MCU only goes back to its
 * initial state through a full WFSYS power cycle.
 */
static int mt6893_wmac_mcu_start(struct mt6893_dev *dev)
{
	ktime_t start;

	mt76_wr(dev, CONN_INFRA_AP2WF_REMAP_1, CONN_MCU_CONFG_HS_BASE);
	mt76_wr(dev, WF_DYNAMIC_MCU_EMI_ENTRY, 0);
	mt76_wr(dev, WF_DYNAMIC_WF_EMI_ENTRY, 0);

	mt6893_wmac_rmw(dev, WFSYS_SW_RST, WFSYS_SW_RST_B, 0);
	mt6893_wmac_rmw(dev, WFSYS_CPU_SW_RST, WFSYS_CPU_SW_RST_B,
			WFSYS_CPU_SW_RST_B);
	start = ktime_get();

	/*
	 * The vendor driver waits 50 ms, but the ROM consistently takes 55 ms
	 * here.
	 */
	if (!mt76_poll_msec_tick(dev, WF_ROM_CODE_INDEX, ~0u,
				 WF_ROM_CODE_INDEX_DONE, 500, 1)) {
		dev_err(dev->mt76.dev,
			"ROM did not finish: WF_ROM_CODE_INDEX 0x%08x, pc 0x%08x\n",
			mt76_rr(dev, WF_ROM_CODE_INDEX),
			mt76_rr(dev, WFSYS_CPUPCR));
		return -ETIMEDOUT;
	}

	dev_info(dev->mt76.dev, "MCU running: WF_ROM_CODE_INDEX 0x%08x, after %lld ms\n",
		 mt76_rr(dev, WF_ROM_CODE_INDEX),
		 ktime_ms_delta(ktime_get(), start));

	return 0;
}

/*
 * The firmware owns the block when the ROM exits: registers respond but the
 * WFDMA does not move until the host takes ownership back, as in
 * mt792x_mcu_drv_pmctrl().
 */
int mt6893_wmac_drv_own(struct mt6893_dev *dev)
{
	mt76_wr(dev, CONN_HOST_LPCTL, CONN_LPCR_HOST_CLR_OWN);

	if (!mt76_poll_msec_tick(dev, CONN_HOST_LPCTL,
				 CONN_LPCR_HOST_OWN_SYNC, 0, 50, 1)) {
		dev_err(dev->mt76.dev,
			"firmware keeps ownership: LPCTL 0x%08x\n",
			mt76_rr(dev, CONN_HOST_LPCTL));
		return -ETIMEDOUT;
	}

	dev_info(dev->mt76.dev, "driver own: LPCTL 0x%08x\n",
		 mt76_rr(dev, CONN_HOST_LPCTL));

	return 0;
}

int mt6893_wmac_enable(struct mt6893_dev *dev)
{
	u32 val;
	int ret;

	ret = pm_runtime_resume_and_get(dev->mt76.dev);
	if (ret)
		return dev_err_probe(dev->mt76.dev, ret,
				     "conn power domain\n");

	ret = regulator_set_voltage(dev->vcn13, MT6893_VCN13_UV,
				    MT6893_VCN13_UV);
	if (ret) {
		dev_err(dev->mt76.dev, "vcn13 at 1.3 V: %d\n", ret);
		goto err_suspend;
	}

	ret = regulator_enable(dev->vcn18);
	if (ret) {
		dev_err(dev->mt76.dev, "vcn18: %d\n", ret);
		goto err_suspend;
	}

	ret = regulator_enable(dev->vcn13);
	if (ret) {
		dev_err(dev->mt76.dev, "vcn13: %d\n", ret);
		goto err_vcn18;
	}

	/* XO clock settle time, from the vendor driver. */
	usleep_range(6000, 7000);

	/* The vendor driver carries on without it; CONN_HW_VER decides. */
	if (!mt76_poll(dev, CONN_ON2OFF_SLP_PROT_ACK,
		       CONN_ON2OFF_SLP_PROT_ACK_BIT, 0, 10000))
		dev_warn(dev->mt76.dev, "sleep protection not acknowledged: 0x%08x\n",
			 mt76_rr(dev, CONN_ON2OFF_SLP_PROT_ACK));

	val = mt76_rr(dev, CONN_HW_VER);
	if (val != CONN_HW_VER_MT6893) {
		dev_err(dev->mt76.dev, "no conn_infra: CONN_HW_VER 0x%08x\n", val);
		ret = -ENODEV;
		goto err_vcn13;
	}

	dev_info(dev->mt76.dev, "CONN_HW_VER 0x%08x, CONN_CFG_ID 0x%08x\n",
		 val, mt76_rr(dev, CONN_CFG_ID));

	ret = mt6893_wmac_set_emi(dev);
	if (ret)
		goto err_vcn13;

	ret = mt6893_wmac_adie_on(dev);
	if (ret)
		goto err_vcn13;

	/*
	 * consys_plt_pmic_wifi_power_ctrl() turns the 3.3 V rails on here.
	 * Without them the firmware still scans but receives no beacon.
	 */
	ret = regulator_bulk_enable(ARRAY_SIZE(dev->vcn33), dev->vcn33);
	if (ret) {
		dev_err(dev->mt76.dev, "vcn33: %d\n", ret);
		goto err_vcn13;
	}

	ret = mt6893_wmac_wfsys_on(dev);
	if (ret)
		goto err_vcn33;

	ret = mt6893_wmac_load_fw(dev);
	if (ret)
		goto err_vcn33;

	ret = mt6893_wmac_mcu_start(dev);
	if (ret)
		goto err_vcn33;

	ret = mt6893_wmac_drv_own(dev);
	if (ret)
		goto err_vcn33;

	ret = mt6893_dma_init(dev);
	if (ret)
		goto err_vcn33;

	/* The second interrupt of the node is for subsystem reset, unused. */
	ret = devm_request_irq(dev->mt76.dev, dev->irq, mt6893_irq_handler, 0,
			       KBUILD_MODNAME, dev);
	if (ret) {
		dev_err(dev->mt76.dev, "interrupt %d: %d\n",
			dev->irq, ret);
		goto err_dma;
	}

	mt6893_irq_enable(dev, dev->mt792x.irq_map->rx.all_complete_mask |
			       MT6893_INT_TX_DONE_ALL);

	dev_info(dev->mt76.dev, "interrupt %d armed, mask 0x%08x\n",
		 dev->irq, mt76_rr(dev, MT6893_INT_MASK_CSR));

	ret = mt7921_register_device(&dev->mt792x);
	if (ret)
		goto err_irq;

	return 0;

err_irq:
	mt6893_irq_disable(dev, ~0);
	devm_free_irq(dev->mt76.dev, dev->irq, dev);
err_dma:
	mt6893_dma_cleanup(dev);
err_vcn33:
	regulator_bulk_disable(ARRAY_SIZE(dev->vcn33), dev->vcn33);
err_vcn13:
	regulator_disable(dev->vcn13);
err_vcn18:
	regulator_disable(dev->vcn18);
err_suspend:
	pm_runtime_put_sync(dev->mt76.dev);
	return ret;
}

void mt6893_wmac_disable(struct mt6893_dev *dev)
{
	cancel_work_sync(&dev->mt792x.init_work);
	mt76_unregister_device(&dev->mt76);
	cancel_delayed_work_sync(&dev->mt792x.pm.ps_work);
	cancel_work_sync(&dev->mt792x.pm.wake_work);
	cancel_work_sync(&dev->mt792x.reset_work);
	mt76_connac2_tx_token_put(&dev->mt76);

	set_bit(MT76_REMOVED, &dev->mt76.phy.state);
	mt6893_irq_disable(dev, ~0);
	devm_free_irq(dev->mt76.dev, dev->irq, dev);

	/* The DMA must be stopped before the domain goes down. */
	mt6893_dma_cleanup(dev);
	tasklet_disable(&dev->mt76.irq_tasklet);

	regulator_bulk_disable(ARRAY_SIZE(dev->vcn33), dev->vcn33);
	regulator_disable(dev->vcn13);
	regulator_disable(dev->vcn18);
	pm_runtime_put_sync(dev->mt76.dev);
}

static int mt6893_wmac_init(struct mt6893_dev *dev)
{
	struct device *pdev = dev->mt76.dev;
	int ret;

	dev->vcn18 = devm_regulator_get(pdev, "vcn18");
	if (IS_ERR(dev->vcn18))
		return dev_err_probe(pdev, PTR_ERR(dev->vcn18),
				     "vcn18 supply\n");

	dev->vcn13 = devm_regulator_get(pdev, "vcn13");
	if (IS_ERR(dev->vcn13))
		return dev_err_probe(pdev, PTR_ERR(dev->vcn13),
				     "vcn13 supply\n");

	dev->vcn33[0].supply = "vcn33-1";
	dev->vcn33[1].supply = "vcn33-2";
	ret = devm_regulator_bulk_get(pdev, ARRAY_SIZE(dev->vcn33), dev->vcn33);
	if (ret)
		return dev_err_probe(pdev, ret, "vcn33 supplies\n");

	dev->pinctrl = devm_pinctrl_get(pdev);
	if (IS_ERR(dev->pinctrl))
		return dev_err_probe(pdev, PTR_ERR(dev->pinctrl), "pinctrl\n");

	dev->pins_strap = pinctrl_lookup_state(dev->pinctrl, "strap");
	if (IS_ERR(dev->pins_strap))
		return dev_err_probe(pdev, PTR_ERR(dev->pins_strap),
				     "strap pinctrl state\n");

	return devm_pm_runtime_enable(pdev);
}

static int mt6893_wmac_probe(struct platform_device *pdev)
{
	struct mt6893_dev *dev;
	void __iomem *mem_base;
	int ret;

	mem_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mem_base))
		return PTR_ERR(mem_base);

	dev = mt6893_mmio_probe(&pdev->dev, mem_base);
	if (IS_ERR(dev))
		return PTR_ERR(dev);

	platform_set_drvdata(pdev, dev);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto free_device;
	dev->irq = ret;

	ret = mt6893_wmac_init(dev);
	if (ret)
		goto free_device;

	ret = mt6893_wmac_enable(dev);
	if (ret)
		goto free_device;

	return 0;

free_device:
	mt76_free_device(&dev->mt76);

	return ret;
}

static void mt6893_wmac_remove(struct platform_device *pdev)
{
	struct mt6893_dev *dev = platform_get_drvdata(pdev);

	mt6893_wmac_disable(dev);
	mt76_free_device(&dev->mt76);
}

/* A DMA left running hangs the reboot. */
static void mt6893_wmac_shutdown(struct platform_device *pdev)
{
	struct mt6893_dev *dev = platform_get_drvdata(pdev);

	mt6893_dma_disable(dev);
}

static const struct of_device_id mt6893_wmac_of_match[] = {
	{ .compatible = "mediatek,mt6893-wmac" },
	{}
};
MODULE_DEVICE_TABLE(of, mt6893_wmac_of_match);

static struct platform_driver mt6893_wmac_driver = {
	.driver = {
		.name = "mt6893-wmac",
		.of_match_table = mt6893_wmac_of_match,
	},
	.probe = mt6893_wmac_probe,
	.remove = mt6893_wmac_remove,
	.shutdown = mt6893_wmac_shutdown,
};
module_platform_driver(mt6893_wmac_driver);

MODULE_FIRMWARE(MT6893_FW_WMMCU);
MODULE_FIRMWARE(MT6893_FW_WIFI);
MODULE_FIRMWARE(MT6893_FW_ROM_PATCH);
MODULE_FIRMWARE(MT6893_FW_WM);
MODULE_FIRMWARE(MT6893_FW_NVRAM);

MODULE_DESCRIPTION("MediaTek MT6893 WiFi driver");
MODULE_LICENSE("GPL");
