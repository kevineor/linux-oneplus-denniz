/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Copyright (c) 2026 Kevin Andrieux
 *
 * MT6893 connectivity subsystem: a connac2 WiFi core on the SoC internal bus,
 * without a WA CPU, with two WFDMA blocks and a firmware the host places in
 * shared memory before releasing the MCU from reset.
 */

#ifndef __MT6893_H
#define __MT6893_H

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include "../mt792x.h"
#include "../mt7921/mt7921.h"
#include "regs.h"

#define MT6893_VCN13_UV			1300000

#define MT6893_EMI_SIZE			0x450000
#define MT6893_EMI_OFFSET		GENMASK(23, 0)

/* Loaded into shared memory before the MCU leaves reset, in this order. */
#define MT6893_FW_WMMCU			"mediatek/mt6893/mt6893_wmmcu.bin"
#define MT6893_FW_WIFI			"mediatek/mt6893/mt6893_wifi.bin"

/*
 * The ROM patch goes to MCU memory through the AP2WF window. Its destination
 * is not in the file: it is SOC3_0_PATCH_START_ADDR in the vendor driver.
 */
#define MT6893_FW_ROM_PATCH		"mediatek/mt6893/mt6893_rom_patch.bin"
#define MT6893_PATCH_ADDR		0x00100000
#define MT6893_AP2WF_WINDOW		0x10000

/* connac2 RAM image; feature_set selects shared memory or the AP2WF window. */
#define MT6893_FW_WM			"mediatek/mt6893/mt6893_wm.bin"
#define MT6893_FW_REGION_EMI		BIT(7)
#define MT6893_FW_REGION_SKIP		BIT(6)

/*
 * Per-device calibration, the APCFG/APRDEB/WIFI file of the Android NVRAM
 * (WIFI_CFG_PARAM_STRUCT): a 256-byte header holding the MAC address, then a
 * chain of TLV elements.
 */
#define MT6893_FW_NVRAM			"mediatek/mt6893/mt6893_nvram.bin"
#define MT6893_NVRAM_HDR_LEN		256
#define MT6893_NVRAM_MAC_OFFSET		4
#define MT6893_NVRAM_FRAG_LEN		1500

#define MT6893_FUNC_INIT_DONE		BIT(0)
#define MT6893_FUNC_WM_DONE		BIT(1)
#define MT6893_FUNC_READY		(MT6893_FUNC_INIT_DONE | \
					 MT6893_FUNC_WM_DONE)

/*
 * ROM_EMI_HEADER. The low 24 bits of the connsys-side address are the offset
 * in the shared memory region.
 */
struct mt6893_rom_hdr {
	u8 date[16];
	u8 platform[4];
	__le16 hw_ver;
	__le16 sw_ver;
	__le32 addr;
	__le32 type;
	__le32 crc[4];
} __packed;

#define MT6893_MCUQ_ID_FWDL		16
#define MT6893_MCUQ_ID_WM		17
#define MT6893_RXQ_ID_MCU		0

#define MT6893_TXQ_ID_BAND0		0
#define MT6893_RXQ_ID_DATA0		0
#define MT6893_RXQ_ID_DATA1		1
#define MT6893_RXQ_ID_TXFREE		2

#define MT6893_TX_MCU_RING_SIZE		256
#define MT6893_TX_FWDL_RING_SIZE	128
#define MT6893_RX_MCU_RING_SIZE		512

/*
 * The MT6893 WiFi core is an MT7961 behind the SoC bus, so this driver is a
 * bus variant of mt7921 like mt7921e/s/u: power-on, firmware download, DMA
 * and interrupts live here, everything else in mt7921.
 */
struct mt6893_dev {
	union {
		struct mt792x_dev mt792x;
		struct mt76_dev mt76;
	};

	struct regulator *vcn18;
	struct regulator *vcn13;
	struct regulator_bulk_data vcn33[2];
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_strap;

	phys_addr_t emi_base;
	resource_size_t emi_size;

	int irq;
};

static inline struct mt6893_dev *to_mt6893(struct mt792x_dev *dev)
{
	return container_of(dev, struct mt6893_dev, mt792x);
}

static inline void mt6893_irq_enable(struct mt6893_dev *dev, u32 mask)
{
	mt76_set_irq_mask(&dev->mt76, MT6893_INT_MASK_CSR, 0, mask);
}

static inline void mt6893_irq_disable(struct mt6893_dev *dev, u32 mask)
{
	mt76_set_irq_mask(&dev->mt76, MT6893_INT_MASK_CSR, mask, 0);
}

struct mt6893_dev *mt6893_mmio_probe(struct device *pdev,
				     void __iomem *mem_base);
irqreturn_t mt6893_irq_handler(int irq, void *dev_instance);
void mt6893_irq_tasklet(struct tasklet_struct *t);
int mt6893_mcu_init(struct mt792x_dev *mdev);
int mt6893_mcu_load_firmware(struct mt792x_dev *mdev);
int mt6893_dma_init(struct mt6893_dev *dev);
void mt6893_dma_cleanup(struct mt6893_dev *dev);
void mt6893_dma_disable(struct mt6893_dev *dev);
int mt6893_wmac_enable(struct mt6893_dev *dev);
void mt6893_wmac_disable(struct mt6893_dev *dev);
int mt6893_wmac_drv_own(struct mt6893_dev *dev);

#endif /* __MT6893_H */
