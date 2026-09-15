// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Copyright (c) 2026 Kevin Andrieux
 *
 * Firmware download and calibration.
 *
 * Images are written directly to MCU memory or shared memory, as
 * downloadImgByDynMemMap() does; only the commands handing them over to the
 * ROM go through the rings, in the regular connac2 MCU message format.
 */

#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/unaligned.h>

#include "mt6893.h"
#include "../mt76_connac_mcu.h"

/* PATCH_FORMAT_T */
struct mt6893_patch_hdr {
	u8 date[16];
	u8 platform[4];
	__le32 hw_sw_ver;
	__le32 patch_ver;
	__le16 crc;
} __packed;

/* INIT_CMD_WIFI_START */
struct mt6893_fw_start {
	__le32 override;
	__le32 addr;
} __packed;

/* INIT_CMD_PATCH_FINISH */
struct mt6893_patch_finish {
	u8 check_crc;
	u8 rsv[3];
} __packed;

/*
 * As mt7921e, except that everything goes to the WM queue: firmware images
 * are never sent through the FWDL ring.
 */
static int mt6893_mcu_send_message(struct mt76_dev *mdev, struct sk_buff *skb,
				   int cmd, int *wait_seq)
{
	struct mt6893_dev *dev = container_of(mdev, struct mt6893_dev, mt76);
	int ret;

	/*
	 * The firmware does not know unified commands and never answers them,
	 * and the resulting timeout would trigger a chip reset.
	 */
	if (cmd & __MCU_CMD_FIELD_UNI) {
		dev_warn_ratelimited(mdev->dev,
				     "unified command 0x%08x rejected\n", cmd);
		dev_kfree_skb(skb);
		return -EOPNOTSUPP;
	}

	ret = mt76_connac2_mcu_fill_message(mdev, skb, cmd, wait_seq);
	if (ret)
		return ret;

	/* ROM RF calibration takes up to five seconds. */
	mdev->mcu.timeout = cmd == MCU_CMD(PHY_ACTION) ? 6 * HZ : 3 * HZ;

	dev_dbg(mdev->dev, "command 0x%08x, seq %d, %u bytes\n", cmd,
		wait_seq ? *wait_seq : -1, skb->len);

	return mt76_tx_queue_skb_raw(dev, mdev->q_mcu[MT_MCUQ_WM], skb, 0);
}

/* Write to MCU memory through the 64 KiB AP2WF window, one window at a time. */
static void mt6893_mcu_wr_window(struct mt6893_dev *dev, u32 addr,
				 const u8 *data, size_t len)
{
	while (len) {
		u32 base = addr & ~(MT6893_AP2WF_WINDOW - 1);
		u32 off = addr - base;
		size_t n = min(len, (size_t)MT6893_AP2WF_WINDOW - off);

		mt76_wr(dev, CONN_INFRA_AP2WF_REMAP_1, base);
		mt76_wr_copy(dev, WF_DYNAMIC + off, data, n);

		addr += n;
		data += n;
		len -= n;
	}
}

static int mt6893_mcu_load_patch(struct mt6893_dev *dev)
{
	const struct mt6893_patch_hdr *hdr;
	const struct firmware *fw;
	size_t size;
	int ret;

	ret = request_firmware(&fw, MT6893_FW_ROM_PATCH, dev->mt76.dev);
	if (ret)
		return dev_err_probe(dev->mt76.dev, ret, "firmware %s\n",
				     MT6893_FW_ROM_PATCH);

	if (fw->size <= sizeof(*hdr)) {
		dev_err(dev->mt76.dev, "%s: %zu bytes, shorter than its header\n",
			MT6893_FW_ROM_PATCH, fw->size);
		ret = -EINVAL;
		goto out;
	}

	hdr = (const struct mt6893_patch_hdr *)fw->data;
	size = fw->size - sizeof(*hdr);

	dev_info(dev->mt76.dev,
		 "ROM patch %.16s, version 0x%08x: 0x%zx bytes to 0x%08x\n",
		 hdr->date, le32_to_cpu(hdr->patch_ver), size,
		 MT6893_PATCH_ADDR);

	mt6893_mcu_wr_window(dev, MT6893_PATCH_ADDR, fw->data + sizeof(*hdr),
			     size);

out:
	release_firmware(fw);

	return ret;
}

/*
 * Regions flagged EMI go to shared memory, at the offset given by the low 24
 * bits of their address; the others go to MCU memory through the window.
 */
static int mt6893_mcu_load_region(struct mt6893_dev *dev,
				  const struct mt76_connac2_fw_region *region,
				  const u8 *data, void __iomem **emi)
{
	u32 addr = le32_to_cpu(region->addr);
	u32 len = le32_to_cpu(region->len);
	u32 offset;

	if (region->feature_set & MT6893_FW_REGION_SKIP) {
		dev_info(dev->mt76.dev, "region 0x%08x: skipped\n", addr);
		return 0;
	}

	if (!(region->feature_set & MT6893_FW_REGION_EMI)) {
		mt6893_mcu_wr_window(dev, addr, data, len);
		dev_info(dev->mt76.dev,
			 "region 0x%08x: 0x%x bytes, through the window\n",
			 addr, len);
		return 0;
	}

	offset = addr & MT6893_EMI_OFFSET;
	if (offset > dev->emi_size || len > dev->emi_size - offset) {
		dev_err(dev->mt76.dev,
			"region 0x%08x: 0x%x bytes out of shared memory\n",
			addr, len);
		return -EINVAL;
	}

	if (!*emi) {
		*emi = ioremap_wc(dev->emi_base, dev->emi_size);
		if (!*emi)
			return -ENOMEM;
	}

	memcpy_toio(*emi + offset, data, len);
	dev_info(dev->mt76.dev,
		 "region 0x%08x: 0x%x bytes at EMI offset 0x%06x\n",
		 addr, len, offset);

	return 0;
}

/* connac2 RAM image: regions, then the region table, then the trailer. */
static int mt6893_mcu_load_ram(struct mt6893_dev *dev)
{
	const struct mt76_connac2_fw_region *regions;
	const struct mt76_connac2_fw_trailer *hdr;
	const struct firmware *fw;
	void __iomem *emi = NULL;
	size_t table, offset = 0;
	int ret, i;

	ret = request_firmware(&fw, MT6893_FW_WM, dev->mt76.dev);
	if (ret)
		return dev_err_probe(dev->mt76.dev, ret, "firmware %s\n",
				     MT6893_FW_WM);

	if (fw->size <= sizeof(*hdr)) {
		dev_err(dev->mt76.dev, "%s: %zu bytes, shorter than its header\n",
			MT6893_FW_WM, fw->size);
		ret = -EINVAL;
		goto out;
	}

	hdr = (const void *)(fw->data + fw->size - sizeof(*hdr));
	table = (size_t)hdr->n_region * sizeof(*regions);

	if (fw->size < sizeof(*hdr) + table) {
		dev_err(dev->mt76.dev, "%s: %u regions, file too short\n",
			MT6893_FW_WM, hdr->n_region);
		ret = -EINVAL;
		goto out;
	}

	regions = (const void *)(fw->data + fw->size - sizeof(*hdr) - table);

	dev_info(dev->mt76.dev, "WM firmware %.10s, %.15s, %u regions\n",
		 hdr->fw_ver, hdr->build_date, hdr->n_region);

	for (i = 0; i < hdr->n_region; i++) {
		u32 len = le32_to_cpu(regions[i].len);

		if (len > fw->size - sizeof(*hdr) - table - offset) {
			dev_err(dev->mt76.dev,
				"region %d: 0x%x bytes, out of the file\n",
				i, len);
			ret = -EINVAL;
			break;
		}

		ret = mt6893_mcu_load_region(dev, &regions[i],
					     fw->data + offset, &emi);
		if (ret)
			break;

		offset += len;
	}

	if (emi)
		iounmap(emi);

out:
	release_firmware(fw);

	return ret;
}

/*
 * As wlanLoadManufactureData(): one element per zero-padded fixed-size
 * message, the first one with the file header, until a null element.
 */
static int mt6893_mcu_load_nvram(struct mt6893_dev *dev)
{
	struct mt76_phy *phy = &dev->mt76.phy;
	size_t start = 0, off = MT6893_NVRAM_HDR_LEN;
	const struct firmware *fw;
	int ret, n = 0;
	u8 *frag;

	ret = firmware_request_nowarn(&fw, MT6893_FW_NVRAM, dev->mt76.dev);
	if (ret) {
		dev_warn(dev->mt76.dev, "%s missing: no calibration\n",
			 MT6893_FW_NVRAM);
		return 0;
	}

	if (fw->size < MT6893_NVRAM_HDR_LEN) {
		dev_err(dev->mt76.dev, "%s: %zu bytes, shorter than its header\n",
			MT6893_FW_NVRAM, fw->size);
		ret = -EINVAL;
		goto out;
	}

	if (is_valid_ether_addr(fw->data + MT6893_NVRAM_MAC_OFFSET))
		memcpy(phy->macaddr, fw->data + MT6893_NVRAM_MAC_OFFSET,
		       ETH_ALEN);

	frag = kmalloc(MT6893_NVRAM_FRAG_LEN, GFP_KERNEL);
	if (!frag) {
		ret = -ENOMEM;
		goto out;
	}

	while (off + 3 <= fw->size) {
		const u8 *tag = fw->data + off;
		size_t len = tag[1] | tag[2] << 8;

		if (!tag[0] && !len)
			break;

		off += 3 + len;
		if (off > fw->size || off - start > MT6893_NVRAM_FRAG_LEN) {
			dev_err(dev->mt76.dev,
				"%s: element %u of %zu bytes, out of bounds\n",
				MT6893_FW_NVRAM, tag[0], len);
			ret = -EINVAL;
			break;
		}

		memset(frag, 0, MT6893_NVRAM_FRAG_LEN);
		memcpy(frag, fw->data + start, off - start);

		ret = mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SET_NVRAM), frag,
					MT6893_NVRAM_FRAG_LEN, false);
		if (ret)
			break;

		start = off;
		n++;
	}

	kfree(frag);

	if (!ret)
		dev_info(dev->mt76.dev, "calibration: %d elements, address %pM\n",
			 n, phy->macaddr);

out:
	release_firmware(fw);

	return ret;
}

#define MT6893_PHY_ACTION_MAGIC		0x556789aa
#define MT6893_PHY_ACTION_FORCE_CAL	0x01

enum {
	MT6893_PHY_TAG_NVRAM,
	MT6893_PHY_TAG_CAL,
	MT6893_PHY_TAG_COM_FEM,
	MT6893_PHY_TAG_LAA,
};

/* NVRAM tags 7 to 9 go with the PhyAction. */
#define MT6893_NVRAM_TAG_2G4_COMMON	7
#define MT6893_NVRAM_TAG_CO_ANT		10
#define MT6893_PHY_NVRAM_MAX		512

struct mt6893_phy_tlv {
	__le16 tag;
	__le16 len;
} __packed;

/*
 * connfem_epaelna_pin_info and connfem_epaelna_laa_pin_info, all zero: this
 * board has no external FEM.
 */
struct mt6893_phy_com_fem {
	__le32 fem_id;
	__le32 count;
	u8 pin[32][3];
} __packed;

struct mt6893_phy_laa {
	__le32 chip_id;
	__le32 count;
	u8 pin[8][4];
} __packed;

struct mt6893_phy_rsp {
	__le32 magic;
	u8 n_tag;
	u8 version;
	__le16 len;
	struct mt6893_phy_tlv tlv;
	u8 event;
	u8 status;
	u8 rsv[2];
	__le32 emi_addr;
	__le32 emi_len;
	__le32 temperature;
} __packed;

/*
 * wlanGetEpaElnaFromNvram(): the 2.4 GHz common, 5 GHz common and system
 * NVRAM tags, headers included.
 */
static int mt6893_mcu_nvram_fem(const struct firmware *fw, const u8 **data,
				size_t *len)
{
	size_t off = MT6893_NVRAM_HDR_LEN, start = 0;

	while (off + 3 <= fw->size) {
		const u8 *tag = fw->data + off;
		size_t n = tag[1] | tag[2] << 8;

		if (!tag[0] && !n)
			break;
		if (tag[0] == MT6893_NVRAM_TAG_2G4_COMMON)
			start = off;
		if (tag[0] == MT6893_NVRAM_TAG_CO_ANT && start) {
			*data = fw->data + start;
			*len = off - start;
			return 0;
		}
		off += 3 + n;
	}

	return -ENOENT;
}

/*
 * wlanSendPhyAction() with HAL_PHY_ACTION_CAL_FORCE_CAL_REQ: between the ROM
 * patch and the firmware, the ROM calibrates the RF and stores the result in
 * shared memory. Without it the radio receives nothing.
 */
static int mt6893_mcu_phy_action(struct mt6893_dev *dev)
{
	struct mt6893_phy_com_fem fem = {};
	struct mt6893_phy_laa laa = {};
	const struct mt6893_phy_rsp *rsp;
	const struct firmware *fw;
	struct sk_buff *skb;
	size_t nv_len, nv_pad;
	const u8 *nv;
	u8 cal[4] = { MT6893_PHY_ACTION_FORCE_CAL };
	u16 buf_len;
	int ret;

	ret = request_firmware(&fw, MT6893_FW_NVRAM, dev->mt76.dev);
	if (ret)
		return ret;

	ret = mt6893_mcu_nvram_fem(fw, &nv, &nv_len);
	if (ret) {
		dev_err(dev->mt76.dev, "NVRAM tags 7 to 9 missing\n");
		goto out;
	}
	nv_pad = min_t(size_t, ALIGN(nv_len, 4), MT6893_PHY_NVRAM_MAX);

	buf_len = 4 * sizeof(struct mt6893_phy_tlv) + sizeof(cal) + nv_pad +
		  sizeof(fem) + sizeof(laa);

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, 8 + buf_len);
	if (!skb) {
		ret = -ENOMEM;
		goto out;
	}

	put_unaligned_le32(MT6893_PHY_ACTION_MAGIC, skb_put(skb, 4));
	skb_put_u8(skb, 4);			/* number of tags */
	skb_put_u8(skb, 1);			/* version */
	put_unaligned_le16(buf_len, skb_put(skb, 2));

#define PUT_TLV(_tag, _data, _len) do {					\
		struct mt6893_phy_tlv *t = skb_put(skb, sizeof(*t));	\
		t->tag = cpu_to_le16(_tag);				\
		t->len = cpu_to_le16(_len);				\
		skb_put_data(skb, _data, _len);				\
	} while (0)

	PUT_TLV(MT6893_PHY_TAG_CAL, cal, sizeof(cal));
	{
		struct mt6893_phy_tlv *t = skb_put(skb, sizeof(*t));

		t->tag = cpu_to_le16(MT6893_PHY_TAG_NVRAM);
		t->len = cpu_to_le16(nv_pad);
		memcpy(skb_put_zero(skb, nv_pad), nv, min(nv_len, nv_pad));
	}
	PUT_TLV(MT6893_PHY_TAG_COM_FEM, &fem, sizeof(fem));
	PUT_TLV(MT6893_PHY_TAG_LAA, &laa, sizeof(laa));
#undef PUT_TLV

	release_firmware(fw);
	fw = NULL;

	ret = mt76_mcu_skb_send_and_get_msg(&dev->mt76, skb,
					    MCU_CMD(PHY_ACTION), true, &skb);
	if (ret) {
		dev_err(dev->mt76.dev, "PhyAction failed: %d\n", ret);
		return ret;
	}

	/* The init event starts four bytes before what mt76 strips. */
	skb_push(skb, 4);
	if (skb->len < sizeof(*rsp)) {
		dev_err(dev->mt76.dev, "PhyAction: %u-byte response\n",
			skb->len);
		print_hex_dump(KERN_INFO, "phy: ", DUMP_PREFIX_OFFSET, 16, 1,
			       skb->data, skb->len, false);
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	rsp = (const struct mt6893_phy_rsp *)skb->data;
	dev_info(dev->mt76.dev,
		 "PhyAction: magic 0x%08x, tag %u, event 0x%02x, status %u, EMI 0x%08x+0x%x, %u degrees\n",
		 le32_to_cpu(rsp->magic), le16_to_cpu(rsp->tlv.tag), rsp->event,
		 rsp->status, le32_to_cpu(rsp->emi_addr),
		 le32_to_cpu(rsp->emi_len), le32_to_cpu(rsp->temperature));
	ret = rsp->status ? -EIO : 0;
	dev_kfree_skb(skb);

	return ret;

out:
	release_firmware(fw);
	return ret;
}

/* Replaces mt792x_load_firmware(). */
int mt6893_mcu_load_firmware(struct mt792x_dev *mdev)
{
	struct mt6893_dev *dev = to_mt6893(mdev);
	struct mt6893_patch_finish req = {};
	struct mt6893_fw_start start = {};
	int ret;

	ret = mt6893_mcu_load_patch(dev);
	if (ret)
		return ret;

	ret = mt76_mcu_send_msg(&dev->mt76,
				MCU_CMD(DYN_MEM_MAP_PATCH_FINISH),
				&req, sizeof(req), true);
	if (ret) {
		dev_err(dev->mt76.dev, "ROM patch rejected: %d\n",
			ret);
		return ret;
	}

	ret = mt6893_mcu_phy_action(dev);
	if (ret)
		return ret;

	ret = mt6893_mcu_load_ram(dev);
	if (ret)
		return ret;

	ret = mt76_mcu_send_msg(&dev->mt76, MCU_CMD(DYN_MEM_MAP_FW_FINISH),
				&start, sizeof(start), true);
	if (ret) {
		dev_err(dev->mt76.dev, "firmware rejected: %d\n",
			ret);
		return ret;
	}

	if (!mt76_poll_msec_tick(dev, CONN_ON_MISC, MT6893_FUNC_READY,
				 MT6893_FUNC_READY, 1000, 10)) {
		dev_err(dev->mt76.dev,
			"firmware not ready: CONN_ON_MISC 0x%08x, pc 0x%08x\n",
			mt76_rr(dev, CONN_ON_MISC), mt76_rr(dev, WFSYS_CPUPCR));
		return -ETIMEDOUT;
	}

	dev_info(dev->mt76.dev, "WM firmware running: CONN_ON_MISC 0x%08x\n",
		 mt76_rr(dev, CONN_ON_MISC));

	return 0;
}

/*
 * As mt7921e, without the ownership handshake. The calibration follows the
 * capabilities query, as in the vendor driver, and provides the MAC address.
 */
int mt6893_mcu_init(struct mt792x_dev *mdev)
{
	static const struct mt76_mcu_ops mt6893_mcu_ops = {
		.headroom = sizeof(struct mt76_connac2_mcu_txd),
		.mcu_skb_send_msg = mt6893_mcu_send_message,
		.mcu_parse_response = mt7921_mcu_parse_response,
	};
	struct mt6893_dev *dev = to_mt6893(mdev);
	int ret;

	dev->mt76.mcu_ops = &mt6893_mcu_ops;

	ret = mt7921_run_firmware(mdev);
	if (ret)
		return ret;

	ret = mt6893_mcu_load_nvram(dev);
	if (ret)
		return ret;

	/*
	 * kalSetSuspendFlagToEMI(): host awake (0x22222222) or suspended
	 * (0x11111111). The firmware sends nothing to a host it thinks asleep.
	 */
	if (mdev->emi_host_status) {
		u32 off = mdev->emi_host_status & MT6893_EMI_OFFSET;
		void __iomem *p;

		if (off + 4 <= dev->emi_size) {
			p = ioremap(dev->emi_base + off, 4);
			if (p) {
				dev_info(dev->mt76.dev,
					 "host status in EMI at 0x%06x: 0x%08x\n",
					 off, readl(p));
				writel(0x22222222, p);
				iounmap(p);
			}
		}
	}

	/* wlanUpdateBasicConfig(), without checksum offload. */
	{
		u8 basic[16] = {};

		ret = mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(BASIC_CONFIG),
					basic, sizeof(basic), false);
	}

	mt76_queue_tx_cleanup(dev, dev->mt76.q_mcu[MT_MCUQ_FWDL], false);

	return ret;
}
