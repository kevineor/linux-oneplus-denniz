// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Copyright (c) 2026 Kevin Andrieux
 *
 * mt7921 glue: register address translation, TX descriptor preparation and
 * the mt792x_hif_ops of the SoC bus.
 */

#include "mt6893.h"
#include "../dma.h"
#include "../mt76_connac2_mac.h"

/*
 * soc3_0_bus2chip_cr_mapping: the mt7921e table, shifted by 0x400000 and
 * relative to the node base at 0x18000000.
 */
static const struct mt76_connac_reg_map mt6893_fixed_map[] = {
	{ 0x54000000, 0x402000, 0x01000 }, /* WFDMA PCIE0 MCU DMA0 */
	{ 0x55000000, 0x403000, 0x01000 }, /* WFDMA PCIE0 MCU DMA1 */
	{ 0x56000000, 0x404000, 0x01000 }, /* WFDMA reserved */
	{ 0x57000000, 0x405000, 0x01000 }, /* WFDMA MCU wrap CR */
	{ 0x58000000, 0x406000, 0x01000 }, /* WFDMA PCIE1 MCU DMA0 (MEM_DMA) */
	{ 0x59000000, 0x407000, 0x01000 }, /* WFDMA PCIE1 MCU DMA1 */
	{ 0x820c0000, 0x408000, 0x04000 }, /* WF_UMAC_TOP (PLE) */
	{ 0x820c8000, 0x40c000, 0x02000 }, /* WF_UMAC_TOP (PSE) */
	{ 0x820cc000, 0x40e000, 0x02000 }, /* WF_UMAC_TOP (PP) */
	{ 0x820e0000, 0x420000, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_CFG) */
	{ 0x820e1000, 0x420400, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_TRB) */
	{ 0x820e2000, 0x420800, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_AGG) */
	{ 0x820e3000, 0x420c00, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_ARB) */
	{ 0x820e4000, 0x421000, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_TMAC) */
	{ 0x820e5000, 0x421400, 0x00800 }, /* WF_LMAC_TOP BN0 (WF_RMAC) */
	{ 0x820ce000, 0x421c00, 0x00200 }, /* WF_LMAC_TOP (WF_SEC) */
	{ 0x820e7000, 0x421e00, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_DMA) */
	{ 0x820cf000, 0x422000, 0x01000 }, /* WF_LMAC_TOP (WF_PF) */
	{ 0x820e9000, 0x423400, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_WTBLOFF) */
	{ 0x820ea000, 0x424000, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_ETBF) */
	{ 0x820eb000, 0x424200, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_LPON) */
	{ 0x820ec000, 0x424600, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_INT) */
	{ 0x820ed000, 0x424800, 0x00800 }, /* WF_LMAC_TOP BN0 (WF_MIB) */
	{ 0x820ca000, 0x426000, 0x02000 }, /* WF_LMAC_TOP BN0 (WF_MUCOP) */
	{ 0x820d0000, 0x430000, 0x10000 }, /* WF_LMAC_TOP (WF_WTBLON) */
	{ 0x40000000, 0x470000, 0x10000 }, /* WF_UMAC_SYSRAM */
	{ 0x00400000, 0x480000, 0x10000 }, /* WF_MCU_SYSRAM */
	{ 0x00410000, 0x490000, 0x10000 }, /* WF_MCU_SYSRAM (configuration) */
	{ 0x820f0000, 0x4a0000, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_CFG) */
	{ 0x820f1000, 0x4a0600, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_TRB) */
	{ 0x820f2000, 0x4a0800, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_AGG) */
	{ 0x820f3000, 0x4a0c00, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_ARB) */
	{ 0x820f4000, 0x4a1000, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_TMAC) */
	{ 0x820f5000, 0x4a1400, 0x00800 }, /* WF_LMAC_TOP BN1 (WF_RMAC) */
	{ 0x820f7000, 0x4a1e00, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_DMA) */
	{ 0x820f9000, 0x4a3400, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_WTBLOFF) */
	{ 0x820fa000, 0x4a4000, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_ETBF) */
	{ 0x820fb000, 0x4a4200, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_LPON) */
	{ 0x820fc000, 0x4a4600, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_INT) */
	{ 0x820fd000, 0x4a4800, 0x00800 }, /* WF_LMAC_TOP BN1 (WF_MIB) */
	{ 0x820c4000, 0x4a8000, 0x04000 }, /* WF_LMAC_TOP (WF_UWTBL) */
	{ 0x820b0000, 0x4ae000, 0x01000 }, /* WFSYS_ON */
	{ 0x80020000, 0x4b0000, 0x10000 }, /* WF_TOP_MISC_OFF */
	{ 0x81020000, 0x4c0000, 0x10000 }, /* WF_TOP_MISC_ON */
	{ 0x7c500000, 0x500000, 0x10000 }, /* CONN_INFRA, dyn mem map */
	{ 0x7c000000, 0x000000, 0x10000 }, /* CONN_INFRA, conn_infra_on */
	{ 0x7c020000, 0x020000, 0x10000 }, /* CONN_INFRA, wfdma */
	{ 0x7c050000, 0x050000, 0x10000 }, /* CONN_INFRA, sysram */
	{ 0x7c060000, 0x060000, 0x10000 }, /* CONN_INFRA, conn_host_csr_top */
};

/*
 * Three kinds of addresses: mt7921 WFDMA offsets (MT_WFDMA0_BASE, 0xd0000),
 * which map to our WFDMA block at 0x20000; offsets already within the node,
 * used by the power-on code; and chip addresses, translated by the table. The
 * MCU memory range 0x400000-0x41ffff is never accessed directly and always
 * goes through the table.
 */
static u32 mt6893_reg_addr(struct mt792x_dev *dev, u32 addr)
{
	int i;

	if (addr >= 0xd0000 && addr < 0xe0000)
		return addr - 0xb0000;

	if (addr < 0x700000 && (addr < 0x400000 || addr >= 0x420000))
		return addr;

	for (i = 0; i < ARRAY_SIZE(mt6893_fixed_map); i++) {
		u32 ofs;

		if (addr < mt6893_fixed_map[i].phys)
			continue;

		ofs = addr - mt6893_fixed_map[i].phys;
		if (ofs >= mt6893_fixed_map[i].size)
			continue;

		return mt6893_fixed_map[i].maps + ofs;
	}

	dev_err(dev->mt76.dev, "address 0x%08x not in the map\n", addr);

	return 0;
}

static u32 mt6893_rr(struct mt76_dev *mdev, u32 offset)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);

	return dev->bus_ops->rr(mdev, mt6893_reg_addr(dev, offset));
}

static void mt6893_wr(struct mt76_dev *mdev, u32 offset, u32 val)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);

	dev->bus_ops->wr(mdev, mt6893_reg_addr(dev, offset), val);
}

static u32 mt6893_rmw(struct mt76_dev *mdev, u32 offset, u32 mask, u32 val)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);

	return dev->bus_ops->rmw(mdev, mt6893_reg_addr(dev, offset), mask, val);
}

/* Same descriptor format as mt7921e_tx_prepare_skb(). */
static int mt6893_tx_prepare_skb(struct mt76_dev *mdev, void *txwi_ptr,
				 enum mt76_txq_id qid, struct mt76_wcid *wcid,
				 struct ieee80211_sta *sta,
				 struct mt76_tx_info *tx_info)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(tx_info->skb);
	struct ieee80211_key_conf *key = info->control.hw_key;
	struct mt76_connac_hw_txp *txp;
	struct mt76_txwi_cache *t;
	u8 *txwi = (u8 *)txwi_ptr;
	int id, pid;

	if (unlikely(tx_info->skb->len <= ETH_HLEN))
		return -EINVAL;

	if (!wcid)
		wcid = &dev->mt76.global_wcid;

	t = (struct mt76_txwi_cache *)(txwi + mdev->drv->txwi_size);
	t->skb = tx_info->skb;
	t->jiffies = jiffies;	/* for mt792x_mac_token_reclaim() */

	id = mt76_token_consume(mdev, &t);
	if (id < 0)
		return id;

	if (sta) {
		struct mt792x_sta *msta = (struct mt792x_sta *)sta->drv_priv;

		if (time_after(jiffies, msta->deflink.last_txs + HZ / 4)) {
			info->flags |= IEEE80211_TX_CTL_REQ_TX_STATUS;
			msta->deflink.last_txs = jiffies;
		}
	}

	pid = mt76_tx_status_skb_add(mdev, wcid, tx_info->skb);
	mt76_connac2_mac_write_txwi(mdev, txwi_ptr, tx_info->skb, wcid, key,
				    pid, qid, 0);

	txp = (struct mt76_connac_hw_txp *)(txwi + MT_TXD_SIZE);
	memset(txp, 0, sizeof(struct mt76_connac_hw_txp));
	mt76_connac_write_hw_txp(mdev, tx_info, txp, id);

	tx_info->skb = NULL;

	return 0;
}

/* Restarting the MCU requires a full power domain cycle, not supported yet. */
static int mt6893_init_reset(struct mt792x_dev *dev)
{
	return -EOPNOTSUPP;
}

static int mt6893_mac_reset(struct mt792x_dev *dev)
{
	return -EOPNOTSUPP;
}

static int mt6893_drv_own(struct mt792x_dev *dev)
{
	return mt6893_wmac_drv_own(to_mt6893(dev));
}

/* Power management is disabled: the firmware never takes ownership back. */
static int mt6893_fw_own(struct mt792x_dev *dev)
{
	return 0;
}

struct mt6893_dev *mt6893_mmio_probe(struct device *pdev,
				     void __iomem *mem_base)
{
	static const struct mt76_driver_ops drv_ops = {
		.txwi_size = MT_TXD_SIZE + sizeof(struct mt76_connac_hw_txp),
		.drv_flags = MT_DRV_TXWI_NO_FREE | MT_DRV_HW_MGMT_TXQ |
			     MT_DRV_AMSDU_OFFLOAD | MT_DRV_TOKEN_CYCLIC,
		.survey_flags = SURVEY_INFO_TIME_TX |
				SURVEY_INFO_TIME_RX |
				SURVEY_INFO_TIME_BSS_RX,
		.token_size = MT7921_TOKEN_SIZE,
		.tx_prepare_skb = mt6893_tx_prepare_skb,
		.tx_complete_skb = mt76_connac_tx_complete_skb,
		.rx_check = mt7921_rx_check,
		.rx_skb = mt7921_queue_rx_skb,
		.rx_poll_complete = mt792x_rx_poll_complete,
		.sta_add = mt7921_mac_sta_add,
		.sta_event = mt7921_mac_sta_event,
		.sta_remove = mt7921_mac_sta_remove,
		.update_survey = mt792x_update_channel,
		.set_channel = mt7921_set_channel,
	};
	static const struct mt792x_hif_ops hif_ops = {
		.init_reset = mt6893_init_reset,
		.reset = mt6893_mac_reset,
		.mcu_init = mt6893_mcu_init,
		.drv_own = mt6893_drv_own,
		.fw_own = mt6893_fw_own,
		.load_firmware = mt6893_mcu_load_firmware,
	};
	static const struct mt792x_irq_map irq_map = {
		.host_irq_enable = MT6893_INT_MASK_CSR,
		.tx = {
			.all_complete_mask = MT6893_INT_TX_DONE_ALL,
			.mcu_complete_mask = MT6893_INT_TX_DONE_MCU,
		},
		.rx = {
			.all_complete_mask = MT6893_INT_RX_DONE_ALL,
			.data_complete_mask = MT6893_INT_RX_DONE_DATA0,
			.wm_complete_mask = MT6893_INT_RX_DONE_MCU,
			.wm2_complete_mask = MT6893_INT_RX_DONE_TXFREE,
		},
	};
	struct ieee80211_ops *ops;
	struct mt76_bus_ops *bus_ops;
	struct mt6893_dev *dev;
	struct mt792x_dev *mdev;
	struct mt76_dev *m;

	/*
	 * The firmware has no CNM support: same fallback as
	 * mt792x_get_mac80211_ops() without the capability.
	 */
	ops = devm_kmemdup(pdev, &mt7921_ops, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return ERR_PTR(-ENOMEM);

	ops->remain_on_channel = NULL;
	ops->cancel_remain_on_channel = NULL;
	ops->add_chanctx = ieee80211_emulate_add_chanctx;
	ops->remove_chanctx = ieee80211_emulate_remove_chanctx;
	ops->change_chanctx = ieee80211_emulate_change_chanctx;
	ops->switch_vif_chanctx = ieee80211_emulate_switch_vif_chanctx;
	ops->assign_vif_chanctx = NULL;
	ops->unassign_vif_chanctx = NULL;
	ops->mgd_prepare_tx = NULL;
	ops->mgd_complete_tx = NULL;

	m = mt76_alloc_device(pdev, sizeof(*dev), ops, &drv_ops);
	if (!m)
		return ERR_PTR(-ENOMEM);

	dev = container_of(m, struct mt6893_dev, mt76);
	mdev = &dev->mt792x;

	mt76_mmio_init(m, mem_base);
	m->rev = 0x6893 << 16;

	mdev->hif_ops = &hif_ops;
	mdev->irq_map = &irq_map;
	mdev->fw_features = 0;
	tasklet_setup(&m->irq_tasklet, mt6893_irq_tasklet);

	mdev->phy.dev = mdev;
	mdev->phy.mt76 = &m->phy;
	m->phy.priv = &mdev->phy;

	mdev->bus_ops = m->bus;
	bus_ops = devm_kmemdup(pdev, mdev->bus_ops, sizeof(*bus_ops),
			       GFP_KERNEL);
	if (!bus_ops) {
		mt76_free_device(m);
		return ERR_PTR(-ENOMEM);
	}

	bus_ops->rr = mt6893_rr;
	bus_ops->wr = mt6893_wr;
	bus_ops->rmw = mt6893_rmw;
	m->bus = bus_ops;

	return dev;
}
