// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Copyright (c) 2026 Kevin Andrieux
 *
 * WFDMA rings. WFDMA0 holds the RX data rings, WFDMA1 the TX rings and the MCU
 * event ring. Ring indexes are those of the vendor driver, and of mt7915:
 * 16 for firmware download, 17 for WM commands. There is no WA CPU.
 */

#include "mt6893.h"
#include "../dma.h"
#include "../mt76_connac.h"
#include "../trace.h"

irqreturn_t mt6893_irq_handler(int irq, void *dev_instance)
{
	struct mt6893_dev *dev = dev_instance;

	if (test_bit(MT76_REMOVED, &dev->mt76.phy.state))
		return IRQ_NONE;

	mt76_wr(dev, MT6893_INT_MASK_CSR, 0);
	tasklet_schedule(&dev->mt76.irq_tasklet);

	return IRQ_HANDLED;
}

void mt6893_irq_tasklet(struct tasklet_struct *t)
{
	struct mt6893_dev *dev = from_tasklet(dev, t, mt76.irq_tasklet);
	const struct mt792x_irq_map *irq_map = dev->mt792x.irq_map;
	u32 intr, mask;

	mt76_wr(dev, MT6893_INT_MASK_CSR, 0);

	intr = mt76_rr(dev, MT6893_INT_SOURCE_CSR);
	intr &= dev->mt76.mmio.irqmask;

	/* Block status first, then the aggregate, as soc3_0ReadExtIntStatus(). */
	mt76_wr(dev, MT6893_WFDMA1(MT6893_WFDMA_HOST_INT_STA),
		intr & ~MT6893_INT_RX_DONE_WFDMA0);
	mt76_wr(dev, MT6893_WFDMA0(MT6893_WFDMA_HOST_INT_STA),
		(intr & MT6893_INT_RX_DONE_WFDMA0) >>
		MT6893_INT_RX_DONE_WFDMA0_SHIFT);
	mt76_wr(dev, MT6893_INT_SOURCE_CSR, intr);

	trace_dev_irq(&dev->mt76, intr, dev->mt76.mmio.irqmask);

	mask = intr & irq_map->rx.all_complete_mask;
	if (intr & irq_map->tx.all_complete_mask)
		mask |= irq_map->tx.all_complete_mask;
	mt6893_irq_disable(dev, mask);

	if (intr & irq_map->tx.all_complete_mask)
		napi_schedule(&dev->mt76.tx_napi);

	if (intr & irq_map->rx.wm_complete_mask)
		napi_schedule(&dev->mt76.napi[MT_RXQ_MCU]);

	if (intr & irq_map->rx.wm2_complete_mask)
		napi_schedule(&dev->mt76.napi[MT_RXQ_MCU_WA]);

	if (intr & irq_map->rx.data_complete_mask)
		napi_schedule(&dev->mt76.napi[MT_RXQ_MAIN]);
}

void mt6893_dma_disable(struct mt6893_dev *dev)
{
	u32 cfg = MT6893_WFDMA_GLO_CFG_TX_DMA_EN | MT6893_WFDMA_GLO_CFG_RX_DMA_EN |
		  MT6893_WFDMA_GLO_CFG_CSR_DISP_BASE_PTR_CHAIN_EN |
		  MT6893_WFDMA_GLO_CFG_OMIT_TX_INFO |
		  MT6893_WFDMA_GLO_CFG_OMIT_RX_INFO |
		  MT6893_WFDMA_GLO_CFG_OMIT_RX_INFO_PFET2;
	u32 busy = MT6893_WFDMA_GLO_CFG_TX_DMA_BUSY |
		   MT6893_WFDMA_GLO_CFG_RX_DMA_BUSY;

	mt6893_irq_disable(dev, ~0);
	mt76_wr(dev, MT6893_WFDMA0(MT6893_WFDMA_HOST_INT_ENA_CLR), ~0);
	mt76_wr(dev, MT6893_WFDMA1(MT6893_WFDMA_HOST_INT_ENA_CLR), ~0);

	/* Same order as soc3_0asicConnac2xWfdmaControl(false). */
	mt76_clear(dev, MT6893_WFDMA0(MT6893_WFDMA_GLO_CFG), cfg);
	mt76_clear(dev, MT6893_WFDMA1(MT6893_WFDMA_GLO_CFG), cfg);

	if (!mt76_poll(dev, MT6893_WFDMA0(MT6893_WFDMA_GLO_CFG), busy, 0, 100000) ||
	    !mt76_poll(dev, MT6893_WFDMA1(MT6893_WFDMA_GLO_CFG), busy, 0, 100000))
		dev_warn(dev->mt76.dev, "WFDMA still busy: 0x%08x 0x%08x\n",
			 mt76_rr(dev, MT6893_WFDMA0(MT6893_WFDMA_GLO_CFG)),
			 mt76_rr(dev, MT6893_WFDMA1(MT6893_WFDMA_GLO_CFG)));

	mt76_wr(dev, MT6893_WFDMA0(MT6893_WFDMA_RST_DTX_PTR), ~0);
	mt76_wr(dev, MT6893_WFDMA0(MT6893_WFDMA_RST_DRX_PTR), ~0);
	mt76_wr(dev, MT6893_WFDMA1(MT6893_WFDMA_RST_DTX_PTR), ~0);
	mt76_wr(dev, MT6893_WFDMA1(MT6893_WFDMA_RST_DRX_PTR), ~0);
}

/* Prefetch windows: soc3_0asicConnac2xWfdmaManualPrefetch(), depth 4. */
static void mt6893_dma_prefetch(struct mt6893_dev *dev)
{
#define PREFETCH(_base)		((_base) << 16 | 0x4)
	mt76_wr(dev, MT6893_WFDMA0(MT6893_RX_RING_EXT_CTRL_BASE) +
		MT6893_RXQ_ID_DATA0 * 4, PREFETCH(0x000));
	mt76_wr(dev, MT6893_WFDMA0(MT6893_RX_RING_EXT_CTRL_BASE) +
		MT6893_RXQ_ID_DATA1 * 4, PREFETCH(0x040));
	mt76_wr(dev, MT6893_WFDMA1(MT6893_RX_RING_EXT_CTRL_BASE) +
		MT6893_RXQ_ID_MCU * 4, PREFETCH(0x200));
	mt76_wr(dev, MT6893_WFDMA0(MT6893_RX_RING_EXT_CTRL_BASE) + 2 * 4,
		PREFETCH(0x080));
	mt76_wr(dev, MT6893_WFDMA0(MT6893_RX_RING_EXT_CTRL_BASE) + 3 * 4,
		PREFETCH(0x0c0));
	mt76_wr(dev, MT6893_WFDMA1(MT6893_RX_RING_EXT_CTRL_BASE) + 1 * 4,
		PREFETCH(0x240));
	mt76_wr(dev, MT6893_WFDMA1(MT6893_RX_RING_EXT_CTRL_BASE) + 2 * 4,
		PREFETCH(0x280));
	mt76_wr(dev, MT6893_WFDMA1(MT6893_TX_RING_EXT_CTRL_BASE) +
		MT6893_TXQ_ID_BAND0 * 4, PREFETCH(0x2c0));
	mt76_wr(dev, MT6893_WFDMA1(MT6893_TX_RING_EXT_CTRL_BASE) +
		MT6893_MCUQ_ID_FWDL * 4, PREFETCH(0x5c0));
	mt76_wr(dev, MT6893_WFDMA1(MT6893_TX_RING_EXT_CTRL_BASE) +
		MT6893_MCUQ_ID_WM * 4, PREFETCH(0x600));
#undef PREFETCH
}

/* OMIT_RX_INFO only on WFDMA1: MCU events carry no RX info. */
static void mt6893_dma_enable(struct mt6893_dev *dev)
{
	u32 cfg = FIELD_PREP(MT6893_WFDMA_GLO_CFG_PDMA_BT_SIZE, 3) |
		  MT6893_WFDMA_GLO_CFG_TX_WB_DDONE |
		  MT6893_WFDMA_GLO_CFG_FIFO_LITTLE_ENDIAN |
		  MT6893_WFDMA_GLO_CFG_CLK_GATE_DIS |
		  MT6893_WFDMA_GLO_CFG_OMIT_TX_INFO |
		  MT6893_WFDMA_GLO_CFG_CSR_DISP_BASE_PTR_CHAIN_EN |
		  MT6893_WFDMA_GLO_CFG_OMIT_RX_INFO_PFET2;
	u32 mask = cfg | MT6893_WFDMA_GLO_CFG_OMIT_RX_INFO |
		   MT6893_WFDMA_GLO_CFG_TX_DMA_EN | MT6893_WFDMA_GLO_CFG_RX_DMA_EN;

	mt76_rmw(dev, MT6893_WFDMA0(MT6893_WFDMA_GLO_CFG), mask, cfg);
	mt76_rmw(dev, MT6893_WFDMA1(MT6893_WFDMA_GLO_CFG), mask,
		 cfg | MT6893_WFDMA_GLO_CFG_OMIT_RX_INFO);

	mt76_set(dev, MT6893_WFDMA0(MT6893_WFDMA_GLO_CFG),
		 MT6893_WFDMA_GLO_CFG_TX_DMA_EN | MT6893_WFDMA_GLO_CFG_RX_DMA_EN);
	mt76_set(dev, MT6893_WFDMA1(MT6893_WFDMA_GLO_CFG),
		 MT6893_WFDMA_GLO_CFG_TX_DMA_EN | MT6893_WFDMA_GLO_CFG_RX_DMA_EN);

	mt76_wr(dev, MT6893_WFDMA1(MT6893_WFDMA_HOST_INT_ENA_SET),
		MT6893_INT_RX_DONE_MCU | MT6893_INT_TX_DONE_ALL);
	mt76_wr(dev, MT6893_WFDMA0(MT6893_WFDMA_HOST_INT_ENA_SET),
		MT6893_INT_RX_DONE_WFDMA0 >> MT6893_INT_RX_DONE_WFDMA0_SHIFT);

	mt76_set(dev, MT6893_WFDMA_AXI_R2A_CTRL0, MT6893_WFDMA_AXI_BID_CHK_BYP);

	mt76_wr(dev, MT6893_WFDMA1(MT6893_WFDMA_INT_RX_PRI_SEL),
		MT6893_WFDMA_INT_RX_PRI_SEL_VAL);
	mt76_wr(dev, MT6893_WFDMA1(MT6893_WFDMA_INT_TX_PRI_SEL),
		MT6893_WFDMA_INT_TX_PRI_SEL_VAL);
}

void mt6893_dma_cleanup(struct mt6893_dev *dev)
{
	int i;

	mt6893_dma_disable(dev);

	/* mt76_dma_cleanup() deletes the RX NAPIs without disabling them. */
	mt76_for_each_q_rx(&dev->mt76, i)
		napi_disable(&dev->mt76.napi[i]);

	mt76_dma_cleanup(&dev->mt76);

	skb_queue_purge(&dev->mt76.mcu.res_q);
}

int mt6893_dma_init(struct mt6893_dev *dev)
{
	int ret;

	mt76_dma_attach(&dev->mt76);
	mt6893_dma_disable(dev);
	mt6893_dma_prefetch(dev);

	ret = mt76_connac_init_tx_queues(dev->mt792x.phy.mt76,
					 MT6893_TXQ_ID_BAND0,
					 MT7921_TX_RING_SIZE,
					 MT6893_WFDMA1(MT6893_TX_RING_BASE),
					 NULL, 0);
	if (ret)
		return ret;

	ret = mt76_init_mcu_queue(&dev->mt76, MT_MCUQ_WM, MT6893_MCUQ_ID_WM,
				  MT6893_TX_MCU_RING_SIZE,
				  MT6893_WFDMA1(MT6893_TX_RING_BASE));
	if (ret)
		return ret;

	ret = mt76_init_mcu_queue(&dev->mt76, MT_MCUQ_FWDL, MT6893_MCUQ_ID_FWDL,
				  MT6893_TX_FWDL_RING_SIZE,
				  MT6893_WFDMA1(MT6893_TX_RING_BASE));
	if (ret)
		return ret;

	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MCU],
			       MT6893_RXQ_ID_MCU, MT6893_RX_MCU_RING_SIZE,
			       MT_RX_BUF_SIZE, MT6893_WFDMA1(MT6893_RX_RING_BASE));
	if (ret)
		return ret;

	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MAIN],
			       MT6893_RXQ_ID_DATA0, MT7921_RX_RING_SIZE,
			       MT_RX_BUF_SIZE, MT6893_WFDMA0(MT6893_RX_RING_BASE));
	if (ret)
		return ret;

	/*
	 * TX free notifications arrive on WFDMA0 RX ring 2 (RX_RING_TXDONE0).
	 * Use the WA event queue, unused here, as it has its own interrupt.
	 */
	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MCU_WA],
			       MT6893_RXQ_ID_TXFREE,
			       MT7921_RX_RING_SIZE,
			       MT_RX_BUF_SIZE, MT6893_WFDMA0(MT6893_RX_RING_BASE));
	if (ret)
		return ret;

	/*
	 * The other RX rings the vendor driver sets up. Nothing is expected on
	 * them, but they are filled so the hardware always has buffers.
	 */
	{
		static const struct { int q, ring; u32 base; } extra[] = {
			{ MT_RXQ_BAND1, MT6893_RXQ_ID_DATA1,
			  MT6893_WFDMA0(MT6893_RX_RING_BASE) },
			{ MT_RXQ_BAND1_WA, 3, MT6893_WFDMA0(MT6893_RX_RING_BASE) },
			{ MT_RXQ_MAIN_WA, 1, MT6893_WFDMA1(MT6893_RX_RING_BASE) },
			{ MT_RXQ_BAND2, 2, MT6893_WFDMA1(MT6893_RX_RING_BASE) },
		};
		int i;

		for (i = 0; i < ARRAY_SIZE(extra); i++) {
			ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[extra[i].q],
					       extra[i].ring, 128,
					       MT_RX_BUF_SIZE, extra[i].base);
			if (ret)
				return ret;
		}
	}

	ret = mt76_init_queues(dev, mt792x_poll_rx);
	if (ret)
		return ret;

	/* mt76_dma_cleanup() unconditionally disables the TX NAPI. */
	netif_napi_add_tx(dev->mt76.tx_napi_dev, &dev->mt76.tx_napi,
			  mt792x_poll_tx);
	napi_enable(&dev->mt76.tx_napi);

	mt6893_dma_enable(dev);

	dev_info(dev->mt76.dev,
		 "WFDMA ready: commands %pad, firmware %pad, events %pad\n",
		 &dev->mt76.q_mcu[MT_MCUQ_WM]->desc_dma,
		 &dev->mt76.q_mcu[MT_MCUQ_FWDL]->desc_dma,
		 &dev->mt76.q_rx[MT_RXQ_MCU].desc_dma);

	return 0;
}
