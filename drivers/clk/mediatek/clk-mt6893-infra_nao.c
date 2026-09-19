// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Kevin Andrieux
 */

#include <dt-bindings/clock/mediatek,mt6893-clk.h>
#include <linux/clk-provider.h>
#include <linux/platform_device.h>

#include "clk-gate.h"
#include "clk-mtk.h"

/*
 * INFRA_AP2MD_GALS_CTL. Bit 0 gates the AP to CONNSYS host clock, needed before
 * the connectivity bus protections can be released.
 */
static const struct mtk_gate_regs infra_nao_cg_regs = {
	.sta_ofs = 0x504,
};

/* No set/clear registers, set to ungate. The parent is unknown. */
#define GATE_INFRA_NAO(_id, _name, _shift)				\
	GATE_MTK(_id, _name, NULL, &infra_nao_cg_regs, _shift,		\
		 &mtk_clk_gate_ops_no_setclr_inv)

static const struct mtk_gate infra_nao_clks[] = {
	GATE_INFRA_NAO(CLK_INFRA_NAO_AP2CONN_HOST, "infra_nao_ap2conn_host", 0),
};

static const struct mtk_clk_desc infra_nao_desc = {
	.clks = infra_nao_clks,
	.num_clks = ARRAY_SIZE(infra_nao_clks),
};

static const struct of_device_id of_match_clk_mt6893_infra_nao[] = {
	{ .compatible = "mediatek,mt6893-infracfg-nao", .data = &infra_nao_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6893_infra_nao);

static struct platform_driver clk_mt6893_infra_nao_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6893-infra_nao",
		.of_match_table = of_match_clk_mt6893_infra_nao,
	},
};
module_platform_driver(clk_mt6893_infra_nao_drv);

MODULE_AUTHOR("Kevin Andrieux");
MODULE_DESCRIPTION("MediaTek MT6893 infracfg-nao clocks driver");
MODULE_LICENSE("GPL");
