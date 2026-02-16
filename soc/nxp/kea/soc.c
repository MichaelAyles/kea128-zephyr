/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "kea_regs.h"

void soc_early_init_hook(void)
{
	/* Disable watchdog early to avoid periodic resets during bring-up. */
	KEA_WDOG->TOVAL = 0xE803u;
	KEA_WDOG->CS2 = KEA_WDOG_CS2_CLK_LPO;
	KEA_WDOG->CS1 = KEA_WDOG_CS1_UPDATE_MASK | KEA_WDOG_CS1_WAIT_MASK | KEA_WDOG_CS1_STOP_MASK;
}
