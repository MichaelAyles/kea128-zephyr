/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "kea_regs.h"

#define KEA_ICS_LOCK_WAIT_LOOPS 1000000u

static void kea_clock_init_fei_20mhz(void)
{
	uint8_t c1;
	uint8_t c2;

	/*
	 * AN4942 FEI setup for KEA128:
	 * - Use internal reference for FLL (IRCLKEN + IREFS)
	 * - Set trim to 0x90 (nominal 31.25 kHz reference)
	 * - Wait FLL lock, then divide bus by 2 (BDIV=1) for 20 MHz bus.
	 */
	c1 = KEA_ICS->C1;
	c1 &= (uint8_t)~KEA_ICS_C1_CLKS_MASK;
	c1 |= KEA_ICS_C1_IRCLKEN_MASK | KEA_ICS_C1_IREFS_MASK;
	KEA_ICS->C1 = c1;

	KEA_ICS->C3 = 0x90u & KEA_ICS_C3_SCTRIM_MASK;

	for (uint32_t i = 0; i < KEA_ICS_LOCK_WAIT_LOOPS; i++) {
		const uint8_t status = KEA_ICS->S;

		if (((status & KEA_ICS_S_CLKST_MASK) == 0u) &&
		    ((status & KEA_ICS_S_IREFST_MASK) != 0u) &&
		    ((status & KEA_ICS_S_LOCK_MASK) != 0u)) {
			break;
		}
	}

	c2 = KEA_ICS->C2;
	c2 &= (uint8_t)~KEA_ICS_C2_BDIV_MASK;
	c2 |= (uint8_t)(1u << KEA_ICS_C2_BDIV_SHIFT);
	KEA_ICS->C2 = c2;

	KEA_ICS->S |= KEA_ICS_S_LOLS_MASK;
}

void soc_early_init_hook(void)
{
	/* Disable watchdog early to avoid periodic resets during bring-up. */
	KEA_WDOG->TOVAL = 0xE803u;
	KEA_WDOG->CS2 = KEA_WDOG_CS2_CLK_LPO;
	KEA_WDOG->CS1 = KEA_WDOG_CS1_UPDATE_MASK | KEA_WDOG_CS1_WAIT_MASK | KEA_WDOG_CS1_STOP_MASK;

	kea_clock_init_fei_20mhz();
}
