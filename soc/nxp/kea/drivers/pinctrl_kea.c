/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>

#include "kea_regs.h"

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0u; i < pin_cnt; i++) {
		const uint32_t pin_cfg = pins[i];
		const uint32_t pinsel_mask = (pin_cfg & 0x7FFFFFFFu);

		if ((pin_cfg & Z_PINCTRL_KEA_PINSEL1_SEL) != 0u) {
			KEA_SIM->PINSEL1 |= pinsel_mask;
		} else {
			KEA_SIM->PINSEL |= pinsel_mask;
		}
	}

	return 0;
}
