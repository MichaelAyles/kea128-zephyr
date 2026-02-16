/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ARM_NXP_KEA_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_NXP_KEA_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * bit31: route register selector
 *   0 -> SIM->PINSEL
 *   1 -> SIM->PINSEL1
 * bits30..0: mask bits to set in the selected register
 */
typedef uint32_t pinctrl_soc_pin_t;

#define Z_PINCTRL_KEA_PINSEL1_SEL 0x80000000u

#define Z_PINCTRL_STATE_PIN_INIT(group, pin_prop, idx) DT_PROP_BY_IDX(group, pin_prop, idx),

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                             \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop),                                   \
				DT_FOREACH_PROP_ELEM, pinmux, Z_PINCTRL_STATE_PIN_INIT)}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_NXP_KEA_PINCTRL_SOC_H_ */
