/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kea_sim_clock

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>

#include "kea_regs.h"

struct kea_sim_clock_config {
	uint32_t bus_clock_hz;
	uint32_t fll_clock_hz;
	bool derive_rate_from_dividers;
};

static int kea_sim_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);

	const uint32_t gate_mask = (uint32_t)(uintptr_t)sys;

	if (gate_mask == 0u) {
		return -EINVAL;
	}

	KEA_SIM->SCGC |= gate_mask;

	return 0;
}

static int kea_sim_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);

	const uint32_t gate_mask = (uint32_t)(uintptr_t)sys;

	if (gate_mask == 0u) {
		return -EINVAL;
	}

	KEA_SIM->SCGC &= ~gate_mask;

	return 0;
}

static int kea_sim_clock_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	const struct kea_sim_clock_config *cfg = dev->config;
	uint32_t root_hz;
	uint32_t bdiv_sel;
	uint32_t outdiv1;
	uint32_t outdiv2;
	uint64_t derived_hz;

	ARG_UNUSED(sys);

	if (!cfg->derive_rate_from_dividers) {
		*rate = cfg->bus_clock_hz;
		return 0;
	}

	root_hz = (cfg->fll_clock_hz != 0u) ? cfg->fll_clock_hz : cfg->bus_clock_hz;
	if (root_hz == 0u) {
		return -EINVAL;
	}

	bdiv_sel = (KEA_ICS->C2 & KEA_ICS_C2_BDIV_MASK) >> KEA_ICS_C2_BDIV_SHIFT;
	outdiv1 = ((KEA_SIM->CLKDIV & KEA_SIM_CLKDIV_OUTDIV1_MASK) >> KEA_SIM_CLKDIV_OUTDIV1_SHIFT) + 1u;
	outdiv2 = ((KEA_SIM->CLKDIV & KEA_SIM_CLKDIV_OUTDIV2_MASK) >> KEA_SIM_CLKDIV_OUTDIV2_SHIFT) + 1u;

	derived_hz = root_hz;
	derived_hz /= BIT(bdiv_sel);
	derived_hz /= outdiv1;
	derived_hz /= outdiv2;

	if (derived_hz == 0u) {
		*rate = cfg->bus_clock_hz;
		return 0;
	}

	*rate = (uint32_t)derived_hz;

	return 0;
}

static enum clock_control_status kea_sim_clock_get_status(const struct device *dev,
							  clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);

	const uint32_t gate_mask = (uint32_t)(uintptr_t)sys;

	if (gate_mask == 0u) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	return ((KEA_SIM->SCGC & gate_mask) != 0u) ? CLOCK_CONTROL_STATUS_ON :
						      CLOCK_CONTROL_STATUS_OFF;
}

static DEVICE_API(clock_control, kea_sim_clock_driver_api) = {
	.on = kea_sim_clock_on,
	.off = kea_sim_clock_off,
	.get_rate = kea_sim_clock_get_rate,
	.get_status = kea_sim_clock_get_status,
};

#define KEA_SIM_CLOCK_INIT(inst)                                                           \
	static const struct kea_sim_clock_config kea_sim_clock_cfg_##inst = {              \
		.bus_clock_hz = DT_INST_PROP(inst, clock_frequency),                       \
		.fll_clock_hz = DT_INST_PROP_OR(inst, nxp_fll_clock_frequency, 0),         \
		.derive_rate_from_dividers = DT_INST_PROP_OR(inst, nxp_derive_rate_from_dividers, false), \
	};                                                                                 \
                                                                                           \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, NULL, &kea_sim_clock_cfg_##inst,          \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,            \
			      &kea_sim_clock_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KEA_SIM_CLOCK_INIT)
