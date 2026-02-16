/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kea_sim_clock

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>

#include "kea_regs.h"

struct kea_sim_clock_config {
	uint32_t bus_clock_hz;
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

	ARG_UNUSED(sys);

	*rate = cfg->bus_clock_hz;

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
	};                                                                                 \
                                                                                           \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, NULL, &kea_sim_clock_cfg_##inst,          \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,            \
			      &kea_sim_clock_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KEA_SIM_CLOCK_INIT)
