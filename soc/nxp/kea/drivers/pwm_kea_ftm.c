/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kea_ftm_pwm

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/pwm/pwm.h>

#include "kea_regs.h"

struct kea_pwm_config {
	kea_ftm_t *base;
	uint32_t clock_frequency;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
	uint8_t prescaler;
	uint8_t channel_count;
};

struct kea_pwm_data {
	uint32_t period_cycles;
};

static int kea_pwm_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
			      uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct kea_pwm_config *cfg = dev->config;
	struct kea_pwm_data *data = dev->data;
	uint32_t cnsc = 0u;

	if (channel >= cfg->channel_count) {
		return -EINVAL;
	}

	if ((flags & ~(PWM_POLARITY_INVERTED)) != 0u) {
		return -ENOTSUP;
	}

	if ((period_cycles == 0u) || (pulse_cycles > period_cycles)) {
		return -EINVAL;
	}

	if ((data->period_cycles != 0u) && (data->period_cycles != period_cycles)) {
		/* Single MOD register shared by all channels. */
		return -ENOTSUP;
	}

	data->period_cycles = period_cycles;

	cfg->base->MODE |= KEA_FTM_MODE_WPDIS_MASK | KEA_FTM_MODE_FTMEN_MASK;
	cfg->base->CNTIN = 0u;
	cfg->base->CNT = 0u;
	cfg->base->MOD = period_cycles - 1u;

	cfg->base->SC &= ~(KEA_FTM_SC_PS_MASK | KEA_FTM_SC_CLKS_MASK);
	cfg->base->SC |= (cfg->prescaler & KEA_FTM_SC_PS_MASK) |
			 (1u << KEA_FTM_SC_CLKS_SHIFT);

	if (pulse_cycles == 0u) {
		cfg->base->CONTROLS[channel].CnSC = 0u;
		cfg->base->CONTROLS[channel].CnV = 0u;
		return 0;
	}

	cnsc = KEA_FTM_CNSC_MSB_MASK;
	if ((flags & PWM_POLARITY_INVERTED) != 0u) {
		cnsc |= KEA_FTM_CNSC_ELSA_MASK;
	} else {
		cnsc |= KEA_FTM_CNSC_ELSB_MASK;
	}

	cfg->base->CONTROLS[channel].CnSC = cnsc;
	cfg->base->CONTROLS[channel].CnV = pulse_cycles;

	return 0;
}

static int kea_pwm_get_cycles_per_sec(const struct device *dev, uint32_t channel, uint64_t *cycles)
{
	const struct kea_pwm_config *cfg = dev->config;

	if (channel >= cfg->channel_count) {
		return -EINVAL;
	}

	*cycles = (uint64_t)cfg->clock_frequency >> cfg->prescaler;
	return 0;
}

static int kea_pwm_init(const struct device *dev)
{
	const struct kea_pwm_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret != 0) {
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if ((ret < 0) && (ret != -ENOENT)) {
		return ret;
	}

	return 0;
}

static DEVICE_API(pwm, kea_pwm_driver_api) = {
	.set_cycles = kea_pwm_set_cycles,
	.get_cycles_per_sec = kea_pwm_get_cycles_per_sec,
};

#define KEA_PWM_INIT(inst)                                                                  \
	PINCTRL_DT_INST_DEFINE(inst);                                                       \
                                                                                               \
	static const struct kea_pwm_config kea_pwm_cfg_##inst = {                           \
		.base = (kea_ftm_t *)DT_INST_REG_ADDR(inst),                                \
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),                     \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                     \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, bits),   \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                             \
		.prescaler = DT_INST_PROP(inst, nxp_prescaler),                             \
		.channel_count = 6,                                                         \
	};                                                                                  \
                                                                                               \
	static struct kea_pwm_data kea_pwm_data_##inst;                                      \
                                                                                               \
	DEVICE_DT_INST_DEFINE(inst, kea_pwm_init, NULL, &kea_pwm_data_##inst,                \
			      &kea_pwm_cfg_##inst, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,      \
			      &kea_pwm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KEA_PWM_INIT)
