/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kea_pit_counter

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include "kea_regs.h"

struct kea_pit_config {
	struct counter_config_info info;
	kea_pit_t *base;
	uint32_t clock_frequency;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(const struct device *dev);
};

struct kea_pit_data {
	counter_top_callback_t top_callback;
	void *top_user_data;
};

static int kea_pit_start(const struct device *dev)
{
	const struct kea_pit_config *cfg = dev->config;

	cfg->base->CHANNEL[0].TCTRL |= KEA_PIT_TCTRL_TIE_MASK | KEA_PIT_TCTRL_TEN_MASK;

	return 0;
}

static int kea_pit_stop(const struct device *dev)
{
	const struct kea_pit_config *cfg = dev->config;

	cfg->base->CHANNEL[0].TCTRL &= ~(KEA_PIT_TCTRL_TIE_MASK | KEA_PIT_TCTRL_TEN_MASK);

	return 0;
}

static int kea_pit_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct kea_pit_config *cfg = dev->config;

	*ticks = cfg->base->CHANNEL[0].CVAL;

	return 0;
}

static int kea_pit_set_top_value(const struct device *dev, const struct counter_top_cfg *top_cfg)
{
	const struct kea_pit_config *cfg = dev->config;
	struct kea_pit_data *data = dev->data;
	const bool running = (cfg->base->CHANNEL[0].TCTRL & KEA_PIT_TCTRL_TEN_MASK) != 0u;

	if (top_cfg->ticks == 0u) {
		return -EINVAL;
	}

	if (running && ((top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) != 0u)) {
		return -ENOTSUP;
	}

	data->top_callback = top_cfg->callback;
	data->top_user_data = top_cfg->user_data;

	cfg->base->CHANNEL[0].LDVAL = top_cfg->ticks - 1u;

	if (running) {
		cfg->base->CHANNEL[0].TCTRL &= ~KEA_PIT_TCTRL_TEN_MASK;
		cfg->base->CHANNEL[0].TCTRL |= KEA_PIT_TCTRL_TEN_MASK;
	}

	return 0;
}

static uint32_t kea_pit_get_pending_int(const struct device *dev)
{
	const struct kea_pit_config *cfg = dev->config;

	return ((cfg->base->CHANNEL[0].TFLG & KEA_PIT_TFLG_TIF_MASK) != 0u) ? 1u : 0u;
}

static uint32_t kea_pit_get_top_value(const struct device *dev)
{
	const struct kea_pit_config *cfg = dev->config;

	return cfg->base->CHANNEL[0].LDVAL + 1u;
}

static int kea_pit_set_alarm(const struct device *dev, uint8_t chan_id,
			     const struct counter_alarm_cfg *alarm_cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan_id);
	ARG_UNUSED(alarm_cfg);

	return -ENOTSUP;
}

static int kea_pit_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan_id);

	return -ENOTSUP;
}

static uint32_t kea_pit_get_freq(const struct device *dev)
{
	const struct kea_pit_config *cfg = dev->config;

	return cfg->clock_frequency;
}

static void kea_pit_isr(const struct device *dev)
{
	const struct kea_pit_config *cfg = dev->config;
	struct kea_pit_data *data = dev->data;

	if ((cfg->base->CHANNEL[0].TFLG & KEA_PIT_TFLG_TIF_MASK) != 0u) {
		cfg->base->CHANNEL[0].TFLG = KEA_PIT_TFLG_TIF_MASK;

		if (data->top_callback != NULL) {
			data->top_callback(dev, data->top_user_data);
		}
	}

	if ((cfg->base->CHANNEL[1].TFLG & KEA_PIT_TFLG_TIF_MASK) != 0u) {
		cfg->base->CHANNEL[1].TFLG = KEA_PIT_TFLG_TIF_MASK;
	}
}

static int kea_pit_init(const struct device *dev)
{
	const struct kea_pit_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret != 0) {
		return ret;
	}

	cfg->base->MCR &= ~KEA_PIT_MCR_MDIS_MASK;
	cfg->base->MCR |= KEA_PIT_MCR_FRZ_MASK;

	cfg->base->CHANNEL[0].TCTRL = 0u;
	cfg->base->CHANNEL[1].TCTRL = 0u;
	cfg->base->CHANNEL[0].LDVAL = UINT32_MAX - 1u;
	cfg->base->CHANNEL[0].TFLG = KEA_PIT_TFLG_TIF_MASK;
	cfg->base->CHANNEL[1].TFLG = KEA_PIT_TFLG_TIF_MASK;

	if (cfg->irq_config_func != NULL) {
		cfg->irq_config_func(dev);
	}

	return 0;
}

static DEVICE_API(counter, kea_pit_driver_api) = {
	.start = kea_pit_start,
	.stop = kea_pit_stop,
	.get_value = kea_pit_get_value,
	.set_alarm = kea_pit_set_alarm,
	.cancel_alarm = kea_pit_cancel_alarm,
	.set_top_value = kea_pit_set_top_value,
	.get_pending_int = kea_pit_get_pending_int,
	.get_top_value = kea_pit_get_top_value,
	.get_freq = kea_pit_get_freq,
};

#define KEA_PIT_IRQ_CONFIG(inst)                                                                 \
	static void kea_pit_irq_config_##inst(const struct device *dev)                            \
	{                                                                                            \
		ARG_UNUSED(dev);                                                                      \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, ch0, irq),                                      \
			    DT_INST_IRQ_BY_NAME(inst, ch0, priority), kea_pit_isr,                    \
			    DEVICE_DT_INST_GET(inst), 0);                                              \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, ch0, irq));                                    \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, ch1, irq),                                      \
			    DT_INST_IRQ_BY_NAME(inst, ch1, priority), kea_pit_isr,                    \
			    DEVICE_DT_INST_GET(inst), 0);                                              \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, ch1, irq));                                    \
	}

#define KEA_PIT_INIT(inst)                                                                       \
	KEA_PIT_IRQ_CONFIG(inst)                                                                   \
                                                                                                   \
	static struct kea_pit_data kea_pit_data_##inst;                                            \
                                                                                                   \
	static const struct kea_pit_config kea_pit_cfg_##inst = {                                  \
		.info = {                                                                            \
			.max_top_value = UINT32_MAX,                                                   \
			.freq = DT_INST_PROP(inst, clock_frequency),                                   \
			.flags = 0u,                                                                   \
			.channels = 0u,                                                                \
		},                                                                                 \
		.base = (kea_pit_t *)DT_INST_REG_ADDR(inst),                                       \
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),                            \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, bits),           \
		.irq_config_func = kea_pit_irq_config_##inst,                                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, kea_pit_init, NULL, &kea_pit_data_##inst,                      \
			      &kea_pit_cfg_##inst, POST_KERNEL, CONFIG_COUNTER_INIT_PRIORITY,       \
			      &kea_pit_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KEA_PIT_INIT)
