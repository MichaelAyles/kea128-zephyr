/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kea_adc

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "kea_regs.h"

struct kea_adc_config {
	kea_adc_t *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(const struct device *dev);
};

struct kea_adc_data {
	uint32_t configured_channels;
	struct k_sem completion;
};

static int kea_adc_channel_setup(const struct device *dev, const struct adc_channel_cfg *channel_cfg)
{
	const struct kea_adc_config *cfg = dev->config;
	struct kea_adc_data *data = dev->data;

	if (channel_cfg->channel_id >= 16U) {
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		return -ENOTSUP;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		return -ENOTSUP;
	}

	if ((channel_cfg->reference != ADC_REF_INTERNAL) &&
	    (channel_cfg->reference != ADC_REF_VDD_1)) {
		return -ENOTSUP;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		return -ENOTSUP;
	}

	cfg->base->APCTL1 |= BIT(channel_cfg->channel_id);
	data->configured_channels |= BIT(channel_cfg->channel_id);

	return 0;
}

static int kea_adc_sequence_init(const struct kea_adc_config *cfg, const struct adc_sequence *sequence)
{
	uint32_t mode;

	switch (sequence->resolution) {
	case 8:
		mode = 0u;
		break;
	case 10:
		mode = 1u;
		break;
	case 12:
		mode = 2u;
		break;
	default:
		return -ENOTSUP;
	}

	cfg->base->SC2 = 0u;
	cfg->base->SC3 = (cfg->base->SC3 & ~(KEA_ADC_SC3_ADICLK_MASK | KEA_ADC_SC3_MODE_MASK)) |
			 (0u << 0) | (mode << KEA_ADC_SC3_MODE_SHIFT);

	/* Disable continuous conversion and completion interrupts. */
	cfg->base->SC1 &= ~(KEA_ADC_SC1_ADCO_MASK | KEA_ADC_SC1_AIEN_MASK);

	return 0;
}

static int kea_adc_convert_channel(const struct device *dev, uint32_t ch, uint16_t *sample_out)
{
	const struct kea_adc_config *cfg = dev->config;
	struct kea_adc_data *data = dev->data;
	uint32_t sc1;

	k_sem_reset(&data->completion);

	sc1 = cfg->base->SC1 & ~(KEA_ADC_SC1_ADCH_MASK | KEA_ADC_SC1_ADCO_MASK |
				 KEA_ADC_SC1_AIEN_MASK);
	sc1 |= ((ch << KEA_ADC_SC1_ADCH_SHIFT) & KEA_ADC_SC1_ADCH_MASK) | KEA_ADC_SC1_AIEN_MASK;
	cfg->base->SC1 = sc1;

	if (!k_is_in_isr()) {
		if (k_sem_take(&data->completion, K_MSEC(100)) != 0) {
			return -ETIMEDOUT;
		}
	} else {
		while ((cfg->base->SC1 & KEA_ADC_SC1_COCO_MASK) == 0u) {
		}
	}

	*sample_out = (uint16_t)(cfg->base->R & 0x0FFFu);
	return 0;
}

static int kea_adc_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct kea_adc_config *cfg = dev->config;
	struct kea_adc_data *data = dev->data;
	uint16_t *buffer = sequence->buffer;
	uint32_t channels = sequence->channels;
	size_t channel_count = 0;
	size_t sample_count;

	if ((channels == 0u) || ((channels & ~BIT_MASK(16)) != 0u)) {
		return -EINVAL;
	}

	if ((channels & ~data->configured_channels) != 0u) {
		return -EINVAL;
	}

	if (sequence->oversampling != 0U) {
		return -ENOTSUP;
	}

	if (sequence->calibrate) {
		return -ENOTSUP;
	}

	if (kea_adc_sequence_init(cfg, sequence) != 0) {
		return -ENOTSUP;
	}

	for (uint32_t ch = 0; ch < 16u; ch++) {
		if ((channels & BIT(ch)) != 0u) {
			channel_count++;
		}
	}

	sample_count = channel_count;
	if (sequence->options != NULL) {
		sample_count *= (size_t)(1u + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < (sample_count * sizeof(uint16_t))) {
		return -ENOMEM;
	}

	for (size_t sample = 0; sample < sample_count; sample += channel_count) {
		for (uint32_t ch = 0; ch < 16u; ch++) {
			int ret;

			if ((channels & BIT(ch)) == 0u) {
				continue;
			}

			ret = kea_adc_convert_channel(dev, ch, buffer);
			if (ret != 0) {
				return ret;
			}

			buffer++;
		}
	}

	return 0;
}

static void kea_adc_isr(const struct device *dev)
{
	const struct kea_adc_config *cfg = dev->config;
	struct kea_adc_data *data = dev->data;

	if ((cfg->base->SC1 & KEA_ADC_SC1_COCO_MASK) != 0u) {
		k_sem_give(&data->completion);
	}
}

static int kea_adc_init(const struct device *dev)
{
	const struct kea_adc_config *cfg = dev->config;
	struct kea_adc_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret != 0) {
		return ret;
	}

	k_sem_init(&data->completion, 0, 1);

	if (cfg->irq_config_func != NULL) {
		cfg->irq_config_func(dev);
	}

	return 0;
}

static DEVICE_API(adc, kea_adc_driver_api) = {
	.channel_setup = kea_adc_channel_setup,
	.read = kea_adc_read,
	.ref_internal = 3300,
};

#define KEA_ADC_IRQ_CONFIG(inst)                                                            \
	static void kea_adc_irq_config_##inst(const struct device *dev)                     \
	{                                                                                     \
		ARG_UNUSED(dev);                                                               \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),                  \
			    kea_adc_isr, DEVICE_DT_INST_GET(inst), 0);                         \
		irq_enable(DT_INST_IRQN(inst));                                                \
	}

#define KEA_ADC_INIT(inst)                                                             \
	KEA_ADC_IRQ_CONFIG(inst)                                                       \
                                                                                         \
	static const struct kea_adc_config kea_adc_cfg_##inst = {                      \
		.base = (kea_adc_t *)DT_INST_REG_ADDR(inst),                           \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                 \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, bits),\
		.irq_config_func = kea_adc_irq_config_##inst,                          \
	};                                                                             \
                                                                                           \
	static struct kea_adc_data kea_adc_data_##inst;                                \
                                                                                           \
	DEVICE_DT_INST_DEFINE(inst, kea_adc_init, NULL, &kea_adc_data_##inst,          \
			      &kea_adc_cfg_##inst, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, \
			      &kea_adc_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KEA_ADC_INIT)
