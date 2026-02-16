/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kea_gpio

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include "kea_regs.h"

struct kea_gpio_config {
	struct gpio_driver_config common;
	kea_gpio_t *base;
	kea_kbi_t *kbi;
	uint32_t kbi_clock_gate_mask;
	void (*irq_config_func)(const struct device *dev);
};

struct kea_gpio_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
	gpio_port_pins_t int_enabled_mask;
	gpio_port_pins_t int_rising_mask;
	gpio_port_pins_t int_level_mask;
	gpio_port_pins_t int_configured_mask;
};

static void kea_gpio_update_kbi_locked(const struct kea_gpio_config *cfg,
				       const struct kea_gpio_data *data);

static int kea_gpio_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct kea_gpio_config *cfg = dev->config;
	struct kea_gpio_data *data = dev->data;
	uint32_t pin_mask;

	if (pin >= 32u) {
		return -EINVAL;
	}

	if (((BIT(pin) & cfg->common.port_pin_mask) == 0u)) {
		return -EINVAL;
	}

	if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN | GPIO_SINGLE_ENDED)) != 0u) {
		return -ENOTSUP;
	}

	if (((flags & GPIO_INPUT) != 0u) && ((flags & GPIO_OUTPUT) != 0u)) {
		return -ENOTSUP;
	}

	pin_mask = BIT(pin);

	if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0u) {
		cfg->base->PSOR = pin_mask;
	}

	if ((flags & GPIO_OUTPUT_INIT_LOW) != 0u) {
		cfg->base->PCOR = pin_mask;
	}

	if ((flags & GPIO_OUTPUT) != 0u) {
		cfg->base->PDDR |= pin_mask;
		cfg->base->PIDR |= pin_mask;

		if (cfg->kbi != NULL) {
			const unsigned int key = irq_lock();

			data->int_enabled_mask &= ~pin_mask;
			data->int_rising_mask &= ~pin_mask;
			data->int_level_mask &= ~pin_mask;
			data->int_configured_mask &= ~pin_mask;
			kea_gpio_update_kbi_locked(cfg, data);
			irq_unlock(key);
		}
	} else {
		cfg->base->PDDR &= ~pin_mask;
		cfg->base->PIDR &= ~pin_mask;
	}

	return 0;
}

static int kea_gpio_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct kea_gpio_config *cfg = dev->config;

	*value = cfg->base->PDIR;

	return 0;
}

static int kea_gpio_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					gpio_port_value_t value)
{
	const struct kea_gpio_config *cfg = dev->config;
	uint32_t reg;

	reg = cfg->base->PDOR;
	reg = (reg & ~mask) | (value & mask);
	cfg->base->PDOR = reg;

	return 0;
}

static int kea_gpio_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct kea_gpio_config *cfg = dev->config;

	cfg->base->PSOR = pins;

	return 0;
}

static int kea_gpio_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct kea_gpio_config *cfg = dev->config;

	cfg->base->PCOR = pins;

	return 0;
}

static int kea_gpio_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct kea_gpio_config *cfg = dev->config;

	cfg->base->PTOR = pins;

	return 0;
}

static void kea_gpio_update_kbi_locked(const struct kea_gpio_config *cfg,
				       const struct kea_gpio_data *data)
{
	uint32_t sc;

	if (cfg->kbi == NULL) {
		return;
	}

	cfg->kbi->PE = data->int_enabled_mask;
	cfg->kbi->ES = data->int_rising_mask & data->int_enabled_mask;

	sc = cfg->kbi->SC;
	if ((data->int_level_mask & data->int_enabled_mask) != 0u) {
		sc |= KEA_KBI_SC_KBMOD_MASK;
	} else {
		sc &= ~KEA_KBI_SC_KBMOD_MASK;
	}

	if (data->int_enabled_mask != 0u) {
		sc |= KEA_KBI_SC_KBIE_MASK;
	} else {
		sc &= ~KEA_KBI_SC_KBIE_MASK;
	}

	sc |= KEA_KBI_SC_KBSPEN_MASK;
	cfg->kbi->SC = sc;
	cfg->kbi->SC |= KEA_KBI_SC_RSTKBSP_MASK;
}

static int kea_gpio_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					    enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct kea_gpio_config *cfg = dev->config;
	struct kea_gpio_data *data = dev->data;
	const gpio_port_pins_t pin_mask = BIT(pin);
	unsigned int key;

	if (pin >= 32u) {
		return -EINVAL;
	}

	if ((pin_mask & cfg->common.port_pin_mask) == 0u) {
		return -EINVAL;
	}

	if (cfg->kbi == NULL) {
		return (mode == GPIO_INT_MODE_DISABLED) ? 0 : -ENOTSUP;
	}

	if ((cfg->base->PDDR & pin_mask) != 0u) {
		return -EINVAL;
	}

	if ((trig & GPIO_INT_WAKEUP) != 0u) {
		return -ENOTSUP;
	}

	key = irq_lock();

#ifdef CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT
	if (mode == GPIO_INT_MODE_DISABLE_ONLY) {
		data->int_enabled_mask &= ~pin_mask;
		kea_gpio_update_kbi_locked(cfg, data);
		irq_unlock(key);
		return 0;
	}

	if (mode == GPIO_INT_MODE_ENABLE_ONLY) {
		if ((data->int_configured_mask & pin_mask) == 0u) {
			irq_unlock(key);
			return -EINVAL;
		}

		data->int_enabled_mask |= pin_mask;
		kea_gpio_update_kbi_locked(cfg, data);
		irq_unlock(key);
		return 0;
	}
#endif

	if (mode == GPIO_INT_MODE_DISABLED) {
		data->int_enabled_mask &= ~pin_mask;
		kea_gpio_update_kbi_locked(cfg, data);
		irq_unlock(key);
		return 0;
	}

	if ((trig != GPIO_INT_TRIG_LOW) && (trig != GPIO_INT_TRIG_HIGH)) {
		irq_unlock(key);
		return -ENOTSUP;
	}

	if ((mode != GPIO_INT_MODE_EDGE) && (mode != GPIO_INT_MODE_LEVEL)) {
		irq_unlock(key);
		return -ENOTSUP;
	}

	data->int_configured_mask |= pin_mask;
	data->int_enabled_mask |= pin_mask;

	if (trig == GPIO_INT_TRIG_HIGH) {
		data->int_rising_mask |= pin_mask;
	} else {
		data->int_rising_mask &= ~pin_mask;
	}

	if (mode == GPIO_INT_MODE_LEVEL) {
		data->int_level_mask |= pin_mask;
	} else {
		data->int_level_mask &= ~pin_mask;
	}

	kea_gpio_update_kbi_locked(cfg, data);
	irq_unlock(key);

	return 0;
}

static int kea_gpio_manage_callback(const struct device *dev, struct gpio_callback *callback,
				    bool set)
{
	struct kea_gpio_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static uint32_t kea_gpio_get_pending_int(const struct device *dev)
{
	const struct kea_gpio_config *cfg = dev->config;
	const struct kea_gpio_data *data = dev->data;

	if (cfg->kbi == NULL) {
		return 0u;
	}

	return cfg->kbi->SP & data->int_enabled_mask;
}

#ifdef CONFIG_GPIO_GET_DIRECTION
static int kea_gpio_port_get_direction(const struct device *dev, gpio_port_pins_t map,
				       gpio_port_pins_t *inputs,
				       gpio_port_pins_t *outputs)
{
	const struct kea_gpio_config *cfg = dev->config;
	const gpio_port_pins_t out_mask = cfg->base->PDDR & map;

	if (outputs != NULL) {
		*outputs = out_mask;
	}

	if (inputs != NULL) {
		*inputs = map & ~out_mask;
	}

	return 0;
}
#endif

static void kea_gpio_isr(const struct device *dev)
{
	const struct kea_gpio_config *cfg = dev->config;
	struct kea_gpio_data *data = dev->data;
	gpio_port_pins_t pending;

	if ((cfg->kbi == NULL) || ((cfg->kbi->SC & KEA_KBI_SC_KBF_MASK) == 0u)) {
		return;
	}

	pending = cfg->kbi->SP & data->int_enabled_mask;
	cfg->kbi->SC |= KEA_KBI_SC_KBACK_MASK;
	cfg->kbi->SC |= KEA_KBI_SC_RSTKBSP_MASK;

	if (pending != 0u) {
		gpio_fire_callbacks(&data->callbacks, dev, pending);
	}
}

static int kea_gpio_init(const struct device *dev)
{
	const struct kea_gpio_config *cfg = dev->config;
	struct kea_gpio_data *data = dev->data;

	sys_slist_init(&data->callbacks);
	data->int_enabled_mask = 0u;
	data->int_rising_mask = 0u;
	data->int_level_mask = 0u;
	data->int_configured_mask = 0u;

	if (cfg->kbi != NULL) {
		KEA_SIM->SCGC |= cfg->kbi_clock_gate_mask;
		cfg->kbi->PE = 0u;
		cfg->kbi->ES = 0u;
		cfg->kbi->SC = KEA_KBI_SC_RSTKBSP_MASK | KEA_KBI_SC_KBSPEN_MASK;
		cfg->kbi->SC |= KEA_KBI_SC_KBACK_MASK;

		if (cfg->irq_config_func != NULL) {
			cfg->irq_config_func(dev);
		}
	}

	return 0;
}

static DEVICE_API(gpio, kea_gpio_driver_api) = {
	.pin_configure = kea_gpio_pin_configure,
	.port_get_raw = kea_gpio_port_get_raw,
	.port_set_masked_raw = kea_gpio_port_set_masked_raw,
	.port_set_bits_raw = kea_gpio_port_set_bits_raw,
	.port_clear_bits_raw = kea_gpio_port_clear_bits_raw,
	.port_toggle_bits = kea_gpio_port_toggle_bits,
	.pin_interrupt_configure = kea_gpio_pin_interrupt_configure,
	.manage_callback = kea_gpio_manage_callback,
	.get_pending_int = kea_gpio_get_pending_int,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = kea_gpio_port_get_direction,
#endif
};

#define KEA_GPIO_IRQ_CONFIG(inst)                                                           \
	COND_CODE_1(DT_INST_IRQ_HAS_IDX(inst, 0),                                           \
		    (static void kea_gpio_irq_config_##inst(const struct device *dev)       \
		     {                                                                      \
			     ARG_UNUSED(dev);                                               \
			     IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),   \
					 kea_gpio_isr, DEVICE_DT_INST_GET(inst), 0);   \
			     irq_enable(DT_INST_IRQN(inst));                                \
		     }),                                                                  \
		    ())

#define KEA_GPIO_INIT(inst)                                                                     \
	KEA_GPIO_IRQ_CONFIG(inst)                                                                \
                                                                                                   \
	static const struct kea_gpio_config kea_gpio_cfg_##inst = {                              \
		.common = {                                                                         \
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),                      \
		},                                                                                  \
		.base = (kea_gpio_t *)DT_INST_REG_ADDR(inst),                                      \
		.kbi = COND_CODE_1(                                                                \
			DT_INST_IRQ_HAS_IDX(inst, 0),                                             \
			(((DT_INST_IRQN(inst) == 24)                                             \
				  ? KEA_KBI0                                                   \
				  : ((DT_INST_IRQN(inst) == 25) ? KEA_KBI1 : NULL))),         \
			(NULL)),                                                              \
		.kbi_clock_gate_mask = COND_CODE_1(                                                \
			DT_INST_IRQ_HAS_IDX(inst, 0),                                             \
			(((DT_INST_IRQN(inst) == 24)                                             \
				  ? KEA_SIM_SCGC_KBI0_MASK                                       \
				  : ((DT_INST_IRQN(inst) == 25) ? KEA_SIM_SCGC_KBI1_MASK : 0u))),\
			(0u)),                                                                \
		.irq_config_func = COND_CODE_1(DT_INST_IRQ_HAS_IDX(inst, 0),                      \
					       (kea_gpio_irq_config_##inst), (NULL)),     \
	};                                                                                          \
                                                                                                   \
	static struct kea_gpio_data kea_gpio_data_##inst;                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, kea_gpio_init, NULL, &kea_gpio_data_##inst,                    \
			      &kea_gpio_cfg_##inst, POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,         \
			      &kea_gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KEA_GPIO_INIT)
