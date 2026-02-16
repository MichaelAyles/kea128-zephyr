/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kea_uart

#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include "kea_regs.h"

struct kea_uart_config {
	kea_uart_t *base;
	uint32_t clock_frequency;
	uint32_t baudrate;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config_func)(const struct device *dev);
};

struct kea_uart_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *callback_data;
#endif
};

static int kea_uart_poll_in(const struct device *dev, unsigned char *c)
{
	const struct kea_uart_config *cfg = dev->config;

	if ((cfg->base->S1 & KEA_UART_S1_RDRF_MASK) == 0U) {
		return -1;
	}

	*c = cfg->base->D;
	return 0;
}

static void kea_uart_poll_out(const struct device *dev, unsigned char c)
{
	const struct kea_uart_config *cfg = dev->config;

	while ((cfg->base->S1 & KEA_UART_S1_TDRE_MASK) == 0U) {
	}

	cfg->base->D = c;
}

static int kea_uart_err_check(const struct device *dev)
{
	const struct kea_uart_config *cfg = dev->config;
	const uint8_t s1 = cfg->base->S1;
	int err = 0;

	if ((s1 & KEA_UART_S1_PF_MASK) != 0U) {
		err |= UART_ERROR_PARITY;
	}

	if ((s1 & KEA_UART_S1_FE_MASK) != 0U) {
		err |= UART_ERROR_FRAMING;
	}

	if ((s1 & KEA_UART_S1_NF_MASK) != 0U) {
		err |= UART_ERROR_NOISE;
	}

	if ((s1 & KEA_UART_S1_OR_MASK) != 0U) {
		err |= UART_ERROR_OVERRUN;
	}

	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int kea_uart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct kea_uart_config *cfg = dev->config;
	int sent = 0;

	while ((sent < len) && ((cfg->base->S1 & KEA_UART_S1_TDRE_MASK) != 0U)) {
		cfg->base->D = tx_data[sent++];
	}

	return sent;
}

static int kea_uart_fifo_read(const struct device *dev, uint8_t *rx_data, const int len)
{
	const struct kea_uart_config *cfg = dev->config;
	int read = 0;

	while ((read < len) && ((cfg->base->S1 & KEA_UART_S1_RDRF_MASK) != 0U)) {
		rx_data[read++] = cfg->base->D;
	}

	return read;
}

static void kea_uart_irq_tx_enable(const struct device *dev)
{
	const struct kea_uart_config *cfg = dev->config;

	cfg->base->C2 |= KEA_UART_C2_TIE_MASK;
}

static void kea_uart_irq_tx_disable(const struct device *dev)
{
	const struct kea_uart_config *cfg = dev->config;

	cfg->base->C2 &= (uint8_t)~KEA_UART_C2_TIE_MASK;
}

static int kea_uart_irq_tx_ready(const struct device *dev)
{
	const struct kea_uart_config *cfg = dev->config;

	return ((cfg->base->S1 & KEA_UART_S1_TDRE_MASK) != 0U) ? 1 : 0;
}

static void kea_uart_irq_rx_enable(const struct device *dev)
{
	const struct kea_uart_config *cfg = dev->config;

	cfg->base->C2 |= KEA_UART_C2_RIE_MASK;
}

static void kea_uart_irq_rx_disable(const struct device *dev)
{
	const struct kea_uart_config *cfg = dev->config;

	cfg->base->C2 &= (uint8_t)~KEA_UART_C2_RIE_MASK;
}

static int kea_uart_irq_tx_complete(const struct device *dev)
{
	const struct kea_uart_config *cfg = dev->config;

	return ((cfg->base->S1 & KEA_UART_S1_TC_MASK) != 0U) ? 1 : 0;
}

static int kea_uart_irq_rx_ready(const struct device *dev)
{
	const struct kea_uart_config *cfg = dev->config;

	return ((cfg->base->S1 & KEA_UART_S1_RDRF_MASK) != 0U) ? 1 : 0;
}

static void kea_uart_irq_err_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void kea_uart_irq_err_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static int kea_uart_irq_is_pending(const struct device *dev)
{
	const struct kea_uart_config *cfg = dev->config;
	const uint8_t c2 = cfg->base->C2;
	const uint8_t s1 = cfg->base->S1;

	const bool rx_pending = ((c2 & KEA_UART_C2_RIE_MASK) != 0U) &&
				((s1 & KEA_UART_S1_RDRF_MASK) != 0U);
	const bool tx_pending = ((c2 & KEA_UART_C2_TIE_MASK) != 0U) &&
				((s1 & KEA_UART_S1_TDRE_MASK) != 0U);

	return (rx_pending || tx_pending) ? 1 : 0;
}

static int kea_uart_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 1;
}

static void kea_uart_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				      void *user_data)
{
	struct kea_uart_data *data = dev->data;

	data->callback = cb;
	data->callback_data = user_data;
}

static void kea_uart_isr(const struct device *dev)
{
	struct kea_uart_data *data = dev->data;

	if (data->callback != NULL) {
		data->callback(dev, data->callback_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int kea_uart_init(const struct device *dev)
{
	const struct kea_uart_config *cfg = dev->config;
	const uint32_t divisor = 16u * cfg->baudrate;
	uint32_t sbr;
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

	if (divisor == 0U) {
		return -EINVAL;
	}

	sbr = cfg->clock_frequency / divisor;
	if ((sbr == 0U) || (sbr > 0x1FFFu)) {
		return -EINVAL;
	}

	cfg->base->BDH = (uint8_t)(sbr >> 8) & KEA_UART_BDH_SBR_MASK;
	cfg->base->BDL = (uint8_t)(sbr & 0xFFu);
	cfg->base->C1 = 0u;
	cfg->base->C2 = KEA_UART_C2_TE_MASK | KEA_UART_C2_RE_MASK;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (cfg->irq_config_func != NULL) {
		cfg->irq_config_func(dev);
	}
#endif

	return 0;
}

static DEVICE_API(uart, kea_uart_driver_api) = {
	.poll_in = kea_uart_poll_in,
	.poll_out = kea_uart_poll_out,
	.err_check = kea_uart_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = kea_uart_fifo_fill,
	.fifo_read = kea_uart_fifo_read,
	.irq_tx_enable = kea_uart_irq_tx_enable,
	.irq_tx_disable = kea_uart_irq_tx_disable,
	.irq_tx_ready = kea_uart_irq_tx_ready,
	.irq_rx_enable = kea_uart_irq_rx_enable,
	.irq_rx_disable = kea_uart_irq_rx_disable,
	.irq_tx_complete = kea_uart_irq_tx_complete,
	.irq_rx_ready = kea_uart_irq_rx_ready,
	.irq_err_enable = kea_uart_irq_err_enable,
	.irq_err_disable = kea_uart_irq_err_disable,
	.irq_is_pending = kea_uart_irq_is_pending,
	.irq_update = kea_uart_irq_update,
	.irq_callback_set = kea_uart_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define KEA_UART_IRQ_CONFIG(inst)                                                           \
	static void kea_uart_irq_config_##inst(const struct device *dev)                    \
	{                                                                                     \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), kea_uart_isr,  \
			    DEVICE_DT_INST_GET(inst), 0);                                      \
		irq_enable(DT_INST_IRQN(inst));                                                 \
	}
#else
#define KEA_UART_IRQ_CONFIG(inst)
#endif

#define KEA_UART_INIT(inst)                                                                     \
	KEA_UART_IRQ_CONFIG(inst)                                                                \
	PINCTRL_DT_INST_DEFINE(inst);                                                           \
                                                                                                   \
	static const struct kea_uart_config kea_uart_cfg_##inst = {                             \
		.base = (kea_uart_t *)DT_INST_REG_ADDR(inst),                                    \
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),                           \
		.baudrate = DT_INST_PROP(inst, current_speed),                                    \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                           \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, bits),         \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
		IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN,                                          \
			   (.irq_config_func = kea_uart_irq_config_##inst,))                         \
	};                                                                                     \
                                                                                                   \
	static struct kea_uart_data kea_uart_data_##inst;                                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, kea_uart_init, NULL, &kea_uart_data_##inst,                \
			      &kea_uart_cfg_##inst, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,    \
			      &kea_uart_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KEA_UART_INIT)
