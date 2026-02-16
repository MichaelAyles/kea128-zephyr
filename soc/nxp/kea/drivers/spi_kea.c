/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kea_spi

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "kea_regs.h"

#define KEA_SPI_WAIT_LOOPS 100000u

struct kea_spi_config {
	kea_spi_t *base;
	uint32_t module_clock_hz;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
};

struct kea_spi_data {
	struct k_mutex lock;
};

struct kea_spi_cursor {
	const struct spi_buf_set *set;
	size_t buf_idx;
	size_t offset;
};

static size_t kea_spi_total_len(const struct spi_buf_set *set)
{
	size_t total = 0u;

	if (set == NULL) {
		return 0u;
	}

	for (size_t i = 0; i < set->count; i++) {
		total += set->buffers[i].len;
	}

	return total;
}

static bool kea_spi_cursor_get_byte(struct kea_spi_cursor *cursor, uint8_t *byte)
{
	while (cursor->buf_idx < cursor->set->count) {
		const struct spi_buf *buf = &cursor->set->buffers[cursor->buf_idx];

		if ((buf->buf == NULL) || (cursor->offset >= buf->len)) {
			cursor->buf_idx++;
			cursor->offset = 0u;
			continue;
		}

		*byte = ((const uint8_t *)buf->buf)[cursor->offset++];
		return true;
	}

	return false;
}

static bool kea_spi_cursor_put_byte(struct kea_spi_cursor *cursor, uint8_t byte)
{
	while (cursor->buf_idx < cursor->set->count) {
		const struct spi_buf *buf = &cursor->set->buffers[cursor->buf_idx];

		if ((buf->buf == NULL) || (cursor->offset >= buf->len)) {
			cursor->buf_idx++;
			cursor->offset = 0u;
			continue;
		}

		((uint8_t *)buf->buf)[cursor->offset++] = byte;
		return true;
	}

	return false;
}

static int kea_spi_wait_flag(volatile uint8_t *reg, uint8_t mask)
{
	for (uint32_t i = 0; i < KEA_SPI_WAIT_LOOPS; i++) {
		if ((*reg & mask) != 0u) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int kea_spi_apply_config(const struct device *dev, const struct spi_config *spi_cfg)
{
	const struct kea_spi_config *cfg = dev->config;
	uint8_t c1 = KEA_SPI_C1_MSTR_MASK | KEA_SPI_C1_SPE_MASK;
	uint8_t br = 0u;
	uint32_t best_err = UINT32_MAX;

	if ((spi_cfg->operation & SPI_OP_MODE_MASTER) == 0u) {
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(spi_cfg->operation) != 8u) {
		return -ENOTSUP;
	}

	if ((spi_cfg->operation & SPI_CS_ACTIVE_HIGH) != 0u) {
		return -ENOTSUP;
	}

	if ((spi_cfg->operation & SPI_FRAME_FORMAT_TI) != 0u) {
		return -ENOTSUP;
	}

	if (spi_cfg->frequency == 0u) {
		return -EINVAL;
	}

	if ((spi_cfg->operation & SPI_MODE_CPOL) != 0u) {
		c1 |= KEA_SPI_C1_CPOL_MASK;
	}

	if ((spi_cfg->operation & SPI_MODE_CPHA) != 0u) {
		c1 |= KEA_SPI_C1_CPHA_MASK;
	}

	if ((spi_cfg->operation & SPI_TRANSFER_LSB) != 0u) {
		c1 |= KEA_SPI_C1_LSBFE_MASK;
	}

	for (uint32_t sppr = 0u; sppr < 8u; sppr++) {
		for (uint32_t spr = 0u; spr < 16u; spr++) {
			const uint32_t div = (sppr + 1u) << (spr + 1u);
			const uint32_t rate = cfg->module_clock_hz / div;
			const uint32_t err = (rate > spi_cfg->frequency) ?
						     (rate - spi_cfg->frequency) :
						     (spi_cfg->frequency - rate);

			if (err < best_err) {
				best_err = err;
				br = (uint8_t)((sppr << KEA_SPI_BR_SPPR_SHIFT) | (spr & KEA_SPI_BR_SPR_MASK));
			}
		}
	}

	cfg->base->C1 = 0u;
	cfg->base->C2 = 0u;
	cfg->base->BR = br;
	cfg->base->C1 = c1;

	return 0;
}

static int kea_spi_transceive(const struct device *dev, const struct spi_config *spi_cfg,
			      const struct spi_buf_set *tx_bufs,
			      const struct spi_buf_set *rx_bufs)
{
	const struct kea_spi_config *cfg = dev->config;
	struct kea_spi_data *data = dev->data;
	struct kea_spi_cursor tx_cursor = { .set = tx_bufs };
	struct kea_spi_cursor rx_cursor = { .set = rx_bufs };
	size_t tx_len = kea_spi_total_len(tx_bufs);
	size_t rx_len = kea_spi_total_len(rx_bufs);
	size_t frames = MAX(tx_len, rx_len);
	int ret;

	if ((tx_bufs == NULL) && (rx_bufs == NULL)) {
		return 0;
	}

	if ((spi_cfg->cs.cs_is_gpio) && (spi_cfg->cs.gpio.port != NULL)) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = kea_spi_apply_config(dev, spi_cfg);
	if (ret != 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	for (size_t i = 0; i < frames; i++) {
		uint8_t tx_byte = 0xFFu;
		uint8_t rx_byte;

		if ((tx_bufs != NULL) && (tx_len > 0u)) {
			(void)kea_spi_cursor_get_byte(&tx_cursor, &tx_byte);
		}

		ret = kea_spi_wait_flag(&cfg->base->S, KEA_SPI_S_SPTEF_MASK);
		if (ret != 0) {
			break;
		}

		cfg->base->D = tx_byte;

		ret = kea_spi_wait_flag(&cfg->base->S, KEA_SPI_S_SPRF_MASK);
		if (ret != 0) {
			break;
		}

		rx_byte = cfg->base->D;

		if ((rx_bufs != NULL) && (rx_len > 0u)) {
			(void)kea_spi_cursor_put_byte(&rx_cursor, rx_byte);
		}
	}

	k_mutex_unlock(&data->lock);

	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int kea_spi_release(const struct device *dev, const struct spi_config *spi_cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(spi_cfg);

	return 0;
}

static int kea_spi_init(const struct device *dev)
{
	const struct kea_spi_config *cfg = dev->config;
	struct kea_spi_data *data = dev->data;
	int ret;

	k_mutex_init(&data->lock);

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

	cfg->base->C1 = 0u;
	cfg->base->C2 = 0u;

	return 0;
}

static DEVICE_API(spi, kea_spi_driver_api) = {
	.transceive = kea_spi_transceive,
	.release = kea_spi_release,
};

#define KEA_SPI_INIT(inst)                                                                  \
	PINCTRL_DT_INST_DEFINE(inst);                                                       \
                                                                                               \
	static const struct kea_spi_config kea_spi_cfg_##inst = {                            \
		.base = (kea_spi_t *)DT_INST_REG_ADDR(inst),                                    \
		.module_clock_hz = DT_INST_PROP(inst, nxp_clock_frequency),                    \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                          \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, bits),        \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                  \
	};                                                                                       \
                                                                                               \
	static struct kea_spi_data kea_spi_data_##inst;                                        \
                                                                                               \
	SPI_DEVICE_DT_INST_DEFINE(inst, kea_spi_init, NULL, &kea_spi_data_##inst,              \
				  &kea_spi_cfg_##inst, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,         \
				  &kea_spi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KEA_SPI_INIT)
