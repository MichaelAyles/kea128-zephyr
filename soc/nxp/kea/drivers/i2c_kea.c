/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kea_i2c

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "kea_regs.h"

#define KEA_I2C_WAIT_LOOPS 100000u

struct kea_i2c_config {
	kea_i2c_t *base;
	uint32_t module_clock_hz;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
};

struct kea_i2c_data {
	uint32_t dev_config;
	struct k_mutex lock;
};

static const uint16_t kea_i2c_scl_div_lut[64] = {
	20, 22, 24, 26, 28, 30, 34, 40,
	28, 32, 36, 40, 44, 48, 56, 68,
	48, 56, 64, 72, 80, 88, 104, 128,
	80, 96, 112, 128, 144, 160, 192, 240,
	160, 192, 224, 256, 288, 320, 384, 480,
	320, 384, 448, 512, 576, 640, 768, 960,
	640, 768, 896, 1024, 1152, 1280, 1536, 1920,
	1280, 1536, 1792, 2048, 2304, 2560, 3072, 3840,
};

static int kea_i2c_wait_bus_idle(kea_i2c_t *base)
{
	for (uint32_t i = 0; i < KEA_I2C_WAIT_LOOPS; i++) {
		if ((base->S & KEA_I2C_S_BUSY_MASK) == 0u) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int kea_i2c_wait_transfer(kea_i2c_t *base)
{
	uint8_t status;

	for (uint32_t i = 0; i < KEA_I2C_WAIT_LOOPS; i++) {
		status = base->S;
		if ((status & KEA_I2C_S_IICIF_MASK) != 0u) {
			if ((status & KEA_I2C_S_ARBL_MASK) != 0u) {
				base->S = KEA_I2C_S_ARBL_MASK | KEA_I2C_S_IICIF_MASK;
				return -EIO;
			}

			base->S = KEA_I2C_S_IICIF_MASK;
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static inline void kea_i2c_stop(kea_i2c_t *base)
{
	base->C1 &= (uint8_t)~(KEA_I2C_C1_MST_MASK | KEA_I2C_C1_TX_MASK |
			      KEA_I2C_C1_TXAK_MASK | KEA_I2C_C1_RSTA_MASK);
}

static int kea_i2c_start(kea_i2c_t *base, bool repeated)
{
	if (repeated) {
		base->C1 |= KEA_I2C_C1_TX_MASK;
		base->C1 |= KEA_I2C_C1_RSTA_MASK;
		return 0;
	}

	if (kea_i2c_wait_bus_idle(base) != 0) {
		return -EBUSY;
	}

	base->C1 |= KEA_I2C_C1_TX_MASK | KEA_I2C_C1_MST_MASK;

	return 0;
}

static int kea_i2c_send_byte(kea_i2c_t *base, uint8_t byte)
{
	base->D = byte;

	if (kea_i2c_wait_transfer(base) != 0) {
		return -EIO;
	}

	if ((base->S & KEA_I2C_S_RXAK_MASK) != 0u) {
		return -EIO;
	}

	return 0;
}

static int kea_i2c_write_msg(kea_i2c_t *base, const struct i2c_msg *msg)
{
	for (uint32_t i = 0; i < msg->len; i++) {
		if (kea_i2c_send_byte(base, msg->buf[i]) != 0) {
			return -EIO;
		}
	}

	return 0;
}

static int kea_i2c_read_msg(kea_i2c_t *base, const struct i2c_msg *msg, bool stop)
{
	uint8_t *buf = msg->buf;

	if (msg->len == 0u) {
		if (stop) {
			kea_i2c_stop(base);
		}
		return 0;
	}

	base->C1 &= (uint8_t)~KEA_I2C_C1_TX_MASK;

	if (msg->len == 1u) {
		base->C1 |= KEA_I2C_C1_TXAK_MASK;
	} else {
		base->C1 &= (uint8_t)~KEA_I2C_C1_TXAK_MASK;
	}

	(void)base->D;

	for (uint32_t i = 0; i < msg->len; i++) {
		if (kea_i2c_wait_transfer(base) != 0) {
			return -EIO;
		}

		if (i == (msg->len - 2u)) {
			base->C1 |= KEA_I2C_C1_TXAK_MASK;
		}

		if ((i == (msg->len - 1u)) && stop) {
			kea_i2c_stop(base);
		}

		buf[i] = base->D;
	}

	return 0;
}

static int kea_i2c_set_bitrate(const struct device *dev, uint32_t bitrate)
{
	const struct kea_i2c_config *cfg = dev->config;
	kea_i2c_t *base = cfg->base;
	uint32_t best_err = UINT32_MAX;
	uint32_t best_mult = 0u;
	uint32_t best_icr = 0u;

	if ((bitrate == 0u) || (cfg->module_clock_hz == 0u)) {
		return -EINVAL;
	}

	for (uint32_t mult_idx = 0u; mult_idx < 3u; mult_idx++) {
		const uint32_t mult = (1u << mult_idx);

		for (uint32_t icr = 0u; icr < ARRAY_SIZE(kea_i2c_scl_div_lut); icr++) {
			const uint32_t clk = cfg->module_clock_hz / (mult * kea_i2c_scl_div_lut[icr]);
			const uint32_t err = (clk > bitrate) ? (clk - bitrate) : (bitrate - clk);

			if (err < best_err) {
				best_err = err;
				best_mult = mult_idx;
				best_icr = icr;
			}
		}
	}

	base->F = (uint8_t)((best_mult << 6) | best_icr);

	return 0;
}

static int kea_i2c_configure(const struct device *dev, uint32_t dev_config)
{
	const struct kea_i2c_config *cfg = dev->config;
	struct kea_i2c_data *data = dev->data;
	uint32_t bitrate;
	int ret;

	if ((dev_config & I2C_MODE_CONTROLLER) == 0u) {
		return -ENOTSUP;
	}

	if ((dev_config & I2C_ADDR_10_BITS) != 0u) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		bitrate = 100000u;
		break;
	case I2C_SPEED_FAST:
		bitrate = 400000u;
		break;
	default:
		return -ENOTSUP;
	}

	ret = kea_i2c_set_bitrate(dev, bitrate);
	if (ret != 0) {
		return ret;
	}

	cfg->base->C1 = KEA_I2C_C1_IICEN_MASK;
	data->dev_config = dev_config;

	return 0;
}

static int kea_i2c_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct kea_i2c_data *data = dev->data;

	*dev_config = data->dev_config;

	return 0;
}

static int kea_i2c_transfer(const struct device *dev, struct i2c_msg *msgs,
			    uint8_t num_msgs, uint16_t addr)
{
	const struct kea_i2c_config *cfg = dev->config;
	struct kea_i2c_data *data = dev->data;
	kea_i2c_t *base = cfg->base;
	bool started = false;
	int ret = 0;

	if ((num_msgs == 0u) || (addr > 0x7Fu)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	for (uint32_t i = 0; i < num_msgs; i++) {
		const bool read = (msgs[i].flags & I2C_MSG_READ) != 0u;
		const bool restart = (!started) || (i > 0u) || ((msgs[i].flags & I2C_MSG_RESTART) != 0u);
		const bool stop = ((msgs[i].flags & I2C_MSG_STOP) != 0u) || (i == (num_msgs - 1u));
		const uint8_t addr_byte = (uint8_t)((addr << 1) | (read ? 1u : 0u));

		ret = kea_i2c_start(base, started && restart);
		if (ret != 0) {
			break;
		}

		started = true;

		ret = kea_i2c_send_byte(base, addr_byte);
		if (ret != 0) {
			break;
		}

		if (read) {
			ret = kea_i2c_read_msg(base, &msgs[i], stop);
		} else {
			ret = kea_i2c_write_msg(base, &msgs[i]);
			if ((ret == 0) && stop) {
				kea_i2c_stop(base);
			}
		}

		if (ret != 0) {
			break;
		}

		if (stop) {
			started = false;
		}
	}

	if (ret != 0) {
		kea_i2c_stop(base);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static int kea_i2c_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);

	return -ENOTSUP;
}

static int kea_i2c_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);

	return -ENOTSUP;
}

static int kea_i2c_recover_bus(const struct device *dev)
{
	const struct kea_i2c_config *cfg = dev->config;

	kea_i2c_stop(cfg->base);

	return 0;
}

static int kea_i2c_init(const struct device *dev)
{
	const struct kea_i2c_config *cfg = dev->config;
	struct kea_i2c_data *data = dev->data;
	const uint32_t default_cfg = I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD);
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
	cfg->base->S = KEA_I2C_S_IICIF_MASK | KEA_I2C_S_ARBL_MASK;

	return kea_i2c_configure(dev, default_cfg);
}

static DEVICE_API(i2c, kea_i2c_driver_api) = {
	.configure = kea_i2c_configure,
	.get_config = kea_i2c_get_config,
	.transfer = kea_i2c_transfer,
	.target_register = kea_i2c_target_register,
	.target_unregister = kea_i2c_target_unregister,
	.recover_bus = kea_i2c_recover_bus,
};

#define KEA_I2C_INIT(inst)                                                                  \
	PINCTRL_DT_INST_DEFINE(inst);                                                       \
                                                                                               \
	static const struct kea_i2c_config kea_i2c_cfg_##inst = {                            \
		.base = (kea_i2c_t *)DT_INST_REG_ADDR(inst),                                    \
		.module_clock_hz = DT_INST_PROP(inst, nxp_module_clock_frequency),              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                          \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, bits),        \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                  \
	};                                                                                       \
                                                                                               \
	static struct kea_i2c_data kea_i2c_data_##inst;                                        \
                                                                                               \
	I2C_DEVICE_DT_INST_DEFINE(inst, kea_i2c_init, NULL, &kea_i2c_data_##inst,              \
				  &kea_i2c_cfg_##inst, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,         \
				  &kea_i2c_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KEA_I2C_INIT)
