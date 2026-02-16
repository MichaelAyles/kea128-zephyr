/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kea_wdog

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>

#include "kea_regs.h"

#define KEA_WDOG_CHANNEL_ID 0
#define KEA_WDOG_CLOCK_HZ 1000u

struct kea_wdog_config {
	kea_wdog_t *base;
};

struct kea_wdog_data {
	uint16_t timeout_ticks;
	bool timeout_valid;
	bool enabled;
};

static void kea_wdog_unlock(kea_wdog_t *base)
{
	base->CNT = KEA_WDOG_UNLOCK_KEY1;
	base->CNT = KEA_WDOG_UNLOCK_KEY2;
}

static int kea_wdog_setup(const struct device *dev, uint8_t options)
{
	const struct kea_wdog_config *cfg = dev->config;
	struct kea_wdog_data *data = dev->data;
	uint8_t cs1 = KEA_WDOG_CS1_UPDATE_MASK | KEA_WDOG_CS1_EN_MASK;

	if (!data->timeout_valid) {
		return -EINVAL;
	}

	if (data->enabled) {
		return -EBUSY;
	}

	if ((options & ~(WDT_OPT_PAUSE_IN_SLEEP | WDT_OPT_PAUSE_HALTED_BY_DBG)) != 0u) {
		return -ENOTSUP;
	}

	if ((options & WDT_OPT_PAUSE_IN_SLEEP) != 0u) {
		cs1 |= KEA_WDOG_CS1_WAIT_MASK | KEA_WDOG_CS1_STOP_MASK;
	}

	if ((options & WDT_OPT_PAUSE_HALTED_BY_DBG) != 0u) {
		cs1 |= KEA_WDOG_CS1_DBG_MASK;
	}

	kea_wdog_unlock(cfg->base);
	cfg->base->TOVAL = data->timeout_ticks;
	cfg->base->WIN = UINT16_MAX;
	cfg->base->CS2 = KEA_WDOG_CS2_CLK_LPO;
	cfg->base->CS1 = cs1;

	data->enabled = true;

	return 0;
}

static int kea_wdog_disable(const struct device *dev)
{
	const struct kea_wdog_config *cfg = dev->config;
	struct kea_wdog_data *data = dev->data;

	if (!data->enabled) {
		return -EFAULT;
	}

	kea_wdog_unlock(cfg->base);
	cfg->base->TOVAL = 0xE803u;
	cfg->base->WIN = UINT16_MAX;
	cfg->base->CS2 = KEA_WDOG_CS2_CLK_LPO;
	cfg->base->CS1 = KEA_WDOG_CS1_UPDATE_MASK | KEA_WDOG_CS1_WAIT_MASK | KEA_WDOG_CS1_STOP_MASK;

	data->enabled = false;
	data->timeout_valid = false;

	return 0;
}

static int kea_wdog_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	struct kea_wdog_data *data = dev->data;
	uint64_t ticks;

	if (data->enabled) {
		return -EBUSY;
	}

	if (data->timeout_valid) {
		return -ENOMEM;
	}

	if (cfg->window.max == 0u) {
		return -EINVAL;
	}

	if (cfg->window.min != 0u) {
		return -ENOTSUP;
	}

	if (cfg->callback != NULL) {
		return -ENOTSUP;
	}

	if (cfg->flags != WDT_FLAG_RESET_SOC) {
		return -ENOTSUP;
	}

	ticks = ((uint64_t)cfg->window.max * KEA_WDOG_CLOCK_HZ + 999u) / 1000u;
	if ((ticks == 0u) || (ticks > UINT16_MAX)) {
		return -EINVAL;
	}

	data->timeout_ticks = (uint16_t)ticks;
	data->timeout_valid = true;

	return KEA_WDOG_CHANNEL_ID;
}

static int kea_wdog_feed(const struct device *dev, int channel_id)
{
	const struct kea_wdog_config *cfg = dev->config;
	struct kea_wdog_data *data = dev->data;

	if (channel_id != KEA_WDOG_CHANNEL_ID) {
		return -EINVAL;
	}

	if (!data->enabled) {
		return -EFAULT;
	}

	cfg->base->CNT = KEA_WDOG_REFRESH_KEY1;
	cfg->base->CNT = KEA_WDOG_REFRESH_KEY2;

	return 0;
}

static int kea_wdog_init(const struct device *dev)
{
	const struct kea_wdog_config *cfg = dev->config;
	struct kea_wdog_data *data = dev->data;

	data->timeout_valid = false;
	data->enabled = (cfg->base->CS1 & KEA_WDOG_CS1_EN_MASK) != 0u;

#ifdef CONFIG_WDT_DISABLE_AT_BOOT
	if (data->enabled) {
		return kea_wdog_disable(dev);
	}
#endif

	return 0;
}

static DEVICE_API(wdt, kea_wdog_driver_api) = {
	.setup = kea_wdog_setup,
	.disable = kea_wdog_disable,
	.install_timeout = kea_wdog_install_timeout,
	.feed = kea_wdog_feed,
};

#define KEA_WDOG_INIT(inst)                                                                     \
	static struct kea_wdog_data kea_wdog_data_##inst;                                        \
                                                                                                   \
	static const struct kea_wdog_config kea_wdog_cfg_##inst = {                               \
		.base = (kea_wdog_t *)DT_INST_REG_ADDR(inst),                                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, kea_wdog_init, NULL, &kea_wdog_data_##inst,                   \
			      &kea_wdog_cfg_##inst, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			      &kea_wdog_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KEA_WDOG_INIT)
