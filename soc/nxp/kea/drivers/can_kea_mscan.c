/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kea_mscan

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "kea_regs.h"

struct kea_can_filter {
	can_rx_callback_t callback;
	void *user_data;
	struct can_filter filter;
};

struct kea_can_config {
	struct can_driver_config common;
	kea_mscan_t *base;
	uint32_t clock_frequency;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config_func)(const struct device *dev);
};

struct kea_can_data {
	struct can_driver_data common;
	struct k_mutex lock;
	struct kea_can_filter filters[CONFIG_CAN_KEA_MSCAN_MAX_FILTER];
};

static int kea_can_wait_set(volatile uint8_t *reg, uint8_t mask)
{
	for (uint32_t i = 0; i < 1000000u; i++) {
		if ((*reg & mask) == mask) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int kea_can_wait_clear(volatile uint8_t *reg, uint8_t mask)
{
	for (uint32_t i = 0; i < 1000000u; i++) {
		if ((*reg & mask) == 0u) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int kea_can_enter_init_mode(const struct kea_can_config *cfg)
{
	cfg->base->CANCTL0 |= KEA_MSCAN_CANCTL0_INITRQ_MASK;

	return kea_can_wait_set(&cfg->base->CANCTL1, KEA_MSCAN_CANCTL1_INITAK_MASK);
}

static int kea_can_leave_init_mode(const struct kea_can_config *cfg)
{
	cfg->base->CANCTL0 &= (uint8_t)~KEA_MSCAN_CANCTL0_INITRQ_MASK;

	return kea_can_wait_clear(&cfg->base->CANCTL1, KEA_MSCAN_CANCTL1_INITAK_MASK);
}

static void kea_can_dispatch_rx(const struct device *dev, const struct can_frame *frame)
{
	struct kea_can_data *data = dev->data;
	struct can_frame frame_tmp = *frame;

	k_mutex_lock(&data->lock, K_FOREVER);

	for (int i = 0; i < ARRAY_SIZE(data->filters); i++) {
		if ((data->filters[i].callback == NULL) ||
		    !can_frame_matches_filter(frame, &data->filters[i].filter)) {
			continue;
		}

		data->filters[i].callback(dev, &frame_tmp, data->filters[i].user_data);
	}

	k_mutex_unlock(&data->lock);
}

static int kea_can_program_timing(const struct device *dev, const struct can_timing *timing)
{
	const struct kea_can_config *cfg = dev->config;
	struct kea_can_data *data = dev->data;
	uint16_t tseg1;
	uint8_t btr0;
	uint8_t btr1;

	if ((timing->prescaler < 1u) || (timing->prescaler > 64u)) {
		return -EINVAL;
	}

	if ((timing->sjw < 1u) || (timing->sjw > 4u)) {
		return -EINVAL;
	}

	tseg1 = timing->prop_seg + timing->phase_seg1;
	if ((tseg1 < 1u) || (tseg1 > 16u)) {
		return -EINVAL;
	}

	if ((timing->phase_seg2 < 1u) || (timing->phase_seg2 > 8u)) {
		return -EINVAL;
	}

	btr0 = (uint8_t)(((timing->prescaler - 1u) << KEA_MSCAN_CANBTR0_BRP_SHIFT) &
			 KEA_MSCAN_CANBTR0_BRP_MASK);
	btr0 |= (uint8_t)(((timing->sjw - 1u) << KEA_MSCAN_CANBTR0_SJW_SHIFT) &
			  KEA_MSCAN_CANBTR0_SJW_MASK);

	btr1 = (uint8_t)(((tseg1 - 1u) << KEA_MSCAN_CANBTR1_TSEG1_SHIFT) &
			 KEA_MSCAN_CANBTR1_TSEG1_MASK);
	btr1 |= (uint8_t)(((timing->phase_seg2 - 1u) << KEA_MSCAN_CANBTR1_TSEG2_SHIFT) &
			  KEA_MSCAN_CANBTR1_TSEG2_MASK);

	if ((data->common.mode & CAN_MODE_3_SAMPLES) != 0u) {
		btr1 |= KEA_MSCAN_CANBTR1_SAMP_MASK;
	}

	cfg->base->CANBTR0 = btr0;
	cfg->base->CANBTR1 = btr1;

	return 0;
}

static void kea_can_rx_isr(const struct device *dev)
{
	const struct kea_can_config *cfg = dev->config;
	struct can_frame frame = { 0 };
	uint8_t rsidr1;
	uint8_t flags;

	flags = cfg->base->CANRFLG;

	if ((flags & KEA_MSCAN_CANRFLG_RXF_MASK) == 0u) {
		if ((flags & (KEA_MSCAN_CANRFLG_OVRIF_MASK |
			      KEA_MSCAN_CANRFLG_CSCIF_MASK |
			      KEA_MSCAN_CANRFLG_WUPIF_MASK)) != 0u) {
			cfg->base->CANRFLG = flags & (KEA_MSCAN_CANRFLG_OVRIF_MASK |
						      KEA_MSCAN_CANRFLG_CSCIF_MASK |
						      KEA_MSCAN_CANRFLG_WUPIF_MASK);
		}
		return;
	}

	rsidr1 = cfg->base->RSIDR1;
	if ((rsidr1 & KEA_MSCAN_RSIDR1_RSIDE_MASK) != 0u) {
		/* Extended ID frames are not supported yet. */
		cfg->base->CANRFLG = KEA_MSCAN_CANRFLG_RXF_MASK;
		return;
	}

	frame.id = ((uint32_t)cfg->base->RSIDR0 << 3) |
		   ((uint32_t)(rsidr1 & KEA_MSCAN_RSIDR1_RSID2_0_MASK) >> 5);
	frame.flags = 0u;
	if ((rsidr1 & KEA_MSCAN_RSIDR1_RSRTR_MASK) != 0u) {
		frame.flags |= CAN_FRAME_RTR;
	}

	frame.dlc = cfg->base->RDLR & KEA_MSCAN_TDLR_TDLC_MASK;
	if (frame.dlc > CAN_MAX_DLC) {
		frame.dlc = CAN_MAX_DLC;
	}

	for (uint8_t i = 0; i < frame.dlc; i++) {
		frame.data[i] = cfg->base->REDSR[i];
	}

	cfg->base->CANRFLG = KEA_MSCAN_CANRFLG_RXF_MASK;

	kea_can_dispatch_rx(dev, &frame);
}

static void kea_can_tx_isr(const struct device *dev)
{
	const struct kea_can_config *cfg = dev->config;
	uint8_t flags = cfg->base->CANRFLG;

	if ((flags & (KEA_MSCAN_CANRFLG_OVRIF_MASK |
		      KEA_MSCAN_CANRFLG_CSCIF_MASK |
		      KEA_MSCAN_CANRFLG_WUPIF_MASK)) != 0u) {
		cfg->base->CANRFLG = flags & (KEA_MSCAN_CANRFLG_OVRIF_MASK |
					      KEA_MSCAN_CANRFLG_CSCIF_MASK |
					      KEA_MSCAN_CANRFLG_WUPIF_MASK);
	}
}

static int kea_can_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_3_SAMPLES;

	return 0;
}

static int kea_can_start(const struct device *dev)
{
	const struct kea_can_config *cfg = dev->config;
	struct kea_can_data *data = dev->data;
	uint8_t ctl1;
	int ret;

	if (data->common.started) {
		return -EALREADY;
	}

	ret = kea_can_enter_init_mode(cfg);
	if (ret != 0) {
		return ret;
	}

	ctl1 = cfg->base->CANCTL1;
	ctl1 |= KEA_MSCAN_CANCTL1_CANE_MASK | KEA_MSCAN_CANCTL1_CLKSRC_MASK;

	if ((data->common.mode & CAN_MODE_LOOPBACK) != 0u) {
		ctl1 |= KEA_MSCAN_CANCTL1_LOOPB_MASK;
	} else {
		ctl1 &= (uint8_t)~KEA_MSCAN_CANCTL1_LOOPB_MASK;
	}

	if ((data->common.mode & CAN_MODE_LISTENONLY) != 0u) {
		ctl1 |= KEA_MSCAN_CANCTL1_LISTEN_MASK;
	} else {
		ctl1 &= (uint8_t)~KEA_MSCAN_CANCTL1_LISTEN_MASK;
	}

	cfg->base->CANCTL1 = ctl1;
	cfg->base->CANRIER = KEA_MSCAN_CANRIER_RXFIE_MASK | KEA_MSCAN_CANRIER_OVRIE_MASK;
	cfg->base->CANTIER = 0u;

	ret = kea_can_leave_init_mode(cfg);
	if (ret != 0) {
		return ret;
	}

	data->common.started = true;

	return 0;
}

static int kea_can_stop(const struct device *dev)
{
	const struct kea_can_config *cfg = dev->config;
	struct kea_can_data *data = dev->data;
	int ret;

	if (!data->common.started) {
		return -EALREADY;
	}

	ret = kea_can_enter_init_mode(cfg);
	if (ret != 0) {
		return ret;
	}

	cfg->base->CANRIER = 0u;
	cfg->base->CANTIER = 0u;

	data->common.started = false;

	return 0;
}

static int kea_can_set_mode(const struct device *dev, can_mode_t mode)
{
	struct kea_can_data *data = dev->data;

	if (data->common.started) {
		return -EBUSY;
	}

	if ((mode & ~(CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_3_SAMPLES)) != 0u) {
		return -ENOTSUP;
	}

	data->common.mode = mode;

	return 0;
}

static int kea_can_set_timing(const struct device *dev, const struct can_timing *timing)
{
	const struct kea_can_config *cfg = dev->config;
	struct kea_can_data *data = dev->data;
	int ret;

	if (data->common.started) {
		return -EBUSY;
	}

	ret = kea_can_enter_init_mode(cfg);
	if (ret != 0) {
		return ret;
	}

	return kea_can_program_timing(dev, timing);
}

static int kea_can_send(const struct device *dev, const struct can_frame *frame,
			k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	const struct kea_can_config *cfg = dev->config;
	struct kea_can_data *data = dev->data;
	uint8_t txe_mask;
	uint8_t tx_buf_mask;
	int tx_buf_idx;

	ARG_UNUSED(timeout);

	if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0u) {
		return -ENOTSUP;
	}

	if ((frame->flags & CAN_FRAME_IDE) != 0u) {
		return -ENOTSUP;
	}

	if (frame->id > CAN_STD_ID_MASK) {
		return -EINVAL;
	}

	if (frame->dlc > CAN_MAX_DLC) {
		return -EINVAL;
	}

	if (!data->common.started) {
		return -ENETDOWN;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	txe_mask = cfg->base->CANTFLG & KEA_MSCAN_CANTFLG_TXE_MASK;
	if (txe_mask == 0u) {
		k_mutex_unlock(&data->lock);
		return -EAGAIN;
	}

	tx_buf_idx = __builtin_ctz(txe_mask);
	tx_buf_mask = BIT(tx_buf_idx);

	cfg->base->CANTBSEL = tx_buf_mask & KEA_MSCAN_CANTBSEL_TX_MASK;

	cfg->base->TSIDR0 = (uint8_t)(frame->id >> 3);
	cfg->base->TSIDR1 = (uint8_t)((frame->id << 5) & KEA_MSCAN_TSIDR1_TSID2_0_MASK);
	if ((frame->flags & CAN_FRAME_RTR) != 0u) {
		cfg->base->TSIDR1 |= KEA_MSCAN_TSIDR1_TSRTR_MASK;
	}

	cfg->base->TDLR = frame->dlc & KEA_MSCAN_TDLR_TDLC_MASK;
	for (uint8_t i = 0; i < frame->dlc; i++) {
		cfg->base->TEDSR[i] = frame->data[i];
	}

	cfg->base->TBPR = 0u;
	cfg->base->CANTFLG = tx_buf_mask;

	k_mutex_unlock(&data->lock);

	callback(dev, 0, user_data);

	return 0;
}

static int kea_can_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
				 void *user_data, const struct can_filter *filter)
{
	struct kea_can_data *data = dev->data;
	int filter_id = -ENOSPC;

	if ((filter->flags & ~(CAN_FILTER_IDE)) != 0u) {
		return -ENOTSUP;
	}

	if ((filter->flags & CAN_FILTER_IDE) != 0u) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	for (int i = 0; i < ARRAY_SIZE(data->filters); i++) {
		if (data->filters[i].callback == NULL) {
			data->filters[i].callback = callback;
			data->filters[i].user_data = user_data;
			data->filters[i].filter = *filter;
			filter_id = i;
			break;
		}
	}

	k_mutex_unlock(&data->lock);

	return filter_id;
}

static void kea_can_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct kea_can_data *data = dev->data;

	if ((filter_id < 0) || (filter_id >= ARRAY_SIZE(data->filters))) {
		return;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->filters[filter_id].callback = NULL;
	k_mutex_unlock(&data->lock);
}

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
static int kea_can_recover(const struct device *dev, k_timeout_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timeout);

	return -ENOSYS;
}
#endif

static int kea_can_get_state(const struct device *dev, enum can_state *state,
			     struct can_bus_err_cnt *err_cnt)
{
	const struct kea_can_config *cfg = dev->config;
	struct kea_can_data *data = dev->data;

	if (state != NULL) {
		*state = data->common.started ? CAN_STATE_ERROR_ACTIVE : CAN_STATE_STOPPED;
	}

	if (err_cnt != NULL) {
		err_cnt->tx_err_cnt = cfg->base->CANTXERR;
		err_cnt->rx_err_cnt = cfg->base->CANRXERR;
	}

	return 0;
}

static void kea_can_set_state_change_callback(const struct device *dev,
				      can_state_change_callback_t callback,
				      void *user_data)
{
	struct kea_can_data *data = dev->data;

	data->common.state_change_cb = callback;
	data->common.state_change_cb_user_data = user_data;
}

static int kea_can_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct kea_can_config *cfg = dev->config;

	*rate = cfg->clock_frequency;

	return 0;
}

static int kea_can_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(dev);

	if (ide) {
		return 0;
	}

	return CONFIG_CAN_KEA_MSCAN_MAX_FILTER;
}

static DEVICE_API(can, kea_can_driver_api) = {
	.get_capabilities = kea_can_get_capabilities,
	.start = kea_can_start,
	.stop = kea_can_stop,
	.set_mode = kea_can_set_mode,
	.set_timing = kea_can_set_timing,
	.send = kea_can_send,
	.add_rx_filter = kea_can_add_rx_filter,
	.remove_rx_filter = kea_can_remove_rx_filter,
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
	.recover = kea_can_recover,
#endif
	.get_state = kea_can_get_state,
	.set_state_change_callback = kea_can_set_state_change_callback,
	.get_core_clock = kea_can_get_core_clock,
	.get_max_filters = kea_can_get_max_filters,
	.timing_min = {
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 1,
		.prescaler = 1,
	},
	.timing_max = {
		.sjw = 4,
		.prop_seg = 8,
		.phase_seg1 = 8,
		.phase_seg2 = 8,
		.prescaler = 64,
	},
};

static int kea_can_init(const struct device *dev)
{
	const struct kea_can_config *cfg = dev->config;
	struct kea_can_data *data = dev->data;
	struct can_timing timing;
	int ret;

	k_mutex_init(&data->lock);

	for (int i = 0; i < ARRAY_SIZE(data->filters); i++) {
		data->filters[i].callback = NULL;
	}

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

	cfg->base->CANCTL1 = KEA_MSCAN_CANCTL1_CANE_MASK | KEA_MSCAN_CANCTL1_CLKSRC_MASK;

	ret = kea_can_enter_init_mode(cfg);
	if (ret != 0) {
		return ret;
	}

	cfg->base->CANIDAC &= (uint8_t)~KEA_MSCAN_CANIDAC_IDAM_MASK;
	for (int i = 0; i < 4; i++) {
		cfg->base->CANIDAR_BANK_1[i] = 0u;
		cfg->base->CANIDAR_BANK_2[i] = 0u;
		cfg->base->CANIDMR_BANK_1[i] = 0xFFu;
		cfg->base->CANIDMR_BANK_2[i] = 0xFFu;
	}

	cfg->base->CANRIER = 0u;
	cfg->base->CANTIER = 0u;
	cfg->base->CANRFLG = KEA_MSCAN_CANRFLG_RXF_MASK |
			    KEA_MSCAN_CANRFLG_OVRIF_MASK |
			    KEA_MSCAN_CANRFLG_CSCIF_MASK |
			    KEA_MSCAN_CANRFLG_WUPIF_MASK;

	data->common.mode = CAN_MODE_NORMAL;
	data->common.started = false;

	ret = can_calc_timing(dev, &timing, cfg->common.bitrate, cfg->common.sample_point);
	if (ret < 0) {
		return ret;
	}

	ret = kea_can_program_timing(dev, &timing);
	if (ret != 0) {
		return ret;
	}

	if (cfg->irq_config_func != NULL) {
		cfg->irq_config_func(dev);
	}

	return 0;
}

#define KEA_CAN_IRQ_CONFIG(inst)                                                                 \
	static void kea_can_irq_config_##inst(const struct device *dev)                            \
	{                                                                                            \
		ARG_UNUSED(dev);                                                                      \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rx, irq),                                       \
			    DT_INST_IRQ_BY_NAME(inst, rx, priority), kea_can_rx_isr,                   \
			    DEVICE_DT_INST_GET(inst), 0);                                              \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rx, irq));                                     \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, tx, irq),                                       \
			    DT_INST_IRQ_BY_NAME(inst, tx, priority), kea_can_tx_isr,                   \
			    DEVICE_DT_INST_GET(inst), 0);                                              \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, tx, irq));                                     \
	}

#define KEA_CAN_INIT(inst)                                                                       \
	KEA_CAN_IRQ_CONFIG(inst)                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                             \
                                                                                                   \
	static const struct kea_can_config kea_can_cfg_##inst = {                                  \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 10000, 1000000),                     \
		.base = (kea_mscan_t *)DT_INST_REG_ADDR(inst),                                       \
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),                              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                               \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, bits),             \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                       \
		.irq_config_func = kea_can_irq_config_##inst,                                        \
	};                                                                                         \
                                                                                                   \
	static struct kea_can_data kea_can_data_##inst;                                            \
                                                                                                   \
	CAN_DEVICE_DT_INST_DEFINE(inst, kea_can_init, NULL, &kea_can_data_##inst,                 \
				  &kea_can_cfg_##inst, POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,           \
				  &kea_can_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KEA_CAN_INIT)
