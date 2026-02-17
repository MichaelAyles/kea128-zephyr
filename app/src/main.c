#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/printk.h>

#define CAN_NODE DT_ALIAS(can_0)
#define PWM_NODE DT_ALIAS(pwm_0)
#define WDT_NODE DT_ALIAS(watchdog0)

#if !DT_NODE_HAS_STATUS(CAN_NODE, okay)
#error "can-0 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(PWM_NODE, okay)
#error "pwm-0 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(WDT_NODE, okay)
#error "watchdog0 alias is not defined"
#endif

#define LED_COUNT 4u
#define LED_PWM_PERIOD_CYCLES 1024u
#define J1939_TABLE_SIZE 32u
#define J1939_PRINT_BATCH_MAX 8u
#define J1939_SAMPLE_PERIOD_MS 100u
#define J1939_STATS_PERIOD_MS 1000u
#define J1939_TARGET_BITRATE 250000u

struct j1939_entry {
	bool used;
	bool newly_discovered;
	uint32_t pgn;
	uint32_t count;
	uint8_t last_data0;
	uint32_t last_seen_ms;
};

struct j1939_discovery {
	uint32_t pgn;
};

static struct j1939_entry j1939_table[J1939_TABLE_SIZE];
static volatile uint32_t can_total_frames;
static volatile uint32_t can_j1939_frames;
static volatile uint32_t can_std_frames;

static bool j1939_parse_pgn(const struct can_frame *frame, uint32_t *pgn, uint8_t *src_addr)
{
	uint32_t can_id;
	uint8_t pdu_format;
	uint32_t raw_pgn;

	if ((frame->flags & CAN_FRAME_IDE) == 0u) {
		return false;
	}

	can_id = frame->id & CAN_EXT_ID_MASK;
	pdu_format = (uint8_t)((can_id >> 16) & 0xFFu);
	raw_pgn = (can_id >> 8) & 0x3FFFFu;

	/* J1939 PDU1 PGNs do not include destination address in low byte. */
	if (pdu_format < 240u) {
		raw_pgn &= 0x3FF00u;
	}

	*pgn = raw_pgn;
	*src_addr = (uint8_t)(can_id & 0xFFu);

	return true;
}

static void j1939_can_ext_rx(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	uint32_t pgn;
	uint8_t src_addr;
	int free_idx = -1;

	can_total_frames++;

	if (!j1939_parse_pgn(frame, &pgn, &src_addr)) {
		can_std_frames++;
		return;
	}

	ARG_UNUSED(src_addr);
	can_j1939_frames++;

	for (int i = 0; i < (int)J1939_TABLE_SIZE; i++) {
		if (!j1939_table[i].used) {
			if (free_idx < 0) {
				free_idx = i;
			}
			continue;
		}

		if (j1939_table[i].pgn == pgn) {
			j1939_table[i].count++;
			j1939_table[i].last_data0 = frame->dlc > 0u ? frame->data[0] : 0u;
			j1939_table[i].last_seen_ms = k_uptime_get_32();
			return;
		}
	}

	if (free_idx >= 0) {
		j1939_table[free_idx].used = true;
		j1939_table[free_idx].newly_discovered = true;
		j1939_table[free_idx].pgn = pgn;
		j1939_table[free_idx].count = 1u;
		j1939_table[free_idx].last_data0 = frame->dlc > 0u ? frame->data[0] : 0u;
		j1939_table[free_idx].last_seen_ms = k_uptime_get_32();
	}
}

static void j1939_can_std_rx(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(frame);
	ARG_UNUSED(user_data);

	can_total_frames++;
	can_std_frames++;
}

static void j1939_snapshot(struct j1939_entry *snapshot,
			   struct j1939_discovery *discoveries,
			   size_t *discovery_count,
			   uint32_t *total_frames,
			   uint32_t *j1939_frames,
			   uint32_t *std_frames)
{
	unsigned int key;

	*discovery_count = 0u;
	key = irq_lock();

	*total_frames = can_total_frames;
	*j1939_frames = can_j1939_frames;
	*std_frames = can_std_frames;

	for (int i = 0; i < (int)J1939_TABLE_SIZE; i++) {
		snapshot[i] = j1939_table[i];
		if (j1939_table[i].newly_discovered && (*discovery_count < J1939_PRINT_BATCH_MAX)) {
			discoveries[*discovery_count].pgn = j1939_table[i].pgn;
			(*discovery_count)++;
		}
		j1939_table[i].newly_discovered = false;
	}

	irq_unlock(key);
}

static void j1939_select_top4(const struct j1939_entry *entries, int top_idx[LED_COUNT], uint32_t *unique)
{
	*unique = 0u;
	for (size_t i = 0; i < LED_COUNT; i++) {
		top_idx[i] = -1;
	}

	for (int i = 0; i < (int)J1939_TABLE_SIZE; i++) {
		if (!entries[i].used) {
			continue;
		}

		(*unique)++;

		for (size_t pos = 0; pos < LED_COUNT; pos++) {
			if ((top_idx[pos] < 0) ||
			    (entries[i].count > entries[top_idx[pos]].count)) {
				for (size_t shift = LED_COUNT - 1; shift > pos; shift--) {
					top_idx[shift] = top_idx[shift - 1];
				}
				top_idx[pos] = i;
				break;
			}
		}
	}
}

static void j1939_update_led_pwm(const struct device *pwm, const struct j1939_entry *entries,
				 int top_idx[LED_COUNT], uint32_t led_duty[LED_COUNT])
{
	for (uint32_t ch = 0u; ch < LED_COUNT; ch++) {
		uint32_t duty = 0u;

		if (top_idx[ch] >= 0) {
			duty = ((uint32_t)entries[top_idx[ch]].last_data0 * LED_PWM_PERIOD_CYCLES) / 255u;
			if ((duty == 0u) && (entries[top_idx[ch]].count > 0u)) {
				duty = 1u;
			}
		}

		(void)pwm_set_cycles(pwm, ch, LED_PWM_PERIOD_CYCLES, duty, 0u);
		led_duty[ch] = duty;
	}
}

int main(void)
{
	const struct device *can = DEVICE_DT_GET(CAN_NODE);
	const struct device *pwm = DEVICE_DT_GET(PWM_NODE);
	const struct device *wdt = DEVICE_DT_GET(WDT_NODE);
	struct wdt_timeout_cfg wdt_cfg = {
		.window = {
			.min = 0u,
			.max = 2000u,
		},
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC,
	};
	struct can_filter ext_filter = {
		.id = 0u,
		.mask = 0u,
		.flags = CAN_FILTER_IDE,
	};
	struct can_filter std_filter = {
		.id = 0u,
		.mask = 0u,
		.flags = 0u,
	};
	int ext_filter_id;
	int std_filter_id;
	int wdt_channel_id;
	uint64_t last_stats_ms = 0u;
	uint32_t led_duty[LED_COUNT] = {0};
	int ret;

	printk("TRK-KEA128 J1939 PGN LED demo\n");

	if (!device_is_ready(can) || !device_is_ready(pwm) || !device_is_ready(wdt)) {
		printk("A required device is not ready\n");
		return -ENODEV;
	}

	ret = can_set_mode(can, CAN_MODE_NORMAL);
	if (ret != 0) {
		printk("can_set_mode(NORMAL) failed: %d\n", ret);
		return ret;
	}

	ret = can_set_bitrate(can, J1939_TARGET_BITRATE);
	if (ret != 0) {
		printk("can_set_bitrate(250k) failed: %d (using DTS bitrate)\n", ret);
	} else {
		printk("CAN bitrate set to %u\n", J1939_TARGET_BITRATE);
	}

	/* Initialize all LED channels to off. */
	for (uint32_t ch = 0u; ch < LED_COUNT; ch++) {
		ret = pwm_set_cycles(pwm, ch, LED_PWM_PERIOD_CYCLES, 0u, 0u);
		if (ret != 0) {
			printk("pwm_set_cycles(ch=%u) failed: %d\n", ch, ret);
			return ret;
		}
	}

	ext_filter_id = can_add_rx_filter(can, j1939_can_ext_rx, NULL, &ext_filter);
	if (ext_filter_id < 0) {
		printk("can_add_rx_filter(ext) failed: %d\n", ext_filter_id);
		return ext_filter_id;
	}

	std_filter_id = can_add_rx_filter(can, j1939_can_std_rx, NULL, &std_filter);
	if (std_filter_id < 0) {
		printk("can_add_rx_filter(std) failed: %d\n", std_filter_id);
		return std_filter_id;
	}

	ret = can_start(can);
	if (ret != 0) {
		printk("can_start failed: %d\n", ret);
		return ret;
	}

	wdt_channel_id = wdt_install_timeout(wdt, &wdt_cfg);
	if (wdt_channel_id >= 0) {
		ret = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
		if (ret != 0) {
			printk("wdt_setup failed: %d\n", ret);
			wdt_channel_id = -1;
		}
	} else {
		printk("wdt_install_timeout failed: %d\n", wdt_channel_id);
	}

	printk("Listening on CAN, decoding J1939 PGNs, mapping top 4 PGNs to LED PWM.\n");

	while (1) {
		struct j1939_entry snapshot[J1939_TABLE_SIZE];
		struct j1939_discovery discoveries[J1939_PRINT_BATCH_MAX];
		size_t discovery_count;
		uint32_t total_frames;
		uint32_t j1939_frames;
		uint32_t std_frames;
		int top_idx[LED_COUNT];
		uint32_t unique_pgns;
		uint64_t now_ms = k_uptime_get();

		j1939_snapshot(snapshot, discoveries, &discovery_count,
			      &total_frames, &j1939_frames, &std_frames);

		for (size_t i = 0; i < discovery_count; i++) {
			printk("J1939 PGN discovered: 0x%05x\n", discoveries[i].pgn);
		}

		j1939_select_top4(snapshot, top_idx, &unique_pgns);
		j1939_update_led_pwm(pwm, snapshot, top_idx, led_duty);

		if ((now_ms - last_stats_ms) >= J1939_STATS_PERIOD_MS) {
			printk("CAN total=%u j1939=%u std=%u unique_pgn=%u\n",
			       total_frames, j1939_frames, std_frames, unique_pgns);

			for (uint32_t ch = 0u; ch < LED_COUNT; ch++) {
				if (top_idx[ch] >= 0) {
					printk(" LED%u <- PGN 0x%05x duty=%u/%u count=%u data0=0x%02x\n",
					       ch,
					       snapshot[top_idx[ch]].pgn,
					       led_duty[ch],
					       LED_PWM_PERIOD_CYCLES,
					       snapshot[top_idx[ch]].count,
					       snapshot[top_idx[ch]].last_data0);
				} else {
					printk(" LED%u <- none duty=0/%u\n", ch, LED_PWM_PERIOD_CYCLES);
				}
			}

			last_stats_ms = now_ms;
		}

		if (wdt_channel_id >= 0) {
			(void)wdt_feed(wdt, wdt_channel_id);
		}

		k_msleep(J1939_SAMPLE_PERIOD_MS);
	}

	return 0;
}
