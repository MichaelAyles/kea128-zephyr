#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define GPIO_NODE DT_ALIAS(gpio_0)
#define I2C_NODE DT_ALIAS(i2c_0)
#define SPI_NODE DT_ALIAS(spi_0)
#define ADC_NODE DT_ALIAS(adc_0)
#define PWM_NODE DT_ALIAS(pwm_0)
#define COUNTER_NODE DT_ALIAS(counter_0)
#define CAN_NODE DT_ALIAS(can_0)
#define WDT_NODE DT_ALIAS(watchdog0)
#define SIMCLK_NODE DT_NODELABEL(simclk)

#if !DT_NODE_HAS_STATUS(GPIO_NODE, okay)
#error "gpio-0 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(I2C_NODE, okay)
#error "i2c-0 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(SPI_NODE, okay)
#error "spi-0 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(ADC_NODE, okay)
#error "adc-0 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(PWM_NODE, okay)
#error "pwm-0 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(COUNTER_NODE, okay)
#error "counter-0 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(CAN_NODE, okay)
#error "can-0 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(WDT_NODE, okay)
#error "watchdog0 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(SIMCLK_NODE, okay)
#error "simclk node is not defined"
#endif

#define LED_MASK (BIT(16) | BIT(17) | BIT(18) | BIT(19))
#define BUTTON_SW1_PIN 24u
#define BUTTON_SW2_PIN 25u
#define BUTTON_MASK (BIT(BUTTON_SW1_PIN) | BIT(BUTTON_SW2_PIN))

static volatile uint32_t pit_wrap_count;
static volatile gpio_port_pins_t button_events;
static volatile uint32_t button_irq_count;
static struct gpio_callback button_callback;

static void pit_top_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	pit_wrap_count++;
}

static void can_tx_handler(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (error != 0) {
		printk("CAN TX error: %d\n", error);
	}
}

static void can_rx_handler(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	printk("CAN RX id=0x%x dlc=%u data0=0x%02x\n", frame->id, frame->dlc,
	       frame->dlc > 0u ? frame->data[0] : 0u);
}

static void button_gpio_handler(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	button_events |= pins;
	button_irq_count++;
}

int main(void)
{
	const struct device *gpio = DEVICE_DT_GET(GPIO_NODE);
	const struct device *i2c = DEVICE_DT_GET(I2C_NODE);
	const struct device *spi = DEVICE_DT_GET(SPI_NODE);
	const struct device *adc = DEVICE_DT_GET(ADC_NODE);
	const struct device *pwm = DEVICE_DT_GET(PWM_NODE);
	const struct device *counter = DEVICE_DT_GET(COUNTER_NODE);
	const struct device *can = DEVICE_DT_GET(CAN_NODE);
	const struct device *wdt = DEVICE_DT_GET(WDT_NODE);
	const struct device *simclk = DEVICE_DT_GET(SIMCLK_NODE);
	struct adc_channel_cfg adc_ch_cfg = {
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_VDD_1,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = 0,
	};
	uint16_t adc_sample = 0;
	struct adc_sequence adc_seq = {
		.channels = BIT(0),
		.buffer = &adc_sample,
		.buffer_size = sizeof(adc_sample),
		.resolution = 12,
	};
	struct counter_top_cfg pit_top_cfg = {
		.ticks = 2000000,
		.callback = pit_top_handler,
		.user_data = NULL,
		.flags = 0,
	};
	struct can_filter can_filter = {
		.id = 0x123,
		.mask = CAN_STD_ID_MASK,
		.flags = 0,
	};
	struct can_filter can_ext_filter = {
		.id = 0x1abcdeu,
		.mask = CAN_EXT_ID_MASK,
		.flags = CAN_FILTER_IDE,
	};
	struct wdt_timeout_cfg wdt_cfg = {
		.window = {
			.min = 0u,
			.max = 2000u,
		},
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC,
	};
	struct spi_config spi_cfg = {
		.frequency = 1000000u,
		.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
		.slave = 0u,
		.cs = {
			.cs_is_gpio = false,
		},
	};
	uint8_t spi_tx_byte = 0x9Fu;
	uint8_t spi_rx_byte = 0u;
	struct spi_buf spi_tx_buf = {
		.buf = &spi_tx_byte,
		.len = sizeof(spi_tx_byte),
	};
	struct spi_buf spi_rx_buf = {
		.buf = &spi_rx_byte,
		.len = sizeof(spi_rx_byte),
	};
	struct spi_buf_set spi_tx = {
		.buffers = &spi_tx_buf,
		.count = 1u,
	};
	struct spi_buf_set spi_rx = {
		.buffers = &spi_rx_buf,
		.count = 1u,
	};
	uint8_t i2c_probe = 0x00u;
	int can_filter_id;
	int can_ext_filter_id;
	int wdt_channel_id;
	int ret;
	bool did_bus_probe = false;
	bool did_can_ext_probe = false;
	uint32_t last_button_irq_count = 0u;
	uint32_t bus_clock_hz = 0u;

	printk("TRK-KEA128 peripheral bring-up\n");

	if (!device_is_ready(gpio) || !device_is_ready(i2c) || !device_is_ready(spi) ||
	    !device_is_ready(adc) || !device_is_ready(pwm) || !device_is_ready(counter) ||
	    !device_is_ready(can) || !device_is_ready(wdt) || !device_is_ready(simclk)) {
		printk("A required device is not ready\n");
		return -ENODEV;
	}

	ret = clock_control_get_rate(simclk, (clock_control_subsys_t)0, &bus_clock_hz);
	if (ret == 0) {
		printk("simclk bus_hz=%u\n", bus_clock_hz);
	} else {
		printk("simclk get_rate failed: %d\n", ret);
	}

	for (uint32_t pin = 16u; pin <= 19u; pin++) {
		ret = gpio_pin_configure(gpio, pin, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			printk("gpio_pin_configure(%u) failed: %d\n", pin, ret);
			return ret;
		}
	}

	ret = gpio_pin_configure(gpio, BUTTON_SW1_PIN, GPIO_INPUT);
	if (ret != 0) {
		printk("gpio_pin_configure(%u) failed: %d\n", BUTTON_SW1_PIN, ret);
		return ret;
	}

	ret = gpio_pin_configure(gpio, BUTTON_SW2_PIN, GPIO_INPUT);
	if (ret != 0) {
		printk("gpio_pin_configure(%u) failed: %d\n", BUTTON_SW2_PIN, ret);
		return ret;
	}

	gpio_init_callback(&button_callback, button_gpio_handler, BUTTON_MASK);
	ret = gpio_add_callback(gpio, &button_callback);
	if (ret != 0) {
		printk("gpio_add_callback failed: %d\n", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure(gpio, BUTTON_SW1_PIN, GPIO_INT_EDGE_FALLING);
	if (ret != 0) {
		printk("gpio_pin_interrupt_configure(%u) failed: %d\n", BUTTON_SW1_PIN, ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure(gpio, BUTTON_SW2_PIN, GPIO_INT_EDGE_FALLING);
	if (ret != 0) {
		printk("gpio_pin_interrupt_configure(%u) failed: %d\n", BUTTON_SW2_PIN, ret);
		return ret;
	}

	ret = adc_channel_setup(adc, &adc_ch_cfg);
	if (ret != 0) {
		printk("adc_channel_setup failed: %d\n", ret);
		return ret;
	}

	ret = pwm_set_cycles(pwm, 0, 1024, 256, 0);
	if (ret != 0) {
		printk("pwm_set_cycles failed: %d\n", ret);
		return ret;
	}

	ret = counter_set_top_value(counter, &pit_top_cfg);
	if (ret != 0) {
		printk("counter_set_top_value failed: %d\n", ret);
		return ret;
	}

	ret = counter_start(counter);
	if (ret != 0) {
		printk("counter_start failed: %d\n", ret);
		return ret;
	}

	ret = can_set_mode(can, CAN_MODE_LOOPBACK);
	if (ret != 0) {
		printk("can_set_mode failed: %d\n", ret);
		return ret;
	}

	can_filter_id = can_add_rx_filter(can, can_rx_handler, NULL, &can_filter);
	if (can_filter_id < 0) {
		printk("can_add_rx_filter failed: %d\n", can_filter_id);
		return can_filter_id;
	}

	can_ext_filter_id = can_add_rx_filter(can, can_rx_handler, NULL, &can_ext_filter);
	if (can_ext_filter_id < 0) {
		printk("can_add_rx_filter(ext) failed: %d\n", can_ext_filter_id);
		return can_ext_filter_id;
	}

	ret = can_start(can);
	if (ret != 0) {
		printk("can_start failed: %d\n", ret);
		return ret;
	}

	wdt_channel_id = wdt_install_timeout(wdt, &wdt_cfg);
	if (wdt_channel_id < 0) {
		printk("wdt_install_timeout failed: %d\n", wdt_channel_id);
		return wdt_channel_id;
	}

	ret = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (ret != 0) {
		printk("wdt_setup failed: %d\n", ret);
		return ret;
	}

	while (1) {
		struct can_frame tx_frame = {
			.id = 0x123,
			.dlc = 1,
			.flags = 0,
			.data = { (uint8_t)pit_wrap_count },
		};

		ret = adc_read(adc, &adc_seq);
		if (ret != 0) {
			printk("adc_read failed: %d\n", ret);
		}

		ret = gpio_port_toggle_bits(gpio, LED_MASK);
		if (ret != 0) {
			printk("gpio_port_toggle_bits failed: %d\n", ret);
		}

			if (!did_bus_probe) {
				ret = spi_transceive(spi, &spi_cfg, &spi_tx, &spi_rx);
				printk("spi_transceive ret=%d rx=0x%02x\n", ret, spi_rx_byte);

				ret = i2c_write(i2c, &i2c_probe, sizeof(i2c_probe), 0x50);
			printk("i2c_write probe ret=%d\n", ret);

				did_bus_probe = true;
			}

			if (!did_can_ext_probe) {
				struct can_frame ext_tx_frame = {
					.id = 0x1abcdeu,
					.dlc = 1,
					.flags = CAN_FRAME_IDE,
					.data = { 0x5au },
				};

				ret = can_send(can, &ext_tx_frame, K_MSEC(10), can_tx_handler, NULL);
				printk("can_send ext ret=%d\n", ret);
				did_can_ext_probe = true;
			}

			ret = can_send(can, &tx_frame, K_MSEC(10), can_tx_handler, NULL);
			if (ret != 0) {
				printk("can_send failed: %d\n", ret);
			}

		ret = wdt_feed(wdt, wdt_channel_id);
		if (ret != 0) {
			printk("wdt_feed failed: %d\n", ret);
		}

		if (button_irq_count != last_button_irq_count) {
			printk("gpio_irq count=%u pins=0x%08x\n", button_irq_count, button_events);
			last_button_irq_count = button_irq_count;
		}

		printk("pit_wrap=%u adc=%u\n", pit_wrap_count, adc_sample);
		k_msleep(1000);
	}

	return 0;
}
