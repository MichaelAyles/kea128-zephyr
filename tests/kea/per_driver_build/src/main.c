#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/toolchain.h>

static int __maybe_unused test_gpio(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(gpio_0));
	gpio_port_value_t value;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	return gpio_port_get_raw(dev, &value);
}

static int __maybe_unused test_uart(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(uart_0));

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	uart_poll_out(dev, 'U');
	return 0;
}

static int __maybe_unused test_i2c(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(i2c_0));
	uint32_t cfg;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	return i2c_get_config(dev, &cfg);
}

static int __maybe_unused test_spi(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(spi_0));
	struct spi_config cfg = {
		.frequency = 1000000u,
		.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8),
		.slave = 0u,
		.cs = {
			.cs_is_gpio = false,
		},
	};

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	return spi_release(dev, &cfg);
}

static int __maybe_unused test_adc(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(adc_0));
	struct adc_channel_cfg ch_cfg = {
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_VDD_1,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = 0u,
	};

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	return adc_channel_setup(dev, &ch_cfg);
}

static int __maybe_unused test_pwm(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(pwm_0));
	uint64_t cycles = 0u;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	return pwm_get_cycles_per_sec(dev, 0u, &cycles);
}

static int __maybe_unused test_counter(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(counter_0));

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	(void)counter_get_frequency(dev);
	return 0;
}

static int __maybe_unused test_can(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(can_0));
	can_mode_t cap = 0u;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	return can_get_capabilities(dev, &cap);
}

static int __maybe_unused test_wdt(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
	struct wdt_timeout_cfg cfg = {
		.window = {
			.min = 0u,
			.max = 1000u,
		},
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC,
	};

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	return wdt_install_timeout(dev, &cfg);
}

int main(void)
{
	int ret;

#if defined(CONFIG_KEA_TEST_GPIO)
	ret = test_gpio();
#elif defined(CONFIG_KEA_TEST_UART)
	ret = test_uart();
#elif defined(CONFIG_KEA_TEST_I2C)
	ret = test_i2c();
#elif defined(CONFIG_KEA_TEST_SPI)
	ret = test_spi();
#elif defined(CONFIG_KEA_TEST_ADC)
	ret = test_adc();
#elif defined(CONFIG_KEA_TEST_PWM)
	ret = test_pwm();
#elif defined(CONFIG_KEA_TEST_COUNTER)
	ret = test_counter();
#elif defined(CONFIG_KEA_TEST_CAN)
	ret = test_can();
#elif defined(CONFIG_KEA_TEST_WDT)
	ret = test_wdt();
#else
#error "One CONFIG_KEA_TEST_* selector must be enabled."
#endif

	printk("per_driver_build ret=%d\n", ret);
	return ret;
}
