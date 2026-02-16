# KEA128 Zephyr Research

## Upstream support status

- Zephyr currently has NXP Kinetis support for families like `KE1x`, `KL2x`, `K2x`, `K6x`, `K8x`, `KWx`, but not KEA/KE06.
- There is no `SOC_*` or board target for `SKEAZ*` / `MKEA*` in current upstream trees.

References:
- https://github.com/zephyrproject-rtos/zephyr/tree/main/soc/nxp/kinetis
- https://github.com/zephyrproject-rtos/zephyr/tree/main/boards/nxp

## HAL/device header availability

- `hal_nxp` does not currently provide KEA/MKEA device directories.
- This blocks immediate reuse of MCUX device headers for KEA from upstream modules.

References:
- https://github.com/zephyrproject-rtos/hal_nxp/tree/main/mcux/mcux-sdk/devices
- https://github.com/zephyrproject-rtos/hal_nxp/tree/main/mcux/mcux-sdk-ng/devices

## Vendor package for KEA devices

- Arm/Keil DFP package `Kinetis_KEAxx_DFP` includes `SKEAZ1284` device headers, startup files, SVDs, and TRK-KEA128 board example files.

References:
- https://www.keil.arm.com/packs/kinetis_keaxx_dfp-keil/devices/
- https://www.keil.com/pack/Keil.Kinetis_KEAxx_DFP.1.3.1.pack

## Existing Zephyr peripheral driver candidates

Potentially reusable (after KEA SoC clock/pinctrl/device-integration work):

- UART: `nxp,kinetis-uart` (`drivers/serial/uart_mcux.c`)
- GPIO: `nxp,kinetis-gpio` (`drivers/gpio/gpio_mcux.c`)
- I2C: `nxp,kinetis-i2c` (`drivers/i2c/i2c_mcux.c`)
- ADC: `nxp,kinetis-adc16` (`drivers/adc/adc_mcux_adc16.c`)
- PWM/TPM: `nxp,kinetis-tpm` (`drivers/pwm/pwm_mcux_tpm.c`)
- Watchdog: `nxp,kinetis-wdog` (`drivers/watchdog/wdt_mcux_wdog.c`)

Bindings:
- https://github.com/zephyrproject-rtos/zephyr/tree/main/dts/bindings/serial/nxp,kinetis-uart.yaml
- https://github.com/zephyrproject-rtos/zephyr/tree/main/dts/bindings/gpio/nxp,kinetis-gpio.yaml
- https://github.com/zephyrproject-rtos/zephyr/tree/main/dts/bindings/i2c/nxp,kinetis-i2c.yaml
- https://github.com/zephyrproject-rtos/zephyr/tree/main/dts/bindings/adc/nxp,kinetis-adc16.yaml
- https://github.com/zephyrproject-rtos/zephyr/tree/main/dts/bindings/pwm/nxp,kinetis-tpm.yaml
- https://github.com/zephyrproject-rtos/zephyr/tree/main/dts/bindings/watchdog/nxp,kinetis-wdog.yaml

## Practical implication

- First milestone is best done with an out-of-tree SoC/board and a direct-register validation app.
- Then add KEA devicetree nodes and progressively enable existing Kinetis MCUX drivers where register compatibility is confirmed.
