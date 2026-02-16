# kea128-zephyr

Out-of-tree Zephyr bring-up for NXP Kinetis KEA128 (`SKEAZ1284`) on the TRK-KEA128 board using SEGGER J-Link.

## Current State

- Custom KEA SoC + TRK-KEA128 board support is implemented.
- Drivers implemented: GPIO, UART, I2C, SPI, ADC, PWM (FTM), PIT counter, CAN (MSCAN), WDOG.
- KEA SIM clock-control and pinctrl backends are integrated.
- Smoke app exercises all major enabled peripherals.
- Build and flash paths are validated.

Detailed status, implementation notes, and roadmap:

- `docs/bringup.md`

Research background and upstream context:

- `docs/research.md`

## Repository Layout

- `soc/nxp/kea/`: SoC support, register map, KEA drivers
- `boards/nxp/trk_kea128/`: board DTS/defconfig/runner files
- `dts/`: KEA SoC DTSI + Devicetree bindings
- `app/`: peripheral smoke test app
- `scripts/`: build/flash/debug helpers
- `docs/`: bring-up and planning documentation

## Prerequisites

- `west`, `cmake`, `ninja`
- Zephyr SDK/toolchain
- Python environment for Zephyr tooling
- SEGGER J-Link tools (`JLinkExe`) in `PATH`

## Workspace Bootstrap

```bash
west init -l .
west update
west zephyr-export
```

## Build

```bash
./scripts/build.sh
```

Equivalent manual build:

```bash
west build -p auto -b trk_kea128 app -- \
  -DBOARD_ROOT=$PWD -DSOC_ROOT=$PWD -DDTS_ROOT=$PWD
```

## Flash and Debug

```bash
./scripts/flash.sh
./scripts/debug.sh
```

Default J-Link device string:

- `SKEAZ128xxx4`

Override device string when needed:

```bash
JLINK_DEVICE="S9KEAZ128xxxx" ./scripts/flash.sh
JLINK_DEVICE="SKEAZ128xxx4 (allow security)" ./scripts/flash.sh
```

Select a specific connected probe:

```bash
JLINK_DEV_ID="<probe-id>" ./scripts/flash.sh
```

## Upstreaming Note

Development can happen in this repo, but upstream Zephyr PRs must be opened from a Zephyr fork branch. Use this repo as the hardware-validation integration branch, then replay clean commits into the Zephyr fork for PR submission.
