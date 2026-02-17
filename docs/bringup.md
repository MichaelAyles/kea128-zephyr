# KEA128 Zephyr Bring-up: Implemented State and Roadmap

Last updated: 2026-02-17

Companion documents:

- Progress tracker: `docs/progress.md`
- Hardware smoke checklist: `docs/hardware-smoke.md`
- Pinmux map and validation matrix: `docs/pinmux-map.md`

## 1) Scope and Goal

This repository is an out-of-tree Zephyr port for:

- SoC: `SKEAZ1284` (NXP Kinetis KEA, Cortex-M0+)
- Board: `TRK-KEA128`
- Probe/debug: SEGGER J-Link

Primary goal:

- Boot Zephyr reliably.
- Enable and validate core peripherals with upstream-style APIs.
- Keep the code structured so it can be upstreamed to Zephyr later.

This document is the source of truth for:

- What has already been implemented.
- What is partially implemented.
- What still needs work before upstream-quality support.

## 2) What Has Been Done

### 2.1 Repository and Workspace Baseline

- West manifest created (`west.yml`) with Zephyr + CMSIS module dependencies.
- Project layout created for out-of-tree SoC/board/drivers/docs/scripts.
- Build scripts and J-Link flash/debug wrappers added.

### 2.2 SoC and Board Port

- Added SoC family/series/part definitions for KEA:
  - `soc/nxp/kea/Kconfig.soc`
  - `soc/nxp/kea/Kconfig`
  - `soc/nxp/kea/Kconfig.defconfig`
  - `soc/nxp/kea/soc.yml`
- Added early SoC init hook to disable watchdog during bring-up:
  - `soc/nxp/kea/soc.c`
- Added FEI bus-clock initialization sequence (AN4942-aligned) in early init:
  - `soc/nxp/kea/soc.c`
- Added register map for SIM/UART/ADC/PIT/FTM/MSCAN/I2C/SPI/KBI/GPIO/WDOG:
  - `soc/nxp/kea/kea_regs.h`
- Added board definition for TRK-KEA128:
  - `boards/nxp/trk_kea128/*`

### 2.3 Devicetree and Bindings

- Added KEA SoC DTSI with memory map and peripheral nodes:
  - `dts/arm/nxp/kea/nxp_skeaz1284.dtsi`
- Added board DTS with aliases, enabled peripherals, and pinctrl states:
  - `boards/nxp/trk_kea128/trk_kea128.dts`
- Added custom bindings for KEA peripherals:
  - UART, GPIO, I2C, SPI, ADC, PWM/FTM, PIT counter, MSCAN, WDOG
- Added KEA-specific clock control binding:
  - `dts/bindings/clock/nxp,kea-sim-clock.yaml`
- Added KEA pinctrl binding:
  - `dts/bindings/pinctrl/nxp,kea-pinctrl.yaml`

### 2.4 Clock and Pinmux Infrastructure

- Added SIM-backed clock-control driver:
  - `soc/nxp/kea/drivers/clock_control_kea_sim.c`
- Added KEA pinctrl backend:
  - `soc/nxp/kea/drivers/pinctrl_kea.c`
  - `soc/nxp/kea/pinctrl_soc.h`
- Migrated major peripherals to `clocks` + `pinctrl` model:
  - UART, I2C, SPI, CAN, PWM, ADC, PIT

### 2.5 Peripheral Drivers Added

All below are implemented in `soc/nxp/kea/drivers/`:

- `gpio_kea.c`
- `uart_kea.c`
- `i2c_kea.c`
- `spi_kea.c`
- `adc_kea.c`
- `pwm_kea_ftm.c`
- `counter_kea_pit.c`
- `can_kea_mscan.c`
- `wdt_kea.c`

### 2.6 Application-Level Smoke Test

`app/src/main.c` exercises these APIs in one loop:

- GPIO outputs and GPIO interrupts
- ADC conversion
- PWM setup
- PIT counter top callback
- CAN loopback send/receive
- SPI one-shot transfer
- I2C one-shot probe
- WDOG setup/feed
- UART console output

### 2.7 Build and Flash Validation

Confirmed on 2026-02-16:

- `./scripts/build.sh` completes successfully.
- `./scripts/flash.sh` completes successfully using J-Link runner.

Confirmed on 2026-02-17:

- Twister build-only driver coverage passes for `trk_kea128/skeaz1284`:
  - `./scripts/twister.sh`
  - Test suite: `tests/kea/per_driver_build` (11 scenarios)

## 3) Current Architecture

### 3.1 High-Level Layout

- `soc/nxp/kea/`: SoC Kconfig/CMake, register map, drivers, early init.
- `boards/nxp/trk_kea128/`: board metadata, DTS, defconfig, runner args.
- `dts/`: SoC DTSI + KEA bindings.
- `app/`: single smoke application.
- `scripts/`: build/flash/debug/helpers.
- `docs/`: bring-up and research notes.

### 3.2 Device Tree Model in Use

- `simclk` node (`nxp,kea-sim-clock`) provides `#clock-cells = <1>`.
- Peripheral nodes consume clock gates via `clocks = <&simclk MASK>`.
- `pinctrl` node (`nxp,kea-pinctrl`) encodes SIM `PINSEL/PINSEL1` route bits.
- Legacy properties (`nxp,clock-gate-mask`, `nxp,pinsel*`) remain in bindings as deprecated compatibility fields.

### 3.3 J-Link Device Strategy

Default runner target:

- `SKEAZ128xxx4`

Other valid IDs observed for this silicon family:

- `S9KEAZ128xxxx`

`(allow security)` device variants should only be used intentionally for security/fuse workflows.

Supported overrides:

- `JLINK_DEVICE=...` for device string
- `JLINK_DEV_ID=...` for selecting a specific connected probe

## 4) Driver Capability Matrix (What Works vs Gaps)

### 4.1 Clock Control (`clock_control_kea_sim.c`)

Implemented:

- Gate on/off via SIM `SCGC` bitmask (`clock_control_on/off`).
- `get_rate` supports runtime bus-rate derivation from `ICS` + `SIM->CLKDIV`
  when enabled in Devicetree.
- AN4942-style FEI setup in early boot for nominal 20 MHz bus operation.
- `get_status` checks current gate state.

Known gaps:

- No runtime frequency switching policy.
- No generalized system/core/bus/flash clock API split yet.

### 4.2 Pinctrl (`pinctrl_kea.c`)

Implemented:

- Applies encoded pin routing masks to `SIM->PINSEL` and `SIM->PINSEL1`.
- Supports Devicetree-driven state application.

Known gaps:

- Current model only ORs bits into route registers.
- No explicit clear/reset/power-state mux management.

### 4.3 GPIO + KBI IRQ (`gpio_kea.c`)

Implemented:

- Input/output mode.
- Port read/set/clear/toggle.
- Callback management.
- KBI-backed GPIO interrupts on banks with IRQ lines.
- Supports edge and level mode, high/low trigger selection.
- KBI gate enable via generic `clock_control` (`clocks` DT property).

Known gaps:

- No pull-up/down or single-ended/open-drain configuration.
- No wakeup flag support.
- Interrupt source mapping is tied to current KBI/IRQ assumptions and needs matrix validation for full coverage.

### 4.4 UART (`uart_kea.c`)

Implemented:

- Polling API (`poll_in/poll_out`).
- Error reporting.
- IRQ-driven API path (`fifo_*`, IRQ enable/disable, callback).
- Baud setup from DT properties during init.
- Clock + pinctrl apply at init.

Known gaps:

- No runtime line-control configuration API.
- No power-management integration.

### 4.5 I2C (`i2c_kea.c`)

Implemented:

- Controller mode.
- Target mode registration/unregistration with callback dispatch.
- 7-bit addressing.
- Standard (100 kHz) and Fast (400 kHz) support.
- Multi-message transfer path with restart/stop handling.
- Bus recover path (basic STOP recovery).
- IRQ path for target transactions.
- Clock + pinctrl apply at init.

Known gaps:

- No 10-bit addressing.
- No HS mode.
- No interrupt-driven/async transfer path.
- Target mode needs cross-board validation against an external controller.

### 4.6 SPI (`spi_kea.c`)

Implemented:

- Controller mode.
- 8-bit word size.
- CPOL/CPHA and MSB/LSB options.
- Synchronous transceive with multi-buffer cursor handling.
- Async callback API path (`transceive_async`, workqueue-backed).
- GPIO-based CS helper path (`spi_cfg->cs` GPIO) with optional setup delay.
- Clock + pinctrl apply at init.

Known gaps:

- No slave mode.
- No IRQ-driven or DMA-backed transfer engine.

### 4.7 ADC (`adc_kea.c`)

Implemented:

- Channel setup for channels 0..15.
- Single-ended reads.
- 8/10/12-bit resolution support.
- IRQ completion path with semaphore synchronization.
- Clock gating at init.

Known gaps:

- No differential mode.
- No oversampling.
- No calibration flow.
- Limited reference/gain/acquisition combinations.

### 4.8 PWM (FTM) (`pwm_kea_ftm.c`)

Implemented:

- `set_cycles` and `get_cycles_per_sec`.
- Per-channel duty control.
- Polarity invert support.
- Clock + pinctrl apply at init.

Known gaps:

- Shared-period restriction enforced (single `MOD` for all channels).
- No advanced FTM feature exposure (deadtime/combine/fault/sync/center-aligned control).
- No interrupt-based PWM features.

### 4.9 PIT Counter (`counter_kea_pit.c`)

Implemented:

- Counter start/stop/get-value.
- Top value configuration + callback on wrap.
- Pending interrupt/top value/frequency reporting.
- Clock gating at init.

Known gaps:

- No alarm channels (`set_alarm/cancel_alarm` return `-ENOTSUP`).
- Basic channel-0 oriented model; channel-1 not exposed as independent counter API.

### 4.10 CAN (MSCAN) (`can_kea_mscan.c`)

Implemented:

- CAN classic mode support.
- Standard-ID and extended-ID TX/RX.
- Loopback/listen-only/3-sample modes.
- Software RX filter table (`CONFIG_CAN_KEA_MSCAN_MAX_FILTER`).
- Basic timing setup using `can_calc_timing`.
- RX/TX IRQ hookup with TX mailbox completion callbacks from ISR.
- State-change callback notifications on controller state-change flags.
- Clock + pinctrl apply at init.

Known gaps:

- Manual recovery path is stubbed (`-ENOSYS` when enabled).
- Timeout-aware TX queueing/blocking semantics are still minimal.

### 4.11 Watchdog (`wdt_kea.c`)

Implemented:

- Install timeout.
- Setup/start.
- Feed.
- Disable.
- Optional disable-at-boot behavior.

Known gaps:

- Single timeout channel only.
- No callback mode (reset-only flow).
- No non-zero minimum window support.

## 5) Application Validation Behavior

`app/src/main.c` currently validates:

1. Device readiness for all enabled blocks.
2. LED GPIO outputs on GPIOA pins 16..19.
3. GPIO interrupts on SW1/SW2 pins (24/25).
4. ADC channel 0 setup + periodic read.
5. PWM setup on channel 0.
6. PIT top callback increments a wrap counter.
7. CAN loopback send/receive on standard ID `0x123`.
8. One-time CAN extended-ID probe frame (`0x1abcde`) and receive path.
9. One-time SPI transceive and I2C write probe.
10. WDOG setup + periodic feed.
11. Continuous console logging over UART2.

This is a smoke test, not a compliance test suite.

## 6) Build, Flash, Debug Commands

Workspace bootstrap:

```bash
west init -l .
west update
west zephyr-export
```

Build:

```bash
./scripts/build.sh
```

Flash:

```bash
./scripts/flash.sh
```

Debug:

```bash
./scripts/debug.sh
```

Select a specific J-Link probe:

```bash
JLINK_DEV_ID="<probe-id>" ./scripts/flash.sh
```

Override device target:

```bash
JLINK_DEVICE="SKEAZ128xxx4" ./scripts/flash.sh
JLINK_DEVICE="S9KEAZ128xxxx" ./scripts/flash.sh
```

## 7) What Needs To Be Done Next (Prioritized)

### P0: Stabilize and Prove Baseline

- Remove ambiguity around pin routing by documenting full board mux map and validated pinout.

### P1: Finish Critical Driver Gaps

- I2C:
  - Cross-board validation of target mode behavior.
  - Broader speed and transfer corner-case validation.
- SPI:
  - Optional IRQ-driven async engine.
- GPIO:
  - Validate/expand interrupt coverage beyond current mapping assumptions.

### P2: Clock/Power and Robustness

- Add ICS/clock-tree model for configurable CPU/bus frequencies.
- Add flash-controller + flash driver support to unlock internal flash-backed
  NVS/settings (EEPROM emulation).
- Add low-power state integration and resume-safe peripheral behavior.
- Add runtime/system power management compatibility checks.

### P3: Upstream Readiness

- Split work into upstreamable chunks:
  - DTS bindings + SoC skeleton.
  - Board definition.
  - Individual drivers (small PRs by subsystem).
- Align coding/binding style with Zephyr maintainer expectations.
- Minimize out-of-tree custom behavior where upstream equivalents exist.
- Ensure all new bindings and drivers have tests/docs needed for CI acceptance.

## 8) Upstreaming Strategy

Recommended path:

1. Keep this repository as the integration and hardware-validation branch.
2. Maintain a Zephyr fork for actual upstream PR branches.
3. Cherry-pick or replay clean commits from this repo into the Zephyr fork.
4. Open PRs from the Zephyr fork to upstream Zephyr.

Rationale:

- This repo is optimized for fast local bring-up.
- Upstream requires narrower, subsystem-specific changes and CI-friendly commit history.

## 9) Definition of "Done Enough" for Initial Upstream Attempt

Minimum bar:

- Board boots reliably.
- UART console stable.
- GPIO, I2C, SPI, ADC, PWM, PIT, CAN, and WDOG each have at least one validated functional path.
- Core known limitations are documented in bindings/docs.
- Build and flash instructions are reproducible.
- Patch set is split so maintainers can review by subsystem.
