# TRK-KEA128 Hardware Smoke Checklist

Last updated: 2026-02-17

## Purpose

This checklist validates that the out-of-tree KEA128 Zephyr port is alive on real hardware and that each enabled peripheral has at least one working path.

## Preconditions

- Board: TRK-KEA128 powered and connected.
- Probe: J-Link connected to target.
- UART console connected to host.
- Workspace bootstrapped (`west update`, toolchain installed).

## Build and Flash

```bash
./scripts/build.sh
./scripts/flash.sh
```

Optional explicit target/probe overrides:

```bash
JLINK_DEVICE="SKEAZ128xxx4" JLINK_DEV_ID="<probe-id>" ./scripts/flash.sh
```

## Runtime Checklist

On boot, confirm these log signatures on UART:

1. Boot banner:
   - `TRK-KEA128 peripheral bring-up`
2. SPI probe:
   - `spi_transceive ret=<n> rx=0x..`
3. I2C probe:
   - `i2c_write probe ret=<n>`
4. Periodic telemetry:
   - `pit_wrap=<n> adc=<n>`
5. CAN loopback receive:
   - `CAN RX id=0x123 dlc=1 data0=0x..`
6. Extended-ID CAN probe:
   - `can_send ext ret=<n>`
   - `CAN RX id=0x1abcde dlc=1 data0=0x5a`
7. GPIO interrupt activity (press SW1/SW2):
   - `gpio_irq count=<n> pins=0x........`

Pass criteria:

- Banner appears once after reset.
- PIT/ADC line increments every second.
- No persistent `... failed: <err>` spam.
- Button presses increment GPIO IRQ count.
- CAN RX lines appear while loopback mode is active.

## Subsystem-to-Check Mapping

- UART: banner + periodic logs
- GPIO: LED toggling + SW interrupts
- ADC: sampled value in periodic log
- PWM: configured at startup (no runtime print; verify by scope if needed)
- PIT: `pit_wrap` increments
- CAN: loopback RX print
- SPI: one-shot transceive status line
- I2C: one-shot write status line
- WDOG: no reset while feed loop runs

## Failure Triage

1. Build fails:
   - Re-run `./scripts/build.sh` and inspect the first compile/link error.
2. Flash fails:
   - Verify J-Link connectivity and `JLINK_DEVICE`.
3. No UART output:
   - Check `uart2` wiring/routing and console terminal settings.
4. Device-not-ready banner:
   - Check DTS `status = "okay"` and pinctrl/clock wiring.
5. Peripheral-specific failures:
   - Cross-check capability limits in `docs/bringup.md` section 4.
