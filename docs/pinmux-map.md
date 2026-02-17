# TRK-KEA128 Pinmux and Validation Map

Last updated: 2026-02-17

## Encoding Model

KEA pinctrl uses encoded values in Devicetree `pinmux` arrays:

- bit31 = 0: write mask into `SIM->PINSEL`
- bit31 = 1: write mask into `SIM->PINSEL1`
- bits30..0: mask bits to OR into selected register

Current implementation ORs routing bits and does not clear previous routes.

## Active Board Routes (Current DTS)

From `boards/nxp/trk_kea128/trk_kea128.dts`:

| Peripheral | Pinctrl state | Encoded value | Register/bit | Route property | Validation |
|---|---|---:|---|---|---|
| UART2 | `uart2_default` | `0x80002000` | `PINSEL1[13]` | `UART2PS` | Validated by console output |
| I2C0 | `i2c0_default` | `0x00000020` | `PINSEL[5]` | `I2C0PS` | Validated by one-shot I2C probe |
| SPI0 | `spi0_default` | `0x00000040` | `PINSEL[6]` | `SPI0PS` | Validated by one-shot SPI transfer |
| CAN0 | `can0_default` | `0x80010000` | `PINSEL1[16]` | `MSCANPS` | Validated by CAN loopback RX/TX |

## Additional Route Bits Present in SoC Register Map

These routes are defined in `soc/nxp/kea/kea_regs.h` but not enabled in board DTS today:

| Register bit | Meaning | Typical peripheral |
|---|---|---|
| `PINSEL1[10]` | `I2C1PS` | I2C1 alternate route |
| `PINSEL1[11]` | `SPI1PS` | SPI1 alternate route |

## GPIO Usage Under Test App

From `app/src/main.c`:

| Signal use | Port/pin | Validation |
|---|---|---|
| LED outputs | GPIOA[16..19] | Validated (toggled periodically) |
| Button SW1 | GPIOA[24] | Validated (KBI/GPIO IRQ callback) |
| Button SW2 | GPIOA[25] | Validated (KBI/GPIO IRQ callback) |

## Validation Matrix

| Category | Status | Notes |
|---|---|---|
| Devicetree pinctrl states for enabled peripherals | Done | UART2/I2C0/SPI0/CAN0 states applied in driver init |
| Runtime validation of configured routes | Done | Covered by smoke app runtime signatures |
| Full package-level pinout annotation | In progress | Board-level physical connector/pad table not yet captured here |
| Unused peripheral route alternatives | Partial | I2C1/SPI1 route bits known; not validated on hardware |

## Next Pinmux Documentation Tasks

1. Add package pin and board connector mapping (silkscreen/header names) for active routes.
2. Add physical verification notes (scope/logic-analyzer captures where relevant).
3. Document conflict matrix if alternate routes are enabled simultaneously.
