# Repository Agent Notes

## Mission

This repository exists to bring Zephyr RTOS up on NXP KEA128 (`SKEAZ1284`) and keep the port in an upstream-friendly shape.

Primary target board:

- `TRK-KEA128`

Primary debug/flash probe:

- SEGGER J-Link

## Source of Truth

- Detailed implementation status and roadmap: `docs/bringup.md`
- Upstream research context: `docs/research.md`

When behavior or status changes, update `docs/bringup.md` in the same change set.

## Working Principles

- Keep changes incremental and testable.
- Prefer Zephyr subsystem APIs over board-local hacks.
- Keep Devicetree and bindings aligned with actual driver behavior.
- Preserve a clean path for eventual upstream PRs.
- Record assumptions and hardware caveats explicitly.

## Expected Project Structure

- `soc/nxp/kea/`: SoC support and KEA peripheral drivers
- `boards/nxp/trk_kea128/`: board definitions
- `dts/`: SoC DTSI and custom bindings
- `app/`: smoke/bring-up firmware
- `scripts/`: reproducible build/flash/debug tooling
- `docs/`: implementation and planning docs

## Required Verification Before Commit

At minimum:

1. Build succeeds:
   - `./scripts/build.sh`
2. If hardware is connected, flash succeeds:
   - `./scripts/flash.sh`
3. If docs/status changed, ensure `docs/bringup.md` and `README.md` remain consistent.

If a change touches a specific driver, include at least one concrete validation note for that subsystem in docs or commit message.

## J-Link Conventions

Default target device string:

- `SKEAZ128xxx4`

Supported overrides:

- `JLINK_DEVICE=...`
- `JLINK_DEV_ID=...`

Use `(... allow security)` variants only when intentionally working with security/fuse behavior.

## Upstreaming Workflow

Use a two-repo flow:

1. Implement and validate in this repository.
2. Replay/cherry-pick clean commits into a Zephyr fork branch.
3. Open upstream PRs from the Zephyr fork.

Do not treat this repository itself as the direct upstream PR branch.

## Priority Order for New Work

1. Keep current peripherals stable and documented.
2. Close known driver API gaps (CAN extended IDs, I2C target mode, SPI CS/async, GPIO IRQ coverage).
3. Add clock-tree/power-management depth.
4. Split and polish for upstream acceptance.
