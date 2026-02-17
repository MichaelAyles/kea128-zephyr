# KEA128 Port Progress Tracker

Last updated: 2026-02-17

## Milestone Timeline

- [x] Repository initialized and pushed (`08bc025`).
- [x] SoC + board definitions in place.
- [x] Core peripheral drivers added.
- [x] Build + flash path validated on hardware.
- [x] Detailed implementation roadmap documented (`docs/bringup.md`).

## Roadmap Progress

### P0: Stabilize and Prove Baseline

- [x] Add repeatable hardware smoke checklist and expected log signatures.
- [x] Document current pinmux map and validation status.
- [x] Add Twister build coverage for board + key subsystems (`scripts/twister.sh` + `tests/kea/per_driver_build`).
- [x] Add minimal per-driver unit/build tests where feasible (11 build-only scenarios, including I2C target + SPI async configs).
- [ ] Add package/connector-level pin mapping details.

### P1: Driver Feature Depth

- [x] CAN extended ID TX/RX support.
- [x] CAN improved state/error callback behavior.
- [x] CAN TX completion semantics via mailbox completion ISR callbacks.
- [x] I2C target mode support (single-target callback flow + IRQ path).
- [ ] I2C target mode cross-board hardware validation (external controller/target interactions).
- [x] SPI GPIO-CS helper path.
- [x] SPI async callback path (`transceive_async`, workqueue-backed).
- [ ] SPI IRQ-driven async engine optimization (optional phase).
- [x] GPIO migrate KBI gate enable to generic clock control.

### P2: Clock/Power

- [ ] ICS/clock-tree integration.
- [ ] Low-power integration and resume-safe peripheral behavior.
- [ ] Runtime/system PM compatibility validation.

### P3: Upstream Readiness

- [ ] Split into subsystem-sized upstream patches.
- [ ] Add/align tests for CI acceptance.
- [ ] Prepare PR series from Zephyr fork branch.

## Working Notes

- Keep this file updated in the same commit that lands milestone changes.
- For major feature steps, add commit hashes under relevant checklist items.
