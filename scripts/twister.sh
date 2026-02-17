#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLATFORM="${PLATFORM:-trk_kea128/skeaz1284}"
TEST_PATH="${TEST_PATH:-${ROOT_DIR}/tests/kea/per_driver_build}"
OUT_DIR="${OUT_DIR:-${ROOT_DIR}/twister-out}"
WEST_PY="$(head -n 1 "$(command -v west)" | sed 's/^#!//')"

if [[ -n "${ZEPHYR_EXTRA_MODULES:-}" ]]; then
  export ZEPHYR_EXTRA_MODULES="${ROOT_DIR};${ZEPHYR_EXTRA_MODULES}"
else
  export ZEPHYR_EXTRA_MODULES="${ROOT_DIR}"
fi

${WEST_PY} - <<'PY'
import importlib.util
import sys

missing = []
for mod in ("junitparser",):
    if importlib.util.find_spec(mod) is None:
        missing.append(mod)

if missing:
    print("Missing Twister python deps: " + ", ".join(missing))
    print("Install with: python3 -m pip install --user " + " ".join(missing))
    sys.exit(1)
PY

west twister \
  -T "${TEST_PATH}" \
  -p "${PLATFORM}" \
  --outdir "${OUT_DIR}" \
  --board-root "${ROOT_DIR}" \
  "$@"
