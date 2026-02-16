#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-${ROOT_DIR}/build/trk_kea128}"

west build -p auto \
  -b trk_kea128 \
  "${ROOT_DIR}/app" \
  --build-dir "${BUILD_DIR}" \
  -- \
  -DBOARD_ROOT="${ROOT_DIR}" \
  -DSOC_ROOT="${ROOT_DIR}" \
  -DDTS_ROOT="${ROOT_DIR}"
