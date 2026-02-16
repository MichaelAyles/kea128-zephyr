#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-${ROOT_DIR}/build/trk_kea128}"
JLINK_DEVICE="${JLINK_DEVICE:-SKEAZ128xxx4}"

cmd=(west flash -r jlink --build-dir "${BUILD_DIR}" -- --device "${JLINK_DEVICE}")
if [[ -n "${JLINK_DEV_ID:-}" ]]; then
  cmd+=(--dev-id "${JLINK_DEV_ID}")
fi
"${cmd[@]}"
