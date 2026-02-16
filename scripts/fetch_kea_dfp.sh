#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE_DIR="${ROOT_DIR}/.cache/kea_dfp"
PACK_NAME="Keil.Kinetis_KEAxx_DFP.1.3.1.pack"
PACK_URL="https://www.keil.com/pack/${PACK_NAME}"

mkdir -p "${CACHE_DIR}"

if [ ! -f "${CACHE_DIR}/${PACK_NAME}" ]; then
  curl -fL --retry 3 -o "${CACHE_DIR}/${PACK_NAME}" "${PACK_URL}"
fi

if [ ! -d "${CACHE_DIR}/extracted" ]; then
  unzip -q "${CACHE_DIR}/${PACK_NAME}" -d "${CACHE_DIR}/extracted"
fi

echo "KEA DFP extracted at: ${CACHE_DIR}/extracted"
