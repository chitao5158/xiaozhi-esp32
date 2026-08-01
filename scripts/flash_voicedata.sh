#!/usr/bin/env bash
# Flash the esp_tts_chinese (xiaole) voice model to the `voice_data`
# partition on a connected ESP32-S3 board. Run AFTER idf.py has flashed
# the firmware, or as a one-shot setup the first time you provision the
# device.
#
# Usage:
#   scripts/flash_voicedata.sh [PORT] [BAUD]
#
# Defaults:
#   PORT = /dev/cu.usbserial-13230 (macOS-friendly)
#   BAUD = 921600
#
# The .dat file ships inside esp-sr (managed_components/...). We pick the
# xiaole voice — ~2.8 MB, the smallest of the four shipped voices.

set -euo pipefail

PORT="${1:-/dev/cu.usbserial-13230}"
BAUD="${2:-921600}"

# Resolve paths relative to this script so it works from anywhere.
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." && pwd )"

VOICE_DAT="${PROJECT_ROOT}/managed_components/espressif__esp-sr/esp-tts/esp_tts_chinese/esp_tts_voice_data_xiaole.dat"
# Must match partitions/v2/16m_custom_wakeword.csv
VOICE_OFFSET=0xd00000

if [[ ! -f "${VOICE_DAT}" ]]; then
    echo "ERROR: voice data file not found:" >&2
    echo "  ${VOICE_DAT}" >&2
    echo "Run 'idf.py build' first to fetch managed_components." >&2
    exit 1
fi

if ! command -v python3 >/dev/null; then
    echo "ERROR: python3 not found in PATH" >&2
    exit 1
fi

VOICE_SIZE=$(wc -c < "${VOICE_DAT}" | tr -d ' ')
VOICE_SIZE_HEX=$(printf '0x%x' "${VOICE_SIZE}")
VOICE_OFFSET_HEX=$(printf '0x%x' "${VOICE_OFFSET}")

echo "Flashing voice data:"
echo "  source : ${VOICE_DAT}"
echo "  size   : ${VOICE_SIZE} bytes (${VOICE_SIZE_HEX})"
echo "  offset : ${VOICE_OFFSET_HEX}"
echo "  port   : ${PORT} @ ${BAUD}"

# Source the IDF env so we get the bundled esptool module on PYTHONPATH.
# shellcheck disable=SC1091
if [[ -f "${HOME}/.espressif/v5.5.2/esp-idf/export.sh" ]]; then
    # shellcheck disable=SC1091
    source "${HOME}/.espressif/v5.5.2/esp-idf/export.sh" >/dev/null 2>&1 || true
fi

python3 -m esptool \
    --chip esp32s3 \
    --port "${PORT}" \
    --baud "${BAUD}" \
    --before default_reset \
    --after hard_reset \
    write_flash "${VOICE_OFFSET_HEX}" "${VOICE_DAT}"

echo
echo "Verifying: reading first 16 bytes back from ${VOICE_OFFSET_HEX}..."
VERIFY_FILE="$(mktemp)"
python3 -m esptool \
    --chip esp32s3 \
    --port "${PORT}" \
    --baud "${BAUD}" \
    read_flash "${VOICE_OFFSET_HEX}" 16 "${VERIFY_FILE}" >/dev/null

# The xiaole .dat always starts with the ASCII magic "xiaole_YYYYMMDD".
MAGIC=$(head -c 7 "${VERIFY_FILE}")
EXPECTED="xiaole_"
if [[ "${MAGIC}" == "${EXPECTED}" ]]; then
    echo "  ✓ Verified: partition starts with '$(head -c 16 "${VERIFY_FILE}")'"
    rm -f "${VERIFY_FILE}"
    echo "Done. Reboot the device (or run idf.py app-flash again) to pick up the new voice data."
else
    echo "  ✗ MISMATCH: read back '${MAGIC}', expected '${EXPECTED}'" >&2
    echo "    Full bytes:" >&2
    xxd "${VERIFY_FILE}" >&2
    rm -f "${VERIFY_FILE}"
    echo "The flash may have failed. Try lowering BAUD to 115200 and rerun." >&2
    exit 1
fi