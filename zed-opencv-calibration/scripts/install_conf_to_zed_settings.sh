#!/usr/bin/env bash
# © 2026, Cargo Robotics
# Offer to copy the newest SN<virtual_sn>.conf from a calibration output directory into
# /usr/local/zed/settings so the ZED SDK loads it by default. Run inside the zed-end-effector
# container (or on-host with matching paths).

set -euo pipefail

usage() {
  echo "Usage: $(basename "$0") [calibration_output_dir]" >&2
  echo "" >&2
  echo "  calibration_output_dir  Directory containing SN*.conf (default: \$ZED_CALIB_OUTPUT_DIR" >&2
  echo "                          or /var/cargo/zed-calibration/calibration_config)." >&2
  echo "  ZED_SETTINGS_DIR        Destination (default: /usr/local/zed/settings)." >&2
  exit 1
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
fi

SRC_DIR="${1:-${ZED_CALIB_OUTPUT_DIR:-/var/cargo/zed-calibration/calibration_config}}"
DEST_DIR="${ZED_SETTINGS_DIR:-/usr/local/zed/settings}"

if [[ ! -d "$SRC_DIR" ]]; then
  echo "Error: calibration output directory does not exist: ${SRC_DIR}" >&2
  exit 1
fi

if [[ ! -d "$DEST_DIR" ]]; then
  echo "Error: ZED settings directory does not exist: ${DEST_DIR}" >&2
  exit 1
fi

shopt -s nullglob
candidates=("${SRC_DIR}"/SN*.conf)
shopt -u nullglob

if [[ ${#candidates[@]} -eq 0 ]]; then
  echo "No SN*.conf files found in ${SRC_DIR}." >&2
  exit 1
fi

latest=""
latest_mtime=-1
for f in "${candidates[@]}"; do
  if [[ ! -f "$f" ]]; then
    continue
  fi
  if stat --version >/dev/null 2>&1; then
    mt=$(stat -c %Y "$f")
  else
    mt=$(stat -f %m "$f")
  fi
  if (( mt > latest_mtime )); then
    latest_mtime=$mt
    latest=$f
  fi
done

if [[ -z "$latest" ]]; then
  echo "Error: could not pick a calibration .conf file." >&2
  exit 1
fi

base=$(basename "$latest")
dest="${DEST_DIR}/${base}"

echo ""
echo "Newest calibration file: ${latest}"
echo "Destination:             ${dest}"
read -r -p "Copy this file into the ZED SDK settings directory? [y/N] " answer
case "${answer:-}" in
  y | Y | yes | YES) ;;
  *)
    echo "Skipped."
    exit 0
    ;;
esac

if [[ -f "$dest" ]]; then
  read -r -p "File already exists: ${dest}. Overwrite? [y/N] " ow
  case "${ow:-}" in
    y | Y | yes | YES) ;;
    *)
      echo "Skipped (not overwriting)."
      exit 0
      ;;
  esac
fi

cp -f "$latest" "$dest"
chmod a+r "$dest" || true
echo "Installed: ${dest}"
