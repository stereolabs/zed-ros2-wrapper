#!/usr/bin/env bash
# © 2026, Cargo Robotics
# Runs zed_stereo_calibration, then (on success) offers to copy the newest SN*.conf to
# /usr/local/zed/settings via install_conf_to_zed_settings.sh.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ZED_OPENCV_CALIB_SRC="${ZED_OPENCV_CALIB_SRC:-/root/ros2_ws/src/zed-opencv-calibration}"
# Same layout as colcon build of the top-level zed_opencv_calibration CMake project.
ZED_CALIB_STEREO_BUILD_DIR="${ZED_CALIB_STEREO_BUILD_DIR:-/root/ros2_ws/build/zed_opencv_calibration/stereo_calibration}"
ZED_CALIB_REPROJ_BUILD_DIR="${ZED_CALIB_REPROJ_BUILD_DIR:-/root/ros2_ws/build/zed_opencv_calibration/stereo_reprojection_viewer}"

INSTALL_HELPER="${SCRIPT_DIR}/install_conf_to_zed_settings.sh"
if [[ ! -f "$INSTALL_HELPER" ]]; then
  INSTALL_HELPER="${ZED_OPENCV_CALIB_SRC}/scripts/install_conf_to_zed_settings.sh"
fi

if ! command -v zed_stereo_calibration >/dev/null 2>&1; then
  export PATH="${ZED_CALIB_STEREO_BUILD_DIR}:${ZED_CALIB_REPROJ_BUILD_DIR}:${PATH}"
fi
if ! command -v zed_stereo_calibration >/dev/null 2>&1; then
  echo "Error: zed_stereo_calibration not found. Build the workspace (colcon) or set PATH or" >&2
  echo "       ZED_CALIB_STEREO_BUILD_DIR and ZED_CALIB_REPROJ_BUILD_DIR." >&2
  exit 127
fi

# Optional: read calibration_output_dir from the same YAML as --config (simple scalar lines only).
extract_calib_output_dir_from_yaml() {
  local yaml_path="$1"
  [[ -f "$yaml_path" ]] || return 1
  grep -E '^[[:space:]]*calibration_output_dir:' "$yaml_path" | head -1 | \
    sed -e 's/^[[:space:]]*calibration_output_dir:[[:space:]]*//' -e 's/^["'\'']//' -e 's/["'\'']$//' -e 's/[[:space:]]*#.*$//'
}

CONFIG_PATH=""
args=("$@")
i=0
while [[ $i -lt ${#args[@]} ]]; do
  if [[ "${args[i]}" == --config ]]; then
    ((i + 1 < ${#args[@]})) && CONFIG_PATH="${args[i + 1]}"
    break
  fi
  ((i++)) || true
done

CALIB_OUT=""
if [[ -n "${ZED_CALIB_OUTPUT_DIR:-}" ]]; then
  CALIB_OUT="${ZED_CALIB_OUTPUT_DIR}"
elif [[ -n "$CONFIG_PATH" ]]; then
  CALIB_OUT="$(extract_calib_output_dir_from_yaml "$CONFIG_PATH" || true)"
fi

zed_stereo_calibration "$@"
ec=$?

if [[ $ec -ne 0 ]]; then
  exit "$ec"
fi

if [[ ! -f "$INSTALL_HELPER" ]]; then
  echo "Warning: install helper not found at ${INSTALL_HELPER}; skipping install prompt." >&2
  exit 0
fi

if [[ -n "$CALIB_OUT" ]]; then
  bash "$INSTALL_HELPER" "$CALIB_OUT"
else
  bash "$INSTALL_HELPER"
fi
