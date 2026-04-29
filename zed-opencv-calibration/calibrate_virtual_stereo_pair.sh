#!/usr/bin/env bash
# © 2026, Cargo Robotics
# Jetson only: build and run zed-opencv-calibration in Docker (ZED SDK 5.1.x, L4T JP installer).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

L4T_MAJOR="${L4T_MAJOR:-35}"
L4T_MINOR="${L4T_MINOR:-4}"
L4T_PATCH="${L4T_PATCH:-1}"

DOCKERFILE="${SCRIPT_DIR}/Dockerfile.calibration"
IMAGE_NAME="${ZED_CALIBRATION_IMAGE:-cargo/zed-virtual-stereo-calibration:zed-5.1.0-l4t${L4T_MAJOR}.${L4T_MINOR}-aarch64}"
HOST_ZED_IMAGES_DIR="${HOST_ZED_IMAGES_DIR:-$HOME/zed-images}"
# Host directory for zed_calibration_*.yml, SN*.conf, etc. (container cwd is set to this mount).
HOST_ZED_CALIB_OUTPUT_DIR="${HOST_ZED_CALIB_OUTPUT_DIR:-$HOME/zed-calibration-output}"
HOST_CALIB_SOURCE_DIR="${HOST_CALIB_SOURCE_DIR:-$SCRIPT_DIR}"

usage() {
  echo "Usage: $(basename "$0") <build|run|shell> [extra docker args...]"
  echo ""
  echo "  build       Build ${IMAGE_NAME} (context: ${SCRIPT_DIR})"
  echo "  run|shell   Run container (ZED X / Argus volumes; see docker-compose-deploy zed)"
  echo ""
  echo "L4T line for Stereolabs URL (defaults ${L4T_MAJOR}.${L4T_MINOR}.${L4T_PATCH}): set L4T_MAJOR, L4T_MINOR, L4T_PATCH"
  echo ""
  echo "Environment:"
  echo "  ZED_CALIBRATION_IMAGE          Override image tag"
  echo "  CALIBRATION_JETSON_BASE_IMAGE  Optional; passed as IMAGE_NAME to docker build"
  echo "  HOST_ZED_IMAGES_DIR            Host directory for capture PNG pairs (default: $HOME/zed-images)"
  echo "  HOST_ZED_CALIB_OUTPUT_DIR      Host directory for calibration result files (default: $HOME/zed-calibration-output)"
  echo "  HOST_CALIB_SOURCE_DIR          Host source dir bind-mounted for live edits (default: script directory)"
  echo ""
  echo "Binaries on PATH in container: zed_stereo_calibration, zed_reprojection_viewer"
  echo "Ensure on host: systemctl is-active nvargus-daemon zed_x_daemon"
  echo "Captures (image_left_*.png) persist to: ${HOST_ZED_IMAGES_DIR} -> /root/zed-images"
  echo "Calibration outputs (*.yml, SN*.conf) persist to: ${HOST_ZED_CALIB_OUTPUT_DIR} -> /root/zed-calibration-out (container cwd)"
  exit 1
}

docker_build() {
  local build_args=(
    --build-arg "L4T_MAJOR=${L4T_MAJOR}"
    --build-arg "L4T_MINOR=${L4T_MINOR}"
    --build-arg "L4T_PATCH=${L4T_PATCH}"
  )
  if [[ -n "${CALIBRATION_JETSON_BASE_IMAGE:-}" ]]; then
    build_args+=(--build-arg "IMAGE_NAME=${CALIBRATION_JETSON_BASE_IMAGE}")
  fi
  docker build \
    -f "${DOCKERFILE}" \
    "${build_args[@]}" \
    -t "${IMAGE_NAME}" \
    "${SCRIPT_DIR}"
}

if [[ $# -lt 1 ]]; then
  usage
fi

cmd="$1"
shift

case "${cmd}" in
  build)
    docker_build
    ;;
  run | shell)
    if ! mkdir -p "${HOST_ZED_IMAGES_DIR}" 2>/dev/null; then
      FALLBACK_ZED_IMAGES_DIR="$HOME/zed-images"
      echo "Warning: cannot create HOST_ZED_IMAGES_DIR='${HOST_ZED_IMAGES_DIR}' (permission denied)." >&2
      echo "         Falling back to '${FALLBACK_ZED_IMAGES_DIR}'." >&2
      HOST_ZED_IMAGES_DIR="${FALLBACK_ZED_IMAGES_DIR}"
      mkdir -p "${HOST_ZED_IMAGES_DIR}"
    fi
    if ! mkdir -p "${HOST_ZED_CALIB_OUTPUT_DIR}" 2>/dev/null; then
      FALLBACK_CALIB_OUT="$HOME/zed-calibration-output"
      echo "Warning: cannot create HOST_ZED_CALIB_OUTPUT_DIR='${HOST_ZED_CALIB_OUTPUT_DIR}' (permission denied)." >&2
      echo "         Falling back to '${FALLBACK_CALIB_OUT}'." >&2
      HOST_ZED_CALIB_OUTPUT_DIR="${FALLBACK_CALIB_OUT}"
      mkdir -p "${HOST_ZED_CALIB_OUTPUT_DIR}"
    fi
    if [[ ! -d "${HOST_CALIB_SOURCE_DIR}" ]]; then
      echo "Error: HOST_CALIB_SOURCE_DIR does not exist: ${HOST_CALIB_SOURCE_DIR}" >&2
      exit 1
    fi
    docker_run_args=(
      docker run --rm -it
      --runtime=nvidia
      --privileged
      --network=host
      --ipc=host
      --pid=host
      -e NVIDIA_DRIVER_CAPABILITIES=all
      -e DISPLAY="${DISPLAY:-:0}"
      -v /dev:/dev
      -v /usr/local/zed/resources/:/usr/local/zed/resources/
      -v /usr/local/zed/settings/:/usr/local/zed/settings/
      -v /dev/shm:/dev/shm
      -v /tmp:/tmp
      -v /var/nvidia/nvcam/settings/:/var/nvidia/nvcam/settings/
      -v /etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service
      -v "${HOST_ZED_IMAGES_DIR}":/root/zed-images
      -v "${HOST_ZED_CALIB_OUTPUT_DIR}":/root/zed-calibration-out
      -v "${HOST_CALIB_SOURCE_DIR}":/root/zed-opencv-calibration
      -w /root/zed-calibration-out
    )
    if [[ -d /usr/lib/aarch64-linux-gnu/tegra ]]; then
      docker_run_args+=(-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra)
    fi
    if [[ -d /usr/lib/aarch64-linux-gnu/tegra-egl ]]; then
      docker_run_args+=(-v /usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu/tegra-egl)
    fi
    if [[ -d /usr/lib/aarch64-linux-gnu/nvidia ]]; then
      docker_run_args+=(-v /usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/nvidia)
    fi
    docker_run_args+=(
      -e "LD_LIBRARY_PATH=/usr/local/zed/lib:/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu/nvidia:/usr/local/cuda/lib64"
      "$@"
      "${IMAGE_NAME}"
      bash -l
    )
    exec "${docker_run_args[@]}"
    ;;
  -h | --help | help)
    usage
    ;;
  *)
    echo "Unknown command: ${cmd}" >&2
    usage
    ;;
esac
