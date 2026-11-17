#!/bin/bash
# =============================================================================
# build_desktop.sh
#
# Builds the ZED ROS 2 Wrapper desktop Docker image on top of a StereoLabs ZED
# base image (https://hub.docker.com/r/stereolabs/zed/tags).
#
# Inputs (all optional, with defaults):
#   --ros-distro <distro>   ROS 2 distribution        (default: jazzy)
#                             Supported: humble, jazzy, lyrical
#   --os <ubuntu-XX.XX>     Ubuntu version            (default: ubuntu-24.04)
#   --sdk <X.Y.Z|latest>    ZED SDK version           (default: latest)
#   --cuda <XX.X>           CUDA version of the base  (default: 13.0, or 13.3
#                             on Ubuntu 26.04)
#                             See the README configuration table for the
#                             CUDA/Ubuntu combinations published per ZED SDK.
#   --sdk-url <URL|path>    Build against a custom ZED SDK '.run' installer
#                             (e.g. a local or pre-release build) instead of a
#                             published 'stereolabs/zed' image. Accepts either
#                             a URL (fetched at build time) or a path to a
#                             local '.run' file (staged into the build
#                             context, since a Docker build cannot see the
#                             host filesystem otherwise). The base image then
#                             becomes a plain 'nvcr.io/nvidia/cuda' image, so
#                             '--cuda' must be a full version (e.g. 12.6.3)
#                             and '--sdk' is ignored.
#
# The desktop images always use the StereoLabs 'devel' base variant (unless
# --sdk-url is used, which builds on a plain CUDA base). The OpenGL runtime
# libraries, needed by graphical tools such as RViz 2, are added by the
# Dockerfile.
#
# Examples:
#   ./build_desktop.sh --ros-distro jazzy --os ubuntu-24.04 --sdk latest
#   ./build_desktop.sh --ros-distro lyrical --os ubuntu-26.04
#   ./build_desktop.sh --ros-distro jazzy --os ubuntu-24.04 --cuda 12.8
#   ./build_desktop.sh --ros-distro humble --os ubuntu-22.04 --cuda 12.6.3 \
#                      --sdk-url https://download.stereolabs.com/.../ZED_SDK_....run
#   ./build_desktop.sh --ros-distro humble --os ubuntu-22.04 --cuda 12.6.3 \
#                      --sdk-url ~/Downloads/ZED_SDK_Ubuntu22_cuda12.6_v5.0.run
# =============================================================================
set -e
cd "$(dirname "$0")"
# shellcheck source=scripts/_common.sh
source ./scripts/_common.sh

require_cmd curl
require_cmd docker

# --- Defaults ----------------------------------------------------------------
ROS_DISTRO="jazzy"
OS="ubuntu-24.04"
SDK="latest"
CUDA=""              # empty => default for the selected Ubuntu, set below
SDK_URL=""           # custom ZED SDK .run installer URL (empty => published image)
VARIANT="devel"      # desktop images always use the 'devel' base variant

# --- Parse arguments ---------------------------------------------------------
while [ "$#" -gt 0 ]; do
  case "$1" in
    --ros-distro) ROS_DISTRO="$2"; shift 2 ;;
    --os)         OS="$2";         shift 2 ;;
    --sdk)        SDK="$2";        shift 2 ;;
    --cuda)       CUDA="$2";       shift 2 ;;
    --sdk-url)    SDK_URL="$2";    shift 2 ;;
    -h|--help)
      grep '^#' "$0" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    *) die "Unknown argument: $1 (use --help)" ;;
  esac
done

validate_ros_distro "${ROS_DISTRO}"

# --- Normalize / validate the Ubuntu version ---------------------------------
UBUNTU="${OS#ubuntu-}"
if ! [[ "${UBUNTU}" =~ ^[0-9]+\.[0-9]+$ ]]; then
  die "Invalid Ubuntu version '${OS}'. Expected e.g. 'ubuntu-24.04'."
fi

# Fail fast on an unsupported ROS 2 distro / Ubuntu combination.
assert_distro_ubuntu_supported "${ROS_DISTRO}" "${UBUNTU}"

# --- Default CUDA version ----------------------------------------------------
# CUDA 13.0 is published for Ubuntu 22.04 and 24.04; Ubuntu 26.04 images ship
# CUDA 13.3 only. With --sdk-url there is no default: a full version is needed.
if [ -z "${CUDA}" ] && [ -z "${SDK_URL}" ]; then
  case "${UBUNTU}" in
    26.04) CUDA="13.3" ;;
    *)     CUDA="13.0" ;;
  esac
fi

# --- Select the base image ---------------------------------------------------
if [ -n "${SDK_URL}" ]; then
  # Custom SDK mode: build on a plain NVIDIA CUDA base and install the ZED SDK
  # from the given .run installer URL. A full CUDA version (XX.X.X) is required
  # to select an 'nvcr.io/nvidia/cuda' tag.
  if ! [[ "${CUDA}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    die "With --sdk-url, --cuda must be a full version (e.g. 12.6.3) to select an 'nvcr.io/nvidia/cuda' base image. Got '${CUDA:-none}'."
  fi
  BASE_IMAGE="nvcr.io/nvidia/cuda:${CUDA}-devel-ubuntu${UBUNTU}"
  IMAGE_TAG="zed_ros2_desktop_${ROS_DISTRO}_u${UBUNTU}_sdkcustom_cuda${CUDA}"
  echo "Custom ZED SDK: ${SDK_URL}"
else
  # Published SDK mode: build on a 'stereolabs/zed' image that already ships the
  # SDK.
  if ! [[ "${CUDA}" =~ ^[0-9]+\.[0-9]+$ ]]; then
    die "Invalid CUDA version '${CUDA}'. Expected e.g. '13.0'."
  fi

  # --- Resolve the ZED SDK version ------------------------------------------
  SUFFIX="${VARIANT}-cuda${CUDA}-ubuntu${UBUNTU}"
  if [ "${SDK}" = "latest" ]; then
    echo "Resolving latest ZED SDK for '${SUFFIX}' ..."
    SDK="$(resolve_latest_sdk "${SUFFIX}")"
    [ -n "${SDK}" ] || die "Could not find any 'stereolabs/zed' image matching '*-${SUFFIX}'. Check the CUDA/Ubuntu combination on https://hub.docker.com/r/stereolabs/zed/tags"
    echo "Latest ZED SDK resolved to: ${SDK}"
  fi

  BASE_TAG="${SDK}-${SUFFIX}"
  BASE_IMAGE="stereolabs/zed:${BASE_TAG}"

  # --- Verify the base image exists -----------------------------------------
  echo "Checking base image: ${BASE_IMAGE}"
  tag_exists "${BASE_TAG}" \
    || die "Base image '${BASE_IMAGE}' does not exist. See available tags at https://hub.docker.com/r/stereolabs/zed/tags"

  IMAGE_TAG="zed_ros2_desktop_${ROS_DISTRO}_u${UBUNTU}_sdk${SDK}_cuda${CUDA}"
fi

# --- Build -------------------------------------------------------------------
echo "============================================================"
echo " Base image : ${BASE_IMAGE}"
echo " ROS distro : ${ROS_DISTRO}"
echo " Output tag : ${IMAGE_TAG}"
echo "============================================================"

copy_wrapper_sources
stage_sdk_url
trap 'cleanup_wrapper_sources; cleanup_tmp_sdk' EXIT

docker build \
  -t "${IMAGE_TAG}" \
  --build-arg BASE_IMAGE="${BASE_IMAGE}" \
  --build-arg ROS2_DISTRO="${ROS_DISTRO}" \
  --build-arg CUSTOM_ZED_SDK_URL="${BUILD_ARG_SDK_URL}" \
  --build-arg CUSTOM_ZED_SDK_LOCAL_FILE="${BUILD_ARG_SDK_LOCAL_FILE}" \
  -f ./Dockerfile .

echo ">>> Built image: ${IMAGE_TAG}"
