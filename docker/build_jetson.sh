#!/bin/bash
# =============================================================================
# build_jetson.sh
#
# Builds the ZED ROS 2 Wrapper Jetson (L4T) Docker image on top of a StereoLabs
# ZED base image (https://hub.docker.com/r/stereolabs/zed/tags).
#
# Inputs (all optional, with defaults):
#   --ros-distro <distro>   ROS 2 distribution     (default: humble)
#                             Supported: humble, jazzy, lyrical
#   --os <version>          JetPack / L4T version   (default: jp6.2.2)
#                             Accepted spellings:
#                               jp6.2.2 | jetson-jp6.2.2 | 6.2.2  -> jetson-jp6.2.2
#                               l4t-r36.5 | r36.5 | 36.5          -> l4t-r36.5
#   --sdk <X.Y.Z|latest>    ZED SDK version         (default: latest)
#   --sdk-url <URL|path>    Build against a custom ZED SDK '.run' installer
#                             (e.g. a local or pre-release build) instead of a
#                             published 'stereolabs/zed' image. Accepts either
#                             a URL (fetched at build time) or a path to a
#                             local '.run' file (staged into the build
#                             context, since a Docker build cannot see the
#                             host filesystem otherwise). The base image then
#                             becomes a plain 'nvcr.io/nvidia/l4t-jetpack'
#                             image, so '--os' must be a full L4T version
#                             (e.g. l4t-r36.4.0) and '--sdk' is ignored.
#
# The Jetson images always use the StereoLabs 'devel' base variant (unless
# --sdk-url is used, which builds on a plain L4T JetPack base).
#
# Note: the ROS 2 distribution must be supported on the JetPack base Ubuntu:
#   * JetPack 6.x (L4T r36.x) = Ubuntu 22.04 -> Humble (APT), or Jazzy (Tier 3, from source)
#   * JetPack 7.x (L4T r38.x) = Ubuntu 24.04 -> Jazzy, or Lyrical (from source)
# Unsupported pairings (e.g. Lyrical on JetPack 6 / Ubuntu 22.04) are rejected
# up front.
#
# To cross-build a Jetson image from a desktop PC, first run:
#   docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
#
# Examples:
#   ./build_jetson.sh --ros-distro humble --os jp6.2.2          # Ubuntu 22.04
#   ./build_jetson.sh --ros-distro jazzy --os jp6.2.2 --sdk latest     # Ubuntu 22.04, from source
#   ./build_jetson.sh --ros-distro lyrical --os jp7.1.0 --sdk latest   # Ubuntu 24.04, from source
# =============================================================================
set -e
cd "$(dirname "$0")"
# shellcheck source=scripts/_common.sh
source ./scripts/_common.sh

require_cmd curl
require_cmd docker

# --- Defaults ----------------------------------------------------------------
# Default JetPack 6.2.2 = Ubuntu 22.04, whose supported ROS 2 distribution is
# Humble. For Jazzy/Lyrical use a JetPack 7 (Ubuntu 24.04) base.
ROS_DISTRO="humble"
OS="jp6.2.2"
SDK="latest"
SDK_URL=""        # custom ZED SDK .run installer URL (empty => published image)
VARIANT="devel"   # Jetson images always use the 'devel' base variant

# --- Parse arguments ---------------------------------------------------------
while [ "$#" -gt 0 ]; do
  case "$1" in
    --ros-distro) ROS_DISTRO="$2"; shift 2 ;;
    --os)         OS="$2";         shift 2 ;;
    --sdk)        SDK="$2";        shift 2 ;;
    --sdk-url)    SDK_URL="$2";    shift 2 ;;
    -h|--help)
      grep '^#' "$0" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    *) die "Unknown argument: $1 (use --help)" ;;
  esac
done

validate_ros_distro "${ROS_DISTRO}"

# --- Normalize the OS version and select the base image ----------------------
OS_CLEAN="${OS#jetson-}"     # strip optional 'jetson-' prefix
OS_CLEAN="${OS_CLEAN#l4t-}"  # strip optional 'l4t-' prefix

if [ -n "${SDK_URL}" ]; then
  # Custom SDK mode: build on a plain NVIDIA L4T (JetPack) base and install the
  # ZED SDK from the given .run installer URL. A full L4T version is required to
  # select an 'nvcr.io/nvidia/l4t-jetpack' tag, so the JetPack ('jp...')
  # spelling is not accepted here.
  if ! [[ "${OS_CLEAN}" =~ ^r?[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    die "With --sdk-url on Jetson, pass --os as a full L4T version (e.g. l4t-r36.4.0) to select an 'nvcr.io/nvidia/l4t-jetpack' base image. Got '${OS}'."
  fi
  L4T="${OS_CLEAN#r}"
  PLATFORM_SUFFIX="l4t-r${L4T}"
  BASE_IMAGE="nvcr.io/nvidia/l4t-jetpack:r${L4T}"
  IMAGE_TAG="zed_ros2_${ROS_DISTRO}_${PLATFORM_SUFFIX}_sdkcustom"
  echo "Custom ZED SDK: ${SDK_URL}"
else
  # Published SDK mode. JetPack (3 numbers) -> jetson-jp<x.y.z> ; L4T (2 numbers)
  # -> l4t-r<maj.min>.
  if [[ "${OS_CLEAN}" =~ ^(jp)?[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    PLATFORM_SUFFIX="jetson-jp${OS_CLEAN#jp}"
  elif [[ "${OS_CLEAN}" =~ ^r?[0-9]+\.[0-9]+$ ]]; then
    PLATFORM_SUFFIX="l4t-r${OS_CLEAN#r}"
  else
    die "Invalid Jetson OS version '${OS}'. Expected e.g. 'jp6.2.2' or 'l4t-r36.5'."
  fi
fi

# Fail fast on an unsupported ROS 2 distro / Ubuntu combination (the Ubuntu
# release is determined by the JetPack/L4T base).
assert_distro_ubuntu_supported "${ROS_DISTRO}" "$(jetson_ubuntu_version "${PLATFORM_SUFFIX}")"

if [ -z "${SDK_URL}" ]; then
  # --- Resolve the ZED SDK version ------------------------------------------
  SUFFIX="${VARIANT}-${PLATFORM_SUFFIX}"
  if [ "${SDK}" = "latest" ]; then
    echo "Resolving latest ZED SDK for '${SUFFIX}' ..."
    SDK="$(resolve_latest_sdk "${SUFFIX}")"
    [ -n "${SDK}" ] || die "Could not find any 'stereolabs/zed' image matching '*-${SUFFIX}'. Check the JetPack/L4T version on https://hub.docker.com/r/stereolabs/zed/tags"
    echo "Latest ZED SDK resolved to: ${SDK}"
  fi

  BASE_TAG="${SDK}-${SUFFIX}"
  BASE_IMAGE="stereolabs/zed:${BASE_TAG}"

  # --- Verify the base image exists -----------------------------------------
  echo "Checking base image: ${BASE_IMAGE}"
  tag_exists "${BASE_TAG}" \
    || die "Base image '${BASE_IMAGE}' does not exist. See available tags at https://hub.docker.com/r/stereolabs/zed/tags"

  IMAGE_TAG="zed_ros2_${ROS_DISTRO}_${PLATFORM_SUFFIX}_sdk${SDK}"
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
