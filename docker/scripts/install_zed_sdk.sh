#!/bin/bash
# =============================================================================
# install_zed_sdk.sh <custom_zed_sdk_url> <local_installer_filename>
#
# Installs the ZED SDK from a custom ".run" installer, either fetched from a
# URL or copied from a local file staged into the build context (see
# build_*.sh --sdk-url <URL|local path>; a Docker build cannot see the host
# filesystem, so a local file is staged into ./tmp_sdk and COPYed in by the
# Dockerfile before this script runs).
#
# This is used only when building on top of a *plain* CUDA / L4T base image,
# i.e. when "--sdk-url" was passed to build_desktop.sh / build_jetson.sh to
# build against an SDK that is NOT published as a "stereolabs/zed" Docker image
# (e.g. a local or pre-release build).
#
# When both arguments are empty (the default), the base image is a
# "stereolabs/zed" image that already ships the SDK, so this script is a
# no-op.
# =============================================================================
set -e

CUSTOM_ZED_SDK_URL="${1:-}"
LOCAL_INSTALLER_FILENAME="${2:-}"

if [ -z "${CUSTOM_ZED_SDK_URL}" ] && [ -z "${LOCAL_INSTALLER_FILENAME}" ]; then
  echo "No custom ZED SDK URL or local installer provided; the base image is expected to already ship the ZED SDK."
  exit 0
fi

# SDK installer prerequisites (build tooling comes from the Dockerfile). The
# ZED SDK '.run' also pulls its own dependencies in silent mode; these are a
# safety net and provide the libusb '.so' link (arch-independent, replacing the
# old x86_64-only manual symlink) the wrapper links against.
apt-get update || true
apt-get install -y --no-install-recommends \
  wget zstd libpng-dev libgomp1 libusb-1.0-0-dev
rm -rf /var/lib/apt/lists/*

# The SDK installer reads the CUDA version from /usr/local/cuda/version.txt.
# Plain CUDA base images expose it through the CUDA_VERSION env variable.
if [ -n "${CUDA_VERSION:-}" ] && [ -d /usr/local/cuda ]; then
  echo "CUDA Version ${CUDA_VERSION}" > /usr/local/cuda/version.txt
fi

if [ -n "${LOCAL_INSTALLER_FILENAME}" ]; then
  echo "Installing the ZED SDK from a local installer staged into the build context: ${LOCAL_INSTALLER_FILENAME}"
  cp "/tmp/sdk_installer/${LOCAL_INSTALLER_FILENAME}" /tmp/ZED_SDK_installer.run
else
  echo "Installing the ZED SDK from a custom URL: ${CUSTOM_ZED_SDK_URL}"
  # Fail fast if the URL does not point to an existing file.
  if [ "$(curl -L -I "${CUSTOM_ZED_SDK_URL}" -o /dev/null -s -w '%{http_code}')" != "200" ]; then
    echo "ERROR: the custom ZED SDK URL is not reachable (expected HTTP 200): ${CUSTOM_ZED_SDK_URL}" >&2
    exit 1
  fi
  wget -q -O /tmp/ZED_SDK_installer.run "${CUSTOM_ZED_SDK_URL}"
fi

chmod +x /tmp/ZED_SDK_installer.run
# silent      : no interactive prompts
# skip_tools  : do not install the ZED tools (not needed in a headless image)
# skip_cuda   : CUDA is already provided by the base image (desktop) or by
#               JetPack / L4T (Jetson)
/tmp/ZED_SDK_installer.run -- silent skip_tools skip_cuda
rm -f /tmp/ZED_SDK_installer.run

echo "ZED SDK installed."
