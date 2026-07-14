#!/bin/bash
# =============================================================================
# _common.sh
#
# Shared helpers for build_desktop.sh and build_jetson.sh.
# Not meant to be run directly: it is sourced by the build scripts.
# =============================================================================

STEREOLABS_REPO="stereolabs/zed"
DOCKERHUB_API="https://hub.docker.com/v2/repositories/${STEREOLABS_REPO}/tags"

# ROS 2 distributions supported by these images.
SUPPORTED_ROS_DISTROS="humble jazzy lyrical"

# Print an error message and exit.
die() {
  echo "ERROR: $*" >&2
  exit 1
}

# validate_ros_distro <distro> : accept only a supported ROS 2 distribution.
validate_ros_distro() {
  local distro="$1" supported
  for supported in ${SUPPORTED_ROS_DISTROS}; do
    [ "${distro}" = "${supported}" ] && return 0
  done
  die "Unsupported ROS 2 distribution '${distro}'. Supported: ${SUPPORTED_ROS_DISTROS}."
}

# supported_ubuntu_for <distro> : Ubuntu release(s) a ROS 2 distribution officially
# targets (REP-2000 + the Lyrical development setup). Empty => not gated.
supported_ubuntu_for() {
  case "$1" in
    humble)  echo "22.04" ;;
    jazzy)   echo "22.04 24.04" ;;
    lyrical) echo "24.04 26.04" ;;
    *)       echo "" ;;
  esac
}

# jetson_ubuntu_version <platform_suffix> : Ubuntu version of a JetPack/L4T base
# (L4T r35/JP5 -> 20.04, r36/JP6 -> 22.04, r38/JP7 -> 24.04).
jetson_ubuntu_version() {
  case "$1" in
    *l4t-r35*|*jp5*) echo "20.04" ;;
    *l4t-r36*|*jp6*) echo "22.04" ;;
    *l4t-r38*|*jp7*) echo "24.04" ;;
    *)               echo "" ;;
  esac
}

# assert_distro_ubuntu_supported <ros_distro> <ubuntu_version>
# Fail fast (before the build) on a ROS 2 distro / Ubuntu pair that is not
# officially supported, so an unsupported combination does not waste time on a
# doomed APT lookup or from-source build.
assert_distro_ubuntu_supported() {
  local distro="$1" ubuntu="$2" supported
  supported="$(supported_ubuntu_for "${distro}")"
  [ -z "${supported}" ] && return 0   # distro not gated
  [ -z "${ubuntu}" ] && return 0       # unknown Ubuntu -> let the build decide
  case " ${supported} " in
    *" ${ubuntu} "*) return 0 ;;
    *) die "ROS 2 '${distro}' is not supported on Ubuntu ${ubuntu}. Supported Ubuntu release(s) for '${distro}': ${supported}. (Ubuntu 22.04 -> Humble, or Jazzy from source; Jazzy -> 22.04/24.04; Lyrical -> 24.04/26.04.)" ;;
  esac
}

# require_cmd <command> : make sure a command is available.
require_cmd() {
  command -v "$1" > /dev/null 2>&1 || die "'$1' is required but not installed."
}

# tag_exists <tag> : return 0 if stereolabs/zed:<tag> exists on Docker Hub.
tag_exists() {
  local tag="$1"
  local code
  code="$(curl -L -s -o /dev/null -w '%{http_code}' "${DOCKERHUB_API}/${tag}")"
  [ "${code}" = "200" ]
}

# resolve_latest_sdk <suffix> : print the highest ZED SDK version (X.Y.Z) that
# has an image whose tag ends with "-<suffix>" (e.g. "devel-cuda12.8-ubuntu24.04"
# or "devel-l4t-r36.5"). Queries the Docker Hub tag list with pagination.
resolve_latest_sdk() {
  local suffix="$1"
  local page=1 names latest
  latest=""
  while [ "${page}" -le 20 ]; do
    names="$(curl -L -s "${DOCKERHUB_API}/?page_size=100&page=${page}")"
    # Stop when the page has no results.
    echo "${names}" | grep -q '"name"' || break
    # Extract "X.Y.Z-<suffix>" tags and keep the version part.
    local found
    found="$(echo "${names}" \
      | grep -oE "\"name\": *\"[0-9]+\.[0-9]+\.[0-9]+-${suffix//./\\.}\"" \
      | grep -oE "[0-9]+\.[0-9]+\.[0-9]+-${suffix//./\\.}" \
      | sed -E "s/-${suffix//./\\.}\$//")"
    if [ -n "${found}" ]; then
      latest="$(printf '%s\n%s\n' "${latest}" "${found}" | grep -v '^$' | sort -V | tail -n 1)"
    fi
    # The API stops returning a "next" link on the last page.
    echo "${names}" | grep -q '"next": *null' && break
    page=$((page + 1))
  done
  if [ -n "${latest}" ]; then
    echo "${latest}"
  fi
  # Always succeed: an empty result is handled by the caller (so that, under
  # 'set -e', the script does not exit silently before printing a clear error).
  return 0
}

# copy_wrapper_sources : copy the wrapper packages of the current checkout into
# ./tmp_sources so they can be COPYed by the Dockerfile.
copy_wrapper_sources() {
  rm -rf ./tmp_sources
  mkdir -p ./tmp_sources
  cp -r ../zed* ./tmp_sources/
}

# cleanup_wrapper_sources : remove the temporary source folder.
cleanup_wrapper_sources() {
  rm -rf ./tmp_sources
}

# stage_sdk_url : given $SDK_URL (empty, a URL, or a local '.run' file path),
# prepare ./tmp_sdk for the Dockerfile's COPY and set BUILD_ARG_SDK_URL /
# BUILD_ARG_SDK_LOCAL_FILE for the 'docker build --build-arg' call. A Docker
# build cannot see the host filesystem, so a local file is staged into the
# build context here; a URL is passed through unchanged and fetched by
# install_zed_sdk.sh at build time.
stage_sdk_url() {
  rm -rf ./tmp_sdk
  mkdir -p ./tmp_sdk
  BUILD_ARG_SDK_URL="${SDK_URL}"
  BUILD_ARG_SDK_LOCAL_FILE=""
  if [ -n "${SDK_URL}" ] && [ -f "${SDK_URL}" ]; then
    echo "Custom ZED SDK: local file ${SDK_URL}"
    cp "${SDK_URL}" ./tmp_sdk/ZED_SDK_installer.run
    BUILD_ARG_SDK_URL=""
    BUILD_ARG_SDK_LOCAL_FILE="ZED_SDK_installer.run"
  fi
}

# cleanup_tmp_sdk : remove the temporary local SDK installer folder.
cleanup_tmp_sdk() {
  rm -rf ./tmp_sdk
}
