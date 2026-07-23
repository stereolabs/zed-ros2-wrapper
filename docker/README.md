# Docker

This folder builds Docker images that run the *ZED ROS 2 Wrapper*.

The images are built **on top of the official StereoLabs ZED images**
([hub.docker.com/r/stereolabs/zed](https://hub.docker.com/r/stereolabs/zed/tags)),
which already ship **CUDA and the ZED SDK**. On top of that base, the build
installs **ROS 2** and compiles the wrapper from the sources of the current
checkout.

A single, parameterized `Dockerfile` is used for every target. Two entry-point
scripts, at the top level of this folder, select the right StereoLabs base
image and pass the build arguments:

* `build_desktop.sh` — desktop / dGPU images (CUDA + Ubuntu base).
* `build_jetson.sh` — NVIDIA Jetson images (L4T base).

Everything else these two scripts and the `Dockerfile` rely on lives in
`scripts/` and is not meant to be run directly:

| File | Role |
|------|------|
| `Dockerfile` | Single image definition, parameterized by `BASE_IMAGE`, `ROS2_DISTRO` and (optional) `CUSTOM_ZED_SDK_URL`. |
| `scripts/install_ros2.sh` | Installs ROS 2 (from APT, or from source if no binaries exist for the base OS). |
| `scripts/install_zed_sdk.sh` | Installs the ZED SDK from a custom `.run` URL (only when `--sdk-url` is used; no-op otherwise). |
| `scripts/build_wrapper.sh` | Installs the wrapper dependencies and builds the wrapper with `colcon`. |
| `scripts/zed-wrapper-deps.repos` | Source-mode wrapper dependencies (used only when ROS 2 is built from source). |
| `scripts/ros_entrypoint.sh` | Container entry point (sets up the ROS 2 environment). |
| `scripts/_common.sh` | Shared helpers for the build scripts. |

## ROS 2 installation: APT or from source

The image installs ROS 2 **from APT when binary packages exist** for the
selected distribution on the base image Ubuntu release, and **builds ROS 2 from
source otherwise**. The decision is made automatically inside the build by
reading the actual Ubuntu codename of the base image.

| ROS 2 distro | Target Ubuntu | Base image OS | Mode |
|--------------|---------------|---------------|------|
| Humble | 22.04 | Ubuntu 22.04 / L4T r36.x | APT |
| Jazzy | 24.04 | Ubuntu 24.04 / L4T r38.x (JP7) | APT |
| Jazzy | 24.04 | Ubuntu 22.04 / L4T r36.x (JP6) | **from source** |
| Lyrical | 26.04 | Ubuntu 26.04 | APT |
| Lyrical | 26.04 | Ubuntu 24.04 / L4T r38.x (JP7) | **from source** |

> :warning: **From-source builds are heavy and best-effort.** Building ROS 2
> from source (e.g. Jazzy on an L4T r36.x / Ubuntu 22.04 base, or Lyrical on a
> 24.04 base) compiles the whole ROS 2 core and can take a long time. ROS 2
> distributions also officially target a specific Ubuntu release, so a
> from-source build on a different release is not guaranteed to compile. If a
> source dependency branch is missing for a new distribution, adjust
> `scripts/zed-wrapper-deps.repos`.

## Available configurations

The base images come from the StereoLabs registry
([hub.docker.com/r/stereolabs/zed](https://hub.docker.com/r/stereolabs/zed/tags)).
The desktop build uses the **`gl-devel`** variant and the Jetson build uses the
**`devel`** variant. The combinations published for those variants are listed
below.

> :calendar: **Updated: 2026-06-22.** The lists below are a snapshot. For the
> live, authoritative set always check
> [hub.docker.com/r/stereolabs/zed/tags](https://hub.docker.com/r/stereolabs/zed/tags);
> any combination is validated against Docker Hub before the build starts.

### Desktop (`gl-devel`)

| ZED SDK | CUDA | Ubuntu |
|---------|------|--------|
| 5.4.x *(latest)* | 12.8 | 22.04, 24.04 |
| 5.3.1, 5.3.0 | 12.8 | 22.04, 24.04 |
| 5.2.0 – 5.2.3 | 12.8 | 22.04, 24.04 |
| 5.1.0 – 5.1.2 | 12.8 | 22.04, 24.04 |
| 5.1.0 – 5.1.2 | 11.8 | 22.04 |

> :pushpin: The plain `devel` variant also offers CUDA 13.0, but the wrapper
> desktop image uses `gl-devel`, which currently ships **only CUDA 12.8** (plus
> CUDA 11.8 for ZED SDK 5.1.x). Pass `--cuda` to select it explicitly.

### Jetson (`devel`)

| ZED SDK | Available JetPack / L4T targets |
|---------|---------------------------------|
| 5.4.x *(latest)* | JP6.1 / r36.4 |
| 5.3.1, 5.3.0 | JP6.1 / r36.4 · JP6.2.2 / r36.5 · JP7.1 / r38.4 |
| 5.2.3 | JP5.1.2 / r35.4 · JP6.0 / r36.3 · JP6.1 / r36.4 · JP6.2.2 / r36.5 · JP7.1 / r38.4 |
| 5.2.0 – 5.2.2 | JP5.1.2 / r35.4 · JP6.0 / r36.3 · JP6.1 / r36.4 |
| 5.1.0 – 5.1.2 | JP5.1.2 / r35.4 · JP6.0 / r36.3 · JP6.1 / r36.4 |

JetPack / L4T / Ubuntu mapping (the Ubuntu base decides whether ROS 2 is
installed from APT or built from source):

| JetPack | L4T | Ubuntu base |
|---------|-----|-------------|
| JP5.1.2 | r35.4 | 20.04 |
| JP6.0 | r36.3 | 22.04 |
| JP6.1 | r36.4 | 22.04 |
| JP6.2.2 | r36.5 | 22.04 |
| JP7.1 | r38.4 | 24.04 |

## Tested configurations

The [Available configurations](#available-configurations) table above lists
what Docker Hub *publishes*; this one is a running record of what has actually
been **built and verified** (build success, and — where noted — a live camera
streaming real data). Untested combinations may still work; they just haven't
been checked yet.

| ROS 2 distro | Platform | Build path | ZED SDK | Result |
|--------------|----------|------------|---------|--------|
| Humble | Jetson AGX Orin, JP6.2.2 (native) | published | 5.4.x (latest) | ✅ Built + live ZED X camera @ ~28 Hz |
| Jazzy | Jetson AGX Orin, JP6.2.2 (native) | published | 5.4.x (latest) | ✅ Built + live ZED X camera @ ~29 Hz |
| Jazzy | Jetson AGX Orin, JP6.2.2 (native) | `--sdk-url` (custom) | 5.4.x | ✅ Built + live ZED X camera @ ~30 Hz |
| Lyrical | Jetson AGX Orin, JP6.2.2 (native) | — | — | ✅ Correctly rejected (needs Ubuntu 24.04/26.04) |
| Humble | x86_64 desktop → arm64 (QEMU) | published | 5.4.x (latest) | ✅ Built, ~15 min total |
| Jazzy | x86_64 desktop → arm64 (QEMU) | published | 5.4.x (latest) | ✅ Built, ~2h 15min total (from-source ROS 2 core) |
| Lyrical | x86_64 desktop → arm64 (QEMU) | — | — | ✅ Correctly rejected (same gate, arch-independent) |
| Jazzy | Jetson Thor, JP7.1 (native) | published | 5.4.x (latest) | ✅ Built, ~3 min total (APT mode; no camera attached to this unit, build-only) |
| Lyrical | Jetson Thor, JP7.1 (native) | published | 5.4.x (latest) | ✅ Built, ~4 min total (from-source ROS 2 core; no camera on this unit, build-only) |
| Humble | x86_64 desktop (native) | published | 5.4.x (latest) | ✅ Built, ~35 s total (build-only, no camera) |
| Jazzy | x86_64 desktop (native) | published | 5.4.x (latest) | ✅ Built, ~47 s total (build-only, no camera) |
| Lyrical | x86_64 desktop (native) | published | 5.4.x (latest) | ✅ Built, ~13 min total (from-source ROS 2 core; build-only, no camera) |
| Lyrical | x86_64 desktop (native) | published, Ubuntu 26.04 | — | ⏳ Not available yet — no `stereolabs/zed` image published for Ubuntu 26.04 |
| Jazzy | x86_64 desktop (native) | `--sdk-url` (local `.run` file) | 5.4.0 | ✅ Built, ~18 min total (from-source ROS 2 core; real ZED SDK `.run` installer, not a stand-in) |

## Build the Docker images

Checkout the tag or commit of the wrapper you want to build first:

```bash
git checkout <branch_or_tag>
```

### Desktop

```bash
./build_desktop.sh [--ros-distro <distro>] [--os ubuntu-<XX.XX>] \
                   [--sdk <X.Y.Z|latest>] [--cuda <XX.X>] [--sdk-url <URL>]
```

Defaults: `--ros-distro jazzy --os ubuntu-24.04 --sdk latest --cuda 12.8`.
These build scripts target the LTS distributions with published StereoLabs base
images — **Humble, Jazzy, Lyrical**. Desktop images always use the StereoLabs
`gl-devel` base variant. (The wrapper *packages* also build on Foxy and Rolling;
see [Supported ROS 2 distributions](../README.md#supported-ros-2-distributions).)

```bash
# Defaults: ROS 2 Jazzy, Ubuntu 24.04, latest ZED SDK, CUDA 12.8
./build_desktop.sh

# ROS 2 Humble on Ubuntu 22.04 with a specific ZED SDK
./build_desktop.sh --ros-distro humble --os ubuntu-22.04 --sdk 5.4.1
```

### Jetson

```bash
./build_jetson.sh [--ros-distro <distro>] [--os <jpX.Y.Z|l4t-rXX.X>] \
                  [--sdk <X.Y.Z|latest>] [--sdk-url <URL>]
```

Defaults: `--ros-distro humble --os jp6.2.2 --sdk latest`
(JetPack 6.2.2 = L4T r36.5, **Ubuntu 22.04** — whose supported distribution is Humble).
These build scripts target the LTS distributions with published StereoLabs base
images — **Humble, Jazzy, Lyrical**. Jetson images always use the StereoLabs
`devel` base variant. (The wrapper *packages* also build on Foxy and Rolling;
see [Supported ROS 2 distributions](../README.md#supported-ros-2-distributions).)

```bash
# Default: ROS 2 Humble on JetPack 6.2.2 (Ubuntu 22.04), latest ZED SDK
./build_jetson.sh

# ROS 2 Lyrical on JetPack 7.1 (Ubuntu 24.04) — built from source
./build_jetson.sh --ros-distro lyrical --os jp7.1.0
```

### Notes on inputs

* **`--sdk latest`** resolves to the newest ZED SDK that has an image for the
  chosen platform on Docker Hub. For example, on a desktop Ubuntu 24.04 base it
  resolves to the latest SDK (e.g. `5.4.1`), while on a JetPack 6.2.2 base it
  resolves to the latest SDK published for that platform (e.g. `5.3.1`, since
  newer SDKs may not yet ship a JP6.2.2 image).
* **`--sdk-url <URL>`** builds against a custom/unreleased ZED SDK `.run`
  installer instead of a published image — see
  [Build against a custom or unreleased ZED SDK](#build-against-a-custom-or-unreleased-zed-sdk).
* The base image tag is validated against Docker Hub before building. If a
  combination does not exist (e.g. a given ZED SDK for a given Ubuntu/CUDA/L4T
  version), the script stops with a clear message. See the available tags at
  [hub.docker.com/r/stereolabs/zed/tags](https://hub.docker.com/r/stereolabs/zed/tags).
* The **ROS 2 distribution must be supported on the base image Ubuntu release**,
  otherwise the script fails fast before building:

  | ROS 2 distro | Supported Ubuntu | Jetson (JetPack) |
  |--------------|------------------|------------------|
  | Humble | 22.04 | JP6 (L4T r36.x) |
  | Jazzy | 22.04 (Tier 3), 24.04 (Tier 1) | JP6 (L4T r36.x) or JP7 (L4T r38.x) |
  | Lyrical | 24.04 (Tier 3), 26.04 (Tier 1) | JP7 (L4T r38.x) |

  Lyrical on Ubuntu 22.04 (JetPack 6), or Humble on Ubuntu 24.04, are rejected
  up front. On a supported pairing with no published APT binaries (e.g.
  **Jazzy on 22.04** or **Lyrical on 24.04**, whose binaries are 26.04-only),
  ROS 2 is built from source automatically.

## Build against a custom or unreleased ZED SDK

By default the image is built on a `stereolabs/zed` base that already ships a
**published** ZED SDK version. To build against a ZED SDK that is **not**
published as a Docker image — a local build or a pre-release `.run` installer —
pass it to `--sdk-url`, either as a **URL** or as a path to a **local `.run`
file**:

```bash
# Desktop: plain CUDA base + custom SDK installer.
# --cuda must be a FULL version (X.Y.Z) to pick an nvcr.io/nvidia/cuda tag.
./build_desktop.sh --ros-distro humble --os ubuntu-22.04 --cuda 12.6.3 \
                   --sdk-url https://download.stereolabs.com/.../ZED_SDK_....run

# Desktop, from a local installer instead of a URL:
./build_desktop.sh --ros-distro humble --os ubuntu-22.04 --cuda 12.6.3 \
                   --sdk-url ~/Downloads/ZED_SDK_Ubuntu22_cuda12.6_v5.0.run

# Jetson: plain L4T JetPack base + custom SDK installer.
# --os must be a FULL L4T version (l4t-rX.Y.Z) to pick an nvcr.io/nvidia/l4t-jetpack tag.
./build_jetson.sh --ros-distro humble --os l4t-r36.4.0 \
                  --sdk-url https://download.stereolabs.com/.../ZED_SDK_....run
```

When `--sdk-url` is set:

* The base image is a **plain NVIDIA image** (`nvcr.io/nvidia/cuda` for desktop,
  `nvcr.io/nvidia/l4t-jetpack` for Jetson) instead of a `stereolabs/zed` image,
  and the SDK is installed inside the image by `scripts/install_zed_sdk.sh`
  (`silent skip_tools skip_cuda`).
* `--sdk` is ignored, and no Docker Hub tag lookup is done. The full version is
  required (`--cuda X.Y.Z` on desktop, `--os l4t-rX.Y.Z` on Jetson) because the
  NVIDIA base image tag is derived from it — make sure that tag exists on
  [nvcr.io](https://catalog.ngc.nvidia.com/) / Docker Hub.
* If `--sdk-url` is a **path to an existing local file**, it is staged into the
  build context (a Docker build cannot see the host filesystem otherwise) and
  copied in directly — no network fetch. Otherwise it's treated as a **URL**:
  it must return HTTP 200 (checked during the build) and point to a ZED SDK
  Linux `.run` installer matching the base CUDA/L4T and Ubuntu version.

## Cross compilation

You can build a Jetson (`arm64`) image from an `x86_64` desktop using QEMU
emulation. The steps below are the exact procedure used to test a ROS 2 Lyrical
image on a JetPack 7 / Ubuntu 24.04 base.

1. Register the QEMU `binfmt` handlers so the host can run `arm64` containers
   (run once per host; repeat after a reboot):

   ```bash
   docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
   ```

2. *(Optional)* Check that `arm64` emulation works — this should print `aarch64`:

   ```bash
   docker run --rm --platform linux/arm64 arm64v8/ubuntu:24.04 uname -m
   ```

3. Build the Jetson image, forcing the `arm64` platform. `build_jetson.sh` calls
   `docker build` without `--platform`, so on an `x86_64` host export
   `DOCKER_DEFAULT_PLATFORM` to make the (arm64-only) base image and every `RUN`
   step run under emulation:

   ```bash
   # ROS 2 Lyrical on JetPack 7.1 (L4T r38.4, Ubuntu 24.04)
   DOCKER_DEFAULT_PLATFORM=linux/arm64 \
     ./build_jetson.sh --ros-distro lyrical --os jp7.1.0
   ```

> :warning: **From-source + emulation is slow — measured, not a guess.**
> Lyrical has no APT binaries for Ubuntu 24.04 (it targets Ubuntu 26.04), so
> `scripts/install_ros2.sh` builds the whole ROS 2 core from source. Under QEMU this is
> **~2h 15min total** for a from-source distro (measured for Jazzy on a
> JetPack 6.2.2 / Ubuntu 22.04 base, on a modern x86_64 desktop: ~1h 50min for
> the ROS 2 core itself, ~25min for the wrapper). Lyrical on JetPack 7 compiles
> a similarly-sized ROS 2 core set, so expect a comparable multi-hour build.
> Building the *wrapper alone* on top of an already-built ROS 2 core (native,
> no emulation) measured ~7 minutes on a Jetson AGX Orin — a useful reference
> for how much of the total is emulation overhead vs. inherent compile time,
> though the ROS 2 core step itself wasn't independently re-timed natively.
> For a fast, binary (APT) Jetson build under QEMU, pick a distribution whose
> target Ubuntu matches the base — this is genuinely fast, measured at **~15
> minutes total** for Humble on JP6.2.2:
>
> ```bash
> DOCKER_DEFAULT_PLATFORM=linux/arm64 ./build_jetson.sh --ros-distro jazzy --os jp7.1.0
> ```
>
> Building natively on the Jetson itself needs neither QEMU nor
> `DOCKER_DEFAULT_PLATFORM`, and avoids this emulation overhead entirely.

## Run the Docker image

> :pushpin: **NOTE:** the entry point sets `ROS_DOMAIN_ID=0` (the ROS 2
> default). Change it in `scripts/ros_entrypoint.sh` before building, or use
> `export ROS_DOMAIN_ID=<new_value>` in each interactive session. See the
> [ROS 2 docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html).

### NVIDIA runtime

NVIDIA drivers must be accessible from the container to run the ZED SDK on the
GPU. You need:

* The `nvidia` container runtime installed, following [this guide](https://www.stereolabs.com/docs/docker/install-guide-linux/#nvidia-docker).
* A docker runtime environment with `--gpus all` or `-e NVIDIA_DRIVER_CAPABILITIES=all`.
* Docker privileged mode with `--privileged`.

### Network

* `--network=host`: remove network isolation between the container and the host.
* `--ipc=host`: use the host Inter-Process Communication namespace.
* `--pid=host`: use the host process ID namespace.

### Display (CUDA-based applications)

Share the host `DISPLAY` with `-e DISPLAY=$DISPLAY` and mount
`/tmp/.X11-unix/:/tmp/.X11-unix`.

### Volumes

* `/tmp/.X11-unix/:/tmp/.X11-unix` — X11 server communication for CUDA apps.
* `/usr/local/zed/settings:/usr/local/zed/settings` — to reuse downloaded camera calibration files ([guide](https://support.stereolabs.com/hc/en-us/articles/21614848880791-How-can-I-use-the-ZED-with-Docker-on-a-robot-with-no-internet-connection)).
* `/usr/local/zed/resources:/usr/local/zed/resources` — to persist the AI models (Object Detection, Skeleton Tracking, NEURAL depth) across runs. Use a different host folder per SDK version.
* `/dev:/dev` — to share the video and other devices.
* `/dev/shm:/dev/shm` — to use ROS 2 with shared memory.
* For GMSL2 cameras (ZED X, ZED X One) also add:
  * `/tmp:/tmp`
  * `/var/nvidia/nvcam/settings/:/var/nvidia/nvcam/settings/`
  * `/etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service`

### Start the container

Allow the container to access EGL display resources (once):

```bash
sudo xhost +si:localuser:root
```

#### USB3 cameras

```bash
docker run --runtime nvidia -it --privileged --network=host --ipc=host --pid=host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix/:/tmp/.X11-unix \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  -v /usr/local/zed/resources/:/usr/local/zed/resources/ \
  -v /usr/local/zed/settings/:/usr/local/zed/settings/ \
  <docker_image_tag>
```

#### GMSL cameras

```bash
docker run --runtime nvidia -it --privileged --network=host --ipc=host --pid=host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=$DISPLAY \
  -v /tmp:/tmp \
  -v /dev:/dev \
  -v /var/nvidia/nvcam/settings/:/var/nvidia/nvcam/settings/ \
  -v /etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service \
  -v /usr/local/zed/resources/:/usr/local/zed/resources/ \
  -v /usr/local/zed/settings/:/usr/local/zed/settings/ \
  <docker_image_tag>
```
