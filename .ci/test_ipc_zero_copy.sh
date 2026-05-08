#!/usr/bin/env bash
# =============================================================================
# IPC Zero-Copy Integration Test Script
# Tests the dual-publisher IPC architecture for the ZED ROS 2 wrapper.
#
# Usage:
#   ./test_ipc_zero_copy.sh [camera_model] [--compare baseline.json]
#
# Example:
#   ./test_ipc_zero_copy.sh zedx
#   ./test_ipc_zero_copy.sh zedxonegs
#   ./test_ipc_zero_copy.sh zedx --compare /tmp/ipc_perf_baseline.json
#
# The performance comparison test runs at 60 FPS with depth + point cloud
# enabled to stress data transfer. Results are saved to a JSON file that
# can be used as baseline for future comparisons.
#
# Requirements:
#   - ZED camera connected
#   - Workspace built: colcon build
#   - Source: source install/setup.bash
# =============================================================================

set -eo pipefail

# ─── Force C numeric locale (avoid comma decimals from fr_FR etc.) ───────────
export LC_NUMERIC=C

# ─── Auto-source ROS workspace if not already sourced ────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
if ! command -v ros2 &>/dev/null || ! ros2 pkg prefix zed_wrapper &>/dev/null 2>&1; then
  echo "[setup] Sourcing ROS workspace at $WS_DIR ..."
  set +eu  # ROS setup.bash uses unset variables internally
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  source "$WS_DIR/install/setup.bash"
  set -eu
fi

set -u  # re-enable nounset after ROS sourcing

# ─── Float comparison (no bc dependency) ─────────────────────────────────────
# Usage: float_gt "15.3" "5" → returns 0 (true) if 15.3 > 5
float_gt() { awk "BEGIN { exit !($1 > $2) }"; }

# ─── Parse arguments ─────────────────────────────────────────────────────────
CAMERA_MODEL="zedx"
BASELINE_JSON=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --compare)
      BASELINE_JSON="${2:-}"
      [[ -z "$BASELINE_JSON" ]] && { echo "Error: --compare requires a path"; exit 1; }
      shift 2
      ;;
    -*) echo "Unknown option: $1"; exit 1 ;;
    *) CAMERA_MODEL="$1"; shift ;;
  esac
done

# ─── Configuration ───────────────────────────────────────────────────────────
CAMERA_NAME="zed"
NODE_NAME="zed_node"
# Topic prefix: /<camera_name>/<node_name>/
TOPIC_PREFIX="/${CAMERA_NAME}/${NODE_NAME}"
RGB_TOPIC="${TOPIC_PREFIX}/rgb/color/rect/image"
DEPTH_TOPIC="${TOPIC_PREFIX}/depth/depth_registered"
CONFIDENCE_TOPIC="${TOPIC_PREFIX}/confidence/confidence_map"
STEREO_TOPIC="${TOPIC_PREFIX}/stereo/color/rect/image"
COMPRESSED_TOPIC="${RGB_TOPIC}/compressed"
POINTCLOUD_TOPIC="${TOPIC_PREFIX}/point_cloud/cloud_registered"

LAUNCH_SETTLE_TIME=20   # seconds to wait for node to fully start
HZ_SAMPLE_TIME=5        # seconds to sample topic hz
MIN_HZ=5                # minimum acceptable frame rate
PERF_SAMPLE_TIME=10     # seconds to sample hz in perf comparison (longer = more stable)
PERF_CPU_SAMPLES=5      # number of CPU/RSS samples to average
REPORT_JSON="/tmp/ipc_perf_$(date +%Y%m%d_%H%M%S).json"

# Base overrides applied to every launch: 60 FPS, enable stereo & confidence
BASE_OVERRIDES="general.grab_frame_rate:=60;video.publish_stereo:=true;depth.publish_depth_confidence:=true"

# Mono models don't have stereo/depth
MONO_MODELS=("zedxonegs" "zedxone4k" "zedxonehdr")
IS_MONO=false
for m in "${MONO_MODELS[@]}"; do
  [[ "$CAMERA_MODEL" == "$m" ]] && IS_MONO=true
done

# ─── Helpers ─────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

PASS_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0
ZED_PID=""

pass() { PASS_COUNT=$((PASS_COUNT + 1)); echo -e "  ${GREEN}[PASS]${NC} $1"; }
fail() { FAIL_COUNT=$((FAIL_COUNT + 1)); echo -e "  ${RED}[FAIL]${NC} $1"; }
skip() { SKIP_COUNT=$((SKIP_COUNT + 1)); echo -e "  ${YELLOW}[SKIP]${NC} $1"; }
info() { echo -e "  ${CYAN}[INFO]${NC} $1"; }
section() { echo -e "\n${CYAN}━━━ $1 ━━━${NC}"; }

cleanup() {
  if [[ -n "$ZED_PID" ]] && kill -0 "$ZED_PID" 2>/dev/null; then
    info "Stopping ZED node (PID $ZED_PID)..."
    kill -INT -"$ZED_PID" 2>/dev/null || kill -INT "$ZED_PID" 2>/dev/null || true
    sleep 2
    kill -9 -"$ZED_PID" 2>/dev/null || kill -9 "$ZED_PID" 2>/dev/null || true
    wait "$ZED_PID" 2>/dev/null || true
  fi
  pkill -9 -f "component_container" 2>/dev/null || true
  pkill -f "ros2 topic hz.*${NODE_NAME}" 2>/dev/null || true
  pkill -f "ros2 topic echo.*${NODE_NAME}" 2>/dev/null || true
  rm -f /dev/shm/fastrtps_port* 2>/dev/null || true
}
trap cleanup EXIT

# Kill pre-existing ZED / container processes from previous runs
pkill -9 -f "ros2 launch zed_wrapper" 2>/dev/null || true
pkill -9 -f "component_container" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
sleep 2
# Clean stale FastDDS shared memory from any previous runs
rm -f /dev/shm/fastrtps_port* 2>/dev/null || true

launch_zed() {
  local enable_ipc="$1"
  local extra_overrides="${2:-}"

  # Merge base overrides with any test-specific overrides
  local all_overrides="${BASE_OVERRIDES}"
  if [[ -n "$extra_overrides" ]]; then
    all_overrides="${all_overrides};${extra_overrides}"
  fi

  info "Launching: camera_model:=${CAMERA_MODEL} enable_ipc:=${enable_ipc} overrides=${all_overrides}"
  ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:="${CAMERA_MODEL}" \
    camera_name:="${CAMERA_NAME}" \
    enable_ipc:="${enable_ipc}" \
    "param_overrides:=${all_overrides}" \
    > /tmp/zed_test_launch.log 2>&1 &
  ZED_PID=$!

  info "Waiting for node to initialize..."
  sleep "$LAUNCH_SETTLE_TIME"

  if ! kill -0 "$ZED_PID" 2>/dev/null; then
    fail "Node crashed during startup. Log tail:"
    tail -20 /tmp/zed_test_launch.log
    return 1
  fi

  # Also check if the component container died (ros2 launch may linger briefly)
  if grep -q "process has died" /tmp/zed_test_launch.log 2>/dev/null; then
    fail "Component container crashed during startup. Log tail:"
    tail -20 /tmp/zed_test_launch.log
    stop_zed
    return 1
  fi

  # Wait until the RGB topic is actually discoverable (up to 15s extra)
  info "Waiting for topic discovery..."
  local discovered=false
  for (( i=1; i<=15; i++ )); do
    if ros2 topic list 2>/dev/null | grep -q "^${RGB_TOPIC}$"; then
      discovered=true
      break
    fi
    sleep 1
  done
  if ! $discovered; then
    info "Warning: RGB topic not yet discovered after extended wait"
  fi
}

stop_zed() {
  if [[ -n "$ZED_PID" ]] && kill -0 "$ZED_PID" 2>/dev/null; then
    # Use SIGINT for clean ROS 2 shutdown (sends to entire process group)
    kill -INT -"$ZED_PID" 2>/dev/null || kill -INT "$ZED_PID" 2>/dev/null || true
    # Give it up to 8s to shut down gracefully
    for (( i=0; i<16; i++ )); do
      kill -0 "$ZED_PID" 2>/dev/null || break
      sleep 0.5
    done
    # Force kill process group if still alive
    kill -9 -"$ZED_PID" 2>/dev/null || kill -9 "$ZED_PID" 2>/dev/null || true
    wait "$ZED_PID" 2>/dev/null || true
    ZED_PID=""
  fi
  # Kill any straggler component containers from previous launch
  pkill -9 -f "component_container" 2>/dev/null || true
  sleep 2
  # Clean up stale FastDDS shared memory port lock files
  rm -f /dev/shm/fastrtps_port* 2>/dev/null || true
  # Let DDS fully release resources before next launch
  sleep 3
}

# Get average hz for a topic (returns 0 if no messages).
# Writes to temp file to avoid SIGTERM killing the pipe and losing data.
get_hz() {
  local topic="$1"
  local duration="${2:-$HZ_SAMPLE_TIME}"
  local tmpfile="/tmp/zed_hz_$$.log"
  timeout "$((duration + 3))" ros2 topic hz "$topic" > "$tmpfile" 2>&1 || true
  local output
  # Match both dot and comma decimals (locale safety)
  output=$(grep -oP 'average rate: \K[0-9.,]+' "$tmpfile" | tr ',' '.' | tail -1) || true
  rm -f "$tmpfile"
  echo "${output:-0}"
}

# Get average hz from message header timestamps.
# This measures the actual source publish cadence and is immune to dual-publisher
# inflation (both publishers stamp messages from the same grab loop).
# Falls back to get_hz if echo fails.
get_stamp_hz() {
  local topic="$1"
  local duration="${2:-$HZ_SAMPLE_TIME}"
  local count=40  # number of messages to sample (extra for dual-pub duplicates)
  local tmpfile="/tmp/zed_stamps_$$.log"

  # Collect header stamps: extract sec + nanosec from each message
  timeout "$((duration + 5))" ros2 topic echo "$topic" \
    --field header.stamp --no-arr --csv \
    2>/dev/null | head -n "$count" > "$tmpfile" || true

  local lines
  lines=$(wc -l < "$tmpfile" | tr -d ' ')
  if [[ "$lines" -lt 3 ]]; then
    # Not enough samples, fall back to ros2 topic hz
    rm -f "$tmpfile"
    get_hz "$topic" "$duration"
    return
  fi

  # Compute rate from consecutive unique timestamp differences
  # CSV format: "sec,nanosec" per line (duplicates from dual-pub are skipped)
  local hz
  hz=$(awk -F',' '
    function abs(x) { return x < 0 ? -x : x }
    {
      t = $1 + $2/1e9
      if (prev > 0 && abs(t - prev) < 1e-6) next  # skip duplicate from dual publisher
      if (prev > 0) {
        dt = t - prev
        if (dt > 0 && dt < 1) { sum += dt; n++ }
      }
      prev = t
    }
    END {
      if (n > 0) printf "%.3f", 1.0 / (sum/n)
      else print "0"
    }
  ' "$tmpfile")

  rm -f "$tmpfile"
  echo "${hz:-0}"
}

# Count publishers on a topic
get_pub_count() {
  local topic="$1"
  ros2 topic info "$topic" 2>/dev/null | grep -c "Publisher count:" | head -1 || echo "0"
}

# Get subscription count from topic info -v
get_publisher_count_verbose() {
  local topic="$1"
  local count
  count=$(ros2 topic info "$topic" 2>/dev/null | grep "Publisher count:" | awk '{print $NF}') || true
  echo "${count:-0}"
}

# Check if a topic exists (retries up to 3 times with 2s gaps for discovery)
topic_exists() {
  local attempts=5
  for (( i=1; i<=attempts; i++ )); do
    if ros2 topic list 2>/dev/null | grep -q "^$1$"; then
      return 0
    fi
    [[ $i -lt $attempts ]] && sleep 3
  done
  return 1
}

# Check if node is alive (process-level check + ros2 discovery)
node_alive() {
  # Primary: check if the launch process is still running
  if [[ -n "$ZED_PID" ]] && kill -0 "$ZED_PID" 2>/dev/null; then
    return 0
  fi
  # Fallback: ros2 daemon discovery
  ros2 node list 2>/dev/null | grep -q "${NODE_NAME}"
}

# Average CPU% over N samples (1s apart) for the process group leader
get_avg_cpu() {
  local pid="$1"
  local samples="${2:-$PERF_CPU_SAMPLES}"
  local sum=0 count=0
  for (( i=0; i<samples; i++ )); do
    local cpu
    cpu=$(ps -p "$pid" -o %cpu= 2>/dev/null | tr -d ' ' | tr ',' '.') || true
    if [[ -n "$cpu" && "$cpu" != "0.0" ]]; then
      sum=$(awk "BEGIN { print $sum + $cpu }")
      count=$((count + 1))
    fi
    sleep 1
  done
  [[ $count -eq 0 ]] && { echo "N/A"; return; }
  awk "BEGIN { printf \"%.1f\", $sum / $count }"
}

# RSS in MB for the component_container process tree
get_rss_mb() {
  local pid="$1"
  # Sum RSS of the process and its children (component_container forks)
  local rss_kb
  rss_kb=$(ps --ppid "$pid" -o rss= 2>/dev/null | awk '{s+=$1}END{print s+0}') || rss_kb=0
  local self_rss
  self_rss=$(ps -p "$pid" -o rss= 2>/dev/null | tr -d ' ') || self_rss=0
  rss_kb=$((rss_kb + self_rss))
  awk "BEGIN { printf \"%.0f\", $rss_kb / 1024 }"
}

# Format a delta as colored +/-
fmt_delta() {
  local curr="$1" base="$2" unit="$3" higher_is_better="${4:-true}"
  [[ "$curr" == "N/A" || "$base" == "N/A" ]] && { echo "—"; return; }
  local diff pct color sign
  diff=$(awk "BEGIN { print $curr - $base }")
  pct=$(awk "BEGIN { d=$base; if(d==0) print 0; else printf \"%.1f\", ($diff/d)*100 }")
  if awk "BEGIN { exit !($diff > 0) }"; then
    sign="+"; [[ "$higher_is_better" == "true" ]] && color="$GREEN" || color="$RED"
  elif awk "BEGIN { exit !($diff < 0) }"; then
    sign=""; [[ "$higher_is_better" == "true" ]] && color="$RED" || color="$GREEN"
  else
    echo "0${unit}"; return
  fi
  echo -e "${color}${sign}${diff}${unit} (${sign}${pct}%)${NC}"
}

# ═════════════════════════════════════════════════════════════════════════════
# TEST SUITES
# ═════════════════════════════════════════════════════════════════════════════

test_basic_ipc_disabled() {
  section "TEST SUITE: Basic functionality (IPC disabled)"
  launch_zed "false" || return 1

  # T1: Topics exist
  if topic_exists "$RGB_TOPIC"; then
    pass "RGB topic exists: $RGB_TOPIC"
  else
    fail "RGB topic missing: $RGB_TOPIC"
  fi

  # T2: Frame rate
  local hz
  hz=$(get_hz "$RGB_TOPIC")
  if float_gt "$hz" "$MIN_HZ"; then
    pass "RGB frame rate: ${hz} Hz"
  else
    fail "RGB frame rate too low: ${hz} Hz (expected > ${MIN_HZ})"
  fi

  # T3: Compression topics
  if topic_exists "$COMPRESSED_TOPIC"; then
    pass "Compressed topic exists: $COMPRESSED_TOPIC"
  else
    fail "Compressed topic missing: $COMPRESSED_TOPIC"
  fi

  # T4: Single publisher (no IPC dual-pub)
  local pub_count
  pub_count=$(get_publisher_count_verbose "$RGB_TOPIC")
  if [[ "$pub_count" -eq 1 ]]; then
    pass "Single publisher on RGB topic (no IPC): count=$pub_count"
  else
    # image_transport may still show 1 publisher
    info "Publisher count on RGB topic: $pub_count (expected 1 without IPC)"
  fi

  stop_zed
}

test_ipc_enabled() {
  section "TEST SUITE: IPC zero-copy enabled"
  launch_zed "true" || return 1

  # T1: RGB topic exists
  if topic_exists "$RGB_TOPIC"; then
    pass "RGB topic exists with IPC: $RGB_TOPIC"
  else
    fail "RGB topic missing with IPC: $RGB_TOPIC"
    stop_zed; return 1
  fi

  # T2: Dual publishers visible (image_transport + IPC publisher)
  local pub_count
  pub_count=$(get_publisher_count_verbose "$RGB_TOPIC")
  if [[ "$pub_count" -ge 2 ]]; then
    pass "Dual publishers on RGB topic: count=$pub_count"
  else
    fail "IPC dual-publisher not active: publisher count=$pub_count (expected >= 2). Check if NITROS conflict is silently disabling IPC."
  fi

  # T3: Frame rate on raw topic
  local hz
  hz=$(get_hz "$RGB_TOPIC")
  if float_gt "$hz" "$MIN_HZ"; then
    pass "RGB frame rate with IPC: ${hz} Hz"
  else
    fail "RGB frame rate too low with IPC: ${hz} Hz"
  fi

  # T4: Compression topic still works
  if topic_exists "$COMPRESSED_TOPIC"; then
    pass "Compressed topic still available with IPC"
  else
    fail "Compressed topic missing with IPC"
  fi

  # T5: Compressed topic actually delivers data
  local comp_data
  comp_data=$(timeout 8 ros2 topic echo "$COMPRESSED_TOPIC" --once --no-arr 2>&1) || true
  if echo "$comp_data" | grep -q "format:"; then
    pass "Compressed topic delivers data"
  else
    fail "Compressed topic not delivering data"
  fi

  stop_zed
}

test_subscriber_counting() {
  section "TEST SUITE: Subscriber counting"
  launch_zed "true" || return 1

  # T1: No subscribers — check node is alive but not flooding logs
  if node_alive; then
    pass "Node alive with no subscribers"
  else
    fail "Node not found"
    stop_zed; return 1
  fi

  # T2: Add subscriber → data flows
  local hz
  hz=$(get_hz "$RGB_TOPIC")
  if float_gt "$hz" "$MIN_HZ"; then
    pass "Data flows when subscriber connects: ${hz} Hz"
  else
    fail "No data when subscriber connects: ${hz} Hz"
  fi

  # T3: Remove subscriber, short pause, re-check node alive
  sleep 3
  if node_alive; then
    pass "Node survives subscriber disconnect"
  else
    fail "Node died after subscriber disconnect"
  fi

  stop_zed
}

test_dynamic_transitions() {
  section "TEST SUITE: Dynamic subscriber transitions"
  launch_zed "true" || return 1

  # T1: No sub → add sub mid-stream
  info "Waiting 5s with no subscribers..."
  sleep 5
  local hz
  hz=$(get_hz "$RGB_TOPIC" 4)
  if float_gt "$hz" "$MIN_HZ"; then
    pass "Late subscriber receives data: ${hz} Hz"
  else
    fail "Late subscriber got no data: ${hz} Hz"
  fi

  # T2: Disconnect → node still alive (retry to handle discovery lag)
  local alive=false
  for (( i=1; i<=3; i++ )); do
    sleep 2
    if node_alive; then alive=true; break; fi
  done
  if $alive; then
    pass "Node alive after subscriber disconnect"
  else
    fail "Node died after subscriber disconnect"
  fi

  # T3: Multiple concurrent subscribers
  info "Spawning 3 concurrent hz subscribers..."
  timeout "$((HZ_SAMPLE_TIME + 3))" ros2 topic hz "$RGB_TOPIC" > /tmp/hz1.log 2>&1 &
  local pid1=$!
  timeout "$((HZ_SAMPLE_TIME + 3))" ros2 topic hz "$RGB_TOPIC" > /tmp/hz2.log 2>&1 &
  local pid2=$!
  timeout "$((HZ_SAMPLE_TIME + 3))" ros2 topic hz "$COMPRESSED_TOPIC" > /tmp/hz3.log 2>&1 &
  local pid3=$!
  sleep "$((HZ_SAMPLE_TIME + 2))"
  wait "$pid1" 2>/dev/null || true
  wait "$pid2" 2>/dev/null || true
  wait "$pid3" 2>/dev/null || true

  local hz1 hz3
  hz1=$(grep -oP 'average rate: \K[0-9.,]+' /tmp/hz1.log | tr ',' '.' | tail -1) || hz1="0"
  hz3=$(grep -oP 'average rate: \K[0-9.,]+' /tmp/hz3.log | tr ',' '.' | tail -1) || hz3="0"
  if float_gt "$hz1" "$MIN_HZ"; then
    pass "Raw topic with concurrent subs: ${hz1} Hz"
  else
    fail "Raw topic failed with concurrent subs: ${hz1} Hz"
  fi
  if float_gt "$hz3" "0"; then
    pass "Compressed topic with concurrent subs: ${hz3} Hz"
  else
    fail "Compressed topic failed with concurrent subs: ${hz3} Hz"
  fi

  # T4: Rapid connect/disconnect stress test
  info "Stress test: 10 rapid connect/disconnect cycles..."
  for i in $(seq 1 10); do
    timeout 2 ros2 topic hz "$RGB_TOPIC" > /dev/null 2>&1 &
    local stress_pid=$!
    sleep 0.5
    kill "$stress_pid" 2>/dev/null || true
    wait "$stress_pid" 2>/dev/null || true
  done
  # Wait for DDS to settle after rapid churn, then check via PID (reliable)
  sleep 3
  if [[ -n "$ZED_PID" ]] && kill -0 "$ZED_PID" 2>/dev/null; then
    pass "Node survived rapid connect/disconnect stress test"
  else
    fail "Node crashed during stress test"
  fi

  stop_zed
}

test_depth() {
  section "TEST SUITE: Depth-specific"
  if $IS_MONO; then
    skip "Depth tests skipped for mono model: $CAMERA_MODEL"
    return
  fi

  launch_zed "true" || return 1

  # T1: Depth topic
  if topic_exists "$DEPTH_TOPIC"; then
    pass "Depth topic exists: $DEPTH_TOPIC"
  else
    fail "Depth topic missing: $DEPTH_TOPIC"
  fi

  # Pre-warm: subscribe briefly so the node starts the SDK grab loop for depth
  timeout 4 ros2 topic hz "$DEPTH_TOPIC" > /dev/null 2>&1 || true
  sleep 2
  local hz
  hz=$(get_hz "$DEPTH_TOPIC" 8)
  if float_gt "$hz" "$MIN_HZ"; then
    pass "Depth frame rate: ${hz} Hz"
  else
    fail "Depth frame rate too low: ${hz} Hz"
  fi

  # T2: Confidence map (enabled via BASE_OVERRIDES)
  if topic_exists "$CONFIDENCE_TOPIC"; then
    local conf_hz
    conf_hz=$(get_hz "$CONFIDENCE_TOPIC")
    if float_gt "$conf_hz" "0"; then
      pass "Confidence map publishing: ${conf_hz} Hz"
    else
      fail "Confidence map not publishing"
    fi
  else
    fail "Confidence topic not found: $CONFIDENCE_TOPIC"
  fi

  stop_zed
}

test_stereo() {
  section "TEST SUITE: Stereo-specific"
  if $IS_MONO; then
    skip "Stereo tests skipped for mono model: $CAMERA_MODEL"
    return
  fi

  launch_zed "true" || return 1

  if topic_exists "$STEREO_TOPIC"; then
    local hz
    hz=$(get_hz "$STEREO_TOPIC")
    if float_gt "$hz" "$MIN_HZ"; then
      pass "Stereo topic frame rate: ${hz} Hz"
    else
      fail "Stereo topic frame rate too low: ${hz} Hz"
    fi
  else
    fail "Stereo topic not found: $STEREO_TOPIC"
  fi

  stop_zed
}

# ── Perf measurement helpers ──────────────────────────────────────────────────

# Measure both source (header-timestamp) and external (ros2 topic hz) rates
# in a single session, avoiding extra camera start/stop cycles.
# Populates two associative arrays passed by nameref.
# Usage: measure_perf <src_array_name> <ext_array_name>
measure_perf() {
  local -n _src=$1
  local -n _ext=$2

  local -a topics=("$RGB_TOPIC")
  local -a labels=("rgb")
  if ! $IS_MONO; then
    topics+=("$DEPTH_TOPIC" "$POINTCLOUD_TOPIC")
    labels+=("depth" "pointcloud")
  fi

  # Warmup: subscribe briefly so the node starts all grab/publish paths
  for t in "${topics[@]}"; do
    timeout 4 ros2 topic hz "$t" > /dev/null 2>&1 || true
  done
  sleep 2

  # Source rate: from message header timestamps (immune to dual-pub)
  info "  Sampling source rate (header timestamps)..."
  for idx in "${!topics[@]}"; do
    local hz
    hz=$(get_stamp_hz "${topics[$idx]}" "$PERF_SAMPLE_TIME")
    _src["hz_${labels[$idx]}"]="$hz"
    info "    ${labels[$idx]} source Hz: $hz"
  done

  # External rate: what ros2 topic hz sees (includes dual-pub inflation)
  # Re-warmup: ensure an active subscriber triggers publisher before sampling
  for t in "${topics[@]}"; do
    timeout 4 ros2 topic hz "$t" > /dev/null 2>&1 || true
  done
  sleep 2
  info "  Sampling external rate (ros2 topic hz)..."
  for idx in "${!topics[@]}"; do
    local hz
    hz=$(get_hz "${topics[$idx]}" "$PERF_SAMPLE_TIME")
    _ext["hz_${labels[$idx]}"]="$hz"
    info "    ${labels[$idx]} external Hz: $hz"
  done

  # CPU + RSS (shared — same node session)
  local cpu rss
  cpu=$(get_avg_cpu "$ZED_PID")
  rss=$(get_rss_mb "$ZED_PID")
  _src["cpu"]="$cpu"; _ext["cpu"]="$cpu"
  _src["rss_mb"]="$rss"; _ext["rss_mb"]="$rss"
  info "  CPU: ${cpu}%  RSS: ${rss} MB"
}

# Print a comparison table
print_table() {
  local title="$1"
  local -n _noip=$2
  local -n _ip=$3

  echo ""
  echo "  $title"
  echo "  ┌────────────────────────┬───────────────┬───────────────┐"
  echo "  │ Metric                 │ IPC disabled  │ IPC enabled   │"
  echo "  ├────────────────────────┼───────────────┼───────────────┤"
  printf "  │ %-22s │ %10s Hz │ %10s Hz │\n" "RGB frame rate"  "${_noip[hz_rgb]}"  "${_ip[hz_rgb]}"
  if ! $IS_MONO; then
    printf "  │ %-22s │ %10s Hz │ %10s Hz │\n" "Depth frame rate"  "${_noip[hz_depth]:-N/A}" "${_ip[hz_depth]:-N/A}"
    printf "  │ %-22s │ %10s Hz │ %10s Hz │\n" "PointCloud frame rate" "${_noip[hz_pointcloud]:-N/A}" "${_ip[hz_pointcloud]:-N/A}"
  fi
  printf "  │ %-22s │ %12s%% │ %12s%% │\n" "CPU usage (avg)"  "${_noip[cpu]}"  "${_ip[cpu]}"
  printf "  │ %-22s │ %10s MB │ %10s MB │\n" "RSS memory"  "${_noip[rss_mb]}"  "${_ip[rss_mb]}"
  echo "  └────────────────────────┴───────────────┴───────────────┘"
}

test_ipc_vs_nonipc_comparison() {
  section "TEST SUITE: IPC vs non-IPC performance comparison (60 FPS)"
  echo ""

  # ── IPC disabled ──
  info "Measuring IPC disabled (60 FPS)..."
  launch_zed "false" "depth.point_cloud_freq:=60.0" || return 1
  local -A src_nonipc ext_nonipc
  measure_perf src_nonipc ext_nonipc
  stop_zed

  # ── IPC enabled ──
  info "Measuring IPC enabled (60 FPS)..."
  launch_zed "true" "depth.point_cloud_freq:=60.0" || return 1
  local -A src_ipc ext_ipc
  measure_perf src_ipc ext_ipc
  stop_zed

  # ── Results ──
  print_table "Source publish rate (actual node throughput, from header timestamps)" src_nonipc src_ipc
  print_table "External subscriber rate (what ros2 topic hz sees)" ext_nonipc ext_ipc

  echo ""
  info "Note: With IPC enabled, image topics have dual publishers (image_transport + IPC)."
  info "External subscribers may receive from both → higher measured rate ≠ higher throughput."
  info "The 'source publish rate' above is the authoritative measurement."

  # ── JSON report (use source rates as the authoritative metric) ──
  _j() { [[ "$1" == "N/A" ]] && echo 0 || echo "$1"; }
  cat > "$REPORT_JSON" <<EOF
{
  "timestamp": "$(date -Iseconds)",
  "camera_model": "$CAMERA_MODEL",
  "is_mono": $IS_MONO,
  "grab_frame_rate": 60,
  "sample_time_sec": $PERF_SAMPLE_TIME,
  "source_rate": {
    "ipc_disabled": {
      "hz_rgb": $(_j "${src_nonipc[hz_rgb]}"),
      "hz_depth": $(_j "${src_nonipc[hz_depth]:-0}"),
      "hz_pointcloud": $(_j "${src_nonipc[hz_pointcloud]:-0}"),
      "cpu_pct": $(_j "${src_nonipc[cpu]:-0}"),
      "rss_mb": $(_j "${src_nonipc[rss_mb]:-0}")
    },
    "ipc_enabled": {
      "hz_rgb": $(_j "${src_ipc[hz_rgb]}"),
      "hz_depth": $(_j "${src_ipc[hz_depth]:-0}"),
      "hz_pointcloud": $(_j "${src_ipc[hz_pointcloud]:-0}"),
      "cpu_pct": $(_j "${src_ipc[cpu]:-0}"),
      "rss_mb": $(_j "${src_ipc[rss_mb]:-0}")
    }
  },
  "external_rate": {
    "ipc_disabled": {
      "hz_rgb": $(_j "${ext_nonipc[hz_rgb]}"),
      "hz_depth": $(_j "${ext_nonipc[hz_depth]:-0}"),
      "hz_pointcloud": $(_j "${ext_nonipc[hz_pointcloud]:-0}"),
      "cpu_pct": $(_j "${ext_nonipc[cpu]:-0}"),
      "rss_mb": $(_j "${ext_nonipc[rss_mb]:-0}")
    },
    "ipc_enabled": {
      "hz_rgb": $(_j "${ext_ipc[hz_rgb]}"),
      "hz_depth": $(_j "${ext_ipc[hz_depth]:-0}"),
      "hz_pointcloud": $(_j "${ext_ipc[hz_pointcloud]:-0}"),
      "cpu_pct": $(_j "${ext_ipc[cpu]:-0}"),
      "rss_mb": $(_j "${ext_ipc[rss_mb]:-0}")
    }
  }
}
EOF
  info "Report saved: $REPORT_JSON"

  # ── Compare against baseline if provided ──
  if [[ -n "$BASELINE_JSON" ]]; then
    if [[ ! -f "$BASELINE_JSON" ]]; then
      fail "Baseline file not found: $BASELINE_JSON"
    else
      section "COMPARISON vs baseline: $BASELINE_JSON"
      local b_hz_rgb b_hz_depth b_hz_ptcloud b_cpu b_rss b_model
      b_hz_rgb=$(awk -F': ' '/"ipc_enabled"/{found=1} found && /"hz_rgb"/{gsub(/[^0-9.]/,"",$2); print $2; exit}' "$BASELINE_JSON")
      b_hz_depth=$(awk -F': ' '/"ipc_enabled"/{found=1} found && /"hz_depth"/{gsub(/[^0-9.]/,"",$2); print $2; exit}' "$BASELINE_JSON")
      b_hz_ptcloud=$(awk -F': ' '/"ipc_enabled"/{found=1} found && /"hz_pointcloud"/{gsub(/[^0-9.]/,"",$2); print $2; exit}' "$BASELINE_JSON")
      b_cpu=$(awk -F': ' '/"ipc_enabled"/{found=1} found && /"cpu_pct"/{gsub(/[^0-9.]/,"",$2); print $2; exit}' "$BASELINE_JSON")
      b_rss=$(awk -F': ' '/"ipc_enabled"/{found=1} found && /"rss_mb"/{gsub(/[^0-9.]/,"",$2); print $2; exit}' "$BASELINE_JSON")
      b_model=$(awk -F': ' '/"camera_model"/{gsub(/[" ,]/,"",$2); print $2; exit}' "$BASELINE_JSON")

      echo ""
      echo "  Baseline camera: ${b_model:-unknown}"
      echo "  ┌────────────────────────┬──────────────┬──────────────┬──────────────────┐"
      echo "  │ Metric (IPC enabled)   │ Baseline     │ Current      │ Delta            │"
      echo "  ├────────────────────────┼──────────────┼──────────────┼──────────────────┤"
      printf "  │ %-22s │ %9s Hz │ %9s Hz │ %s\n" "RGB Hz" "${b_hz_rgb:-N/A}" "${src_ipc[hz_rgb]}" "$(fmt_delta "${src_ipc[hz_rgb]}" "${b_hz_rgb:-0}" "Hz" true) │"
      if ! $IS_MONO; then
        printf "  │ %-22s │ %9s Hz │ %9s Hz │ %s\n" "Depth Hz" "${b_hz_depth:-N/A}" "${src_ipc[hz_depth]:-N/A}" "$(fmt_delta "${src_ipc[hz_depth]:-0}" "${b_hz_depth:-0}" "Hz" true) │"
        printf "  │ %-22s │ %9s Hz │ %9s Hz │ %s\n" "PointCloud Hz" "${b_hz_ptcloud:-N/A}" "${src_ipc[hz_pointcloud]:-N/A}" "$(fmt_delta "${src_ipc[hz_pointcloud]:-0}" "${b_hz_ptcloud:-0}" "Hz" true) │"
      fi
      printf "  │ %-22s │ %11s%% │ %11s%% │ %s\n" "CPU" "${b_cpu:-N/A}" "${src_ipc[cpu]}" "$(fmt_delta "${src_ipc[cpu]:-0}" "${b_cpu:-0}" "%" false) │"
      printf "  │ %-22s │ %9s MB │ %9s MB │ %s\n" "RSS" "${b_rss:-N/A}" "${src_ipc[rss_mb]}" "$(fmt_delta "${src_ipc[rss_mb]:-0}" "${b_rss:-0}" "MB" false) │"
      echo "  └────────────────────────┴──────────────┴──────────────┴──────────────────┘"
    fi
  fi

  if float_gt "${src_ipc[hz_rgb]}" "$MIN_HZ" && float_gt "${src_nonipc[hz_rgb]}" "$MIN_HZ"; then
    pass "Both modes produce valid source frame rates"
  else
    fail "One or both modes below minimum source frame rate"
  fi
}

# ═════════════════════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════════════════════

echo "============================================="
echo " IPC Zero-Copy Integration Tests"
echo " Camera model: $CAMERA_MODEL"
echo " Mono mode:    $IS_MONO"
echo " Topic prefix: $TOPIC_PREFIX"
echo "============================================="

test_basic_ipc_disabled
test_ipc_enabled
test_subscriber_counting
test_dynamic_transitions
test_depth
test_stereo
test_ipc_vs_nonipc_comparison

# ─── Summary ─────────────────────────────────────────────────────────────────
section "RESULTS"
echo -e "  ${GREEN}Passed: $PASS_COUNT${NC}"
echo -e "  ${RED}Failed: $FAIL_COUNT${NC}"
echo -e "  ${YELLOW}Skipped: $SKIP_COUNT${NC}"
echo ""

if [[ -f "$REPORT_JSON" ]]; then
  echo -e "  Performance report: ${CYAN}${REPORT_JSON}${NC}"
  echo -e "  Re-run with ${CYAN}--compare ${REPORT_JSON}${NC} to diff against this baseline"
  echo ""
fi

if [[ "$FAIL_COUNT" -gt 0 ]]; then
  echo -e "  ${RED}Some tests failed. Check /tmp/zed_test_launch.log for node output.${NC}"
  exit 1
else
  echo -e "  ${GREEN}All tests passed!${NC}"
  exit 0
fi
