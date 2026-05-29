#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/middleware/ros2/control/base/${SROBOTIS_TEST_NAME:-base-invalid-rpmsg-device}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/base_invalid_rpmsg_device.log"
run_log_file="$log_dir/base_invalid_rpmsg_device.run.log"
ros_log_dir="$artifact_dir/ros_logs"
invalid_ctrl_dev="/tmp/base-test-missing-rpmsg-ctrl-${RANDOM}"
invalid_data_dev="/tmp/base-test-missing-rpmsg-data-${RANDOM}"
expected_exception_pattern="esos_base_control_node exception: Failed to create chassis device"
expected_init_failure_pattern="Failed to initialize RPMsg"

mkdir -p "$log_dir" "$ros_log_dir"
: >"$log_file"
: >"$run_log_file"

trap 'set +e
if [[ -n "${node_pid:-}" ]]; then
    kill -- "-${node_pid}" >/dev/null 2>&1 || kill "${node_pid}" >/dev/null 2>&1 || true
    wait "${node_pid}" >/dev/null 2>&1 || true
fi' EXIT

log() {
    echo "[base-invalid-rpmsg-device] $*" | tee -a "$log_file"
}

source_ros_setup() {
    set +u
    if [[ -f "${SROBOTIS_OUTPUT_STAGING:-}/setup.sh" ]]; then
        # shellcheck disable=SC1091
        source "${SROBOTIS_OUTPUT_STAGING}/setup.sh"
    elif [[ -f "${SROBOTIS_OUTPUT_STAGING:-}/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "${SROBOTIS_OUTPUT_STAGING}/setup.bash"
    elif [[ -f "${SROBOTIS_ROOT:-$(pwd)}/output/staging/setup.sh" ]]; then
        # shellcheck disable=SC1091
        source "${SROBOTIS_ROOT:-$(pwd)}/output/staging/setup.sh"
    elif [[ -f "${SROBOTIS_ROOT:-$(pwd)}/output/staging/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "${SROBOTIS_ROOT:-$(pwd)}/output/staging/setup.bash"
    elif [[ -f "${SROBOTIS_ROOT:-$(pwd)}/install/setup.sh" ]]; then
        # shellcheck disable=SC1091
        source "${SROBOTIS_ROOT:-$(pwd)}/install/setup.sh"
    elif [[ -f "${SROBOTIS_ROOT:-$(pwd)}/install/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "${SROBOTIS_ROOT:-$(pwd)}/install/setup.bash"
    elif [[ -f "/opt/ros/humble/setup.sh" ]]; then
        # shellcheck disable=SC1091
        source "/opt/ros/humble/setup.sh"
    elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "/opt/ros/humble/setup.bash"
    fi
    set -u
}

source_ros_setup

export ROS_LOG_DIR="$ros_log_dir"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-46}"
export PYTHONUNBUFFERED=1

if ! command -v ros2 >/dev/null 2>&1; then
    log "ERROR: ros2 command not found"
    exit 1
fi

if ! ros2 pkg prefix base >>"$log_file" 2>&1; then
    log "ERROR: ROS2 package base is not available in the sourced environment"
    exit 1
fi

log "Verifying esos_base_control_node fails fast for invalid RPMsg device paths"
log "module_root=$module_root"
log "artifact_dir=$artifact_dir"
log "invalid_ctrl_dev=$invalid_ctrl_dev"
log "invalid_data_dev=$invalid_data_dev"

setsid ros2 run base esos_base_control_node --ros-args \
    -p rpmsg_ctrl_dev:="$invalid_ctrl_dev" \
    -p rpmsg_data_dev:="$invalid_data_dev" \
    -p cfg_send_on_startup:=false \
    -p feedback_enable:=false \
    >>"$run_log_file" 2>&1 &
node_pid=$!

deadline=$((SECONDS + 30))
while [[ $SECONDS -lt $deadline ]]; do
    if ! kill -0 "$node_pid" >/dev/null 2>&1; then
        set +e
        wait "$node_pid"
        node_status=$?
        set -e
        node_pid=""

        if [[ $node_status -eq 0 ]]; then
            log "ERROR: esos_base_control_node unexpectedly succeeded with invalid RPMsg devices"
            tee -a "$log_file" <"$run_log_file" >&2
            exit 1
        fi

        if [[ $node_status -ne 1 ]]; then
            log "ERROR: expected esos_base_control_node to exit with status 1, got $node_status"
            tee -a "$log_file" <"$run_log_file" >&2
            exit 1
        fi

        tee -a "$log_file" <"$run_log_file"
        if grep -Fq "$expected_exception_pattern" "$run_log_file" && grep -Fq "$expected_init_failure_pattern" "$run_log_file"; then
            log "Observed expected RPMsg initialization failure."
            log "esos_base_control_node exited with status 1 as expected."
            log "ALL TESTS PASSED: base-invalid-rpmsg-device"
            exit 0
        fi

        log "ERROR: missing expected initialization failure details in esos_base_control_node output"
        break
    fi

    sleep 0.5
done

log "ERROR: esos_base_control_node did not exit with the expected initialization failure within 30s"
tee -a "$log_file" <"$run_log_file" >&2
exit 1