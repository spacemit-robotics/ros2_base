#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/middleware/ros2/control/base/${SROBOTIS_TEST_NAME:-base-launch-contract}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/base_launch_contract.log"
check_launch_script="$script_dir/check_launch_contract.py"

mkdir -p "$log_dir"
exec > >(tee "$log_file") 2>&1

staging_root="${SROBOTIS_OUTPUT_STAGING:-}"
setup_script="$staging_root/setup.sh"

if [[ -z "$staging_root" ]]; then
    echo "[error] SROBOTIS_OUTPUT_STAGING is not set"
    exit 1
fi

if [[ ! -f "$setup_script" ]]; then
    echo "[error] setup script not found: $setup_script"
    exit 1
fi

set +u
# shellcheck source=/dev/null
source "$setup_script"
set -u

package_share_dir="$staging_root/share/base"
launch_file="$package_share_dir/launch/esos_base_control.launch.py"

echo "[info] module_root=$module_root"
echo "[info] artifact_dir=$artifact_dir"
echo "[info] staging_root=$staging_root"
echo "[info] package_share_dir=$package_share_dir"
echo "[info] launch_file=$launch_file"

[[ -f "$launch_file" ]]
[[ -f "$check_launch_script" ]]

python3 "$check_launch_script" "$launch_file"

echo "ALL TESTS PASSED: base-launch-contract"