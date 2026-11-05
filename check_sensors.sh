#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo 'ROS 2 Jazzy 환경이 없습니다: /opt/ros/jazzy/setup.bash' >&2
    exit 2
fi
source /opt/ros/jazzy/setup.bash
if [[ -f "$workspace_dir/install/local_setup.bash" ]]; then
    source "$workspace_dir/install/local_setup.bash"
fi
exec /usr/bin/python3 "$workspace_dir/tools/check_sensors.py" "$@"
