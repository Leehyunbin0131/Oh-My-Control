#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "$workspace_dir/install/local_setup.bash" ]]; then
    source "$workspace_dir/install/local_setup.bash"
fi
exec /usr/bin/python3 "$workspace_dir/tools/hardware_tui.py" "$@"
