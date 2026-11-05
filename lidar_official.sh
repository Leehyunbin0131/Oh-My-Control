#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
lidar_port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_3915548a17a0164cb877f2ab2ade5994-if00-port0
for argument in "$@"; do
    case "$argument" in
        serial_port:=*) lidar_port="${argument#serial_port:=}" ;;
    esac
done
if [[ ! -r "$lidar_port" || ! -w "$lidar_port" ]]; then
    echo "라이다 장치가 없거나 접근 권한이 없습니다: $lidar_port" >&2
    exit 1
fi
if command -v fuser >/dev/null && fuser "$lidar_port" >/dev/null 2>&1; then
    echo '라이다 포트를 다른 프로그램이 사용 중입니다. 대시보드는 q로 종료한 뒤 실행하세요.' >&2
    exit 1
fi
if [[ ! -x "$workspace_dir/install/rplidar_ros/lib/rplidar_ros/rplidar_node" ]]; then
    echo '공식 드라이버를 먼저 빌드하세요: source /opt/ros/jazzy/setup.bash && colcon build --packages-select rplidar_ros --symlink-install --allow-overriding rplidar_ros' >&2
    exit 2
fi
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/local_setup.bash"
exec ros2 launch rplidar_ros rplidar_a2m12_launch.py \
    serial_port:="$lidar_port" \
    serial_baudrate:=256000 "$@"
