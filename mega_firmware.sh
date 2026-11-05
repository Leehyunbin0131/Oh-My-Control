#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
action="${1:-build}"
if [[ $# -gt 0 ]]; then shift; fi
mode=i2c
address=0x01
port=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_24235313336351619002-if00
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode|--address|--port)
            if [[ $# -lt 2 ]]; then echo "$1 값이 필요합니다." >&2; exit 2; fi
            case "$1" in
                --mode) mode="$2" ;;
                --address) address="$2" ;;
                --port) port="$2" ;;
            esac
            shift 2 ;;
        *) echo "알 수 없는 인자: $1" >&2; exit 2 ;;
    esac
done
if [[ "$action" != build && "$action" != upload ]]; then
    echo '사용: ./mega_firmware.sh build|upload [--mode encoder|i2c|serial1] [--address 0x01..0x08] [--port 경로]'
    exit 2
fi
case "$mode" in
    encoder) link=0 ;;
    i2c) link=1 ;;
    serial1) link=2 ;;
    *) echo 'mode는 encoder, i2c, serial1 중 하나입니다.' >&2; exit 2 ;;
esac
if [[ ! "$address" =~ ^(0x0?[1-8]|[1-8])$ ]]; then
    echo 'I2C 주소는 매뉴얼의 0x01..0x08 범위입니다.' >&2; exit 2
fi
cli="$workspace_dir/.tools/arduino/bin/arduino-cli"
config="$workspace_dir/.tools/arduino/arduino-cli.yaml"
if [[ ! -x "$cli" ]]; then
    echo '로컬 Arduino CLI가 없습니다. README의 펌웨어 도구 설치 항목을 확인하세요.' >&2
    exit 2
fi
build_dir="$workspace_dir/build/mega_motor_test_$mode"
"$cli" compile --fqbn arduino:avr:mega --config-file "$config" \
    --build-path "$build_dir" --build-property "build.extra_flags=-DMOTOR_LINK=$link -DMOTOR_I2C_ADDRESS=$address -DSERIAL_RX_BUFFER_SIZE=256" \
    "$workspace_dir/firmware/mega_motor_test"
if [[ "$action" == upload ]]; then
    if [[ ! -c "$port" || ! -w "$port" ]]; then echo "접근 가능한 시리얼 포트가 아닙니다: $port" >&2; exit 2; fi
    if command -v fuser >/dev/null && fuser "$port" 2>/dev/null; then
        echo '테스트 프로그램을 종료한 뒤 업로드하세요.' >&2; exit 2
    fi
    echo "메가 펌웨어 교체: mode=$mode / port=$port (모터 전원 OFF 상태에서 실행)"
    "$cli" upload --fqbn arduino:avr:mega --config-file "$config" --port "$port" --input-dir "$build_dir"
fi
