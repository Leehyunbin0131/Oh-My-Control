# ROS 2 로봇 하드웨어 테스트

SSH에서 센서·모터·엔코더를 확인하는 ROS 2 Jazzy 워크스페이스입니다.
최종 정리: **2026-11-05**. 현재 소스는 **Mega v8 / `signed_pwm_v1`**이며, 오른쪽 모터 전원선 극성을 바꾼 배선을 유지합니다.
기본 목표는 **72 RPM**, 실측 보정한 주행 상한은 **95 RPM**, MCU의 절대 목표·과속 보호 기준은 **100 RPM**입니다. PID 계산은 Mega에서 실제 1000 Hz로 실행합니다.

**무부하 PID 재보정·오토튠과 주행·0 RPM 제동 검증을 완료했습니다.**
오토튠 36개 실험 묶음과 최종 18조건을 통과했습니다. 최대 평균 속도 오차는 0.39 RPM이며 `config/motor_pid.json`에 검증된 계수를 저장했습니다.
위치 유지는 `hold.enabled=true`입니다. 정지 후 위치를 저장하고 위치 오차에 직접 비례하는 출력과 속도 피드백으로 버팁니다. 기존 PWM 120 설정이 약하다는 사용자 시험 결과를 반영해 **위치 보정 출력을 강화하고 상한을 350으로 높였습니다.** 개선판을 Mega에 업로드했고, 2026-11-05 사용자가 손으로 바퀴를 돌려보며 강한 유지력을 확인했습니다. 실제 주행 중 정지 거리·방향 변화는 별도 확인 항목입니다.

기존 주행·제동 검증 근거: [재보정·오토튠 결과](reports/pid_recalibration_v8_20261105_124619/summary.json). 바퀴를 띄운 상태에서 검증했으며 지면 주행·위치 유지 검증은 별도입니다.
변경 내용과 검증 기준: [모터 제어 명세서](docs/motor_control_spec.md).
제어 주기 결정과 무구동 통신 비교: [제어·통신 주기 기준](docs/control_timing.md). 현재 **PID 1000 Hz·목표 송신 50 Hz·상태 수신 20 Hz를 유지**합니다. Nav2는 ROS 연동 후 SLAM 동시 실행 상태에서 **20·30·50 Hz를 비교**해 결정합니다. 아직 ROS 모터 어댑터·오도메트리·Nav2 연동은 구현하지 않았습니다.

## Git 저장소 사용

처음 내려받을 때는 `git clone --recurse-submodules <저장소주소>`를 사용합니다.
이미 내려받았다면 아래 명령으로 공식 라이다 소스를 가져옵니다.

```bash
git submodule update --init --recursive
```

`src/rplidar_ros`는 검증 당시 커밋으로 고정한 서브모듈입니다. 소스·설정·문서는 Git으로 관리하고, `reports/`, `build/`, `install/`, `log/`, `.tools/`와 전체 백업은 로봇에 보관합니다. 문서의 `reports/` 링크는 로봇의 원본 워크스페이스에서 확인할 수 있습니다. 새 기기에서는 ROS·센서 라이브러리와 Arduino 도구 설치 및 빌드가 별도로 필요합니다.

## 바로 실행

로봇에 SSH로 접속한 뒤 실행합니다. 아래 프로그램은 하나씩 사용하세요.

```bash
cd ~/ros2_dev_ws
./hardware_tui.sh --motors       # 센서·모터 상태 대시보드; 모터 구동 없음
./check_sensors.sh              # 외장 IMU, 라이다, 카메라·내장 IMU 검사
./teleop.sh                     # 기본 72 RPM PID 조종, 한 번 입력하면 방향 유지
```

기본 의존성은 이 기기에 설치되어 있습니다. Python 도구는 바로 실행하며, IMU 보정용 카메라 수집기는 필요할 때 자동 빌드합니다.
SSH 명령에서 바로 실행할 때는 `ssh -t hyunlee@로봇주소 'cd ~/ros2_dev_ws && ./hardware_tui.sh'`처럼 터미널을 할당합니다.

## 키보드 조종과 정지

방향키를 **한 번 누르면 주행을 유지**합니다. 키를 놓아도 정지하지 않습니다.

| 키 | 동작 |
| --- | --- |
| `q` / `w` / `e` | 왼쪽 곡선 전진 / 직진 / 오른쪽 곡선 전진 |
| `a` / `d` | 제자리 좌회전 / 우회전 — PID 모드 최대 50 RPM |
| `z` / `x` / `c` | 왼쪽 / 직선 / 오른쪽 후진 |
| `s` / `Space` | 정지 |
| `+` / `-` | PWM 10씩 조절; PID 모드에서는 2 RPM씩 조절 |
| `Ctrl+C` / `Ctrl+D` | 정지 후 종료 |

기본 `./teleop.sh`는 엔코더 피드백으로 72 RPM을 유지하며, `./teleop.sh --rpm 50`처럼 목표를 지정할 수 있습니다.
제자리 회전(`a`/`d`, 좌우 화살표)은 각 바퀴 목표를 최대 **50 RPM**으로 제한합니다. 선택한 주행 속도가 50 RPM보다 낮으면 그 속도를 사용합니다. 전·후진과 곡선 주행은 기존 속도 설정을 사용하며, 회전 중 `+`를 눌러도 회전 목표는 50 RPM을 넘지 않습니다.
`./teleop.sh --pwm 180`은 PID 없이 출력을 직접 지정하는 진단용 모드이며, 실제 180 RPM을 뜻하지 않습니다.
선택 사항인 `--key-timeout 0.6`은 입력이 0.6초 동안 없으면 정지합니다.

PID 모드의 `s`/`Space`는 **목표 0 RPM → 피드백 제동 → 실제 정지 확인 → 위치 유지**로 처리합니다.
실제로 멈춘 위치를 한 번 저장하며, 반복되는 정지 명령은 기준 위치를 바꾸지 않습니다. 유지 전용 위치 P와 속도 피드백이 제한된 출력을 계산합니다. 새 방향 명령을 받으면 즉시 주행 제어로 전환합니다.
속도·방향 변경은 주행 중 갱신하며, 기존 200 ms 대기와 별도 `BRAKE` 루틴은 제거했습니다.
제동 중 PID가 반대 극성 PWM을 계산할 수 있습니다. 전류·토크를 직접 제어하는 방식은 아닙니다.

`STOP`은 제어 해제·출력 0을 뜻하며, 직접 PWM 진단과 오류 정지에서 사용합니다.
목표는 50 Hz로 갱신하며 유효기간은 300 ms입니다. `PING`만으로 이동 목표가 연장되지 않습니다.
위치 유지도 유효한 0 목표가 갱신되는 동안 계속됩니다. 10초가 지났다는 이유로 해제하지 않으며, 명령 만료·오류·종료 시에는 출력을 해제합니다.
정상 종료는 최대 1초 동안 피드백 정지를 시도한 뒤 출력을 해제합니다. 오류·통신 만료에서는 바로 출력 해제를 요청합니다.
`STOP`은 기존 오류를 지우지 않습니다. 원인을 해소한 뒤 `./teleop.sh --clear-fault`로 명시적으로 해제합니다.
USB 연결로 Mega가 재부팅되면 오류 기록도 초기화되지만, 새 구동 명령 없이 움직이지 않습니다.

현재 기기에는 유지력 개선판 업로드를 마쳤고, 새 기능 식별과 출력 해제를 확인했습니다. 구동·유지력 시험은 사용자가 직접 진행했습니다. 다른 Mega에는 모터 전원을 끄고 `./mega_firmware.sh upload`로 올립니다. 기존 기능 마스크 15 펌웨어에서는 강화 설정을 시작하기 전에 업로드 필요 오류로 차단합니다.
사용자가 직접 `./teleop.sh --rpm 30`으로 짧게 주행 후 `s`를 누르고, 양쪽 상태가 `HOLD`로 바뀌는지 확인합니다. 10초 이상 기다려도 `HOLD`가 유지되는지와 화면의 위치 오차·PWM, 재출발 시 반응을 확인합니다. 처음 실행하자마자 `s`만 누르면 출력 해제 상태를 유지하므로 주행 후 시험합니다.
실내 평탄한 바닥과 지속적인 외력이 거의 없는 조건을 대상으로 합니다. 직접 위치 P는 18 PWM/count, 속도 Kp 8·Ki 0, 위치 데드밴드는 1 count입니다. 한도는 위치 오차 100 counts, 유지 PWM 350이며 출력 포화가 1초 지속되면 오류로 해제합니다. 이때 `hold_limit (fault 10)`으로 원인을 표시합니다. 상세 설정은 [모터 제어 명세서](docs/motor_control_spec.md)에 있습니다.

직접 PWM 진단도 최대 650으로 제한합니다. 측정 속도가 100 RPM을 초과한 상태가 100 ms 지속되면 과속 오류로 해제합니다.
100 RPM은 명령 상한이며 순간적인 실제 속도 초과까지 막는 보장은 아닙니다. 실측에서 100 RPM 목표는 오른쪽 과속 보호가 동작해, 검증된 주행 범위를 95 RPM까지로 정했습니다. 지면 정지 거리·방향 변화는 별도 검증이 필요합니다.

## 센서·엔코더 검사

```bash
./check_sensors.sh --sensors imu
./check_sensors.sh --sensors lidar --lidar-baud 256000
./check_sensors.sh --sensors camera --duration 10
./check_sensors.sh --sensors motors --motor-test encoders --duration 20
```

`camera`는 RGB·깊이 영상과 내장 가속도계·자이로를 함께 검사합니다.
`encoders` 검사에서는 측정 중 두 바퀴를 각각 손으로 돌립니다.
종료 코드는 전체 통과 `0`, 실패 `1`, 인자 오류 `2`, 사용자 중단 `130`입니다.

대시보드는 로봇 자세, 카메라 문자 영상, 라이다 분포와 엔코더를 표시합니다.

| 대시보드 키 | 동작 |
| --- | --- |
| `1` / `2`, `z` | 외장 / 카메라 IMU 선택, 현재 자세를 표시 기준으로 설정 |
| `c`, `+` / `-` | 컬러·깊이 전환, 라이다 표시 반경 조절 |
| `s`, `r`, `q` | 상태 저장, 드라이버 재시작, 종료 |
| `m`, `Space` / `x` | 구동 검사 허용 시 시작, 검사 중단·정지 |

카메라 IMU의 자세는 가속도·자이로로 추정한 상대 자세이며 yaw는 드리프트할 수 있습니다.
화면이 좁으면 터미널을 120열 × 42행 정도로 넓히고, 색이 맞지 않으면 `--color none`을 사용합니다.

SLAM 준비용 정지 자이로 보정은 `./calibrate_imu.sh --stationary`로 두 IMU를 함께 측정합니다.
보정값은 `config/imu_calibration.json`에 저장하며 현재 주행·센서 표시에 자동 적용하지 않습니다. 재실행 방법, 결과와 향후 TF·엔코더·시간 보정 항목은 [IMU 보정 문서](docs/imu_calibration.md)에 정리했습니다. 대시보드의 `z`는 화면 기준 설정이며 센서 캘리브레이션이 아닙니다.

모터 구동 검사는 **바퀴를 띄운 상태**에서 실행합니다.

```bash
./hardware_tui.sh --motors --motor-test drive   # 화면에서 m으로 시작
./check_sensors.sh --sensors motors --motor-test drive
./check_sensors.sh --sensors motors --motor-test bench --motor-bench-seconds 30
```

일반 모터 검사와 시간 제한 종료는 출력 0으로 정지합니다. 키보드 조종의 빠른 제동과 구분합니다.

## 현재 하드웨어와 방향

| 장치 | 설정 |
| --- | --- |
| 외장 IMU HFI-A9 | USB 시리얼 921600 bps |
| RPLIDAR A2M12 | USB 시리얼 **256000 bps**, 공식 `rplidar_ros` |
| RealSense | RGB·깊이 640×480 @ 15 fps, 내장 IMU 사용 |
| Arduino Mega 2560 | USB 115200 bps, 엔코더 샘플링 20 kHz |
| NT-S-DCDM1210 | I²C 400 kHz, 주소 `0x01`, DIP 모두 ON, Mega SDA20/SCL21 |

USB 장치는 `/dev/serial/by-id/` 경로를 사용합니다. 공통 장치 인자는 `./check_sensors.sh --help`에서 확인합니다.
라이다·카메라 검사는 ROS 도메인 77을 사용합니다. 카메라는 이 기기의 `~/.local/opt/librealsense-2.58.1-rsusb/lib`를 사용합니다.

전방은 RealSense가 보는 방향이며, 좌우는 로봇 뒤에서 전방을 볼 때 기준입니다.

| 실제 바퀴 | 드라이버 | 엔코더 A/B | 전진 명령 부호 | 전진 시 원시 엔코더 부호 |
| --- | --- | --- | --- | --- |
| 왼쪽 | L / M2 | D50 / D51 | − | − |
| 오른쪽 | R / M1 | D40 / D41 | − | + |

현재 방향의 기준 파일은 [config/drive_calibration.json](config/drive_calibration.json)입니다.
호스트가 실제 전진을 양수로 변환합니다. 펌웨어의 PID 피드백 부호는 드라이버 `[L, R]` 순서로 `[+1, -1]`입니다.
드라이버 매뉴얼은 [NT-S-DCDM1210.pdf](NT-S-DCDM1210.pdf)에 있습니다.

## PID 보정과 오토튠

PID 계산은 Mega Timer1에서 **1000 Hz / 1 ms**로 실행합니다. I²C 출력 전송 주기는 이 계산 주기와 별개입니다.
이번 오토튠에서 PID 계산 주기 누락은 0회였으며, 활성 구간의 I²C 출력 갱신은 평균 약 580 Hz였습니다.
속도 환산은 명목 엔코더 해상도 2652 counts/rev를 사용하며, 실제 한 바퀴당 카운트는 별도 보정하지 않았습니다.

[config/motor_pid.json](config/motor_pid.json)은 v8 제어 모델과 일치하는 schema 2 설정입니다.
현재 `validated: true`, `autotune.completed: true`이며 주행·제동 검증을 통과했습니다.
손으로 확인한 유지력은 `hold_validation.status=user_confirmed_manual_hold`로 기록했습니다. 정량 유지력과 지면 제동 검증은 별도이므로 `validation_status.holding`과 `ground`는 아직 `false`입니다. RPM은 명목 엔코더 해상도에 따른 값입니다.

모터 전원을 켜고 바퀴를 띄운 뒤 보정합니다. **아래 앞의 두 명령은 실제 모터를 구동합니다.**

```bash
./tune_motors.sh              # 새 주행 PI 보정 + 0 RPM 제동 검사, 기본 상한 95 RPM
./tune_motors.sh --autotune   # 검증된 v8 설정에서 주행 Kp/Ki/Kd 탐색 + 제동 재검사
./teleop.sh --rpm 50         # 50 RPM 조종
```

과거 배선·v7 검증을 새 검증으로 재사용하지 않습니다. 후보가 통과한 경우에만 현재 설정을 교체합니다.
현재 PWM 상한 650에서 100 RPM 목표는 왼쪽 출력 포화와 오른쪽 과속 보호가 관찰되어 95 RPM을 채택했습니다. `--max-rpm`으로 보정 범위를 지정할 수 있으며, 기준에 미달하면 기존 설정을 교체하지 않습니다.
절대 목표 상한은 100 RPM이며 초과 인자는 거부합니다. 주행 계수 자동 탐색은 위치 유지 PI를 보정하거나 위치 유지의 실기 검증 상태를 변경하지 않습니다.

오류 원인을 해소한 경우 보정 명령에 `--clear-fault`를 추가할 수 있습니다.
지면 정지 거리·방향 틀어짐과 위치 유지 시험은 별도로 진행하고, 측정 전에는 검증 완료로 표시하지 않습니다.

통신 오류 수정에서는 수신 버퍼를 256바이트로 늘리고, 상태 정보 생성 중 I²C 호출을 제거했습니다.
진단 프레임을 나누어 명령 수신을 먼저 처리합니다. `motors_commands.jsonl`에 송신 명령을, `motors_serial.log`에 원본 응답을 저장합니다.
수신 버퍼 사용량·처리 지연·CRC/프레임/순서 오류는 `R` 진단으로 확인합니다. PID 계산과 I²C 출력 갱신 속도는 각각 기록합니다.

## 야외 SSH용 핫스팟

```bash
sudo ./hotspot_on.sh             # 부팅 자동 시작 켜기 + 5초 후 현재 연결을 핫스팟으로 전환
sudo ./hotspot_on.sh --boot-only # 현재 연결을 유지하고 다음 부팅부터 핫스팟 사용
sudo ./hotspot_off.sh            # 자동 시작 끄기; 현재 Wi-Fi/핫스팟 연결은 유지
sudo ./hotspot_on.sh --status    # 현재 Wi-Fi 이름·비밀번호·SSH 주소 확인
```

핫스팟 이름은 **ORINBOT**, Wi-Fi 비밀번호는 **12345678**, 접속 명령은 `ssh hyunlee@10.42.0.1`입니다.
Wi-Fi 비밀번호와 SSH 계정 인증은 별개입니다. 핫스팟으로 전환되면 노트북도 ORINBOT에 연결한 뒤 SSH로 재접속합니다.
자동 시작을 끄면 다음 부팅에는 저장된 일반 Wi-Fi 연결을 시도합니다.
로그는 `sudo journalctl -u robot-hotspot.service -u robot-hotspot-activate.service -n 50 --no-pager`로 확인합니다.

## 빌드와 검증

현재 기기의 펌웨어 도구 설치 위치는 `.tools/arduino/`이며, CLI와 AVR 코어가 설치되어 있습니다.
CLI는 `bin/`, AVR 코어는 `arduino-cli.yaml`이 가리키는 `data/`를 사용합니다.

```bash
./mega_firmware.sh build         # 현재 I²C 펌웨어 빌드만 수행
./mega_firmware.sh upload        # 모터 전원을 끄고, 장치 사용 프로그램 종료 후 업로드
/usr/bin/python3 -m unittest discover -s tests -v
```

자동 테스트는 가상 시리얼·모터 모델과 C++ 코어를 사용하며 실제 모터를 구동하지 않습니다.
2026-11-05 유지력 개선·IMU 보정을 포함한 전체 **102개 테스트**를 통과했습니다. [검증·업로드 기록](reports/motor_hold_stiffness_20261105/summary.json)
공식 라이다만 실행하려면 대시보드를 종료한 뒤 `./lidar_official.sh`를 사용합니다.
라이다를 다시 빌드할 때는 다음을 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rplidar_ros --symlink-install --allow-overriding rplidar_ros
```

## 파일과 기록

| 경로 | 용도 |
| --- | --- |
| 루트의 `*.sh` | 센서 검사, 조종, 보정, 펌웨어, 핫스팟 실행 진입점 |
| `tools/`, `firmware/mega_motor_test/`, `tests/` | 현재 Python 도구, Mega 소스, 자동 테스트 |
| `config/` | 방향·PID 설정과 적용 전 IMU 자이로 보정값 |
| `src/rplidar_ros/`, `resources/` | 공식 라이다 소스, 표시용 URDF·라이선스 |
| `reports/` | 센서·모터 실측 JSON, 원본 로그와 과거 실험 기록 |
| `build/`, `install/`, `log/`, `.tools/` | 현재 빌드 결과, ROS 실행 환경, 빌드 로그, 로컬 도구 |

현재 검증 결과는 [보고서 안내](reports/README.md)에 모았습니다. 과거 보고서는 당시 버전의 이력이며 현재 성능으로 간주하지 않습니다.
전체 작업 백업은 워크스페이스 밖 `~/robot_backups/`에 저장합니다. 압축 파일과 SHA-256 체크섬을 함께 보관하며, 빌드 환경과 원본 보고서도 포함합니다.

## 다음 개발

1. 바퀴 반지름·윤거·실제 엔코더 CPR을 측정하고 ROS 2 모터 어댑터와 엔코더 오도메트리를 연결합니다.
2. 센서·SLAM·Nav2 동시 실행 상태에서 Nav2 20·30·50 Hz를 비교합니다. 명령 지연, 데이터 나이, 주기 누락과 실제 경로 추종 오차를 기록합니다.
3. 사용자가 지면 정지 거리·방향 변화와 위치 유지를 시험합니다. 결과를 기록하기 전까지 위치 유지의 실기 검증 상태는 미완료로 둡니다.
4. 메가의 실제 실행 여유가 부족할 때만 PID 500 Hz를 별도 비교합니다. 시간 계산과 필터를 수정하고 다시 검증하며, Nav2 주기와 동시에 변경하지 않습니다.

하드웨어·모델 출처: [Oh-My-MobileRobot](https://github.com/Leehyunbin0131/Oh-My-MobileRobot).
라이다 출처: [Slamtec/rplidar_ros](https://github.com/Slamtec/rplidar_ros/tree/ros2), 로컬 참조 커밋 `24cc9b6dea97e045bda1408eaa867ce730fd3fc3`.
모델의 라이선스·고지는 [LICENSE.robot-model](resources/LICENSE.robot-model), [NOTICE.robot-model](resources/NOTICE.robot-model)에 보존했습니다.
