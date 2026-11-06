<p align="center">
  <img src="docs/images/robot-lineup.png" alt="기본 베이스, 적재함형, 로봇암형, 디스플레이형 모바일 로봇의 CAD 렌더링" width="960">
</p>

<h1 align="center">Oh My Control</h1>

<p align="center">
  <em>모바일 로봇 하드웨어 브링업 · Mega 1 kHz 모터 제어 · 구동/센서 캘리브레이션</em>
</p>

<p align="center">
  <a href="#completed-robot">완성 사진</a> ·
  <a href="#robot-video">주행 영상</a> ·
  <a href="#hardware-overview">하드웨어 구성</a> ·
  <a href="#quick-start">바로 실행</a> ·
  <a href="#teleop-and-stop">키보드 조종</a> ·
  <a href="#sensor-check">센서 검사</a> ·
  <a href="#calibration">구동 · IMU 보정</a> ·
  <a href="docs/motor_control_spec.md">모터 제어 명세</a> ·
  <a href="docs/control_timing.md">제어 주기</a> ·
  <a href="docs/imu_calibration.md">IMU 보정</a>
</p>

---

**Oh My Control**은 [`Oh-My-MobileRobot`](https://github.com/Leehyunbin0131/Oh-My-MobileRobot)에서 제작한 모바일 로봇 하드웨어에 **저수준 구동 제어와 센서 보정·진단 계층**을 구성한 프로젝트입니다.

상위 ROS 2 자율주행 스택을 연결하기에 앞서, 모터·엔코더·IMU·라이다·깊이 카메라가 물리적 제어 명령과 좌표계 기준대로 동작하는지 독립적으로 검증하고 보정하는 도구와 펌웨어를 제공합니다.

| 단계 | 저장소 | 역할 |
| :--- | :--- | :--- |
| **기구 · 모델링** | [`Oh-My-MobileRobot`](https://github.com/Leehyunbin0131/Oh-My-MobileRobot) | 로봇 프레임·외장 설계, 출력용 STL·도면, CAD 기반 URDF 모델 |
| **제어 · 보정** | **`Oh-My-Control`** | **Mega 1 kHz 속도·위치 유지 제어 펌웨어, 모터·IMU 보정, 센서 통합 진단** |
| **자율주행** | ROS 2 Sim & Bringup | 보정 설정 및 시리얼 프로토콜(`OMR1`) 기반 ROS 2 드라이버와 SLAM·Nav2 주행 |

<a id="completed-robot"></a>

## 실제 완성 모습

<p align="center">
  <img src="docs/images/photos/completed-front.jpg" alt="상판에 라이다와 전면에 카메라가 장착된 완성 로봇의 전면 사진" width="600"><br>
  <sub>완성된 기본 베이스의 전면</sub>
</p>

| 전면 사시 | 측면·상판 |
| :---: | :---: |
| ![완성 로봇의 전면 사시 사진](docs/images/photos/completed-angle.jpg) | ![완성 로봇의 측면과 상판 사진](docs/images/photos/completed-side.jpg) |

<a id="robot-video"></a>

### 주행 영상

<table align="center"><tr><td>

https://github.com/user-attachments/assets/bb98cf7c-ee6c-49f0-b458-9154ca7bda5a

</td></tr></table>

<p align="center">
  <sub>원본 영상 파일: <a href="docs/videos/robot-driving.mp4"><code>docs/videos/robot-driving.mp4</code></a></sub>
</p>

<a id="hardware-overview"></a>

## 하드웨어 구성과 제어 규격

전방은 RealSense 카메라가 바라보는 방향이며, 좌우는 로봇 뒤에서 전방을 바라볼 때의 기준입니다.

| 구분 | 장치 | 연결 · 동작 규격 |
| --- | --- | --- |
| 메인 연산 | Jetson Orin Nano Super | 센서 수집, 시리얼 명령 송수신(목표 50 Hz / 상태 20 Hz), 터미널 진단·보정 |
| 모터 제어 MCU | Arduino Mega 2560 | USB 시리얼 115200 bps (`OMR1`), 1 kHz 속도·위치 유지 제어, 20 kHz 엔코더 샘플링 |
| 모터 드라이버 | NT-S-DCDM1210 | I²C 400 kHz (주소 `0x01`, Mega SDA20/SCL21), 부호 있는 PWM 구동 |
| 구동 바퀴 · 엔코더 | DFRobot KIT0085 × 2 | 명목 2652 counts/rev, 기본 주행 목표 72 RPM (운용 상한 95 RPM, 보호 상한 100 RPM) |
| 거리 · 깊이 센서 | RPLIDAR A2M12, RealSense D435i | 라이다 256000 bps (`rplidar_ros`), RGB·깊이 640 × 480 @ 15 fps 및 내장 IMU |
| 외장 IMU | HFI-A9 | USB 시리얼 921600 bps, 3축 자이로·가속도·자력계 및 자세 출력 |

### 바퀴 채널과 부호 매핑

| 실제 바퀴 | 드라이버 채널 | 엔코더 핀 (A / B) | 전진 명령 부호 | 전진 시 원시 엔코더 부호 |
| --- | --- | --- | --- | --- |
| 왼쪽 | L / M2 | D50 / D51 | − | − |
| 오른쪽 | R / M1 | D40 / D41 | − | + |

오른쪽 모터는 전원선 극성을 반전한 배선을 사용합니다. 좌우 채널과 전진 부호 기준은 [`config/drive_calibration.json`](config/drive_calibration.json)에 정의되어 있으며, 호스트 도구가 실제 전진 방향을 양수로 변환합니다. 드라이버 세부 사양은 [`NT-S-DCDM1210.pdf`](NT-S-DCDM1210.pdf)를 참고합니다.

<a id="quick-start"></a>

## 준비와 실행

저장소를 처음 내려받을 때는 서브모듈(`src/rplidar_ros`)을 함께 초기화합니다.

```bash
git clone --recurse-submodules https://github.com/Leehyunbin0131/Oh-My-Control.git
cd Oh-My-Control
```

이미 클론한 환경이라면 다음 명령으로 서브모듈을 가져옵니다.

```bash
git submodule update --init --recursive
```

로봇에 접속한 뒤 아래 실행 스크립트를 하나씩 사용합니다.

```bash
./hardware_tui.sh --motors       # 센서·모터 상태 통합 대시보드 (모터 비구동)
./check_sensors.sh               # 외장 IMU, 라이다, 카메라·내장 IMU 일괄 점검
./teleop.sh                      # 키보드 PID 주행 조종 (기본 72 RPM)
```

원격 SSH 명령으로 대시보드를 바로 열 때는 `ssh -t hyunlee@<로봇주소> 'cd ~/ros2_dev_ws && ./hardware_tui.sh'`와 같이 의사 터미널(`-t`)을 할당합니다.

<a id="teleop-and-stop"></a>

## 키보드 조종과 정지 제어

방향키를 한 번 누르면 해당 주행 목표를 유지하며, 정지 키(`s` 또는 `Space`)를 누르면 정지합니다.

| 키 | 동작 |
| --- | --- |
| `q` / `w` / `e` | 좌측 곡선 전진 / 직진 / 우측 곡선 전진 |
| `a` / `d` | 제자리 좌회전 / 우회전 (PID 모드 최대 50 RPM 제한) |
| `z` / `x` / `c` | 좌측 곡선 후진 / 직선 후진 / 우측 곡선 후진 |
| `s` / `Space` | 정지 및 위치 유지(`HOLD`) |
| `+` / `-` | 목표 속도 2 RPM씩 조절 (직접 PWM 모드에서는 10씩 조절) |
| `Ctrl+C` / `Ctrl+D` | 정지 후 프로그램 종료 |

* **속도 지정 및 진단 모드**: `./teleop.sh --rpm 50`으로 목표 RPM을 지정할 수 있으며, `./teleop.sh --pwm 180`은 피드백 제어 없이 PWM을 직접 인가하는 진단 모드입니다. `--key-timeout 0.6`을 추가하면 입력이 없을 때 자동으로 정지합니다.
* **정지 및 위치 유지(`HOLD`)**: PID 모드에서 정지 명령(`SET_SPEED 0 0`)을 내리면 제어기가 현재 엔코더 위치를 기준점으로 저장하고 위치 비례항(18 PWM/count)과 유지 전용 속도 피드백(Kp 8, PWM 상한 350)으로 바퀴 위치를 유지합니다.
* **안전 제한 및 오류 해제**: 목표 속도 유효기간(Lease)은 300 ms이며, 통신이 끊기거나 위치 이탈 한도(100 counts)·출력 포화 시간(1000 ms)·과속 한도(100 RPM, 100 ms)를 초과하면 출력을 즉시 해제하고 오류 상태를 기록합니다. 원인을 해소한 뒤 `./teleop.sh --clear-fault`로 오류를 해제합니다.

<a id="sensor-check"></a>

## 센서 · 엔코더 점검과 대시보드

개별 센서와 엔코더는 아래 명령으로 검사합니다.

```bash
./check_sensors.sh --sensors imu
./check_sensors.sh --sensors lidar --lidar-baud 256000
./check_sensors.sh --sensors camera --duration 10
./check_sensors.sh --sensors motors --motor-test encoders --duration 20
```

* `camera` 검사는 RGB·깊이 영상 프레임과 내장 가속도계·자이로 수신 상태를 함께 확인합니다.
* `encoders` 검사는 측정 시간 동안 두 바퀴를 손으로 회전시켜 엔코더 카운트 변화를 확인합니다.
* 바퀴를 공중에 띄운 상태에서 실제 구동을 포함한 모터 검사를 실행할 때는 `./check_sensors.sh --sensors motors --motor-test drive` 또는 `bench`를 사용합니다.

터미널 대시보드(`./hardware_tui.sh`)는 로봇 3D 와이어프레임 자세, 카메라 문자 영상, 라이다 스캔 분포, 엔코더·모터 상태를 한 화면에 표시합니다.

| 대시보드 키 | 동작 |
| --- | --- |
| `1` / `2`, `z` | 외장 / 카메라 내장 IMU 전환, 현재 자세를 화면 영점으로 설정 |
| `c`, `+` / `-` | 컬러·깊이 영상 전환, 라이다 표시 반경 조절 |
| `s`, `r`, `q` | 현재 상태 스냅샷 저장, 센서 드라이버 재시작, 종료 |
| `m`, `Space` / `x` | 모터 구동 검사 시작 (`--motor-test drive` 지정 시), 즉시 중단·정지 |

<a id="calibration"></a>

## 구동 · PID · IMU 보정

모터 피드포워드·PID 보정은 **바퀴를 지면에서 띄운 상태**에서 수행합니다. 보정 결과는 [`config/motor_pid.json`](config/motor_pid.json)에 저장됩니다.

```bash
./tune_motors.sh              # 출력-속도 특성 측정 기반 피드포워드·PI 보정 및 제동 검증
./tune_motors.sh --autotune   # 계단 응답 기반 바퀴별·방향별 Kp/Ki/Kd 탐색 및 검증
```

외장 IMU(HFI-A9)와 카메라 내장 IMU(RealSense D435i)의 정지 자이로 바이어스 보정은 로봇을 수평 정지 상태로 두고 실행합니다. 보정값은 [`config/imu_calibration.json`](config/imu_calibration.json)에 저장됩니다.

```bash
./calibrate_imu.sh --stationary                 # 외장 IMU와 카메라 내장 IMU 동시 측정·검증
./calibrate_imu.sh --stationary --sensors imu   # 외장 IMU 단독 측정
./calibrate_imu.sh --stationary --sensors camera
```

## 야외 SSH용 핫스팟

실외 시험 등 공유기가 없는 환경에서 로봇에 직접 접속할 수 있도록 NetworkManager 기반 AP 전환 스크립트를 제공합니다.

```bash
sudo ./hotspot_on.sh             # 부팅 시 자동 시작 활성화 + 5초 후 핫스팟 전환
sudo ./hotspot_on.sh --boot-only # 현재 연결 유지, 다음 부팅부터 핫스팟 적용
sudo ./hotspot_off.sh            # 부팅 자동 시작 해제 (현재 연결 유지)
sudo ./hotspot_on.sh --status    # 현재 Wi-Fi 연결 상태 및 SSH 접속 주소 확인
```

기본 핫스팟 SSID는 `ORINBOT`, 비밀번호는 `12345678`, 고정 주소는 `ssh hyunlee@10.42.0.1`입니다.

## 펌웨어 빌드와 자동 테스트

새 환경에서 Python 의존성과 로컬 Arduino CLI(`.tools/arduino/`)를 구성할 때는 다음 명령을 실행합니다.

```bash
/usr/bin/python3 -m pip install pyserial numpy Pillow wcwidth

mkdir -p .tools/arduino/bin .tools/arduino/data .tools/arduino/downloads .tools/arduino/sketchbook
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=.tools/arduino/bin sh
cat > .tools/arduino/arduino-cli.yaml << 'EOF'
directories:
  data: .tools/arduino/data
  downloads: .tools/arduino/downloads
  user: .tools/arduino/sketchbook
EOF
./.tools/arduino/bin/arduino-cli --config-file .tools/arduino/arduino-cli.yaml core update-index
./.tools/arduino/bin/arduino-cli --config-file .tools/arduino/arduino-cli.yaml core install arduino:avr
```

펌웨어 빌드·업로드와 단위 테스트는 아래 명령으로 실행합니다.

```bash
./mega_firmware.sh build                              # Mega 모터 제어 펌웨어 빌드
./mega_firmware.sh upload                             # 펌웨어 업로드 (모터 전원 차단 상태에서 실행)
/usr/bin/python3 -m unittest discover -s tests -v     # C++ 제어 코어·프로토콜·도구 단위 테스트
```

공식 라이다 패키지(`src/rplidar_ros`)를 빌드하고 단독 실행할 때는 다음 명령을 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rplidar_ros --symlink-install --allow-overriding rplidar_ros
./lidar_official.sh
```

## 문서와 파일

| 문서 · 경로 | 제공 내용 |
| --- | --- |
| [모터 제어 명세](docs/motor_control_spec.md) | 1 kHz 속도 PID, 정지 위치 유지(`HOLD`), `OMR1` 시리얼 프로토콜 및 안전 제한 규격 |
| [제어·통신 주기 기준](docs/control_timing.md) | MCU 제어 루프와 호스트 송수신 주기 설계 기준 및 UART 링크 프로파일링 |
| [IMU 보정](docs/imu_calibration.md) | HFI-A9·RealSense D435i 정지 자이로 영점 보정 절차와 좌표계 적용 기준 |
| [`config/`](config/) | 구동 방향([`drive_calibration.json`](config/drive_calibration.json)), PID 계수([`motor_pid.json`](config/motor_pid.json)), IMU 바이어스([`imu_calibration.json`](config/imu_calibration.json)) |
| [`firmware/mega_motor_test/`](firmware/mega_motor_test/) | Arduino Mega 2560 타이머 인터럽트 제어 코어, 엔코더 디코더, 프레임 파서 |
| [`tools/`](tools/) · [`tests/`](tests/) | 센서 점검·키보드 조종·TUI 대시보드·보정 도구 및 자동 테스트 스위트 |
| [`resources/`](resources/) · [`src/rplidar_ros/`](src/rplidar_ros) | 대시보드 표시용 참조 URDF 모델 및 공식 RPLIDAR ROS 2 드라이버 |

## 라이선스

저장소의 펌웨어, 제어·보정 도구, 테스트 및 문서에는 [Apache-2.0](LICENSE)을 적용합니다.
하드웨어·모델 출처는 [Oh-My-MobileRobot](https://github.com/Leehyunbin0131/Oh-My-MobileRobot)이며, 참조 URDF 모델의 라이선스([CERN-OHL-P-2.0](resources/LICENSE.robot-model))와 고지([NOTICE.robot-model](resources/NOTICE.robot-model))를 보존했습니다.
라이다 드라이버 출처: [Slamtec/rplidar_ros](https://github.com/Slamtec/rplidar_ros/tree/ros2).
