# 구동 소프트웨어 아키텍처와 검증 가이드

[README](../README.md) · [모터 제어 명세](motor_control_spec.md) · [제어·통신 주기](control_timing.md) · [IMU 보정](imu_calibration.md)

Jetson Orin 호스트 구동 인터페이스(`WheelDrive`)와 Arduino Mega 2560 저수준 제어 펌웨어의 모듈별 책임, 명령·피드백 계약, 오프라인·실물 검증 절차를 설명합니다.

---

## 1. 구동 소프트웨어 계층과 모듈 책임

호스트(`Jetson Orin Nano Super`)는 물리적 좌우 바퀴 목표 속도(RPM) 전달과 명령 수명 관리를 담당하며, 실시간 1 kHz 폐루프 속도 PID 제어와 하드웨어 안전 감시는 MCU(`Arduino Mega 2560`)가 전담합니다.

```text
keyboard_control.py / 상위 구동 호출자
        │ 물리적 좌·우 바퀴의 목표 RPM (전진 양수)
        ▼
wheel_drive.py (호스트 기본 구동 인터페이스)
        │ 명령 수명(300 ms) · 최대 50 Hz 송신 · 측정 피드백 수집
        ▼
motor_pid.py + drive_calibration.py
        │ 단위·채널·부호 정규화, 바퀴별/방향별 PID 설정 로드
        ▼
motor_link.py ── serial_port.py
        │ OMR1 프레임, CRC16, ACK, 20 Hz 텔레메트리, 시리얼 연결
        ▼
Arduino Mega: 명령 파서 → 1 kHz 속도 PID / HOLD → 모터 드라이버(I²C)
        ▲                                              │
        └──────────── 20 kHz 엔코더 피드백 ────────────┘
```

| 모듈 | 역할 및 설계 원칙 |
| --- | --- |
| [`tools/wheel_drive.py`](../tools/wheel_drive.py) | 호스트 기본 구동 인터페이스(`WheelDrive`). 목표 RPM 수명 관리, 최대 50 Hz 송신, 피드백 수신 및 이상 발생 시 출력 차단 |
| [`tools/drive_calibration.py`](../tools/drive_calibration.py) | 물리적 좌우 바퀴(전진 양수)와 실제 모터 드라이버 채널·배선 부호 사이의 정규화 변환 |
| [`tools/motor_pid.py`](../tools/motor_pid.py) | [`config/motor_pid.json`](../config/motor_pid.json) 설정 검증·로드, RPM ↔ `rad/s` 변환 및 측정 피드백 해석 |
| [`tools/motor_link.py`](../tools/motor_link.py) | 공통 시리얼 통신 계층(`MotorClient`). `OMR1` 프레임 송수신·CRC16·ACK 처리 (모듈 import 시 장치를 자동으로 열지 않음) |
| [`tools/keyboard_control.py`](../tools/keyboard_control.py) | 키보드 입력 처리, 연속 목표 갱신 및 정지(`HOLD` / `STOP`) 요청 |
| [`tools/motor_test.py`](../tools/motor_test.py) | 제한 시간 모터·엔코더 하드웨어 진단 절차 및 결과 판정 |

---

## 2. 명령과 피드백 인터페이스 계약

| 항목 | 규격 및 동작 계약 |
| --- | --- |
| **물리적 바퀴 속도** | 전진을 양수(`+`)로 하는 좌·우 바퀴 RPM (상위 ROS 2 연동 시 `rad/s`에서 명시적 변환) |
| **`SET_SPEED L R lease_ms`** | 드라이버 `[L, R]` 순서의 `0.01 RPM` 단위 정수 및 명령 유효기간(`100–300 ms`) |
| **원시 배열 순서 구분** | 엔코더 카운트는 `[오른쪽(D40/41), 왼쪽(D50/51)]`, 속도·PWM은 드라이버 `[왼쪽(L/M2), 오른쪽(R/M1)]` 순서로 전송되므로 [`drive_calibration.py`](../tools/drive_calibration.py)를 통해서만 접근 |
| **`SET_SPEED 0 0`** | `0 RPM` 제어 및 정지 위치 유지(`HOLD`) 진입 |
| **`STOP`** | 제어 및 모터 출력을 즉시 해제(`PWM 0`). 기존 오류 코드는 유지 |
| **정지 판정 기준** | 명령 수락(`ACK`)만으로 정지를 판정하지 않으며, 최신 측정 속도와 제어 상태를 함께 확인 |

---

## 3. 호스트 구동 API (`WheelDrive`)

`WheelDrive`는 시리얼 연결과 PID 설정이 완료된 `MotorClient`를 주입받아 단일 I/O 스레드에서 수신(`client.poll`)과 목표 송신을 수행합니다.

| 메서드 | 동작 |
| --- | --- |
| `set_target_rpm(left, right)` | 물리적 좌·우 바퀴 목표 RPM(전진 양수)을 설정하고 명령 생성 시각을 기록 |
| `tick()` | 최신 텔레메트리를 수신하고 최대 50 Hz로 목표 속도를 재전송 (원본 명령 생성 시각은 연장하지 않음) |
| `stop()` | 즉시 `0 RPM` 목표를 적용 (지속적인 `HOLD` 유지를 위해서는 상위에서 새로운 0 명령 공급 필요) |
| `disable()` | `STOP` 프레임으로 모터 출력을 해제하고 이전 목표를 폐기 |
| `feedback()` | 목표·측정 RPM, 누적 엔코더 카운트 및 라디안 위치, 출력 PWM, 오류 상태, 데이터 나이(age) 반환 |

### 명령 수명 및 보호 규칙

- **원본 수명 보존**: 마지막 목표 입력 후 **300 ms**가 경과하면 출력을 해제합니다. `tick()`이 재전송할 때도 원본 명령의 남은 유효기간만 MCU `lease_ms`로 전달하며, 남은 시간이 MCU 최소 허용값(**100 ms**) 미만이면 재전송을 중단합니다.
- **피드백 중단 및 오류 차단**: 텔레메트리가 **400 ms** 동안 갱신되지 않거나 MCU 오류·전송 실패가 감지되면 즉시 `STOP`을 요청하고 해당 구동 인스턴스의 재사용을 차단합니다.
- **속도 범위**: 보정된 운용 상한(`95 RPM`) 이내의 유한한 값만 허용하며, 저속 명령을 임의로 `30 RPM`으로 끌어올리지 않습니다(키보드 조종기의 `30 RPM` 하한은 수동 조종 UI 제한입니다).

---

## 4. 보정 파일 운용 기준

- [`config/drive_calibration.json`](../config/drive_calibration.json): 좌우 바퀴 채널 매핑과 전진 방향 부호를 정의합니다.
- [`config/motor_pid.json`](../config/motor_pid.json): 무부하 계단 응답 및 구간별 특성 측정으로 산출한 바퀴별·방향별 피드포워드(`slope`, `offset`)와 PID 게인(`30–95 RPM` 주행 구간 및 곡선 `15 RPM` 조건 포함)을 담고 있습니다.
- 실물 로봇에 배포할 때는 로봇 현장에서 검증된 보정 설정 파일을 유지하며, 예시값으로 덮어쓰지 않습니다.

---

## 5. 검증 절차

### 5.1 오프라인 빌드 및 단위 테스트

모터 제어 펌웨어 컴파일 검증(업로드 제외)과 호스트 구동 계층 단위 테스트는 하드웨어 연결 없이 실행할 수 있습니다.

```bash
# 1. Arduino Mega 펌웨어 타깃 빌드 확인
./mega_firmware.sh build

# 2. 전체 단위 테스트 스위트 실행 (C++ 제어 코어 · 프로토콜 · WheelDrive · 키보드 조종)
/usr/bin/python3 -m unittest discover -s tests -v

# 3. 호스트 구동 계층(WheelDrive 및 키보드 조종 연동) 핵심 검증만 선택 실행
PYTHONPATH=tools:tests /usr/bin/python3 -m unittest \
  test_wheel_drive \
  test_keyboard_control.ControllerTests.test_pid_in_place_limit_preserves_selected_forward_speed \
  test_keyboard_control.TerminalIntegrationTests.test_default_pid_targets_curve_stop_and_sigterm
```

### 5.2 단계별 실물 로봇 검증 순서

실물 로봇에서 구동 계층과 상위 스택을 검증할 때는 아래 순서로 진행합니다.

| 단계 | 점검 대상 | 합격 기준 |
| --- | --- | --- |
| **1. 무부하 구동 (바퀴 공중 부양)** | 좌·우 및 전·후진 회전 방향, 목표 대비 실측 RPM 추종, 정지(`HOLD`), 명령 만료 차단 | `./check_sensors.sh --sensors motors` 및 `./teleop.sh`에서 부호·속도·정지 정상 |
| **2. 지면 기본 주행** | 출발·전진·후진·제자리 회전·정지·재출발 응답 | 지면 마찰 조건에서 발진·진동·오류 차단 없이 안정 주행 |
| **3. 차체 오도메트리 · 센서 정합** | 바퀴 회전량 대비 실제 이동 거리·회전각, 라이다·카메라·IMU 축 방향 | 실측 거리·각도와 센서/TF 방향 일치 |
| **4. 상위 자율주행 연동 (`Oh_My_SLAM`)** | ROS 2 브리지 연결, SLAM 지도 작성, `Nav2` 목표 주행 | [`Oh_My_SLAM`](https://github.com/Leehyunbin0131/Oh_My_SLAM) 단계별 인수 기준 충족 |
