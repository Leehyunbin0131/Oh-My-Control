# 모터 제어 명세

[README](../README.md) · [제어·통신 주기](control_timing.md) · [IMU 보정](imu_calibration.md)

Arduino Mega 2560과 NT-S-DCDM1210 모터 드라이버를 이용한 양륜 차동 구동 제어기의 동작 규격과 시리얼 프로토콜을 정의합니다.

## 제어기 기본 사양

| 항목 | 규격 |
| --- | --- |
| 제어 루프 주기 | Mega Timer1 인터럽트 기반 **1000 Hz (1 ms)** |
| 엔코더 샘플링 | 20 kHz 구적(Quadrature) 디코딩, 명목 2652 counts/rev |
| 호스트 송신 / 수신 주기 | 목표 속도 송신 **50 Hz**, 상태 텔레메트리 수신 **20 Hz** |
| 모터 드라이버 인터페이스 | I²C 400 kHz, 주소 `0x01` (`signed_pwm_v1`) |
| 목표 속도 범위 | 기본 72 RPM, 운용 상한 **95 RPM**, 과속 보호 상한 **100 RPM** |
| 출력 PWM 제한 | 주행 중 최대 **650**, 0 RPM 감속 최대 **150**, 위치 유지(`HOLD`) 최대 **350** |
| 명령 유효기간 (Lease) | **300 ms** (주기적 갱신이 중단되면 자동 출력 해제) |

좌우 바퀴 채널과 전진 부호는 [`config/drive_calibration.json`](../config/drive_calibration.json), 바퀴별·방향별 피드포워드 및 PID 계수는 [`config/motor_pid.json`](../config/motor_pid.json)을 기준으로 합니다.

## 주행 제어와 정지 위치 유지 (`HOLD`)

호스트는 목표 속도를 설정하고, Mega는 엔코더 피드백을 바탕으로 부호 있는 PWM을 계산합니다. 주행 중 새로운 목표 속도를 수신하면 제어 상태를 초기화하지 않고 연속적으로 목표를 갱신합니다.

* **주행(`TRACK`)**: 바퀴별·회전 방향별 선형 피드포워드(`slope`, `offset`)와 속도 PID(`kp`, `ki`, `kd`)를 합산해 구동 PWM을 출력합니다.
* **정지 및 위치 유지(`HOLD`)**: `SET_SPEED 0 0`을 수신한 첫 제어 주기에 현재 엔코더 카운트를 목표 위치(`holdReference`)로 저장하고 `HOLD` 상태로 전환합니다. 위치 오차에 직접 비례하는 PWM 항과 유지 전용 속도 피드백을 결합해 제동 및 정지 위치를 유지합니다.
* **출력 해제(`STOP`)**: PID 및 위치 유지 제어를 즉시 해제하고 양쪽 모터 PWM을 0으로 설정합니다.
* **오류 보호(`FAULT`)**: 통신 만료, 엔코더 배선 이상, 과속(100 RPM 초과 100 ms 지속), 위치 유지 한도 초과 시 즉시 출력을 차단하고 오류 코드를 기록합니다. 원인을 해소한 뒤 `CLEAR_FAULT` 명령으로 해제하기 전까지 재구동하지 않습니다. 같은 방향의 RPM 갱신은 방향 오류·출력 포화 감시의 누적 시간을 초기화하지 않습니다.

### 위치 유지 (`HOLD`) 파라미터

| 항목 | 설정값 | 설명 |
| --- | --- | --- |
| 위치 $\rightarrow$ 속도 비례항 / 상한 | 0.25 RPM/count / 5 RPM | 위치 오차를 감쇠하기 위한 내부 목표 속도 |
| 직접 위치 비례항 (`HOLD_POS`) | 18 PWM/count | 속도 감쇠와 독립된 정지 위치 복원 강성 |
| 유지 전용 속도 PI (`HOLD_PI`) | Kp 8.0, Ki 0.0 | 외력 제거 후 적분 누적으로 인한 역방향 밀림 방지를 위해 Ki는 0으로 설정 |
| 출력 상한 / 위치 데드밴드 | PWM 350 / 1 count | 데드밴드 이내 정지 상태에서는 출력 0 유지 |
| 이탈 한도 / 포화 지속 한도 | 100 counts / 1000 ms | 기준 위치에서 100 counts 이상 밀리거나 출력 포화가 1초 지속되면 `HOLD_LIMIT (fault 10)`으로 해제 |

## 시리얼 프로토콜 (`OMR1`)

호스트와 Mega는 115200 bps UART에서 시퀀스 번호와 CRC16-CCITT를 포함한 줄 단위 프레임(`OMR1,...*CRC`)으로 통신합니다.

| 명령 | 동작 |
| --- | --- |
| `ARM` | 출력 0 상태에서 목표 속도 수신 준비 |
| `SET_SPEED L R lease_ms` | 드라이버 `[L, R]` 순서 목표 속도(0.01 RPM 단위) 및 유효기간(100~300 ms) 설정 |
| `PING` | 연결 상태 확인 (이동 목표의 유효기간은 연장하지 않음) |
| `STOP` | 제어 해제 및 출력 0 요청 |
| `CLEAR_FAULT` | 기록된 오류 상태 해제 (재구동 시 별도 `ARM` 필요) |
| `PID channel dir kp ki kd slope offset` | 바퀴별·방향별 주행 PID 및 피드포워드 계수 설정 |
| `ZERO channel kp ki pwm` | 0 RPM 감속용 PI 계수 및 PWM 상한 설정 |
| `HOLD_PI channel kp ki` | 위치 유지 전용 속도 PI 계수 설정 |
| `HOLD_POS channel kp` | 위치 유지용 직접 위치 비례항(PWM/count) 설정 |
| `HOLD channel enable kp speed pwm deadband error duration saturation` | 위치 유지 활성화 및 안전 한도 설정 |

> **호스트 파서 구현 시 좌/우 인덱스 규칙**:
> `OMR1,T` 상태 텔레메트리 프레임에서 원시 엔코더 카운트(`counts`) 및 무효 전이(`invalid`) 필드는 **`[오른쪽(D40/41), 왼쪽(D50/51)]` 순서**(`ENC_INDEX = {'L': 1, 'R': 0}`)로 전송되며, 목표/측정 속도(`target_rpm_centi`, `measured_rpm_centi`)·출력(`command_pwm`)·제어 상태 필드는 **드라이버 `[왼쪽(L/M2), 오른쪽(R/M1)]` 순서**(`VEL_INDEX = {'L': 0, 'R': 1}`)로 전송됩니다. 상위 ROS 2 드라이버 작성 시 [`tools/motor_link.py`](../tools/motor_link.py)의 인덱스 매핑을 동일하게 적용합니다.

## 구현 파일

| 경로 | 역할 |
| --- | --- |
| [`tools/wheel_drive.py`](../tools/wheel_drive.py) | Orin의 물리적 좌우 RPM 명령, 명령 만료·송신 주기, 측정 피드백과 정지 |
| [`firmware/mega_motor_test/velocity_core.h`](../firmware/mega_motor_test/velocity_core.h) | 속도 추정, 주행 PID, 0 RPM 감속 및 위치 유지(`HOLD`) 연산 코어 |
| [`firmware/mega_motor_test/test_core.h`](../firmware/mega_motor_test/test_core.h) | 구동 허가 상태기, 명령 유효기간 및 오류 감시 |
| [`firmware/mega_motor_test/mega_motor_test.ino`](../firmware/mega_motor_test/mega_motor_test.ino) | Timer1 인터럽트, 20 kHz 엔코더 샘플링, 시리얼 파서 및 I²C 출력 스케줄러 |
| [`tools/motor_link.py`](../tools/motor_link.py) · [`tools/serial_port.py`](../tools/serial_port.py) | 공통 시리얼 연결, OMR1 프레임·응답 처리, 장치 접근 검사 |
| [`tools/motor_pid.py`](../tools/motor_pid.py) · [`tools/drive_calibration.py`](../tools/drive_calibration.py) | 설정 유효성 검사, 물리적 바퀴의 채널·부호와 단위 변환(`velocity_commands`, `joint_states`) |
| [`tools/motor_test.py`](../tools/motor_test.py) | 모터·엔코더 진단 절차와 결과 판정 |
| [`tools/keyboard_control.py`](../tools/keyboard_control.py) | 키보드 조종, 연속 목표 갱신 및 정지 제어 |
| [`tools/tune_motors.py`](../tools/tune_motors.py) · [`tools/motor_autotune.py`](../tools/motor_autotune.py) | 피드포워드·PI 특성 측정 및 계단 응답 기반 PID 자동 탐색 도구 |
