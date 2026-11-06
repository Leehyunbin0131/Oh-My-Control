# IMU 정지 자이로 보정

[README](../README.md) · [모터 제어 명세](motor_control_spec.md) · [제어·통신 주기](control_timing.md)

외장 IMU(HFI-A9)와 깊이 카메라 내장 IMU(RealSense D435i)의 정지 상태 자이로 영점 바이어스를 측정·검증하는 절차와 보정 데이터 규격을 설명합니다.

## 보정 실행 방법

로봇을 수평이고 진동이 없는 바닥에 정지시킨 뒤, 다른 센서 대시보드나 구동 프로그램을 종료한 상태에서 실행합니다.

```bash
./calibrate_imu.sh --stationary                 # 외장 IMU와 카메라 내장 IMU 동시 측정
./calibrate_imu.sh --stationary --sensors imu   # 외장 IMU(HFI-A9)만 측정
./calibrate_imu.sh --stationary --sensors camera
```

1. **측정 절차**: 안정화 대기(60초) $\rightarrow$ 바이어스 추정용 수집(60초) $\rightarrow$ 독립 검증 구간 수집(30초) 순서로 진행합니다.
2. **품질 검사**: 수집 중 진동·움직임, 타임스탬프 역행, 수신 공백, CRC 오류가 기준치를 초과하면 보정을 중단하고 기존 설정 파일을 유지합니다.
3. **유효성 판정**: 검증 구간에서 바이어스 차감 후 잔여 각속도 벡터 크기가 기존 대비 20% 이상 감소할 때만 차감값(`gyro_bias_rad_s`)을 활성화하며, 기존 영점이 이미 충분히 작은 경우에는 추정값만 `estimated_gyro_bias_rad_s`에 기록하고 차감값은 `0`으로 유지합니다.

측정 및 검증을 통과한 보정 파라미터는 [`config/imu_calibration.json`](../config/imu_calibration.json)에 저장됩니다.

## 센서별 수집 규격과 보정식

| 센서 | 인터페이스 · 수집 주기 | 원시 좌표계 기준 |
| --- | --- | --- |
| 외장 IMU (HFI-A9) | USB 시리얼 921600 bps | 센서 패킷 고유 XYZ 축 (각속도 단위: `rad/s`) |
| 카메라 IMU (RealSense D435i) | librealsense RSUSB (자이로 200 Hz · 가속도 100 Hz) | 카메라 SDK Motion XYZ 광학 좌표계 (`rad/s`, `m/s²`) |

### 보정값 적용 원칙

* **자이로 바이어스 차감**: 센서 원시 축에서 먼저 바이어스를 차감(`gyro_corrected = gyro_input - gyro_bias_rad_s`)한 뒤, 로봇 몸체 좌표계(`base_link`: X 전방, Y 좌측, Z 상방)로 회전 변환합니다. [`tools/imu_calibration.py`](../tools/imu_calibration.py)의 `subtract_gyro_bias()` 함수를 사용할 수 있습니다.
* **중력 가속도 참조값(`gravity_reference_m_s2`)**: 정지 자세에서 측정된 중력 포함 가속도 평균 벡터입니다. 단일 자세 측정만으로는 3축 가속도계 바이어스·스케일 오차와 장착 기울기를 분리할 수 없으므로, 이 값을 선형 가속도에서 단순 차감하지 않습니다.
* **정지 공분산**: 정지 검증 구간의 샘플 분산 행렬이 함께 기록되며, 상위 상태 추정기(EKF 등) 구성 시 초기 참조값으로 활용합니다.
