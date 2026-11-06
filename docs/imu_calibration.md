# SLAM 준비용 IMU 보정

목적은 나중에 라이다·엔코더 오도메트리와 IMU를 사용할 때 방향·위치 오차를 줄이는 것이다. 현재 작업 범위는 **외장 HFI-A9와 RealSense D435IF의 정지 자이로 영점 보정**이다. 보정값은 [config/imu_calibration.json](../config/imu_calibration.json)에 저장하며, 기존 센서 표시·주행·ROS 토픽에 자동 적용하지 않는다.

## 측정과 재실행

로봇을 수평이고 흔들리지 않는 곳에 놓고 주행과 센서 대시보드를 종료한다. 전원을 켠 채 정지 상태를 유지하고 실행한다.

```bash
cd ~/ros2_dev_ws
./calibrate_imu.sh --stationary                 # 두 IMU 동시에
./calibrate_imu.sh --stationary --sensors imu   # 외장 IMU만
./calibrate_imu.sh --stationary --sensors camera
```

기본 절차는 안정화 60초, 보정용 수집 60초, 뒤이은 독립 검증 30초다. 자이로 평균을 바이어스로 구하고 검증 구간에서만 보정 전·후 오차를 비교한다. 측정 중 움직임·진동, 프레임/시간 역행, 수신 공백, CRC 오류, 구간별 편차가 기준을 넘으면 실패 처리한다. 선택한 센서가 모두 통과해야 현재 보정 파일을 교체한다. 실패하거나 중단하면 기존 파일을 보존한다.

추가 차감은 검증 구간의 평균 각속도 벡터 크기를 20% 이상 줄일 때만 권장한다. 이득이 없으면 추정값은 `estimated_gyro_bias_rad_s`에 남기고, 실제 차감용 `gyro_bias_rad_s`는 0으로 선택해 기존 출력을 유지한다. 측정 통과와 추가 보정의 필요성은 별개다.

HFI-A9는 시리얼 값을 읽고, 카메라는 설치된 librealsense RSUSB SDK로 자이로 200 Hz·가속도계 100 Hz 프로필을 우선 선택한다. 실제 선택 주기는 보고서에 기록한다. 카메라 SDK의 기존 motion correction 설정과 intrinsic 값을 함께 저장한다. Python `numpy`, `pyserial`, `g++`, SDK 헤더·라이브러리가 필요하며 카메라 수집기는 처음 실행할 때 `build/imu_calibration/`에 빌드한다. 다른 SDK 경로는 `--sdk`로 지정한다.

보고서는 `reports/imu_calibration_날짜_시간/`에 생성한다. CSV에는 원본 센서 시각과 호스트 monotonic 시각을 각각 보존하며, HFI 바이너리도 저장한다. `summary.json`의 PASS는 **해당 정지 측정의 자이로 보정·검증**만 뜻한다. 검사 한계값은 도구의 측정 품질 기준이며 제조사 정확도 사양이 아니다. 정지 확인은 사용자의 준비 상태를 전제로 하며, 일정한 저속 회전을 자이로 편향과 완전히 구분할 수는 없다.

## 보정값의 의미와 이후 적용

- `gyro_corrected = gyro_input - gyro_bias_rad_s`. 저장한 센서와 같은 축·단위의 값에 먼저 적용한 뒤 로봇 좌표계로 회전한다.
- HFI-A9 값은 제조사 패킷 XYZ 기준이다. 각속도 단위는 제조사 ROS 예제가 패킷 값을 `angular_velocity`에 직접 넣는 규약에 따른 rad/s다. 장착 방향과 회전 부호는 실제 회전 시험으로 별도 확인한다.
- 카메라 값은 SDK motion XYZ 기준이다. SDK 버전·장치 시리얼·기존 intrinsic 보정 상태가 같은 입력에 적용한다. 향후 카메라 EEPROM 보정을 새로 하면 이 잔여 바이어스를 다시 측정해야 한다. 중복 차감하지 않는다.
- `gravity_reference_m_s2`는 현재 자세의 중력 포함 가속도 평균이다. 이것을 가속도에서 통째로 빼지 않는다. 한 자세로는 가속도계 3축 바이어스·스케일과 장착 기울기를 분리할 수 없다.
- 공분산은 해당 정지 구간의 샘플 분산이다. Allan variance로 구한 잡음 밀도·random walk나 모든 주행 조건의 EKF 공분산이 아니다.
- HFI의 내장 융합 자세·자력계와 카메라의 EEPROM은 변경하지 않는다. 자이로 영점 차감이 HFI의 내장 yaw를 직접 보정하는 것은 아니다.
- 프로필의 `enabled_for_navigation=false`는 저장만 했다는 뜻이다. 추후 어댑터에서 장치·축을 확인해 명시적으로 적용한다. `tools/imu_calibration.py`의 `subtract_gyro_bias()`는 그때 사용할 수 있는 명시적 보정 함수다.

현재 온도와 전원 상태에서의 보정이다. 온도 변화·전원 재투입 후 재현성, 실제 주행 중 진동과 동적 정확도는 아직 검증하지 않았다. 외장 장치 식별자는 USB 어댑터 ID이므로 같은 어댑터에 IMU 모듈을 교체하면 재보정한다.

## 2026-11-05 측정 결과

두 센서 모두 정지 측정과 별도 구간 검증을 통과했다. (로컬 원본·검증 보고서: `reports/imu_calibration_20261105_192335_778415/summary.json`, 보정값: [imu_calibration.json](../config/imu_calibration.json))

| 센서 | 검증 구간의 평균 각속도 벡터 크기 | 선택 |
| --- | --- | --- |
| HFI-A9 | 기존 약 0.00318 °/s | 추가 차감의 이득이 없어 기존 출력 유지; 추정값 보관 |
| RealSense D435IF | 보정 전 약 0.27904 → 후보 보정 후 0.00430 °/s | 추가 자이로 바이어스 차감 권장, 주행 적용은 꺼짐 |

위 수치는 3축 평균 벡터의 크기이며 yaw 정확도·순간 잡음·SLAM 위치 정확도가 아니다. 카메라의 후보 차감값은 XYZ 순서로 `[-0.0044683773, -0.0017854584, -0.0001467458] rad/s`다. 보고서에는 평균뿐 아니라 표준편차·RMS·샘플 공분산도 저장했다.

정지 가속도 벡터 크기는 표준중력 9.80665 m/s² 대비 HFI 약 **+4.71%**, 카메라 약 **−0.99%**였다. 이 한 자세만으로 축별 오차 원인을 분리하거나 보정하지 않았다. 가속도를 융합하기 전 다음 여섯 방향 보정에서 확인한다.

## SLAM 적용 전에 남은 항목

| 항목 | 확인할 내용 | 상태 |
| --- | --- | --- |
| 정지 자이로 영점 | 각 센서 바이어스와 별도 구간의 잔여 오차 | 이번 측정 |
| 가속도계 보정 | 여섯 방향에서 3축 스케일·바이어스 | 사용자 선택으로 다음에 진행 |
| IMU·라이다·카메라 장착 TF | 로봇 기준 위치와 회전, 실제 좌·우 회전 부호 | 미실시 |
| 엔코더 오도메트리 | 실제 CPR, 좌우 유효 바퀴 반지름, 유효 윤거; 직진·회전 거리 검증 | 미실시; 현재 명목 2652 counts/rev |
| 시간 정렬 | 장치 시각↔ROS 시각, 수신 지연·드롭과 라이다 스캔 시각 | 미실시; 이번 두 캡처는 하드웨어 동기화가 아님 |
| 센서 융합·SLAM 검증 | 동일 rosbag에서 엔코더만/IMU 추가 결과와 지도·복귀 오차 비교 | ROS 어댑터 개발 후 |

로봇 몸체 좌표계는 ROS의 x 전방·y 좌측·z 위쪽 규약을 따른다. 카메라 광학 좌표는 x 우측·y 아래·z 전방이라 장착 TF 없이 같은 축으로 취급하면 안 된다. [ROS REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)

향후 2D SLAM 구조의 한 예는 `엔코더 오도메트리 + 보정 IMU → EKF의 odom→base_link`, `라이다 스캔 + 위 TF → slam_toolbox의 map→odom`이다. 두 IMU를 처음부터 모두 융합하기보다 각각의 축·시간·동적 오차를 비교해 사용 센서를 정한다. 이 구조는 아직 구현하지 않았다. [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/ros2)

## 제조사 자료

- [HFI-A9 참조 드라이버](https://github.com/HANDS-FREE/handsfree_ros_imu/blob/master/scripts/hfi_a9_ros.py): 패킷 값과 단위 변환 기준.
- [HFI 제조사 Q&A](https://docs.taobotics.com/docs/hfi-imu/hfi-imu-en/05_Questions_and_Answers/doc.html): 공장 보정과 모듈 설정 제약. 이번 도구는 미확인 보정 명령을 보내지 않는다.
- [RealSense 공식 IMU 보정 도구](https://github.com/realsenseai/librealsense/tree/v2.58.1/tools/rs-imu-calibration): 여섯 자세 측정과 장치 보정 저장 절차. 이번 정지 잔여 자이로 보정과 구분한다.
- [RealSense IMU 좌표·SDK 처리](https://github.com/realsenseai/librealsense/blob/v2.58.1/doc/d435i.md): SDK 축과 기존 보정 적용 방식.
