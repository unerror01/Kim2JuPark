# wheelchair_ws

라즈베리파이 5 + ROS 2 Jazzy 기반 자율주행 휠체어. 기존 전동 휠체어의 조이스틱
신호를 ESP32로 에뮬레이션해서 Nav2 스택으로 자율주행시키는 프로젝트.

## 하드웨어

- Raspberry Pi 5 / Ubuntu 24.04 LTS / ROS 2 Jazzy
- YDLIDAR G2 (삼각측량, 360도, 0.12~12m, `/dev/ydlidar`)
- ESP32-S3 (조이스틱 에뮬레이터, USB, `/dev/esp32`)
- 차동구동 2륜 + 캐스터. 휠 엔코더는 아직 미장착 (오도메트리는 open-loop 적분)
- 휠체어 치수: 길이 1.10m, 폭 0.70m, 구동륜 반지름 0.15m, 트레드 0.56m

라이다와 ESP32는 둘 다 동일한 CP2102 USB-UART 브릿지를 써서 vendor/product ID로
구분이 안 되기 때문에, `src/autonomous_wheelchair/udev/99-ydlidar.rules`가 물리
USB 포트 경로(KERNELS)로 `/dev/ydlidar`, `/dev/esp32` 심볼릭 링크를 고정한다.
포트를 바꿔 꽂으면 이 규칙의 KERNELS 값을 다시 확인해서 수정해야 한다.

## 구조

```
src/autonomous_wheelchair/   # 메인 ROS 2 패키지 (ament_python)
  autonomous_wheelchair/
    base_driver.py           # /cmd_vel -> 시리얼(ESP32) 송신, open-loop 오도메트리 발행
    safety_monitor.py        # /scan 기반 저속/정지 감속 (직사각 통로 판정)
    waypoint_navigator.py    # nav2_simple_commander 기반 웨이포인트 순회
  config/                    # ekf, slam_toolbox, nav2, ydlidar 파라미터
  launch/                    # bringup / mapping / navigation
  firmware/esp32_joystick_emulator/
                              # ESP32-S3용 Arduino 스케치. throttle/steering을
                              # I2C DAC(MCP4725) 전압으로 변환해 순정 조이스틱을 대신함
src/ydlidar_ros2_driver/     # YDLIDAR 공식 드라이버 (업스트림 별도 저장소, 여기선 미추적)
```

## 빌드

```bash
cd ~/wheelchair_ws
colcon build --symlink-install
source install/setup.bash
```

## 실행

```bash
# 라이다/베이스 없이 노드만 확인
ros2 launch autonomous_wheelchair bringup.launch.py use_base:=false use_ekf:=false

# 매핑
ros2 launch autonomous_wheelchair mapping.launch.py

# 주행 (map 인자 필수)
ros2 launch autonomous_wheelchair navigation.launch.py map:=/path/to/map.yaml
```

## ESP32 펌웨어 업로드

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 src/autonomous_wheelchair/firmware/esp32_joystick_emulator
arduino-cli upload  --fqbn esp32:esp32:esp32s3 -p /dev/esp32 src/autonomous_wheelchair/firmware/esp32_joystick_emulator
```

업로드 전에 `/dev/esp32`(`/dev/ttyUSB0`)를 잡고 있는 ROS 노드(`base_driver`)가
있으면 반드시 먼저 종료할 것 — 포트가 점유된 상태에서는 업로드가 체크섬 에러로 실패한다.

## 속도 명령 체인

```
controller_server -/cmd_vel_nav-> velocity_smoother -/cmd_vel_smoothed->
collision_monitor -/cmd_vel_raw-> safety_monitor -/cmd_vel-> base_driver -> ESP32 -> 순정 컨트롤러
```

## 현재 상태 / 알려진 이슈

- **휠 엔코더 미장착**: `/wheel/odometry`, `/joint_states`는 명령 속도를 그대로
  따른다고 가정하는 open-loop 적분치. 엔코더 장착 시 `base_driver.py`의
  `_integrate_odometry`를 실측 틱 기반으로 교체 필요.
- **순정 컨트롤러 저속 데드밴드**: 전진은 민감하게 반응하지만 후진/조향은 중립
  근처 명령을 무시하는 비대칭 데드밴드가 실측으로 확인되어(2026-08-10),
  ESP32 펌웨어(`esp32_joystick_emulator.ino`)에 데드밴드 보정 로직 반영함.
  우회전 데드밴드는 좌회전과 동일하다고 가정한 값이라 추후 재검증 필요.
- ESP32 ↔ base_driver 시리얼 파이프라인, 4방향 극성, 워치독 실주행 테스트 완료.
- 남은 작업: 바닥에 내려서 저속 실주행 테스트, Wi-Fi 전환 확인.
