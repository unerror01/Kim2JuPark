# wheelchair_ws

자율주행 휠체어 ROS 2 Jazzy 워크스페이스. 프로젝트 개요/구조/빌드법은 README.md 참고.

## Git / GitHub

- GitHub 저장소: `unerror01/Kim2JuPark` (public), 2026-08-10에 사용자 요청으로 생성.
- **코드 변경이 있으면 확인 없이 바로 commit + push할 것.** 사용자가 명시적으로
  요청함 ("코드 같은 경우 변경이 있으면 너가 알아서 푸시 해줘"). 일반적으로는
  push 전에 확인받는 게 기본값이지만, 이 프로젝트에 한해서는 예외.
- `src/ydlidar_ros2_driver/`는 업스트림 별도 저장소를 clone한 것이라 `.gitignore`로
  제외하고 이 저장소에서는 추적하지 않는다.
- `build/`, `install/`, `log/`는 colcon 산출물이라 추적하지 않는다.
