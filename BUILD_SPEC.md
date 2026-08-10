# 자율주행 휠체어 패키지 생성 사양서

이 문서를 읽고 ~/wheelchair_ws/src/autonomous_wheelchair ROS 2 Jazzy 패키지를
처음부터 전부 생성한 뒤 colcon build 까지 완료하라.
아래 수치는 이미 튜닝된 값이므로 임의로 바꾸지 말고 그대로 반영할 것.

## 하드웨어
- Raspberry Pi 5 / Ubuntu 24.04 LTS / ROS 2 Jazzy
- YDLIDAR G2 (삼각측량, 360도, 0.12~12m, 5kHz, 230400bps, /dev/ydlidar)
- ESP32 모터 컨트롤러 (USB, /dev/ttyUSB0, 115200bps)
- 차동구동 2륜 + 캐스터, 휠 엔코더 있음
- 휠체어 치수: 길이 1.10m, 폭 0.70m, 구동륜 반지름 0.15m, 트레드 0.56m
- base_link = 구동륜 축 중심. 라이다 장착 = base_link 기준 x 0.55, y 0.0, z 0.35

## 패키지 구조 (ament_python)
autonomous_wheelchair/
  package.xml, setup.py, setup.cfg, resource/autonomous_wheelchair
  urdf/wheelchair.urdf.xacro
  config/ ydlidar_g2.yaml, ekf.yaml, slam_toolbox_mapping.yaml, nav2_params.yaml, waypoints.yaml
  launch/ bringup.launch.py, mapping.launch.py, navigation.launch.py
  autonomous_wheelchair/ base_driver.py, safety_monitor.py, waypoint_navigator.py
  udev/99-ydlidar.rules
entry_points: base_driver, safety_monitor, waypoint_navigator
setup.py data_files 에 launch/ config/ urdf/ 를 반드시 install 하라.

## 속도 명령 체인 (이 순서를 정확히 지킬 것)
controller_server -/cmd_vel_nav-> velocity_smoother -/cmd_vel_smoothed->
collision_monitor -/cmd_vel_raw-> safety_monitor -/cmd_vel-> base_driver
- controller_server, behavior_server 는 launch 에서 cmd_vel -> cmd_vel_nav 리매핑
- velocity_smoother 도 cmd_vel -> cmd_vel_nav 리매핑 (입력)
- 전부 geometry_msgs/Twist (enable_stamped_cmd_vel: false)

## config/ydlidar_g2.yaml
port /dev/ydlidar, frame_id laser_frame, baudrate 230400, lidar_type 1(TRIANGLE),
device_type 0(SERIAL), sample_rate 5, abnormal_check_count 4, fixed_resolution true,
reversion true, inverted true, auto_reconnect true, isSingleChannel false,
intensity false, support_motor_dtr false, invalid_range_is_inf false,
angle_min -180.0, angle_max 180.0, range_min 0.12, range_max 12.0, frequency 10.0

## config/ekf.yaml (robot_localization)
frequency 30.0, two_d_mode true, publish_tf true, sensor_timeout 0.2
map_frame map / odom_frame odom / base_link_frame base_footprint / world_frame odom
odom0 = /wheel/odometry, odom0_config 는 vx, vy, vyaw 만 true
imu0 = /imu/data, imu0_config 는 yaw, vyaw, ax 만 true (IMU 없으면 주석 처리 안내)

## config/slam_toolbox_mapping.yaml
mode mapping, scan_topic /scan, base_frame base_footprint, resolution 0.05,
map_update_interval 2.0, minimum_travel_distance 0.2, minimum_travel_heading 0.2,
min_laser_range 0.15, max_laser_range 11.0, do_loop_closing true,
loop_search_maximum_distance 3.0, transform_publish_period 0.02,
solver CeresSolver / SPARSE_NORMAL_CHOLESKY / SCHUR_JACOBI

## config/nav2_params.yaml  <-- 핵심 튜닝값
공통 풋프린트 (local, global 코스트맵 둘 다):
  footprint: "[[-0.45,-0.35],[0.65,-0.35],[0.65,0.35],[-0.45,0.35]]"
  footprint_padding: 0.03

amcl: base_frame_id base_footprint, laser_model_type likelihood_field,
  max_beams 120, min_particles 500, max_particles 2000, laser_max_range 11.0,
  laser_min_range 0.15, update_min_d 0.25, update_min_a 0.2,
  recovery_alpha_slow 0.001, recovery_alpha_fast 0.1, alpha1~5 = 0.2,
  robot_model_type "nav2_amcl::DifferentialMotionModel", transform_tolerance 0.5

controller_server: controller_frequency 15.0, failure_tolerance 0.3,
  progress_checker_plugins ["progress_checker"] (Jazzy 는 복수형),
  goal_checker xy_goal_tolerance 0.20 / yaw_goal_tolerance 0.20,
  FollowPath = nav2_mppi_controller::MPPIController
    time_steps 40, model_dt 0.075, batch_size 1000, iteration_count 1,
    vx_max 0.60, vx_min -0.25, wz_max 0.9, vx_std 0.15, wz_std 0.30,
    ax_max 0.4, ax_min -0.4, az_max 1.0,
    temperature 0.3, gamma 0.015, motion_model "DiffDrive",
    prune_distance 1.7, visualize false, regenerate_noises false,
    critics: ConstraintCritic, CostCritic, GoalCritic, GoalAngleCritic,
             PathAlignCritic, PathFollowCritic, PathAngleCritic, PreferForwardCritic
    ConstraintCritic weight 4.0
    GoalCritic weight 5.0 threshold_to_consider 1.4
    GoalAngleCritic weight 3.0 threshold_to_consider 0.5
    PreferForwardCritic weight 8.0 threshold_to_consider 0.5   <- 후진 억제
    CostCritic weight 4.0, critical_cost 300.0, consider_footprint true,
               collision_cost 1000000.0, near_goal_distance 1.0
    PathAlignCritic weight 14.0, offset_from_furthest 20,
               max_path_occupancy_ratio 0.05, trajectory_point_step 4   <- 지그재그 억제
    PathFollowCritic weight 5.0, offset_from_furthest 5, threshold 1.4
    PathAngleCritic weight 2.0, offset_from_furthest 4, max_angle_to_furthest 1.0

local_costmap: rolling_window true, width 5, height 5, resolution 0.05,
  update_frequency 8.0, publish_frequency 4.0, global_frame odom,
  plugins [obstacle_layer, inflation_layer],
  obstacle_layer scan: topic /scan, sensor_frame laser_frame,
    obstacle_max_range 6.0, raytrace_max_range 11.0, marking/clearing true,
  inflation_layer: cost_scaling_factor 2.5, inflation_radius 0.85

global_costmap: update_frequency 1.0, global_frame map, track_unknown_space true,
  plugins [static_layer, obstacle_layer, inflation_layer],
  obstacle_max_range 8.0, inflation 동일 (2.5 / 0.85)

planner_server: GridBased = nav2_smac_planner::SmacPlannerHybrid
  motion_model_for_search "DUBIN", minimum_turning_radius 0.55,
  angle_quantization_bins 72, analytic_expansion_ratio 3.5, reverse_penalty 2.1,
  non_straight_penalty 1.2, cost_penalty 2.0, smooth_path true, tolerance 0.5,
  max_planning_time 3.0, cache_obstacle_heuristic true

velocity_smoother: smoothing_frequency 20.0, scale_velocities true,
  feedback OPEN_LOOP, odom_topic /odometry/filtered,
  max_velocity [0.60, 0.0, 0.9], min_velocity [-0.25, 0.0, -0.9],
  max_accel [0.35, 0.0, 0.7], max_decel [-0.5, 0.0, -1.0]

collision_monitor: cmd_vel_in cmd_vel_smoothed -> cmd_vel_out cmd_vel_raw,
  base_frame_id base_footprint, source scan(/scan), source_timeout 1.0
  PolygonStop  (action stop):     "[[0.80,0.32],[0.80,-0.32],[-0.20,-0.32],[-0.20,0.32]]"
  PolygonSlow  (action slowdown, slowdown_ratio 0.4):
                                  "[[1.40,0.50],[1.40,-0.50],[-0.35,-0.50],[-0.35,0.50]]"

bt_navigator: odom_topic /odometry/filtered, robot_base_frame base_link
behavior_server: max_rotational_vel 0.6, simulate_ahead_time 2.0
smoother_server: nav2_smoother::SimpleSmoother

## base_driver.py
- /cmd_vel 구독 -> 차동구동 역기구학 v_l = v - w*L/2, v_r = v + w*L/2
- 시리얼로 "<v_left,v_right>\n" 송신, "E,<ltick>,<rtick>\n" 수신 파싱
- 엔코더 틱 -> 원호 적분 오도메트리 -> /wheel/odometry 발행 (TF 는 발행하지 않음, EKF 담당)
- /joint_states 발행, cmd_timeout 0.5s 워치독 -> 정지
- 파라미터: serial_port /dev/ttyUSB0, baudrate 115200, wheel_radius 0.15,
  wheel_separation 0.56, ticks_per_rev 1024, max_linear 0.6, max_angular 0.9,
  invert_left, invert_right
- _send_motor(), _parse_encoder() 는 "★ 프로토콜에 맞게 수정" 주석을 크게 달 것

## safety_monitor.py
- 입력 /cmd_vel_raw + /scan, 출력 /cmd_vel + /obstacle_distance + /safety_state
- 부채꼴이 아니라 휠체어 폭 직사각 통로로 최근접 장애물 판정
  (라이다 극좌표 -> x,y 변환 후 abs(y) > robot_half_width 는 무시, 진행방향만 고려,
   차체 외곽 기준 거리 = x - front_offset(또는 rear_offset))
- 거리 -> 배율 변환은 smoothstep t*t*(3-2t) 사용 (급감속 방지)
- 감속은 즉시 반영, 가속은 저역통과(alpha)로 서서히 -> 승차감
- 제자리 회전(abs(v)<0.02)은 통로 판정 예외
- scan/cmd 타임아웃 워치독 -> 정지
- 파라미터: stop_distance 0.45, slow_distance 1.60, robot_half_width 0.40,
  front_offset 0.55, rear_offset 1.00, min_scale 0.15, smooth_alpha 0.25,
  scan_timeout 0.6, cmd_timeout 0.5
- 루프 20Hz, /scan 은 BEST_EFFORT QoS 로 구독

## waypoint_navigator.py
nav2_simple_commander BasicNavigator 사용. waypoints.yaml 의 이름 목록으로 goToPose,
tour:=true 면 followWaypoints 로 전체 순회.

## launch 파일
- bringup.launch.py: robot_state_publisher(xacro) + ydlidar LifecycleNode +
  base_driver + ekf_node. 인자 use_base, use_ekf, serial_port (기본 true/true)
- mapping.launch.py: bringup include + async_slam_toolbox_node + rviz2
- navigation.launch.py: bringup include + map_server/amcl/lifecycle_manager_localization
  + controller/smoother/planner/behavior/bt_navigator/waypoint_follower/
    velocity_smoother/collision_monitor/lifecycle_manager_navigation
  + safety_monitor + rviz2. 인자 map(필수), params_file, autostart, use_rviz.
  nav2_common.launch.RewrittenYaml 로 yaml_filename 에 map 경로 주입.

## 작업 순서
1. 위 구조대로 전체 파일 생성
2. cd ~/wheelchair_ws && colcon build --symlink-install
3. 빌드 에러가 나면 원인을 찾아 수정하고 성공할 때까지 반복
4. ros2 pkg list | grep autonomous_wheelchair 로 확인
5. 완료 후 다음 명령을 안내:
   ros2 launch autonomous_wheelchair bringup.launch.py use_base:=false use_ekf:=false
