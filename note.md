total_sequence.py (~/map5.yaml)
    run_nav2_sequence.py (my_map0.yaml)
        localization, navigation
        go2_move.py
        example_simple_waypoints_junbeom.py
        점프 준비 위치까지 이동, 점프, 점프 후 위치까지 이동

    run_nav2_sequence_1.py (my_map.yaml)
        localization, navigation
        go2_move.py
        example_simple_waypoints_junbeom_1.py
        점프 후 위치에서부터 wp 2개 이동

    udp_bridge_node.py
        켜두고
    
    test_exe.py
        엎드려서 이동

    run_nav2_sequence2.py
        localization, navigation
        go2_move.py
        example_simple_waypoints_junbeom2.py
        저자세 끝 지점부터 wp 5개 이동 후 계단 진입 준비 위치까지



### 여기부터 주석되어있고 연결이 부족하다. 계단 올라간 다음 왼쪽 턴 해서 내려오는거랑 포트홀/경사 이동하는 코드가 없음.

    stair.py
        그냥 10초 올라가기

    crosswalk_slow_stop_cmd_imu.py
        횡단보도 앞에서 
        
    avoid_go_oneshot_move.py
        이건 왜 여기 끝에 있지?

### 다른 코드 뒤져봄.
    climb_stair.py
        -main에서 현재 yaw 값 측정해서 yaw, 벽 일정거리 유지하면서 3초 전진, 2.5초 전진(앞 목표 50 cm부터 p제어)
        자세히는,
        -Laserscan, LowState 받고 SportClient로 이동해줌        
        -Set_rpy 노드에서 /lowstate의 rpy에서 yaw 받아서 2초간 평균 값으로 현재 yaw값 구함.
        -Move_rpy 노드에서
            /lowstate에서 yaw받아서 wz p제어, 
            /laserscan에서 거리값 받아서 vy p제어,
            2.5초 이동하거나 그 전에 laserscan으로 앞 목표 가까워지면 멈춤 (vx p제어)
        -run_stage에서 노드 done될 때까지 굴림

    executor.py


    good_stair.py
        직진, 좌회전, 직진, 자회전 --> 계단 내려오려면 직진 한 번 더 해야하는데?

    lidar_test.py
        테스트용: 라이다로 벽 가운데 유지하게 vy p제어 하면서 vx 1.0 테스트
    lowlevel.py

    map0_total.py
        하드코딩으로 벽간 거리 일정하게 유지하며 점프 앞까지 이동, 점프, 장애물 앞까지 이동하도록
    map3_total.py
        하드코딩임
        wallfollower로 1초 전진
        좌회전
        5.2초 전진
        5초 전진
        localization켜기, 기다린다음,
        좌회전
        -->이게 계단 내려온 다음인 듯??


    obstacle_lidar.py
        라이다로 장애물 보고 지나가기

    pose_estimator.py
        rviz 없이 pose estimate 가능하도록 함
        known pose 모드: /initialpose로 발행한 위치로부터 localization
        global localization 모드: 맵 전체에 대해 localization
        이는 코드에서 사용 안함

    process_executor.py
        ExecConfig, BaseProcessExecutor 정의해둠.
        다른 코드에서 이를 import해 사용한다.
    run_localization_hardcoding0.py
        계단 내려온 다음 맵임
        localization켜고 initial pose잡고 
        map0_total.py 실행
        obstacle_lidar.py 실행
        udp_bridge_node.py 켠 상태로
        lowlevel.py 실행
        다시 포즈 잡고 navigation 켜고
        go2_move.py실행
        example_simple_waypoints_junbeom2.py 실행 (저자세 끝부터 계단 진입 지점까지 이동)
        climb_stair.py 실행
        올라간 다음, localization, pose잡기, navigation 켜고
        good_stair.py 실행 (위에서 돌기)
        climb_stair.py 실행 (내려오기)
        localization 켜고 pose잡고 
        map3_total.py 실행
        포즈 잡고 crosswalk_amcl_lidar_clean_finish_different_node.py 실행

    run_localization_hardcoding2.py
        오류난 경우 마지막 직선 코스의 crosswalk에서 부터 시작하는 코드. 
        localization 켜고 /initialpose 보내서 
        crosswalk 건너기

    wall_follower.py

    crosswalk_amcl_lidar_clean_finish_different_node.py
        신호등 본 다음에 trotrun으로 달리는데
        시간 제한 두고, yaw, vy 각각 p제어 하면서 달리기













go2_move.py
mission이 없을 땐 cmd_vel을 구독하여 운동명령을 내린다.


example_simple_waypoints_junbeom.py
example_simple_waypoints_junbeom_1.py
example_simple_waypoints_junbeom2.py
nav2의 /follow_waypoints 서버를 이용해 설정한 waypoints를 이동하고 subprocess로 mission을 수행한다.