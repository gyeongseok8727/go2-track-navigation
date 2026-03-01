# go2-track-navigation  

## Overview
Code for unitree go2 navigation around given course in POSCO Industrial AI-Robot Challenge 2025, made by Team 용과함께 from MR, KAIST  
Unitree Go2 로봇의 자율주행을 위해 NAV2, 학습시킨 RL policy, YOLO, unitree sdk의 highlevel api인 SportClient등을 통합하여 작성했다. 



## Mission Courses  
<img width="607" height="384" alt="Image" src="https://github.com/user-attachments/assets/06e09826-1f00-40db-a01c-d7b7c3dcbae5" />

코스별 미션과 주행전략은 다음과 같다.  

트랙의 밖에서 출발하여, 
코스 1: 방지턱 점프 구간 
--> 점프 위치까지 nav2로 이동한 뒤 SportClient로 점프하려 했으나 특징점이 없는 긴 통로에서 localization이 안됨
--> 하드코딩으로 속도와 시간을 찾은 뒤 /laserscan, /lowstate를 구독하여 wz, vy를 p제어하며 이동 

코스 2: 동적 장애물 구간   
--> 점프 후 위치에서부터 장애물 앞 까지 nav2로 이동한 후 YOLO로 장애물을 피해 이동하려 했으나 마찬가지로 localization 문제, 대회장의 가변적 환경으로 인해 YOLO 불안정
--> 이동은 코스 1에서와 마찬가지로 하드코딩, /laserscan을 이용해 장애물 인식 후 돌파

코스 3: 저자세 직진 주행 구간  
--> 저자세 위치까지 nav2로 이동 후 udp_bridge와 학습시킨 RL policy로 이동
--> 코스 시작점까지 이동은 localization 문제로 인해 하드코딩으로 변경

코스 4: 협소 공간 통과 구간 --> nav2로 이동

코스 5: 계단 주행 구간 
--> 계단은 벽을 따라 정해둔 시간만큼 주행하며 올라가고, 위에서 방향 전환 한 뒤 내려오기
--> 2d laserscan을 이용한 amcl로는 계단을 주행할 수 없어 하드코딩함

코스 6: 경사로 돌파 구간 
코스 7: 포트폴 주행 구간 --> nav2로 이동하려 했으나 불규칙한 지형에서 로봇이 기울어지면 localization이 불안정하여 p제어 하드코딩으로 변경

코스 8: 신호등 인지 구간 --> YOLO로 신호등 인식하여 통과  
코스 9: 전력질주 주행 구간 --> SportClient로 달리기 모드 켠 뒤 통과

## Workspace Layout
nav2_ws, ros2_ws, unitree_rl_lab  
메인 ws는 nav2_ws이고, 여기서 ros2_ws, unitree_rl_lab의 코드를 불러와 실행하도록 한다.
해당 README에선 별 다른 표시가 없으면 ~/nav2_ws 내의 경로이다.  

## Setup & Dependencies
미리 설치가 필요한 것은 unitree 공식 깃허브에서 다음과 같다.   
-unitree_sdk2  
-unitree_sdk2_python  
-unitree_ros2  
-unitree_rl_lab  

nav2도 필요하다.  
바이너리로 다운 받아서 쓰려면 아래 둘을 깐다.  
sudo apt install ros-$ROS_DISTRO-navigation2  
sudo apt install ros-$ROS_DISTRO-nav2-bringup  
또는 공식 nav2 깃헙에서 git clone 한 뒤 src에 넣어두어 build해도 된다.  
여기선 바이너리로 받은 nav2 위에 nav2_bringup만을 복사하여 nav2_ws에 둔 뒤 nav2_params.yaml과 launch파일만을 수정하고 빌드하여 오버레이하는 식으로 사용한다.

build, install, log, pycache 등은 지우고 업로드한다.





### Runtime Pipeline 

파이썬 subprocess를 이용하여 순차적 미션 수행을 위한 컨트롤러 파일을 여럿을 두고, 각 파일이 또 다른 파일을 부르며 트리처럼 순차적으로 실행한다.

run_localization_hardcoding0.py
    

    출발

    localization켜고 /initialpose 보내서 localization하고 (시작점 ~ 계단 진입까지의 맵인 my_map_edit.pgm에서)
    map0_total.py 실행
        unitree sdk의 SportClient 이용
        WallFollower노드로 /laserscan, /lowstate 구독하여 wz, vy 각각 p제어하며 이동
        BalanceStand, 4.7초 전진, 점프, 0.8초 전진

    점프 끝, 장애물 코스 진입

    obstacle_lidar.py 실행
        라이다로 장애물 보고 지나가기
        SportClient를 이용해서 /scan, /amcl_pose로 p제어하며 이동한다.
        이전에 켜둔 localization에서 /amcl_pose 나오면 이를 이용해 yaw제어하는데, localization이 안되면 /scan으로 vy p제어만으로 이동한다.
        /lowstate의 yaw값 대신에 /amcl_pose의 yaw값을 이용하는 이유는 적분 드리프트 때문이다.

        장애물 탐지 및 회피 로직은 다음과 같다.
            로봇이 벽 중앙, yaw 기준 적당히 맞췄다는 가정하에
            전방 영역(30도~150도)의 laserscan에 대하여 임계거리를 다르게 주어서
            cos로 투영시킨 전방거리가
            30~60, 120~150도는 40cm
            60~120도는 80cm로 임계거리를 잡고, 
            해당 거리 내에 들어온 점들을 장애물로 탐지하여 거리 평균을 내서 장애물 까지 전방거리를 구하는 방식이다.
            전방에 장애물이 없으면 0.5로 전진,
            장애물 보이면 50cm까지 다가가서 멈추도록 vx p제어,  
            그 다음 더 좁은 75~105도 영역에서 수직 투영한 거리 60cm안에 장애물 안보이면 빠르게 통과

    저자세 구간 진입

    udp_bridge_node.py 켠 상태로
    lowlevel.py 실행
        

    협소구간 진입

    저자세 끝 위치로 다시 /initialpose주고 이전에 켜둔 localization에서 /amcl_pose 잡기를 최대 60초 기다리고, 위치 잡으면 navigation 켠다.

    go2_move.py 실행
        mission이 없을 땐 cmd_vel을 구독하여 SportClient로 운동명령을 내린다.
        mission 토픽으로 받은 숫자에 따라서 SportClient로 보행모드 변경 등을 수행한다.

    example_simple_waypoints_junbeom2.py 실행 (저자세 끝부터 계단 진입 지점까지 이동)
        mission 토픽으로 7 한 번 pub해서 EconomicGait켠다. (협소공간에 안정적인 보행모드)
        waypoint로 5개 점 이동 후 mission 토픽으로 8 한 번 pub 해서 BalanceStand 켠다. (계단 주행 위한 안정적인 보행모드)

    이후 kill_all()로 localization, navigation, go2_move.py, udp_bridge도 다 끈다.


    계단 구간 진입

    

    climb_stair.py 실행
        계단 올라간다.
        계단 올라가는 동안에는 localization이 꺼져있으므로 /amcl_pose로 yaw 못 받기 때문에 /lowstate로 yaw_target을 측정하여 유지하도록 하여 쌓인 적분 drift 영향을 없앤다.

        현재 yaw 값 측정해서 yaw 이에 맞춰 유지, 벽 일정거리 유지하면서 3초 전진, 2.5초 전진(계단 끝을 알 수 있는 벽 50 cm부터 p제어해서 정지)

        -Laserscan, LowState 받고 SportClient로 이동해줌        

    올라간 다음, localization 켜기 (my_map2_edit 계단 위 맵), /initialpose pub하고 /amcl_pose 최대 60초 기다림

    good_stair.py 실행 (계단 위에서 돌아서 내려갈 위치까지 이동)
        navigation은 쓰지 않고 하드코딩 하는데, 방금 켠 /amcl_pose를 이용해 yaw_target에 대해서 p제어한다.
        직진, 좌회전, 직진, 좌회전해서 내려올 준비

    climb_stair.py 실행 (내려오기)

    kill_all()로 navigation 끄기

    계단 내려와서 경사로, 포트홀 구간
    localization 켜고 pose잡고 (my_map3_edit.yaml)
    map3_total.py 실행
        /amcl_pose로 yaw wz p제어
        /scan으로 vy p제어
        하드코딩임
        wallfollower로 1초 전진
        좌회전
        5.2초 전진
        5초 전진, 전방 벽까지 50cm에 정지하도록 p제어
        /initialpose 다시 pub해주고
        좌회전


    신호등, 전력질주 구간 진입

    localization 켜고 pose잡고 
    crosswalk_amcl_lidar_clean_finish_different_node.py 실행


    map3_total.py 실행
    /initialpose 한 번 pub해서 /amcl_pose 정확히 해주고
    
    crosswalk_amcl_lidar_clean_finish_different_node.py 실행
        DetectionNode
            YOLO로 신호등 박스 검출
            ROI 영역별 밝기로 색 분석
            색 변경 검출로직에서 GO조건에서 finish()호출
        RunningNode
            SportClient로 TrotRun 켜기
            /scan로 vy p제어
            /amcl_pose로 yaw wz p제어
            일정조건(6초 후 양쪽 벽 없음) 또는 timeout 10초로 종료

    kill_all()로 종료
    도착점 도달



#### 1. total_sequence.py
-~/map5.yaml을 사용하지만 코드 내에선 쓰지 않는다.

-map1_cmd --> run_nav2_sequence.py  
-끝나기를 기다린다.

-map_1_cmd --> run_nav2_sequence_1.py  
-끝나기를 기다린다.

-udp_cmd --> ~/ros2_ws/src/run_policy/udp_bridge_node.py  
-끝나기를 기다리지 않는다.

-low_cmd --> test_exe.py  
-끝나기를 기다린다.

-map2_cmd --> run_nav2_sequence2.py  
-끝나기를 기다린다.

///////////여기부터 아래 3개는 주석 처리를 해두었다.  
-stair_cmd --> stair.py  
-끝나기를 기다린다.

-run_cmd --> ~/ros2_ws/src/traffic_pkg/traffic_pkg/crosswalk_slow_stop_cmd_imu.py  
-끝나기를 기다린다.

-obs_cmd --> ~/ros2_ws/src/obstacle_pkg/obstacle_pkg/avoid_go_oneshot_move.py  
-끝나기를 기다린다.  



#### 2. run_nav2_sequence.py
-my_map0.yaml
-`ros2 topic echo ... | head -n 1` + timeout으로 첫 토픽 발행 감지를 기다리는 함수를 정의한다.

-loc_cmd --> nav2에서 localization을 켠다. (nav2_bringup의 localization_launch.py)
-map yaml 인자를 넘겨준다.
-localization을 계속 켜두어야 하므로 실행이 끝나기를 기다리지 않는다.

-2초 후 jaewon_cmd --> localization 시작을 위해 2D pose estimate를 위한 미리 정해둔 pose를 pub한다.

-wait_cmd --> localization이 되면 계속 발행되는 /amcl_pose 를 8초간 기다린다.
humble 환경에서 ros2 topic echo -n1이 존재하지 않아 임시방편으로 그냥 8초 기다리도록 했다.

-nav_cmd --> nav2에서 navigation을 켠다. (nav2_bringup의 navigation_launch.py)
-navigation 실행 파악을 위해 /local_costmap/costmap과 /global_costmap/costmap이 pub 되기를 최대 60초 기다린다.

-go_cmd --> go2_move.py를 켠다. 이는 go2 로봇의 운동을 통제하는 코드이다.
-이후 7초를 기다린다.

-junbeom_cmd --> example_simple_waypoints_junbeom.py
-끝나기를 기다린다.

-이후 전체 종료한다.



3. go2_move.py
요약하면, go2_move.py는
1. go2가 mission이 없는 평소에는 cmd_vel을 구독하여 움직이도록 하고
2. 특별 미션 코스에 진입하면 이에 맞는 미리 정의된 동작을 실행하도록 한다.
-go2에 내장된 highlevel api, 
-강화학습으로 만든 바이너리 제어 명령 코드, 
-또는 또다른 파이썬 코드
-(점프, 달리기 모드 켜기, 횡단보도 건너기, 장애물 회피 코스, 계단 주행 코스 등)
#####의문!! cmd_vel, mission, stop_cmd는 누가 pub하고 /mode_cmd는 누가 sub할까?


자세한 설명은 다음과 같다.
go2_move에서는 CmdVelSubscriber 노드를 정의하고 실행한다.
해당 노드는 unitree python sdk2를 이용해 go2에 몇 가지 운동 명령을 보낸다.
대표적으로 import하는 것은 unitree_sdk2py에서 SportClient(이하 sc)와 MotionSwitcherClient(이하 msc), 
subprocess, 
그리고 low level 제어 바이너리 코드 실행을 위한 process_executor의 BaseProcessExecutor, ExecConfig 이다.


노드(클래스)의 플래그 변수는 self.mission, self.stop, self.mode이 있다.

노드를 생성하면 다음을 진행한다.
sc, msc 연결, 후에 걸을 수 있도록 BalanceStand해두기, 
저자세 주행에 필요한 low level 코드 실행 위한 프로세스 객체인 self.ex 생성, (unitree_rl_lab/deploy/robots/go2/build/go2_ctrl)

publisher로 /mode_cmd를 pub한다.

subscriber로 'mission', 'cmd_vel', 'stop_cmd'을 sub한다. 
이 셋의 콜백 함수는 아래의 기능을 한다.

cmd_vel의 콜백함수에선 self.mission이 None, self.stop이 False라면 msg: Twist를 vx, vy, wz로 변환 후 속도 클리핑 등을 거쳐서 SportClient에서 받는 형식으로 변환하여 highlevel 제어 명령을 go2에 보낸다.

mission의 콜백 함수에선 self.mission에 토픽으로 받은 데이터를 담고,
그 값이
1--> sc로 전방점프
2--> sc로 달리기 모드 켜기
3--> self.stop=True, subprocess로 ros2_ws의 crosswalk_slow_stop_cmd.py 실행하고 끝나기를 기다림
4--> self.stop=True, 마찬가지로 avoid_go_oneshot.py 실행 후 기다림
5--> self.enter_lowmode()
6--> self.exit_lowmode()
7--> sc로 EconomicGait 모드 켜기
8--> sc로 BalanceStand
를 진행 한 뒤 다시 self.mission = None으로 설정해둔다.

stop_cmd의 콜백함수에선 받은 메시지에 따라 self.stop의 T/F를 설정한다.


기타 메소드는 다음과 같다.
destroy_node(self)는 sc.StopMove()와 노드 파괴 담당,

send_mode(self, mode_value: int)는 받은 mode_value를 pub_mode로 pub한다.

enter_lowmode()에선 sc와 msc로
sc 엎드리기,
msc로 sc의 제어권 종료,
self.ex(미리 정의한 low level control subprocess 객체) 실행
send_mode(1)
send_mode(2)

exit_lowmode()에선 sc와 msc로
send_mode(3)
self.ex 종료
msc로 mcf모드 선택 (





4. example_simple_waypoints_junbeom.py
action 구조, pose 생성, callback 체인의 형태는 기존 nav2, ros2 예제를 참고하였으나 queue를 통해 여럿을 등록하고 mission_cmd를 subprocess로 추가하는 것은 커스텀하였다.

main에서는 
FollowWaypointsClient 노드 생성, 
subprocess로 "mission" topic으로 7을 한 번 pub,

그리고 여러 waypoints와 mission_cmd 쌍을 정의하고 순서에 맞춰 queue에 등록한다.
node.enqueue_goal(wp, mission_cmd)로 
wp1과 mission_cmd1
wp2와 mission_cmd2(=None)
wp3는 주석처리

그리고 FollowWaypointsClient 노드를 실행한다.


해당 노드의 구조는 다음과 같다.

init시 
/follow_waypoints 라는 이름의 액션을 위한 액션 클라이언트 self._wp_client를 생성하고
self.queue를 deque()로 만들어둔다. 메인의 enqueue_goal을 통해 wp, mission_cmd의 쌍을 여럿 담게 된다.

노드 클래스의 메서드는 다음과 같다.

enqueue_goal(self, poses_list, mission_cmd=None)는 
다음 waypoint와 mission_cmd를 self.queue에 추가하고
현재 goal 진행 중이 아니라면 _dequeue_and_send_next로 즉시 시작한다.

_dequeue_and_send_next(self)는 self.queue가 비어있으면 노드를 닫고
그렇지 않으면 self.queue.popleft()로 poses, mission_cmd를 받아서 self._send_goal함수를 부른다.

self._send_goal(self, poses, mission_cmd)는 
goal_msg를 만들고 
/follow_waypoints 서버(nav2)가 켜져있는지 확인해서 켜질 때까지 기다리고
goal_msg를 해당 서버에게 전달해서 서버가 응답하면 self._feedback_cb 구독을 시작한다.
서버가 해당 goal을 수행할지 여부를 받아서 self._goal_response_cb 콜백 함수에 전달한다.

_goal_response_cb(self, future)는
서버가 goal을 수락했으면 self._result_cb을 부르도록 등록해두고
그렇지 않으면 self._dequeue_and_send_next()로 다음 waypoints로 넘어간다.

_feedback_cb(self, feedback_msg)는
현재 총 waypoint 중 몇 번째에 있는지를 계속 보내는데, 시끄러워서 주석해뒀다.

_result_cb(self, future)는 
goal의 success/canceled/failed 여부를 받고, 
-success라면 deque에 waypoints와 함께 mission_cmd가 있었다면 subprocess로 블로킹 호출한다.
-canceled라면 warn 메시지,
-failed라면 error 메시지와 missed_waypoints를 출력한다.


메서드 체인들의 호출 흐름은 플래그 변수 self._active로 제어하는데, 현재 goal로 이동하고 있는지를 담는다.
self._active는 init에서 False로 시작해서
enqueue_goal에서 인자로 받은 waypoints, mission_cmd를 deque에 등록한 다음에 현재 바로 _dequeue_and_send_next를 통해 _send_goal 할지를 판단할 때 쓰고
_goal_response_callback 함수에서 액션 서버가 해당 goal을 거부한다면 False로 설정해주고
goal로의 이동이 끝나서 _result_cb이 불리면 해당 함수에서 mission_cmd를 실행한 다음 False로 설정해준다.

따라서 제어 흐름은 다음과 같다.
goal을 액션 서버에 등록하는 것은 _send_goal에서 이루어지고
이는 _dequeue_and_send_next에서 한 번 부른다.
_dequeue_and_send_next는 enqueue_goal에서 queue에 goal 넣은 다음에 현재 진행 중인 goal이 없으면 부르거나 아님 목적지에 도달하면 불리는 _result_cb에서 mission_cmd 수행 후 부른다. 
따라서, self._active는 _send_goal을 처음에 enqueue_goal에서 부르기 위해 필요하고 해당 goal로의 주행 또는 미션 실행 중 또다른 enqueue_goal로 queue 등록을 할 때 _send_goal을 부르지 않기 위해 필요하다.

기타로
make_pose(x, y, ori_z, ori_w) 함수는 geometry_msgs/msg/PoseStamped로 변환한 값을 리턴한다.


5. run_nav2_sequence_1.py
-my_map.yaml
-`ros2 topic echo ... | head -n 1` + timeout으로 첫 토픽 발행 감지를 기다리는 함수를 정의한다.

-loc_cmd --> nav2에서 localization을 켠다. (nav2_bringup의 localization_launch.py)
-map yaml 인자를 넘겨준다.
-localization을 계속 켜두어야 하므로 실행이 끝나기를 기다리지 않는다.

-end_of_jump로 subprocess에서 localization 시작을 위해 /initialpose를 정해둔 값으로 한 번 pub해준다.

-wait_cmd --> localization이 되면 계속 발행되는 /amcl_pose 를 8초간 기다린다.
humble 환경에서 ros2 topic echo -n1이 존재하지 않아 임시방편으로 그냥 8초 기다리도록 했다.

-nav_cmd --> nav2에서 navigation을 켠다. (nav2_bringup의 navigation_launch.py)
-navigation 실행 파악을 위해 /local_costmap/costmap과 /global_costmap/costmap이 pub 되기를 최대 60초 기다린다.

-go_cmd --> go2_move.py를 켠다. 이는 go2 로봇의 운동을 통제하는 코드이다.
-이후 7초를 기다린다.

-junbeom_cmd --> example_simple_waypoints_junbeom_1.py
-끝나기를 기다린다.

-이후 전체 종료한다.


6. example_simple_waypoints_junbeom_1.py
wp2, wp3를 enqueue하고 실행한다. misison_cmd는 두 경우 다 None이다.

7. udp_bridge_node.py
여기서는 /cmd_vel, /mode_cmd 토픽을 구독하고 udp 통신을 통해 해당 정보를 로봇에 제공한다.
저자세 주행에서는 low-level 제어가 필요하기 때문에 udp 브릿지를 직접 켜야하는 듯하다.
/cmd_vel은 nav2에서가 아니라, 저자세 주행 중에는 양쪽의 벽을 라이다로 본 뒤 계산한 값이고(또는 그냥 하드코딩), /mode_cmd는 udp 통신의 모드를 설정하는 것.
///의문: 왜 같은 pc로 udp를 보내는가? 그리고 이건 왜 저자세 주행할 때는 따로 돌리는가? nav2할 때 /cmd_vel은 누가 받은건가? 어떻게.

go2_driver켜야 /cmd_vel 받게 된다.
unitree_sdk는 low/high level api 쓸 때 쓰는 것 같다. 아마 해당 sdk 내에 go2_driver 같은 것 있을 듯하다.
go2_driver는 이의 단순한 버전으로 여러 api들 안 쓰는 것 같고.
아무튼 그런데 go2_driver에서 하면 되지 왜 따로 udp bridge를 켜는건가?



8. test_exe.py
sc, msc 이용
Stair클래스 정의
init에서 
-sc연결, 
-yaw의 p 제어 위한 변수 튜닝값 설정
-timer로 18.5초 가도록 설정 --> 
-/lowstate 구독 --> self.stair_callback
-라이다 2D 스캔 값인 /scan 구독 --> self.lidar_cb
-bash process 객체로 self.ex만들어서 unitree_rl_lab/deploy/robots/go2/build 실행 가능하도록 해둠
-msc 연결
-/mode_cmd, /cmd_vel pub
-sc로 엎드리기, msc로 sc의 제어권 박탈, self.ex 실행해서 rl 주행
-/mode_cmd로 1 pub
-/mode_cmd로 2 pub

메서드
_on_exit_timer 콜백에서 _cleanup함수 부름.
_cleanup은 
timer 객체 해제, 
/lowstate 구독 해제, 
/cmd_vel로 정지신호 0,0,0 pub,
/mode_cmd로 3 pub
bash 프로세스 객체 self.ex 정지
/mode_cmd로 4 pub
msc로 mcf 모드 선택
sc 해제
등으로 잘 정지하도록 한다.

/lowstate의 콜백 stair_callback은 
msg에서 imu_state.rpy에서 yaw 값 받아서 yaw p제어 한다.
yaw target과의 error 계산, 
error 값 크기가 너무 작으면 0 처리, 최대 4.0 한계둠.
vx=1.5, vy=vy=0.0로 두고 wz=kp * yaw_error로 계산해서
sc.Move 대신 send_cmd 통해서 /cmd_vel로 pub한다. 
하지만 계단에서 yaw를 조정하면 레이다가 벽이 아닌 계단을 보기 때문에 yaw는 건드리지 않기 위해 해당 함수에서 결과 pub부분은 주석처리 했다.
////의문? cmd_vel로 다 pub할거면 sportclient 부를 필요 없었던거 아닌가?


lidar_cb은 Laserscan msg를 받아서 
왼쪽 (150~180도), 오른쪽 (0~30도) 영역의 거리 10cm 밖의 값들을 각각 평균내서 거리 구하고
self.vy = kp * (left - right)해서 vy p 제어 해서 계단의 양쪽 벽의 중간에 있을 수 있도록 했다.

즉, self.ex의 go2_ctrl 바이너리가 /mode_cmd나 /cmd_vel을 udp로 변환한 것을 받아서 계단을 오르는 듯하다.


### go2_ctrl 바이너리
unitree_rl_lab/deploy/robots/go2/src
ros2_ws/src/


~/unitree_rl_lab/deploy/robots/go2/main.cpp

udp_cmd_is(int want) 함수에서 take_mode_cmd() 함수를 이용해 udp로 읽은 값을 받는다.
take_mode_cmd()는 cmd_bufer.h에 있고 cmd_buffer.h와 udp_receiver.h를 include한다.

cmd_buffer.h
이는 ~/unitree_rl_lab/deploy/include에 버퍼 구조체가 정의되어 있다.
버퍼 구조체는 vz, vy, wz, mode_cmd를 담는다. 
그리고 해당 버퍼의 주소를 전달하는 cmd_buffer() 함수와 값을 1회성으로 소비하는 take_mode_cmd()가 정의됨.

udp_receiver.h
해당 버퍼에다가 udp로 받은 값을 저장하는 것은 같은 폴더의 udp_receiver.h에 있다.
float 형식으로 3개가 오면 vx, vy, wz로 인식하여 buf의 해당 멤버 변수에 저장하고 아니면 정수로 읽어서 buf의 mode_cmd에 저장한다.


UDP로 mode_cmd로 수신한 값에 따라 FSM에서 정의한 모드 4개 사이에서 순차적으로 바꾼다.
1 Passive -> FixStand
2 FixStand -> Velocity(RLBase)
3 Velocity -> SitDown
4 udp_bridge_node.py에서 여기로 전달하기 전에 스스로 종료(ctrl + c)

그리고 ~/unitree_rl_lab/deploy/include/FSM/CtrlFSM.h 가 FSM을 돌린다.










9. run_nav2_sequence2.py
-my_map.yaml (점프 후 ~ 계단 전까지)
-localization켜고
-end_of_lowmode로 /initialpose 하드코딩 한 번 pub
-/amcl_pose 2d pose estimate 결과 듣고
-navigation 켜고
-local, global costmap pub 결과 듣고
-go2_move.py 실행
-example_simple_waypoints_junbeom2.py 실행
다 끝나면 종료

10. example_simple_waypoints_junbeom2.py
mission 토픽으로 7 한 번 pub하고 (economic gait켜기)
wp5로 5개 점과 이동 후 mission_cmd5로 mission 8 한 번 pub하도록 (balance stand 켜기)
enqueue 한다.

11. stair.py

sc 연결
yaw값 p제어 위한 kp튜닝
auto_exit_sec 10초 설정
/lowstate 대신 /amcl_pose를 구독해서 여기서 현재 yaw 값을 받는다.
msg의 pose orientation에서 q를 뽑는다.
vx=vy=0.0, wz는 msg의 q에서 계산한 현재 yaw와 yaw target 간의 차로 p제어해서
sc.Move로 이동한다.

12. avoid_go_oneshot_move.py
동적 장애물의 위치를 YOLO로 바운딩 박스를 그려서 알아내고, 해당 박스의 위치가 시야의 가장자리(박스 중심의 x값이 하위 20% 또는 상위 80% 위치)에 있을 때 빠르게 지나가도록 한다.

import 하는 것은
unitree_sdk2py에서 SportClient, VideoClient
YOLO
LowState
등이다.

ObstacleDecisionNode
ros2_ws/src/obstacle_pkg/models/best.pt의 장애물 탐지를 위해 학습해둔 신경망 로드
cmd_pub 퍼블리셔로 'stop_cmd' 토픽 발행하도록 함
sc연결, yaw p제어 위한 값 지정, 달릴 시간 2초 지정

멀티스레드로 video_loop 스레드와 yolo_worker 스레드로 나눴다.

메서드는
stair_callback에서는 LowState에서 yaw를 구해 p제어를 수행한하고 sc.Move로 명령한다.

publish_cmd에선 'stop_cmd' 토픽으로 앞으로 가게 하는 0을 발행한다.

yolo_worker는 

GPU 없는 NUC에서 iGPU가 아닌 CPU로 YOLO추론을 돌린다. 
iGPU를 시도해봤고 cpu보다 훨씬 빨리 추론이 가능하지만 얼마 못 버티고 다시 cpu로 롤백해서 그냥 CPU만 이용한다. 
Coral TPU도 시도했고 훨씬 빠르지만, 메인 개발 컴에서 NUC으로 옮기며 모델이 가벼워지고, 따라서 출력 텐서가 복잡해졌는데, 여기서 바운딩 박스의 위치값을 뽑기가 힘들어 포기하고 cpu를 썼다.



끝??

그래서 코드 뭐 올려야 하는지??

