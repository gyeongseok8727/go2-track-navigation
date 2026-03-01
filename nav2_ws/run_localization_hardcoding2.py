#!/usr/bin/env python3
import os, signal, subprocess, sys, time

# === 사용자 환경 설정 ===
MAP_YAML = "/home/mr/nav2_ws/test3.yaml"
RVIZ_CONFIG = "/home/mr/nav2_ws/src/navigation2/nav2_bringup/rviz/nav2_default_view.rviz"

# ROS 환경을 한 번에 source 하고 명령을 실행하도록 bash -lc 로 감쌉니다.
# 필요에 맞게 setup.bash 경로를 조정하세요.
ROS_SETUP = "source /opt/ros/humble/setup.bash && source ~/nav2_ws/install/setup.bash"

# ---- (선택) Fast DDS SHM 이슈 우회 옵션들 ----
# 1) CycloneDDS 사용 (설치 필요: ros-humble-rmw-cyclonedds-cpp)
USE_CYCLONE = False
# 2) Fast DDS UDP-only 프로파일 적용 (XML 파일 만들어둔 경우)
FASTDDS_PROFILE = ""  # 예: "/home/mr/.ros/fastdds_udp_only.xml"  (비우면 미적용)

# ---------------------------------------------

procs = []  # [Popen, ...]
def popen_bash(cmd: str) -> subprocess.Popen:
    """
    login shell처럼 bash -lc 로 실행. 프로세스 그룹 분리해서 한 번에 종료하기 쉽게 함.
    """
    env = os.environ.copy()
    if USE_CYCLONE:
        env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    if FASTDDS_PROFILE:
        env["FASTRTPS_DEFAULT_PROFILES_FILE"] = FASTDDS_PROFILE

    return subprocess.Popen(
        ["bash", "-lc", cmd],
        preexec_fn=os.setsid,  # 자식들을 새 프로세스 그룹으로
        env=env,
        stdout=sys.stdout, stderr=sys.stderr
    )

def kill_all():
    # 실행중인 자식 프로세스들을 그룹 단위로 종료
    for p in procs:
        try:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        except Exception:
            pass
    # 약간 기다렸다가 남으면 SIGTERM
    time.sleep(1.0)
    for p in procs:
        try:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGTERM)
        except Exception:
            pass

def run_bash_wait(cmd: str) -> int:
    """동기 실행 (반환코드 리턴)"""
    env = os.environ.copy()
    if USE_CYCLONE:
        env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    if FASTDDS_PROFILE:
        env["FASTRTPS_DEFAULT_PROFILES_FILE"] = FASTDDS_PROFILE
    return subprocess.call(["bash", "-lc", cmd], env=env)

def wait_for_first_message(topic: str, timeout_s: int = 30) -> bool:
    """
    해당 토픽에서 '첫 메시지'가 나타날 때까지 대기.
    Humble에서 `-n 1` 옵션이 환경에 따라 안 먹는 경우가 있어,
    `ros2 topic echo ... | head -n 1` + timeout으로 감지한다.
    """
    cmd = f"""{ROS_SETUP} && timeout {timeout_s}s sh -c 'ros2 topic echo {topic} | head -n 1'"""
    print(f"[wait] first message on {topic} (timeout {timeout_s}s)")
    rc = run_bash_wait(cmd)
    return rc == 0

def main():
    try:
        # clean memory
        # kill_cmd1 = ["pkill", "-9", "ros2"]
        # kill_cmd2 = ["pkill", "-9", "python3"]
        # kill_cmd3 = ["pkill", "-9", "go2_ctrl"]
        # clean_cmd1 = ["rm", "-f", "/dev/shm/fastrtps_port*"]

        # junbeom_cmd = f"""{ROS_SETUP} && python3 climb_stair.py"""
        # print("[run] junbeom_waypoints:", junbeom_cmd)
        # junbeom = popen_bash(junbeom_cmd)
        # procs.append(junbeom)

        # print("[info] waiting for junbeom script to finish...")
        # junbeom.wait()

        # 1) Localization 먼저
        loc_cmd = f"""{ROS_SETUP} && ros2 launch nav2_bringup localization_launch.py map:={MAP_YAML}"""
        print("[run] localization:", loc_cmd)
        loc = popen_bash(loc_cmd)
        procs.append(loc)

        # 2) RViz 바로 실행 (2D Pose Estimate 클릭)
        # time.sleep(0.3)  # 아주 짧은 지연
        # rviz_cmd = f"""{ROS_SETUP} && rviz2 -d {RVIZ_CONFIG} --ros-args --log-level rviz:=error""" # Added to suppress warnings (Jaewon)
        # print("[run] rviz2:", rviz_cmd)
        # rvz = popen_bash(rviz_cmd)
        # procs.append(rvz)

        time.sleep(0.5)
        jaewon_cmd = f"""{ROS_SETUP} && ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{{
                            header: {{
                                stamp: {{sec: 0, nanosec: 0}},
                                frame_id: map
                            }},
                            pose: {{
                                pose: {{
                                position: {{x: -6.179600715637207, y: 1.584541082382202, z: 0.0}},
                                orientation: {{x: 0.0, y: 0.0, z: -0.13178851904133532, w: 0.9912778552196612}}
                                }},
                                covariance: [
                                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
                                ]
                            }}
                        }}" """
        # or: --global --wait-amcl
        print("[init] auto_localize:", jaewon_cmd)
        subprocess.call(["bash", "-lc", jaewon_cmd])


        # 3) /amcl_pose 1건 대기 (사용자 2D Pose Estimate 입력 신호)
        #    출력은 버리지만, 한 건 수신되면 즉시 리턴해서 다음 단계로 넘어갑니다.
        wait_cmd = f"{ROS_SETUP} && timeout 3s ros2 topic echo /amcl_pose > /dev/null" # -n1 Doesn't exist (Jaewon)
        print("[wait] /amcl_pose 1 message? ...")
        ret = subprocess.call(["bash", "-lc", wait_cmd])
        if ret != 0:
            print("[warn] /amcl_pose 대기 중 비정상 종료. 그래도 navigation을 시도합니다.")

        # time.sleep(0.5)
        # good_stair_cmd = f"""{ROS_SETUP} && python3 good_stair.py"""
        # print("[run] junbeom_waypoints:", good_stair_cmd)
        # good_stair = popen_bash(good_stair_cmd)
        # procs.append(good_stair)

        # print("[info] waiting for junbeom script to finish...")
        # good_stair.wait()

        
        time.sleep(0.5)
        good_stair_cmd = f"""{ROS_SETUP} && python3 /home/mr/ros2_ws/src/traffic_pkg/traffic_pkg/crosswalk_amcl_lidar_clean_finish.py"""
        print("[run] junbeom_waypoints:", good_stair_cmd)
        good_stair = popen_bash(good_stair_cmd)
        procs.append(good_stair)

        print("[info] waiting for junbeom script to finish...")
        good_stair.wait()

        # 끝나면 전체 종료
        print("[info] junbeom finished, shutting down all processes...")
        kill_all()
        return  # main 함수 종료

    except KeyboardInterrupt:
        print("\n[info] CTRL+C detected, shutting down...")
    finally:
        kill_all()

if __name__ == "__main__":
    main()
