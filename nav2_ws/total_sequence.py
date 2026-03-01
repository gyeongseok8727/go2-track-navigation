#!/usr/bin/env python3
import os, signal, subprocess, sys, time

# === 사용자 환경 설정 ===
MAP_YAML = "/home/mr/map5.yaml"
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
        map1_cmd = f"""{ROS_SETUP} && python3 /home/mr/nav2_ws/run_nav2_sequence.py"""
        print("[run] map1:", map1_cmd)
        map1 = popen_bash(map1_cmd)
        procs.append(map1)

        print("[info] waiting for map1 script to finish...")
        map1.wait()
        time.sleep(0.1)

        # ----------------------------------------------------------------

        map_1_cmd = f"""{ROS_SETUP} && python3 /home/mr/nav2_ws/run_nav2_sequence_1.py"""
        print("[run] map1:", map_1_cmd)
        map_1 = popen_bash(map_1_cmd)
        procs.append(map_1)

        print("[info] waiting for map1 script to finish...")
        map_1.wait()
        time.sleep(0.1)


        # ------------------------------------------------------------------

        udp_cmd = f"""{ROS_SETUP} && python3 /home/mr/ros2_ws/src/run_policy/run_policy/udp_bridge_node.py"""
        print("[run] udp:", udp_cmd)
        udp = popen_bash(udp_cmd)
        procs.append(udp)
        
        low_cmd = f"""{ROS_SETUP} && python3 /home/mr/nav2_ws/test_exe.py"""
        print("[run] low:", low_cmd)
        low = popen_bash(low_cmd)
        procs.append(low)

        print("[info] waiting for low script to finish...")
        low.wait()
        time.sleep(0.1)

        # ------------------------------------------------------------------

        map2_cmd = f"""{ROS_SETUP} && python3 /home/mr/nav2_ws/run_nav2_sequence2.py"""
        print("[run] map1:", map2_cmd)
        map2 = popen_bash(map2_cmd)
        procs.append(map2)

        print("[info] waiting for map1 script to finish...")
        map2.wait()
        time.sleep(0.1)
    
        # stair_cmd = f"""{ROS_SETUP} && python3 /home/mr/nav2_ws/stair.py"""
        # print("[run] stair:", stair_cmd)
        # stair = popen_bash(stair_cmd)
        # procs.append(stair)

        # print("[info] waiting for stair script to finish...")
        # stair.wait()
        # time.sleep(3)
        


        # run_cmd = f"""{ROS_SETUP} && python3 /home/mr/ros2_ws/src/traffic_pkg/traffic_pkg/crosswalk_slow_stop_cmd_imu.py"""
        # print("[run] run:", run_cmd)
        # run = popen_bash(run_cmd)
        # procs.append(run)

        # print("[info] waiting for run script to finish...")
        # run.wait()
        # time.sleep(3)



        # obs_cmd = f"""{ROS_SETUP} && python3 /home/mr/ros2_ws/src/obstacle_pkg/obstacle_pkg/avoid_go_oneshot_move.py"""
        # print("[run] obs:", obs_cmd)
        # obs = popen_bash(obs_cmd)
        # procs.append(obs)

        # print("[info] waiting for run script to finish...")
        # obs.wait()
        # time.sleep(3)

        # 끝나면 전체 종료
        print("[info] map1 finished, shutting down all processes...")
        kill_all()
        return  # main 함수 종료

    except KeyboardInterrupt:
        print("\n[info] CTRL+C detected, shutting down...")
    finally:
        kill_all()

if __name__ == "__main__":
    main()
