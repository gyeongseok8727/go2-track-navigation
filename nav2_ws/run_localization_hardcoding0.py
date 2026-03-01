#!/usr/bin/env python3
import os, signal, subprocess, sys, time

# === 사용자 환경 설정 ===
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

        MAP_YAML = "/home/mr/nav2_ws/my_map_edit.yaml"

        # 1) Localization 먼저
        loc_cmd1 = f"""{ROS_SETUP} && ros2 launch nav2_bringup localization_launch.py map:={MAP_YAML}"""
        print("[run] localization:", loc_cmd1)
        loc1 = popen_bash(loc_cmd1)
        procs.append(loc1)

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
                                position: {{x: -0.9593238830566406, y: -4.256654262542725, z: 0.0}},
                                orientation: {{x: 0.0, y: 0.0, z: 0.9562548154860727, w: 0.2925350027940196}}
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

        print("[wait] /amcl_pose 1 message waiting ...")
        wait_for_first_message("/amcl_pose",  timeout_s=60)
        time.sleep(3)

        time.sleep(0.5)
        map0_cmd = f"""{ROS_SETUP} && python3 map0_total.py"""
        map0 = popen_bash(map0_cmd)
        procs.append(map0)

        print("[info] waiting for junbeom script to finish...")
        map0.wait()

        time.sleep(0.5)
        obstacle_cmd = f"""{ROS_SETUP} && python3 obstacle_lidar.py"""
        # obstacle_cmd = f"""{ROS_SETUP} && /home/mr/ros2_ws/src/obstacle_pkg/obstacle_pkg/avoid_go_oneshot_geometry_lidar_safe_clean_different_node.py"""
        obstacle = popen_bash(obstacle_cmd)
        procs.append(obstacle)

        print("[info] waiting for junbeom script to finish...")
        obstacle.wait()


        time.sleep(0.5)
        udp_cmd = f"""{ROS_SETUP} && python3 /home/mr/ros2_ws/src/run_policy/run_policy/udp_bridge_node.py"""
        udp = popen_bash(udp_cmd)
        procs.append(udp)


        time.sleep(0.5)
        lowlevel_cmd = f"""{ROS_SETUP} && python3 lowlevel.py"""
        lowlevel = popen_bash(lowlevel_cmd)
        procs.append(lowlevel)

        print("[info] waiting for junbeom script to finish...")
        lowlevel.wait()
        time.sleep(0.1)

        # # ---------------------------------------------------------
        # MAP_YAML = "/home/mr/nav2_ws/my_map_edit.yaml"
        # 1) Localization 먼저

        time.sleep(2)
        end_of_lowmode = f"""{ROS_SETUP} && ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{{
                            header: {{
                                stamp: {{sec: 0, nanosec: 0}},
                                frame_id: map
                            }},
                            pose: {{
                                pose: {{
                                position: {{x: -7.652982711791992, y: 0.5075767040252686, z: 0.0}},
                                orientation: {{x: 0.0, y: 0.0, z: 0.9647117067140524, w: 0.263308417884541}}
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
        print("[init] auto_localize:", end_of_lowmode)
        subprocess.call(["bash", "-lc", end_of_lowmode])


        # 3) /amcl_pose 1건 대기 (사용자 2D Pose Estimate 입력 신호)
        #    출력은 버리지만, 한 건 수신되면 즉시 리턴해서 다음 단계로 넘어갑니다.
        # wait_cmd = f"{ROS_SETUP} && timeout 8s ros2 topic echo /amcl_pose > /dev/null" # -n1 Doesn't exist (Jaewon)
        print("[wait] /amcl_pose 1 message waiting ...")
        # ret = subprocess.call(["bash", "-lc", wait_cmd])
        # if ret != 0:
        #     print("[warn] /amcl_pose 대기 중 비정상 종료. 그래도 navigation을 시도합니다.")
        wait_for_first_message("/amcl_pose",  timeout_s=60)
        time.sleep(3)
        # 4) Navigation 실행
        nav_cmd = f"""{ROS_SETUP} && ros2 launch nav2_bringup navigation_launch.py params_file:=map2_nav2_params.yaml"""
        print("[run] navigation:", nav_cmd)
        nav = popen_bash(nav_cmd)
        procs.append(nav)

        wait_for_first_message("/local_costmap/costmap",  timeout_s=60)
        wait_for_first_message("/global_costmap/costmap", timeout_s=60)

        time.sleep(1.5)
        go_cmd = f"""{ROS_SETUP} && python3 /home/mr/nav2_ws/go2_move.py"""
        print("[run] go2_move:", go_cmd)
        go = popen_bash(go_cmd); 
        procs.append(go)

        time.sleep(7)
        # 5) junbeom.py 실행
        junbeom_cmd = f"""{ROS_SETUP} && python3 example_simple_waypoints_junbeom2.py"""
        print("[run] junbeom_waypoints:", junbeom_cmd)
        junbeom = popen_bash(junbeom_cmd)
        procs.append(junbeom)

        print("[info] waiting for junbeom script to finish...")
        junbeom.wait()
        kill_all()

        # map2_cmd = f"""{ROS_SETUP} && python3 /home/mr/nav2_ws/run_nav2_sequence2.py"""
        # print("[run] map1:", map2_cmd)
        # map2 = popen_bash(map2_cmd)
        # procs.append(map2)

        # print("[info] waiting for map1 script to finish...")
        # map2.wait()
        time.sleep(3.0)
        # Jaewon
        # -------------------------------------------------------------------------

        MAP_YAML = "/home/mr/nav2_ws/my_map2_edit.yaml"

        climb_stair_cmd = f"""{ROS_SETUP} && python3 climb_stair.py"""
        print("[run] climb_stair:", climb_stair_cmd)
        climb_stair = popen_bash(climb_stair_cmd)
        procs.append(climb_stair)

        print("[info] waiting for junbeom script to finish...")
        climb_stair.wait()

        loc_cmd3 = f"""{ROS_SETUP} && ros2 launch nav2_bringup localization_launch.py map:={MAP_YAML}"""
        print("[run] localization:", loc_cmd3)
        loc3 = popen_bash(loc_cmd3)
        procs.append(loc3)

        time.sleep(0.5)
        localization2_cmd = f"""{ROS_SETUP} && ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{{
                            header: {{
                                stamp: {{sec: 0, nanosec: 0}},
                                frame_id: map
                            }},
                            pose: {{
                                pose: {{
                                position: {{x: 2.727153778076172, y: 2.549814462661743, z: 0.0}},
                                orientation: {{x: 0.0, y: 0.0, z: 0.9999751939800637, w: 0.007043537785371638}}
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
        print("[init] auto_localize:", localization2_cmd)
        subprocess.call(["bash", "-lc", localization2_cmd])
        
        print("[wait] /amcl_pose 1 message waiting ...")
        wait_for_first_message("/amcl_pose",  timeout_s=60)
        time.sleep(5)

        time.sleep(0.5)
        good_stair_cmd = f"""{ROS_SETUP} && python3 /home/mr/nav2_ws/good_stair.py"""
        good_stair = popen_bash(good_stair_cmd)
        procs.append(good_stair)

        print("[info] waiting for junbeom script to finish...")
        good_stair.wait()

        climb_stair_cmd = f"""{ROS_SETUP} && python3 climb_stair.py"""
        print("[run] climb_stair:", climb_stair_cmd)
        climb_stair = popen_bash(climb_stair_cmd)
        procs.append(climb_stair)

        print("[info] waiting for junbeom script to finish...")
        climb_stair.wait()

        kill_all()
        time.sleep(1.5)

        # --------------------------------------------------------------------------------------------------------------

        MAP_YAML = "/home/mr/nav2_ws/my_map3_edit.yaml"

        loc_cmd4 = f"""{ROS_SETUP} && ros2 launch nav2_bringup localization_launch.py map:={MAP_YAML}"""
        print("[run] localization:", loc_cmd4)
        loc4 = popen_bash(loc_cmd4)
        procs.append(loc4)

        time.sleep(0.5)
        localization3_cmd = f"""{ROS_SETUP} && ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{{
                            header: {{
                                stamp: {{sec: 0, nanosec: 0}},
                                frame_id: map
                            }},
                            pose: {{
                                pose: {{
                                position: {{x: 6.440149307250977, y: 1.724936842918396, z: 0.0}},
                                orientation: {{x: 0.0, y: 0.0, z: -0.008334371881327926, w: 0.999965268519534}}
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
        print("[init] auto_localize:", localization3_cmd)
        subprocess.call(["bash", "-lc", localization3_cmd])
        
        print("[wait] /amcl_pose 1 message waiting ...")
        wait_for_first_message("/amcl_pose",  timeout_s=60)
        time.sleep(3)

        time.sleep(0.5)
        map3_total_cmd = f"""{ROS_SETUP} && python3 /home/mr/nav2_ws/map3_total.py"""
        map3_total = popen_bash(map3_total_cmd)
        procs.append(map3_total)

        print("[info] waiting for junbeom script to finish...")
        map3_total.wait()

        time.sleep(0.5)
        localization4_cmd = f"""{ROS_SETUP} && ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{{
                            header: {{
                                stamp: {{sec: 0, nanosec: 0}},
                                frame_id: map
                            }},
                            pose: {{
                                pose: {{
                                position: {{x: 6.637019634246826, y: 8.035152435302734, z: 0.0}},
                                orientation: {{x: 0.0, y: 0.0, z: 0.9999999999980848, w: 1.957183671412763e-06}}
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
        print("[init] auto_localize:", localization4_cmd)
        subprocess.call(["bash", "-lc", localization4_cmd])

        time.sleep(0.5)

        traffic_cmd = f"""{ROS_SETUP} && python3 /home/mr/ros2_ws/src/traffic_pkg/traffic_pkg/crosswalk_amcl_lidar_clean_finish_different_node.py"""
        print("[run] junbeom_waypoints:", traffic_cmd)
        traffic = popen_bash(traffic_cmd)
        procs.append(traffic)

        print("[info] waiting for junbeom script to finish...")
        traffic.wait()

        # 끝나면 전체 종료
        print("[info] junbeom finished, shutting down all processes...")
        kill_all()
        time.sleep(1.5)

        return  # main 함수 종료

    except KeyboardInterrupt:
        print("\n[info] CTRL+C detected, shutting down...")
    finally:
        kill_all()

if __name__ == "__main__":
    main()
