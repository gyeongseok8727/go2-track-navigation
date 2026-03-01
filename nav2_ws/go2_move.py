#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from rclpy.executors import MultiThreadedExecutor
import time
import os
#import math
#from enum import Enum

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

import sys
import math
import time
import subprocess

# Unitree SDK2 (python)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

from process_executor import BaseProcessExecutor, ExecConfig

#sdk class definition
class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.mission = None
        self.stop = False

        # Unitree SportClient 준비
        # 참고: 네트워크/리스 환경에 따라 파라미터 조정
        self.get_logger().info('Initializing Unitree SportClient...')
        # Channel factory must be initialized before creating clients.
        ChannelFactoryInitialize(0)  # 0: default domain

        #init sportclient
        self.sc = SportClient(enableLease=False)
        self.sc.SetTimeout(10.0)
        self.sc.Init()
        self.sc.BalanceStand()

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        self.pub_mode = self.create_publisher(Int32, '/mode_cmd', 10)
        self.mode = None

        cfg = ExecConfig(
            cmd=["bash", "-i", "-c", "./go2_ctrl --network eno1"],
            cwd="/home/mr/unitree_rl_lab/deploy/robots/go2/build",
        )
        self.ex = BaseProcessExecutor(cfg)


        # /cmd_vel 구독
        self.sub_mission = self.create_subscription(
            Int32,
            'mission',
            self.sub_mission_callback,
            3
        )
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            3
        )
        self.sub_stop = self.create_subscription(
            Int32,          # 메시지 타입
            'stop_cmd',     # 토픽 이름
            self.sub_stop_callback,  # 콜백 함수
            10              # QoS (queue size)
        )
        
    def cmd_vel_callback(self, msg: Twist):
        #시형: nav_stop topic Subscriber
        #
        #
        if self.mission is None:
            vx = msg.linear.x
            vy = msg.linear.y
            wz = msg.angular.z
            if abs(vx) < 0.5 and vx != 0:
                vx = vx/abs(vx) * 0.5
            if abs(vy) < 0.5 and vy != 0:
                vy = vy/abs(vy) * 0.5
            if abs(wz) < 0.6 and wz != 0:
                wz = wz/abs(wz) * 0.6
                
            try:
                # Unitree SDK2py: 보통 Move(vx, vy, wz) 형태 (SDK 버전에 따라 달라질 수 있음)
                if self.stop:
                    pass
                else:
                    ret = self.sc.Move(vx, vy, wz)
                    self.get_logger().debug(f"Move({vx:.3f}, {vy:.3f}, {wz:.3f}) -> {ret}")
            except Exception as e:
                self.get_logger().error(f"SportClient.Move() failed: {e}")
    
    def sub_mission_callback(self, msg: Int32):
        self.mission = msg.data
        self.get_logger().info(f"Mission received: {self.mission}")
        if self.mission == 1:
            ret = self.sc.FrontJump()
            time.sleep(3)
            self.get_logger().info(f"FrontJump() -> {ret}")

        elif self.mission == 2:
            ret = self.sc.TrotRun()
            time.sleep(0.3)
        elif self.mission == 3:
            self.stop = True
            subprocess.run(
                ["python3", "/home/mr/ros2_ws/src/traffic_pkg/traffic_pkg/crosswalk_slow_stop_cmd.py", "eno1"], 
                check=True
            )
        elif self.mission == 4:
            self.stop = True
            subprocess.run(
                ["python3", "/home/mr/ros2_ws/src/obstacle_pkg/obstacle_pkg/avoid_go_oneshot.py", "eno1"], 
                check=True
            )
        elif self.mission == 5:
            self.enter_lowmode()
        elif self.mission == 6:
            self.exit_lowmode()
        elif self.mission == 7:
            self.sc.EconomicGait()
        elif self.mission == 8:
            self.sc.BalanceStand()
        
        
        self.mission = None

    def sub_stop_callback(self, msg: Int32):
        self.get_logger().info(f"Received STOP cmd: {msg.data}")
        if msg.data == 0:
            self.stop = False
        elif msg.data == 1:
            self.stop = True

    def destroy_node(self):
        try:
            self.sc.StopMove()
        except Exception:
            pass
        super().destroy_node()

    def send_mode(self, mode_value: int):
        msg = Int32()
        msg.data = mode_value
        self.pub_mode.publish(msg)
        self.get_logger().info(f'📤 Sent mode command: {mode_value}')

    def enter_lowmode(self):
        print("----- enter lowmode -----")
        self.sc.StandDown()
        time.sleep(3)
        self.msc.ReleaseMode()
        time.sleep(1)

        self.ex.start()
        time.sleep(4)

        self.send_mode(1)
        time.sleep(2)
        self.send_mode(2)

    def exit_lowmode(self):
        self.send_mode(3)
        time.sleep(1)
        self.ex.stop()
        print("----- lowlevel exit -----")
        time.sleep(3)

        self.msc.SelectMode("mcf")

def main():
    rclpy.init()

    cmd_sub_node = CmdVelSubscriber()

    try:
        rclpy.spin(cmd_sub_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # 초기화 실패 등 치명적 오류
        print(f"[ERROR] {e}", file=sys.stderr)
    finally:
        cmd_sub_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
