#!/usr/bin/env python3
import sys
import time
from dataclasses import dataclass
from typing import Optional, Callable, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3

# Unitree SDK
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

@dataclass
class TestOption:
    name: str
    id: int

# 원본과 동일한 매핑 (필요 시 확장 가능)
OPTION_LIST = [
    TestOption(name="damp", id=0),
    TestOption(name="stand_up", id=1),
    TestOption(name="stand_down", id=2),
    TestOption(name="move forward", id=3),
    TestOption(name="move lateral", id=4),
    TestOption(name="move rotate", id=5),
    TestOption(name="stop_move", id=6),
    TestOption(name="hand stand", id=7),
    TestOption(name="balanced stand", id=9),
    TestOption(name="recovery", id=10),
    TestOption(name="left flip", id=11),
    TestOption(name="back flip", id=12),
    TestOption(name="free walk", id=13),
    TestOption(name="free bound", id=14),
    TestOption(name="free avoid", id=15),
    TestOption(name="walk upright", id=17),
    TestOption(name="cross step", id=18),
    TestOption(name="free jump", id=19),
    TestOption(name="move", id=20),
]

ID_TO_NAME = {opt.id: opt.name for opt in OPTION_LIST}

class Go2SportSubscriber(Node):
    """
    /go2/sport_cmd (std_msgs/Int32) 로 들어오는 정수 ID에 따라 SportClient 동작 실행
    - 연속/토글형 액션은 원본 예제와 동일하게 일정 시간 후 자동으로 False 전환
    """
    def __init__(self):
        super().__init__("go2_sport_subscriber")

        # DDS 초기화 (원본 로직 유지)
        # 인자로 멀티캐스트 IP 등을 넘겼다면 그대로 사용 가능
        if len(sys.argv) > 1:
            ChannelFactoryInitialize(0, sys.argv[1])
        else:
            ChannelFactoryInitialize(0)

        # SportClient 구성
        self.sport = SportClient()
        self.sport.SetTimeout(10.0)
        self.sport.Init()
        self.get_logger().info("SportClient initialized.")

        # ROS 2 Subscriber
        self.sub = self.create_subscription(
            Int32, "/go2/sport_cmd", self.cmd_callback, 10
        )

        self.sub_move = self.create_subscription(
            Vector3, "/go2/move_cmd", self.move_callback, 10
        )

        # 중복 명령 과도 실행 방지
        self.last_cmd_id: Optional[int] = None
        self.last_exec_time = 0.0
        self.min_interval_sec = 0.2  # 동일 명령 연타시 0.2초 쿨다운

        # 토글형 기능의 auto-off를 위한 타이머 관리
        self._deferred_timers = []

        # move를 위한 x,y,z(rotate) command
        self.x = 0.0 # m/s
        self.y = 0.0 # m/s
        self.z = 0.0 #rad/s

        # 명령 처리 테이블
        self.dispatch: Dict[int, Callable[[], None]] = {
            0: self._damp,
            1: self._stand_up,
            2: self._stand_down,
            3: self._move_forward,
            4: self._move_lateral,
            5: self._move_rotate,
            6: self._stop_move,
            7: self._hand_stand_pulse,
            9: self._balance_stand,
            10: self._recovery_stand,
            11: self._left_flip,
            12: self._back_flip,
            13: self._free_walk,
            14: self._free_bound_pulse,
            15: self._free_avoid_pulse,
            17: self._walk_upright_pulse,
            18: self._cross_step_pulse,
            19: self._free_jump_pulse,
            20: self._move,
        }

        self.get_logger().info("Ready. Publish Int32 to /go2/sport_cmd")

    # ----------------------------
    # ROS2 Callback
    # ----------------------------
    def cmd_callback(self, msg: Int32):
        cmd_id = int(msg.data)
        now = time.time()

        # 간단한 디바운스
        if self.last_cmd_id == cmd_id and (now - self.last_exec_time) < self.min_interval_sec:
            return

        self.last_cmd_id = cmd_id
        self.last_exec_time = now

        if cmd_id not in self.dispatch:
            self.get_logger().warn(f"Unknown cmd id: {cmd_id}")
            return

        name = ID_TO_NAME.get(cmd_id, "unknown")
        self.get_logger().info(f"Executing: {name} (id={cmd_id})")
        try:
            self.dispatch[cmd_id]()
        except Exception as e:
            self.get_logger().error(f"Command {cmd_id} failed: {e}")

    def move_callback(self, msg: Vector3):
        cmd_id = 20
        now = time.time()

        # 간단한 디바운스
        if self.last_cmd_id == cmd_id and (now - self.last_exec_time) < self.min_interval_sec:
            return

        self.last_cmd_id = cmd_id
        self.last_exec_time = now

        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self._move()


    # ----------------------------
    # Helpers: Auto-off 타이머
    # ----------------------------
    def _schedule_after(self, delay_sec: float, fn: Callable[[], None]):
        # rclpy Timer로 지연 실행
        def _wrap():
            try:
                fn()
            except Exception as e:
                self.get_logger().error(f"Deferred action failed: {e}")

        timer = self.create_timer(delay_sec, lambda: self._cancel_timer_and_run(timer, _wrap))
        self._deferred_timers.append(timer)

    def _cancel_timer_and_run(self, timer, fn: Callable[[], None]):
        try:
            timer.cancel()
        except Exception:
            pass
        try:
            fn()
        finally:
            if timer in self._deferred_timers:
                self._deferred_timers.remove(timer)

    # ----------------------------
    # Command implementations
    # ----------------------------
    def _damp(self):
        self.sport.Damp()

    def _stand_up(self):
        self.sport.StandUp()

    def _stand_down(self):
        self.sport.StandDown()

    def _move_forward(self):
        ret = self.sport.Move(0.3, 0.0, 0.0)
        self.get_logger().info(f"Move forward ret={ret}")

    def _move_lateral(self):
        ret = self.sport.Move(0.0, 0.3, 0.0)
        self.get_logger().info(f"Move lateral ret={ret}")

    def _move_rotate(self):
        ret = self.sport.Move(0.0, 0.0, 0.5)
        self.get_logger().info(f"Move rotate ret={ret}")

    def _stop_move(self):
        self.sport.StopMove()

    def _hand_stand_pulse(self):
        self.sport.HandStand(True)
        # 원본: 4초 유지 후 해제
        self._schedule_after(4.0, lambda: self._safe_call("HandStand False", lambda: self.sport.HandStand(False)))

    def _balance_stand(self):
        self.sport.BalanceStand()

    def _recovery_stand(self):
        self.sport.RecoveryStand()

    def _left_flip(self):
        ret = self.sport.LeftFlip()
        self.get_logger().info(f"LeftFlip ret={ret}")

    def _back_flip(self):
        ret = self.sport.BackFlip()
        self.get_logger().info(f"BackFlip ret={ret}")

    def _free_walk(self):
        ret = self.sport.FreeWalk()
        self.get_logger().info(f"FreeWalk ret={ret}")

    def _free_bound_pulse(self):
        ret = self.sport.FreeBound(True)
        self.get_logger().info(f"FreeBound(True) ret={ret}")
        self._schedule_after(2.0, lambda: self._safe_call("FreeBound False", lambda: self.sport.FreeBound(False)))

    def _free_avoid_pulse(self):
        ret = self.sport.FreeAvoid(True)
        self.get_logger().info(f"FreeAvoid(True) ret={ret}")
        self._schedule_after(2.0, lambda: self._safe_call("FreeAvoid False", lambda: self.sport.FreeAvoid(False)))

    def _walk_upright_pulse(self):
        ret = self.sport.WalkUpright(True)
        self.get_logger().info(f"WalkUpright(True) ret={ret}")
        self._schedule_after(4.0, lambda: self._safe_call("WalkUpright False", lambda: self.sport.WalkUpright(False)))

    def _cross_step_pulse(self):
        ret = self.sport.CrossStep(True)
        self.get_logger().info(f"CrossStep(True) ret={ret}")
        self._schedule_after(4.0, lambda: self._safe_call("CrossStep False", lambda: self.sport.CrossStep(False)))

    def _free_jump_pulse(self):
        ret = self.sport.FreeJump(True)
        self.get_logger().info(f"FreeJump(True) ret={ret}")
        self._schedule_after(4.0, lambda: self._safe_call("FreeJump False", lambda: self.sport.FreeJump(False)))
    
    def _move(self):
        ret = self.sport.Move(self.x, self.y, self.z)
        self.get_logger().info(f"Move forward ret={ret}")
    
    def _safe_call(self, label: str, fn: Callable[[], None]):
        try:
            fn()
            self.get_logger().info(f"{label} OK")
        except Exception as e:
            self.get_logger().error(f"{label} failed: {e}")

def main():
    rclpy.init()
    node = Go2SportSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
