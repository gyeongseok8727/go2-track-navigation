#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from dataclasses import dataclass

@dataclass
class TestOption:
    name: str
    id: int

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
]

ID_TO_NAME = {opt.id: opt.name for opt in OPTION_LIST}
NAME_TO_ID = {opt.name: opt.id for opt in OPTION_LIST}

HELP_TEXT = """\
[Go2 Sport Command Publisher]
- 정수 ID 또는 이름으로 퍼블리시합니다.
- 'list' : 사용 가능한 명령 목록 출력
- 'q' 또는 'quit' : 종료

예)
  3            # move forward
  move rotate  # name으로도 입력 가능
"""

class Go2SportPublisher(Node):
    def __init__(self):
        super().__init__('go2_sport_publisher')
        self.pub = self.create_publisher(Int32, '/go2/sport_cmd', 10)
        self.get_logger().info("Publisher ready on /go2/sport_cmd")
        print(HELP_TEXT)
        self.print_list()

    def print_list(self):
        print("Available options:")
        for opt in OPTION_LIST:
            print(f"  {opt.id:>2} : {opt.name}")
        print()

    def send(self, cmd_id: int):
        msg = Int32()
        msg.data = int(cmd_id)
        self.pub.publish(msg)
        self.get_logger().info(f"Published id={cmd_id} ({ID_TO_NAME.get(cmd_id, 'unknown')})")

def main():
    rclpy.init()
    node = Go2SportPublisher()
    try:
        # 간단한 동기 입력 루프 (spin 없이도 publish 동작함)
        while rclpy.ok():
            try:
                user_in = input("Enter id or name ('list' for menu, 'q' to quit): ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not user_in:
                continue
            if user_in.lower() in ["q", "quit"]:
                break
            if user_in.lower() == "list":
                node.print_list()
                continue

            # 이름으로 입력된 경우
            if user_in in NAME_TO_ID:
                node.send(NAME_TO_ID[user_in])
                continue

            # 정수로 입력된 경우
            try:
                cmd_id = int(user_in)
                if cmd_id in ID_TO_NAME:
                    node.send(cmd_id)
                else:
                    node.get_logger().warn(f"Unknown id: {cmd_id}")
            except ValueError:
                node.get_logger().warn(f"Unknown command: {user_in}")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
