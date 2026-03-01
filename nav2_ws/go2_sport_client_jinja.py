import time
import sys
import math
from dataclasses import dataclass
from typing import Optional, Callable, Dict

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
)

# =========================
# 유틸 파서
# =========================
def parse_float(prompt: str, default: Optional[float] = None) -> float:
    while True:
        s = input(f"{prompt}{' ['+str(default)+']' if default is not None else ''}: ").strip()
        if s == "" and default is not None:
            return default
        try:
            return float(s)
        except ValueError:
            print("숫자를 입력하세요.")

def parse_int(prompt: str, default: Optional[int] = None) -> int:
    while True:
        s = input(f"{prompt}{' ['+str(default)+']' if default is not None else ''}: ").strip()
        if s == "" and default is not None:
            return default
        try:
            return int(s)
        except ValueError:
            print("정수를 입력하세요.")

def parse_bool(prompt: str, default: Optional[bool] = None) -> bool:
    guide = " (y/n"
    if default is not None:
        guide += f", default={'y' if default else 'n'}"
    guide += ")"
    while True:
        s = input(f"{prompt}{guide}: ").strip().lower()
        if s == "" and default is not None:
            return default
        if s in ("y", "yes", "true", "1", "on", "t"):
            return True
        if s in ("n", "no", "false", "0", "off", "f"):
            return False
        print("y 또는 n을 입력하세요.")

def confirm_danger(action_name: str) -> bool:
    print(f"[안전 확인] '{action_name}' 동작은 위험할 수 있습니다.")
    print("주변 장애물이 없고, 충분한 공간이 있는지 반드시 확인하세요.")
    s = input("정말 실행하려면 'YES'를 입력하세요: ").strip()
    return s == "YES"

# =========================
# 옵션 정의
# =========================
@dataclass
class TestOption:
    name: str
    id: int

# 기존 ID 유지 + 신규 기능 추가
option_list = [
    TestOption(name="damp", id=0),
    TestOption(name="stand_up", id=1),
    TestOption(name="stand_down", id=2),

    TestOption(name="move_forward", id=3),
    TestOption(name="move_lateral", id=4),
    TestOption(name="move_rotate", id=5),
    TestOption(name="stop_move", id=6),

    TestOption(name="hand_stand", id=7),
    TestOption(name="sit", id=8),                 # 추가
    TestOption(name="balance_stand", id=9),
    TestOption(name="recovery", id=10),
    TestOption(name="left_flip", id=11),
    TestOption(name="back_flip", id=12),
    TestOption(name="free_walk", id=13),

    TestOption(name="free_bound", id=14),
    TestOption(name="free_avoid", id=15),
    TestOption(name="speed_level", id=16),        # 추가
    TestOption(name="walk_upright", id=17),
    TestOption(name="cross_step", id=18),
    TestOption(name="free_jump", id=19),

    TestOption(name="move_custom", id=20),        # 추가 (vx, vy, vyaw)
    TestOption(name="euler_deg", id=21),          # 추가 (deg 입력 → rad 변환)
    TestOption(name="hello", id=22),              # 추가
    TestOption(name="stretch", id=23),            # 추가
    TestOption(name="content", id=24),            # 추가
    TestOption(name="dance1", id=25),             # 추가
    TestOption(name="dance2", id=26),             # 추가
    TestOption(name="switch_joystick", id=27),    # 추가 (bool)
    TestOption(name="pose", id=28),               # 추가 (bool)
    TestOption(name="scrape", id=29),             # 추가
    TestOption(name="front_flip", id=30),         # 추가
    TestOption(name="front_jump", id=31),         # 추가
    TestOption(name="front_pounce", id=32),       # 추가
    TestOption(name="heart", id=33),              # 추가
    TestOption(name="static_walk", id=34),        # 추가
    TestOption(name="trot_run", id=35),           # 추가
    TestOption(name="classic_walk", id=36),       # 추가 (bool)
    TestOption(name="auto_recovery_set", id=37),  # 추가 (bool)
    TestOption(name="auto_recovery_get", id=38),  # 추가
    TestOption(name="switch_avoid_mode", id=39),  # 추가
    TestOption(name="rise_sit", id=40),           # 추가
]

# =========================
# 사용자 인터페이스
# =========================
class UserInterface:
    def __init__(self, options):
        self.options = options
        self.selected: Optional[TestOption] = None

    def convert_to_int(self, s: str):
        try:
            return int(s)
        except ValueError:
            return None

    def print_list(self):
        print("=== 옵션 목록 ===")
        for o in self.options:
            print(f"{o.name:20s}  id: {o.id}")
        print("================")

    def print_help(self):
        print("명령 예시:")
        print("  list              - 옵션 목록 출력")
        print("  help              - 이 도움말")
        print("  quit / q          - 종료")
        print("  <id> 또는 <name>  - 해당 옵션 실행")
        print("실행 후, 파라미터가 필요한 옵션은 추가 입력을 요청합니다.")

    def ask(self):
        s = input("Enter id or name (list/help/quit): ").strip()
        if s.lower() in ("quit", "q"):
            return "QUIT"
        if s.lower() == "list":
            self.print_list()
            return None
        if s.lower() == "help":
            self.print_help()
            return None

        # select by id or name
        as_int = self.convert_to_int(s)
        for o in self.options:
            if s == o.name or (as_int is not None and as_int == o.id):
                self.selected = o
                print(f"Selected: {o.name} (id={o.id})")
                return "RUN"
        print("일치하는 옵션이 없습니다. 'list'로 확인하세요.")
        return None

# =========================
# 핸들러 구현
# =========================
def handle_move_forward(sc: SportClient):
    ret = sc.Move(0.3, 0.0, 0.0)
    print("ret:", ret)

def handle_move_lateral(sc: SportClient):
    ret = sc.Move(0.0, 0.3, 0.0)
    print("ret:", ret)

def handle_move_rotate(sc: SportClient):
    ret = sc.Move(0.0, 0.0, 0.5)
    print("ret:", ret)

def handle_move_custom(sc: SportClient):
    vx = parse_float("vx [m/s]", 0.0)
    vy = parse_float("vy [m/s]", 0.0)
    vyaw = parse_float("vyaw [rad/s]", 0.0)
    ret = sc.Move(vx, vy, vyaw)
    print("ret:", ret)

def handle_stop_move(sc: SportClient):
    ret = sc.StopMove()
    print("ret:", ret)

def handle_speed_level(sc: SportClient):
    lvl = parse_int("speed level (예: 0,1,2)", 1)
    code = sc.SpeedLevel(lvl)
    print("ret:", code)

def handle_euler_deg(sc: SportClient):
    roll_d = parse_float("roll [deg]", 0.0)
    pitch_d = parse_float("pitch [deg]", 0.0)
    yaw_d = parse_float("yaw [deg]", 0.0)
    roll = math.radians(roll_d)
    pitch = math.radians(pitch_d)
    yaw = math.radians(yaw_d)
    code = sc.Euler(roll, pitch, yaw)
    print("ret:", code)

def handle_toggle_for_seconds(start_fn, stop_fn, label: str, default_sec=4.0):
    if not confirm_danger(label):
        print("사용자 취소")
        return
    sec = parse_float("지속 시간 [sec]", default_sec)
    code, code2 = None, None
    try:
        code = start_fn(True)
        print("ret(start):", code)
        time.sleep(sec)
    finally:
        code2 = stop_fn(False)
        print("ret(stop):", code2)

def handle_simple_call(sc_fn: Callable[[], int], label: str, dangerous=False):
    if dangerous and not confirm_danger(label):
        print("사용자 취소")
        return
    ret = sc_fn()
    print("ret:", ret)

def handle_bool_toggle(sc_fn: Callable[[bool], int], label: str, default_on=True):
    on = parse_bool(f"{label} 시작할까요?", default_on)
    ret = sc_fn(on)
    print("ret:", ret)

def handle_classic_walk(sc: SportClient):
    on = parse_bool("classic_walk on?", True)
    ret = sc.ClassicWalk(on)
    print("ret:", ret)

def handle_auto_recovery_set(sc: SportClient):
    en = parse_bool("Auto Recovery 활성화할까요?", True)
    ret = sc.AutoRecoverySet(en)
    print("ret:", ret)

def handle_auto_recovery_get(sc: SportClient):
    code, enabled = sc.AutoRecoveryGet()
    print(f"ret: {code}, enabled: {enabled}")

# =========================
# 메인
# =========================
def main():
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    sc = SportClient(enableLease=False)
    sc.SetTimeout(10.0)
    sc.Init()

    ui = UserInterface(option_list)

    # id -> handler 매핑
    handlers: Dict[int, Callable[[SportClient], None]] = {
        0:  lambda s: handle_simple_call(s.Damp, "damp"),
        1:  lambda s: handle_simple_call(s.StandUp, "stand_up"),
        2:  lambda s: handle_simple_call(s.StandDown, "stand_down"),

        3:  handle_move_forward,
        4:  handle_move_lateral,
        5:  handle_move_rotate,
        6:  handle_stop_move,

        7:  lambda s: handle_toggle_for_seconds(s.HandStand, s.HandStand, "hand_stand"),
        8:  lambda s: handle_simple_call(s.Sit, "sit"),
        9:  lambda s: handle_simple_call(s.BalanceStand, "balance_stand"),
        10: lambda s: handle_simple_call(s.RecoveryStand, "recovery", dangerous=True),
        11: lambda s: handle_simple_call(s.LeftFlip, "left_flip", dangerous=True),
        12: lambda s: handle_simple_call(s.BackFlip, "back_flip", dangerous=True),
        13: lambda s: handle_simple_call(s.FreeWalk, "free_walk"),

        14: lambda s: handle_toggle_for_seconds(s.FreeBound, s.FreeBound, "free_bound", default_sec=2.0),
        15: lambda s: handle_toggle_for_seconds(s.FreeAvoid, s.FreeAvoid, "free_avoid", default_sec=2.0),
        16: handle_speed_level,
        17: lambda s: handle_toggle_for_seconds(s.WalkUpright, s.WalkUpright, "walk_upright", default_sec=4.0),
        18: lambda s: handle_toggle_for_seconds(s.CrossStep, s.CrossStep, "cross_step", default_sec=4.0),
        19: lambda s: handle_toggle_for_seconds(s.FreeJump, s.FreeJump, "free_jump", default_sec=4.0),

        20: handle_move_custom,
        21: handle_euler_deg,
        22: lambda s: handle_simple_call(s.Hello, "hello"),
        23: lambda s: handle_simple_call(s.Stretch, "stretch"),
        24: lambda s: handle_simple_call(s.Content, "content"),
        25: lambda s: handle_simple_call(s.Dance1, "dance1"),
        26: lambda s: handle_simple_call(s.Dance2, "dance2"),
        27: lambda s: handle_bool_toggle(s.SwitchJoystick, "switch_joystick"),
        28: lambda s: handle_bool_toggle(s.Pose, "pose"),
        29: lambda s: handle_simple_call(s.Scrape, "scrape"),
        30: lambda s: handle_simple_call(s.FrontFlip, "front_flip", dangerous=True),
        31: lambda s: handle_simple_call(s.FrontJump, "front_jump", dangerous=True),
        32: lambda s: handle_simple_call(s.FrontPounce, "front_pounce", dangerous=True),
        33: lambda s: handle_simple_call(s.Heart, "heart"),
        34: lambda s: handle_simple_call(s.StaticWalk, "static_walk"),
        35: lambda s: handle_simple_call(s.TrotRun, "trot_run"),
        36: handle_classic_walk,
        37: handle_auto_recovery_set,
        38: handle_auto_recovery_get,
        39: lambda s: handle_simple_call(s.SwitchAvoidMode, "switch_avoid_mode"),
        40: lambda s: handle_simple_call(s.RiseSit, "rise_sit"),
    }

    # 루프
    while True:
        action = ui.ask()
        if action == "QUIT":
            print("종료합니다.")
            break
        if action != "RUN":
            continue

        opt = ui.selected
        if opt is None:
            continue

        handler = handlers.get(opt.id)
        if handler is None:
            print("아직 구현되지 않은 옵션입니다.")
            continue

        try:
            handler(sc)
        except KeyboardInterrupt:
            print("\n사용자 중단(Ctrl+C)")
        except Exception as e:
            print(f"오류 발생: {e}")

        # Move 명령과 혼동 방지용 짧은 대기
        time.sleep(0.5)

if __name__ == "__main__":
    main()
