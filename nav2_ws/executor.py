#!/usr/bin/env python3
import os
import time
import shutil
import subprocess
from typing import List

WORKDIR = os.path.expanduser("~/nav2_ws")
ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP  = os.path.join(WORKDIR, "install", "setup.bash")

CMD_LAUNCH = "ros2 launch nav2_simple_commander example_simple_waypoints_junbeom.launch.py"
CMD_RUN    = "ros2 run nav2_simple_commander example_simple_waypoints_junbeom"
DELAY_SEC  = 10  # launch 이후 run까지 지연(초)

# 우선순위대로 사용 가능한 터미널 탐색
CANDIDATES: List[str] = [
    "gnome-terminal", "konsole", "xfce4-terminal", "tilix",
    "alacritty", "kitty", "xterm"
]

def find_terminal() -> str:
    for t in CANDIDATES:
        if shutil.which(t):
            return t
    raise RuntimeError(
        "사용할 수 있는 터미널 에뮬레이터가 없습니다. "
        "gnome-terminal/konsole/xfce4-terminal/tilix/alacritty/kitty/xterm 중 하나를 설치해주세요."
    )

def build_shell_command(user_cmd: str) -> str:
    """
    새 터미널 내부에서 실행할 bash 커맨드 문자열.
    - ROS humble + 워크스페이스 오버레이를 source
    - 작업 디렉터리 이동
    - 실제 명령 실행
    - 끝나도 창 유지(Exec bash)
    """
    lines = [
        f'cd "{WORKDIR}"',
        f'source "{ROS_SETUP}" >/dev/null 2>&1 || true',
        f'[ -f "{WS_SETUP}" ] && source "{WS_SETUP}" >/dev/null 2>&1 || true',
        user_cmd,
        'exec bash'  # 명령 종료 후에도 창 유지
    ]
    return "; ".join(lines)

def open_terminal(terminal: str, title: str, user_cmd: str) -> subprocess.Popen:
    cmdline = build_shell_command(user_cmd)

    if terminal == "gnome-terminal":
        args = [terminal, f"--title={title}", "--", "bash", "-ilc", cmdline]
    elif terminal == "konsole":
        args = [terminal, "--new-window", "-p", f"tabtitle={title}", "-e", "bash", "-ilc", cmdline]
    elif terminal == "xfce4-terminal":
        # -e는 문자열 하나로 받는 편이 호환성 좋음
        args = [terminal, "--title", title, "-e", f"bash -ilc \"{cmdline}\""]
    elif terminal == "tilix":
        args = [terminal, "--title", title, "-e", "bash", "-ilc", cmdline]
    elif terminal == "alacritty":
        args = [terminal, "-t", title, "-e", "bash", "-ilc", cmdline]
    elif terminal == "kitty":
        args = [terminal, "--title", title, "bash", "-ilc", cmdline]
    elif terminal == "xterm":
        args = [terminal, "-T", title, "-hold", "-e", "bash", "-ilc", cmdline]
    else:
        raise RuntimeError(f"알 수 없는 터미널: {terminal}")

    return subprocess.Popen(args, env=os.environ.copy())

def main():
    os.makedirs(WORKDIR, exist_ok=True)
    term = find_terminal()

    print(f"[info] 터미널: {term}")
    print(f"[info] 작업 디렉터리: {WORKDIR}")
    print(f"[info] 실행 1: {CMD_LAUNCH}")
    print(f"[info] 실행 2(지연 {DELAY_SEC}s): {CMD_RUN}")

    # 1) 새 창에서 launch 즉시 실행
    open_terminal(term, "NAV2 LAUNCH", CMD_LAUNCH)

    # 2) 지연 후 새 창에서 run 실행
    time.sleep(DELAY_SEC)
    open_terminal(term, "NAV2 RUN", CMD_RUN)

if __name__ == "__main__":
    main()
