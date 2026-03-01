import os
import signal
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

BUILD_DIR = "/home/mr/unitree_rl_lab/deploy/robots/go2/build"
CMD = ["./go2_ctrl", "--network", "lo"]

# go2_ctrl에 “ROS 환경변수”가 절대 안 가도록 최소 환경 구성
MINIMAL_ENV = {
    "PATH": "/usr/bin:/bin",
    "HOME": str(Path.home()),
    "LANG": "C.UTF-8",
    # 필요한 게 있으면 여기에만 추가 (예: LD_LIBRARY_PATH 등)
    # "LD_LIBRARY_PATH": "/usr/local/lib",
}

class ProcessManagerNode(Node):
    def __init__(self):
        super().__init__("process_manager")
        self.proc = None

        # 서비스 등록
        self.start_srv = self.create_service(Trigger, "start_exe", self.handle_start)
        self.stop_srv  = self.create_service(Trigger, "stop_exe", self.handle_stop)

        # autostart 파라미터(기본 True)
        self.declare_parameter("autostart", True)
        if self.get_parameter("autostart").get_parameter_value().bool_value:
            self._start()

    # ---- 내부 구현 ----
    def _start(self):
        if self.proc and self.proc.poll() is None:
            self.get_logger().info("Process already running.")
            return True
        try:
            self.proc = subprocess.Popen(
                CMD,
                cwd=BUILD_DIR,
                env=MINIMAL_ENV,       # ← ROS env 차단 포인트
                preexec_fn=os.setsid,  # 프로세스 그룹 생성(회수 시 유용)
            )
            self.get_logger().info(f"Started: PID={self.proc.pid}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to start: {e}")
            self.proc = None
            return False

    def _stop(self):
        if not self.proc or self.proc.poll() is not None:
            self.get_logger().info("Process not running.")
            return True

        pgid = os.getpgid(self.proc.pid)
        self.get_logger().info("Stopping process (SIGINT → SIGTERM → SIGKILL)...")

        # 1) SIGINT
        try:
            os.killpg(pgid, signal.SIGINT)
            self.proc.wait(timeout=3)
            return True
        except Exception:
            pass

        # 2) SIGTERM
        try:
            os.killpg(pgid, signal.SIGTERM)
            self.proc.wait(timeout=2)
            return True
        except Exception:
            pass

        # 3) SIGKILL
        try:
            os.killpg(pgid, signal.SIGKILL)
            self.proc.wait(timeout=2)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to kill: {e}")
            return False

    # ---- 서비스 핸들러 ----
    def handle_start(self, req, res):
        ok = self._start()
        res.success = ok
        res.message = "started" if ok else "failed to start"
        return res

    def handle_stop(self, req, res):
        ok = self._stop()
        res.success = ok
        res.message = "stopped" if ok else "failed to stop"
        return res

def main():
    rclpy.init()
    node = ProcessManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node._stop()   # 노드 종료 시 자식 프로세스 정리
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
