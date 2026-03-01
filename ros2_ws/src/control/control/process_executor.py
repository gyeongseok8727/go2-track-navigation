import os
import signal
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Union, IO

# 기본 환경: ROS 변수 미상속 (필요 항목만 지정)
DEFAULT_MINIMAL_ENV = {
    "PATH": "/usr/bin:/bin",
    "HOME": str(Path.home()),
    "LANG": "C.UTF-8",
    # 필요 시 여기에만 추가 (예: "LD_LIBRARY_PATH": "/usr/local/lib")
}

@dataclass
class ExecConfig:
    cmd: List[str]                                 # 실행할 명령 (예: ["./go2_ctrl","--network","lo"])
    cwd: Union[str, Path]                          # 작업 디렉토리
    env: Dict[str, str] = field(default_factory=lambda: DEFAULT_MINIMAL_ENV.copy())
    use_process_group: bool = True                 # 프로세스 그룹 생성 (회수 시 유리)
    stdout: Optional[IO] = None                    # 표준출력 리다이렉션 (예: subprocess.PIPE)
    stderr: Optional[IO] = None                    # 표준에러 리다이렉션  (예: subprocess.PIPE)

class BaseProcessExecutor:
    """ROS와 독립된 실행/회수기. 환경 격리 + 안전한 종료 순서(SIGINT→TERM→KILL) 지원."""
    def __init__(self, config: ExecConfig):
        self.config = config
        self._proc: Optional[subprocess.Popen] = None

    @property
    def pid(self) -> Optional[int]:
        return self._proc.pid if self._proc else None

    def is_running(self) -> bool:
        return (self._proc is not None) and (self._proc.poll() is None)

    def start(self) -> None:
        if self.is_running():
            return  # 이미 실행 중
        kwargs = dict(
            cwd=str(self.config.cwd),
            env=self.config.env,
            stdout=self.config.stdout,
            stderr=self.config.stderr,
        )
        if self.config.use_process_group and os.name == "posix":
            kwargs["preexec_fn"] = os.setsid  # 새 프로세스 그룹
        self._proc = subprocess.Popen(self.config.cmd, **kwargs)

    def stop(self,
             timeout_sigint: float = 3.0,
             timeout_sigterm: float = 2.0) -> None:
        """부드럽게 종료(SIGINT)→안 되면 SIGTERM→그래도 안 되면 SIGKILL."""
        if not self.is_running():
            return
        assert self._proc is not None

        def _killgrp(sig):
            if os.name == "posix" and self.config.use_process_group:
                try:
                    os.killpg(os.getpgid(self._proc.pid), sig)
                except ProcessLookupError:
                    pass
            else:
                try:
                    self._proc.send_signal(sig)
                except ProcessLookupError:
                    pass

        # 1) SIGINT
        _killgrp(signal.SIGINT)
        try:
            self._proc.wait(timeout=timeout_sigint)
            return
        except subprocess.TimeoutExpired:
            pass

        # 2) SIGTERM
        _killgrp(signal.SIGTERM)
        try:
            self._proc.wait(timeout=timeout_sigterm)
            return
        except subprocess.TimeoutExpired:
            pass

        # 3) SIGKILL
        if os.name == "posix":
            _killgrp(signal.SIGKILL)
        else:
            try:
                self._proc.kill()
            except ProcessLookupError:
                pass
        try:
            self._proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            pass

    def wait(self, timeout: Optional[float] = None) -> Optional[int]:
        if self._proc is None:
            return None
        try:
            return self._proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None
