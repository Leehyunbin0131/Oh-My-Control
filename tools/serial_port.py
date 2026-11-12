"""Shared read-only serial device availability check."""

import os
from pathlib import Path
import shutil
import stat
import subprocess


def check_serial_port(port):
    path = Path(port)
    if not path.exists():
        raise RuntimeError(f'장치를 찾을 수 없습니다: {port} (USB 연결/전원 확인)')
    if not stat.S_ISCHR(path.stat().st_mode):
        raise RuntimeError(f'시리얼 장치 파일이 아닙니다: {port}')
    if not os.access(path, os.R_OK | os.W_OK):
        raise RuntimeError(f'시리얼 접근 권한이 없습니다: {port} (dialout 그룹 확인)')
    if shutil.which('fuser'):
        result = subprocess.run(['fuser', str(path.resolve())], capture_output=True,
                                text=True, timeout=3)
        if result.returncode == 0:
            raise RuntimeError(f'다른 프로세스가 사용 중입니다: {port}, PID {result.stdout.strip()}')
