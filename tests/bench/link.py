# -*- coding: utf-8 -*-
"""串口收发层：只负责字节/行收发与超时，不懂业务命令。

设计目标（都是上一轮实车踩出来的坑）：
  * 只有一行超时/等待逻辑，避免每个脚本各写一套；
  * 所有等待都带超时上限，绝不无限阻塞；
  * 记录所有收到的原始行，测试报告里可回放；
  * open 失败/端口被占用时给出可操作的错误信息。
"""
from __future__ import annotations

import os
import time
from typing import Callable, Iterable, Optional

try:
    import serial  # pyserial
except ImportError as exc:  # pragma: no cover - 环境问题
    raise SystemExit("缺少 pyserial：pip install pyserial") from exc


class LinkError(RuntimeError):
    """串口打不开 / 写入失败。"""


def default_port() -> str:
    return (os.environ.get("BENCH_PORT")
            or os.environ.get("M4_PORT")
            or os.environ.get("M3_PORT")
            or "COM3")


class SerialLink:
    """一行一条命令的串口连接。"""

    def __init__(self, port: Optional[str] = None, baud: int = 115200,
                 read_timeout: float = 0.05, write_timeout: float = 1.0):
        self.port = port or default_port()
        self.baud = baud
        self._read_timeout = read_timeout
        self._write_timeout = write_timeout
        self._ser = None
        self.lines: list[str] = []      # 本会话收到的所有非空行（原始）
        self.dropped = 0                # 解码失败的行数

    # -- 打开/关闭 -------------------------------------------------------
    def open(self) -> "SerialLink":
        if self._ser is not None:
            return self
        try:
            self._ser = serial.Serial(self.port, self.baud,
                                      timeout=self._read_timeout,
                                      write_timeout=self._write_timeout)
        except Exception as exc:  # serial.SerialException 等
            raise LinkError(
                "打不开串口 %s：%r\n"
                "  1) 确认 USB-TTL/蓝牙已连接（本车一般是 COM3）\n"
                "  2) 关掉别的串口工具（Serial Monitor / 上位机）\n"
                "  3) 换端口：set BENCH_PORT=COMx（或 --port COMx）" % (self.port, exc)) from exc
        time.sleep(0.15)
        self._ser.reset_input_buffer()
        return self

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def __enter__(self) -> "SerialLink":
        return self.open()

    def __exit__(self, *exc_info) -> None:
        self.close()

    # -- 发送 -------------------------------------------------------------
    def send(self, command: str, settle: float = 0.12) -> None:
        """发一行命令（自动加回车换行）。settle 给固件一点解析时间。"""
        if self._ser is None:
            raise LinkError("串口未打开")
        if isinstance(command, bytes):
            payload = command
        else:
            payload = (command + "\r\n").encode("utf-8", "replace")
        try:
            self._ser.write(payload)
            self._ser.flush()
        except Exception as exc:
            raise LinkError("写入串口失败（%s）：%r" % (self.port, exc)) from exc
        if settle > 0:
            time.sleep(settle)

    # -- 接收 -------------------------------------------------------------
    def _readline(self, deadline: float) -> Optional[str]:
        while time.monotonic() < deadline:
            raw = self._ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", "replace").strip()
            if not text:
                continue
            self.lines.append(text)
            return text
        return None

    def collect(self, seconds: float) -> list[str]:
        """在 seconds 秒内尽量多地收行。"""
        deadline = time.monotonic() + seconds
        out = []
        while True:
            text = self._readline(deadline)
            if text is None:
                return out
            out.append(text)

    def wait_for(self, predicate: Callable[[str], bool], timeout: float) -> tuple[Optional[str], list[str]]:
        """收行直到 predicate(line) 为真或超时。返回 (命中的行, 这期间收到的行)。"""
        deadline = time.monotonic() + timeout
        got = []
        while True:
            text = self._readline(deadline)
            if text is None:
                return None, got
            got.append(text)
            if predicate(text):
                return text, got

    def wait_prefix(self, prefixes: Iterable[str], timeout: float) -> tuple[Optional[str], list[str]]:
        prefixes = tuple(prefixes)
        return self.wait_for(lambda line: line.startswith(prefixes), timeout)

    def wait_regex(self, pattern, timeout: float) -> tuple[Optional[str], list[str]]:
        import re
        rx = re.compile(pattern)
        return self.wait_for(lambda line: bool(rx.search(line)), timeout)

    def drain(self, seconds: float = 0.2) -> list[str]:
        return self.collect(seconds)

    # -- 便于带上下文打印 -------------------------------------------------
    def clear(self) -> None:
        self.lines = []
