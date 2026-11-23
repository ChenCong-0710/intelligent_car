# -*- coding: utf-8 -*-
"""小车台架功能测试程序（独立于固件功能代码，只通过固件功能命令驱动）。

模块划分：
  protocol.py  固件日志解析 + 判据（纯函数，可离线自检）
  link.py      串口收发（只懂字节/行，不懂业务）
  device.py    功能命令封装（status/us/set/turn/side/leg/probe/stop）
  cases.py     用例：status / turn / side-tilt / side-parallel / faces2
  report.py    PASS/FAIL 打印 + 原始日志/JSON 落盘
  cli.py       命令行入口
  selftest.py  判据的离线自检（不需要小车）
  run.py       入口脚本
"""
