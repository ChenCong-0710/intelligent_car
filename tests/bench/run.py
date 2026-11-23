#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""台架功能测试程序入口：python tests/bench/run.py <case> [options]

用例：status | turn | side-tilt | side-parallel | faces2 | selftest
帮助：python tests/bench/run.py --help
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tests.bench.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
