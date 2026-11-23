# -*- coding: utf-8 -*-
"""测试报告：打印 + 落盘（原始日志 + JSON 结果）。"""
from __future__ import annotations

import datetime
import json
import os
from pathlib import Path
from typing import Iterable, Optional

from . import protocol

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA_DIR = REPO_ROOT / "data" / "bench"


def _fmt(value) -> str:
    if isinstance(value, float):
        return "%.2f" % value
    return str(value)


def print_result(result: protocol.CaseResult) -> None:
    print("\n" + "=" * 72)
    print("CASE: %s   ->   %s   (%d/%d checks, %.1f s)"
          % (result.case, "PASS" if result.ok else "FAIL",
             len(result.checks) - result.fail_count(), len(result.checks),
             (result.elapsed_ms or 0) / 1000.0))
    print("=" * 72)
    for check in result.checks:
        print("  " + str(check))
    if result.metrics:
        print("  -- metrics --")
        for key in sorted(result.metrics):
            value = result.metrics[key]
            if isinstance(value, dict):
                inner = " ".join("%s=%s" % (k, _fmt(v)) for k, v in sorted(value.items()))
                print("    %-16s %s" % (key + ":", inner))
            else:
                print("    %-16s %s" % (key + ":", _fmt(value)))
    for note in result.notes:
        print("  note: " + note)


def save_case(result: protocol.CaseResult, lines: Iterable[str],
              data_dir: Optional[Path] = None, port: str = "") -> tuple[Path, Path]:
    """把原始日志和结构化结果落盘，返回 (log_path, json_path)。"""
    root = Path(data_dir) if data_dir else DEFAULT_DATA_DIR
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    base = "%s_%s" % (result.case.replace("/", "-"), stamp)
    log_path = root / (base + ".log")
    json_path = root / (base + ".json")
    log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    payload = result.to_dict()
    payload.update({
        "port": port,
        "saved_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "log_file": log_path.name,
    })
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return log_path, json_path


def print_summary(results: Iterable[protocol.CaseResult]) -> bool:
    results = list(results)
    print("\n" + "#" * 72)
    print("# SUMMARY")
    print("#" * 72)
    all_ok = True
    for res in results:
        all_ok = all_ok and res.ok
        print("  %-14s %-4s  %d/%d checks  %6.1f s"
              % (res.case, "PASS" if res.ok else "FAIL",
                 len(res.checks) - res.fail_count(), len(res.checks),
                 (res.elapsed_ms or 0) / 1000.0))
    print("\n  RESULT: %s" % ("ALL PASS" if all_ok else "FAIL"))
    return all_ok
