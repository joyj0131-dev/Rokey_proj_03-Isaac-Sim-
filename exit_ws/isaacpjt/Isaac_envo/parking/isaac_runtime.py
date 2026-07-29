#!/usr/bin/env python3
"""주차장 도구가 설치 위치와 무관하게 Isaac Sim Python을 찾도록 돕는다."""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _candidates() -> list[Path]:
    candidates: list[Path] = []
    explicit = os.environ.get("ISAAC_SIM_PYTHON")
    if explicit:
        candidates.append(Path(explicit).expanduser())

    for variable in ("ISAAC_SIM_ROOT", "ISAAC_SIM_PATH"):
        root = os.environ.get(variable)
        if root:
            candidates.append(Path(root).expanduser() / "python.sh")

    home = Path.home()
    candidates.extend(
        (
            home / "isaacsim/python.sh",
            home / "isaac-sim/python.sh",
            home / "dev_ws/isaac_sim/python.sh",
            home / "dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh",
            Path("/opt/isaacsim/python.sh"),
            Path("/opt/isaac-sim/python.sh"),
        )
    )
    candidates.extend(sorted((home / ".local/share/ov/pkg").glob("isaac-sim-*/python.sh")))
    candidates.extend(sorted((home / ".local/share/ov/pkg").glob("isaac_sim-*/python.sh")))
    return candidates


def resolve_isaac_python() -> Path:
    checked: list[str] = []
    for candidate in _candidates():
        resolved = candidate.resolve()
        if str(resolved) in checked:
            continue
        checked.append(str(resolved))
        if resolved.is_file():
            return resolved
    searched = "\n  - ".join(checked)
    raise FileNotFoundError(
        "Isaac Sim python.sh를 찾을 수 없습니다. 설치 경로를 지정하세요:\n"
        "  export ISAAC_SIM_PYTHON=/path/to/isaac-sim/python.sh\n"
        f"검색한 경로:\n  - {searched}"
    )


def restart_with_isaac_python(script_path: Path) -> None:
    """일반 Python이면 현재 인자를 보존해 Isaac Sim Python으로 교체한다."""
    if os.environ.get("CARB_APP_PATH"):
        return
    isaac_python = resolve_isaac_python()
    print(f"[parking] Isaac Sim Python으로 전환: {isaac_python}", flush=True)
    os.execv(
        str(isaac_python),
        [str(isaac_python), str(script_path.resolve()), *sys.argv[1:]],
    )
