"""Project-relative paths (pathlib only, no absolute paths) and the import bootstrap for P5's aebtest."""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent          # .../P5_AEB法规场景测试/signal_level
P5_ROOT = HERE.parent                           # .../P5_AEB法规场景测试
DBC_PATH = HERE / "aeb_demo.dbc"
RESULTS = HERE / "results"
VENDOR_FILE = P5_ROOT / "vendor" / "canbus_analyzer_aeb" / "aeb_function.py"


def bootstrap() -> None:
    """Make ``aebtest`` (P5) and ``signal_level`` importable without writing bytecode into P5's folders."""
    sys.dont_write_bytecode = True
    root = str(P5_ROOT)
    if root not in sys.path:
        sys.path.insert(0, root)
