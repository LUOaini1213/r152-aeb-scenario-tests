"""Project-relative paths (pathlib only, no absolute paths) and the import bootstrap for aebtest."""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent          # <repo>/signal_level
REPO_ROOT = HERE.parent                         # <repo>
DBC_PATH = HERE / "aeb_demo.dbc"
RESULTS = HERE / "results"
VENDOR_FILE = REPO_ROOT / "vendor" / "canbus_analyzer_aeb" / "aeb_function.py"


def bootstrap() -> None:
    """Make ``aebtest`` and ``signal_level`` importable without writing bytecode into the repo."""
    sys.dont_write_bytecode = True
    root = str(REPO_ROOT)
    if root not in sys.path:
        sys.path.insert(0, root)
