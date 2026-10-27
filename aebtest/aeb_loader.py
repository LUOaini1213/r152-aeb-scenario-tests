"""Load the UNCHANGED upstream AEB state machine (vendor/canbus_analyzer_aeb/aeb_function.py).

aeb_function.py has two package-relative imports:
  * ``from .analyzer import ttc``  -- analyzer.py imports python-can (not installed here), so the
    5-line ``ttc()`` function is cut out of the upstream analyzer.py source text with ``ast`` and executed
    as-is; MIN_CLOSING_MPS is read from the upstream config.py source text.
  * ``from .dbcutil import ...``   -- only used by the CAN shell (Supervisor / AebEcu), which this
    project does not use; replaced by stubs that raise if called.
The state-machine module itself is executed from the vendored file without modification
(SHA-256 checked by scripts/check_vendor.py).
"""
from __future__ import annotations

import ast
import importlib.util
import sys
import types
from pathlib import Path

VENDOR = Path(__file__).resolve().parents[1] / "vendor" / "canbus_analyzer_aeb"
PKG = "canbus_analyzer"


def _const_from_source(src: str, name: str):
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise KeyError(name)


def _function_source(src: str, name: str) -> str:
    for node in ast.parse(src).body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return ast.get_source_segment(src, node)
    raise KeyError(name)


def _not_used(*_a, **_k):
    raise NotImplementedError("CAN/DBC layer of can-bus-analyzer is not used in this project")


def load():
    """Return the upstream ``canbus_analyzer.aeb_function`` module (cached in sys.modules)."""
    full = f"{PKG}.aeb_function"
    if full in sys.modules:
        return sys.modules[full]

    pkg = types.ModuleType(PKG)
    pkg.__path__ = []          # mark as package; nothing is imported from disk through it
    sys.modules[PKG] = pkg

    analyzer_src = (VENDOR / "analyzer.py").read_text(encoding="utf-8")
    config_src = (VENDOR / "config.py").read_text(encoding="utf-8")
    analyzer = types.ModuleType(f"{PKG}.analyzer")
    analyzer.MIN_CLOSING_MPS = _const_from_source(config_src, "MIN_CLOSING_MPS")
    analyzer.TTC_SOURCE = _function_source(analyzer_src, "ttc")
    exec(compile(analyzer.TTC_SOURCE, str(VENDOR / "analyzer.py"), "exec"), analyzer.__dict__)
    sys.modules[analyzer.__name__] = analyzer

    dbcutil = types.ModuleType(f"{PKG}.dbcutil")
    dbcutil.load_db = dbcutil.signal_limits = dbcutil.toyota_checksum = _not_used
    sys.modules[dbcutil.__name__] = dbcutil

    spec = importlib.util.spec_from_file_location(full, VENDOR / "aeb_function.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[full] = mod
    spec.loader.exec_module(mod)
    mod.upstream_ttc = analyzer.ttc            # convenience handle
    return mod
