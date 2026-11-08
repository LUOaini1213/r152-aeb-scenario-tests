"""Tracked text files must not contain local machine paths or the old local folder name.
Run:  python -I -m pytest -q tests/test_no_local_paths.py"""
import re
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PATTERN = re.compile(r"[A-Za-z]:[\/](Users|Documents and Settings)[\/]|/home/[a-z]|/Users/[A-Za-z]|"
                     r"Desktop[\\/]|P5_AEB", re.IGNORECASE)
TEXT = {".py", ".md", ".csv", ".json", ".txt", ".xosc", ".xodr", ".dbc", ".ini", ".yml", ".yaml", ".toml", ""}


def tracked_text_files():
    try:
        out = subprocess.run(["git", "ls-files", "-z"], cwd=ROOT, capture_output=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        pytest.skip("not a git checkout")
    for name in out.decode("utf-8").split("\0"):
        p = ROOT / name
        if name and p.suffix.lower() in TEXT and p.is_file() and not name.startswith("vendor/"):
            yield name, p


def test_no_local_paths():
    hits = []
    for name, p in tracked_text_files():
        if p.resolve() == Path(__file__).resolve():
            continue
        for i, line in enumerate(p.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
            if PATTERN.search(line):
                hits.append(f"{name}:{i}: {line.strip()[:120]}")
    assert not hits, "local paths in tracked files:\n" + "\n".join(hits)
