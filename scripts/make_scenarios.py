"""Write the OpenSCENARIO test-grid files (scenarios/*_grid.xosc) from aebtest/regulations.py.

Run:  python -I scripts/make_scenarios.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # -I does not add the project to sys.path

from aebtest.scenarios import read_grid, write_grids  # noqa: E402

for path in write_grids():
    scen, sets = read_grid(path.name)
    print(f"{path.name}: {len(sets)} permutations of {scen} -> " + ", ".join(s["CaseId"] for s in sets))
