"""Run lists of test points (optionally in parallel worker processes, one esmini instance per process)."""
from __future__ import annotations

import csv
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

from .backends import EsminiBackend, PythonBackend
from .closed_loop import RunConfig, RunResult, run_case
from .scenarios import all_points, grid_index

_BACKEND = None


def make_backend(kind: str, esmini_lib: str | None):
    if kind == "esmini":
        if not esmini_lib:
            raise SystemExit("--esmini-lib <path to esminiLib.dll> is required for the esmini backend")
        return EsminiBackend(esmini_lib)
    return PythonBackend()


def _init_worker(kind, lib):
    global _BACKEND
    _BACKEND = make_backend(kind, lib)


def _run_one(job):
    case_id, cfg, record, log_stem = job[:4]
    overrides = job[4] if len(job) > 4 else None       # e.g. {"RoadMu": "0.5"} for the wet-road runs
    tp = POINTS[case_id]
    grid, scen, perm, params = GRID[case_id]
    if overrides:
        params = {**params, **overrides}
    return run_case(tp, scen, params, _BACKEND, cfg, record=record, log_stem=log_stem)


POINTS = {p.case_id: p for p in all_points()}
GRID = grid_index()


def run_many(jobs: list[tuple[str, RunConfig, bool, str | None]], kind: str, esmini_lib: str | None,
             workers: int = 1) -> list[RunResult]:
    if workers <= 1:
        _init_worker(kind, esmini_lib)
        return [_run_one(j) for j in jobs]
    with ProcessPoolExecutor(max_workers=workers, initializer=_init_worker, initargs=(kind, esmini_lib)) as ex:
        return list(ex.map(_run_one, jobs, chunksize=4))


def write_series(r: RunResult, path: Path) -> None:
    if not r.series:
        return
    keys = list(r.series)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for row in zip(*(r.series[k] for k in keys)):
            w.writerow(["" if v is None else (f"{v:.4f}" if isinstance(v, float) else v) for v in row])


def flat(r: RunResult) -> dict:
    d = asdict(r)
    d.pop("series", None)
    d["transitions"] = " ".join(f"{t:.2f}:{a}>{b}({rid})" for t, a, b, rid in r.transitions)
    d["fcw_lead_s"] = r.fcw_lead_s
    return d
