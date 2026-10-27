"""Experiment definitions shared by run_signal_level.py, make_figures.py and the tests.

Conventions copied from P5 (aebtest/experiments.py): dt = 2 ms, ECU task 10 ms, radar 50 ms, brake dead time
80 ms + tau 0.12 s, five radar sampling phases, original vs proposed AebConfig.  R152 verdicts use P5's
``evaluate.r152_verdict`` (5.2.1.4 impact speed, 5.2.1.1 warning lead, 5.2.1.2 demand >= 5 m/s2).
"""
from __future__ import annotations

import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace

from .paths import bootstrap

bootstrap()

from aebtest.backends import PythonBackend  # noqa: E402
from aebtest.closed_loop import run_case  # noqa: E402
from aebtest.evaluate import r152_verdict  # noqa: E402
from aebtest.experiments import CONFIGS, PHASES, config  # noqa: E402
from aebtest.regulations import r152_points  # noqa: E402
from aebtest.runner import GRID, POINTS  # noqa: E402
from aebtest.scenarios import regulatory_points  # noqa: E402

from .sl_loop import BusSetup, load_db, run_case_bus  # noqa: E402

OFFICIAL = [p.case_id for p in r152_points(sweep=False)]          # 6 official R152 M1 test points
SWEEP = [p.case_id for p in r152_points(sweep=True)]              # + the other 5.2.1.4 table speeds (19)
REGULATORY = [p.case_id for p in regulatory_points()]             # R152 sweep + Euro NCAP grid (49)
MAX_WORKERS = 4                                                   # hard cap (shared machine)

# ---- sweep values of this project (design choices) ----
LOSSES = (0.0, 0.01, 0.05, 0.10, 0.20)
LOSS_SEEDS = 20
BURST_DURATIONS = (0.1, 0.2, 0.5)
BURST_OFFSETS = tuple(round(-0.5 + 0.1 * i, 1) for i in range(11))   # burst start - fault-free brake onset [s]
LATENCIES = (0.0, 0.010, 0.020, 0.050)
HOLD_S = 2.0                                                      # mitigation variant: max brake hold in FAULT

E2E_BITS = {s: tuple(load_db().get_message_by_name("RADAR_TGT").signal(s).bit_positions())
            for s in ("TgtValid", "TgtQuality", "TgtDist", "TgtRelSpd")}


VARIANTS = {                      # degradation-policy variants evaluated in E2 / E3
    "U": ((), 0.0),                                  # baseline: upstream reaction (FAULT releases, 1.0 s heal)
    "H": ((), HOLD_S),                               # + hold an ongoing intervention through a comm fault
    "R": ((("fault_recover_s", 0.2),), 0.0),         # + AebConfig.fault_recover_s 1.0 -> 0.2 s (parameter only)
    "HR": ((("fault_recover_s", 0.2),), HOLD_S),     # both
}


@dataclass(frozen=True)
class Job:
    case_id: str
    cfg_name: str                  # "original" | "proposed"
    phase: float = 0.0
    setup: BusSetup | None = None  # None = P5 direct closed loop (no bus)
    tag: str = ""
    record: bool = False
    aeb_overrides: tuple = ()      # ((AebConfig field, value), ...) applied with dataclasses.replace


def run_job(job: Job):
    tp = POINTS[job.case_id]
    _, scen, _, params = GRID[job.case_id]
    aeb = CONFIGS[job.cfg_name]
    if job.aeb_overrides:
        aeb = replace(aeb, **dict(job.aeb_overrides))
    cfg = config(aeb, 0.0, 0.12, job.phase)
    if job.setup is None:
        return run_case(tp, scen, params, PythonBackend(), cfg, record=job.record)
    return run_case_bus(tp, scen, params, PythonBackend(), cfg, job.setup, record=job.record)


def to_row(job: Job, r) -> dict:
    tp = POINTS[job.case_id]
    row = {"case_id": job.case_id, "config": job.cfg_name, "phase": job.phase, "tag": job.tag,
           "loop": "direct" if job.setup is None else job.setup.label(),
           "contact": r.contact, "v_rel_impact_kph": round(r.v_rel_impact_kph, 4),
           "v_rel_impact_ncap_kph": round(r.v_rel_impact_ncap_kph, 4), "t_contact": r.t_contact,
           "brake_time_s": round(r.brake_time_s, 4), "peak_decel_mps2": round(r.peak_decel_mps2, 4),
           "t_fcw": r.t_fcw, "t_brake": r.t_brake, "t_full": r.t_full, "ttc_brake": r.ttc_brake,
           "fcw_lead_s": r.fcw_lead_s, "max_decel_req": r.max_decel_req, "min_gap_m": round(r.min_gap_m, 4),
           "v_min_kph": round(r.v_min_kph, 3) if r.v_min_kph is not None else None,
           "states": r.states, "end_reason": r.end_reason}
    if tp.protocol == "R152":
        v = r152_verdict(r, tp)
        row.update(limit_kph=v["limit_kph"], impact_ok=v["impact_ok"], lead_ok=v["lead_ok"],
                   demand_ok=v["demand_ok"], all_ok=v["all_ok"])
    for k in ("t_fault", "fault_time_s", "t_telltale", "telltale_time_s", "brake_lost_s", "hold_time_s",
              "frames_tx", "frames_dropped", "frames_corrupted", "corrupted_accepted", "input_faults"):
        row[k] = getattr(r, k, None)
    if hasattr(r, "rx_status"):
        row["rx_status"] = json.dumps(r.rx_status, sort_keys=True)
    return row


def _work(job: Job) -> dict:
    return to_row(job, run_job(job))


def run_rows(jobs: list[Job], workers: int = 3) -> list[dict]:
    workers = max(1, min(workers, MAX_WORKERS))
    if workers == 1 or len(jobs) < 8:
        return [_work(j) for j in jobs]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        return list(ex.map(_work, jobs, chunksize=max(1, min(32, len(jobs) // (4 * workers)))))


__all__ = ["OFFICIAL", "SWEEP", "REGULATORY", "PHASES", "CONFIGS", "LOSSES", "LOSS_SEEDS", "BURST_DURATIONS",
           "BURST_OFFSETS", "LATENCIES", "HOLD_S", "VARIANTS", "E2E_BITS", "Job", "run_job", "to_row", "run_rows"]
