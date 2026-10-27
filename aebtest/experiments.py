"""Shared experiment helpers: phase sweep aggregation, AEB parameter sets, condition grid."""
from __future__ import annotations

from collections import defaultdict
from dataclasses import replace

from .closed_loop import AEB, RunConfig, RunResult
from .evaluate import ncap_cell, ncap_simplified_score, r152_verdict
from .models import ActuatorParams, SensorParams
from .regulations import NCAP_SUBSCORE
from .runner import POINTS, run_many

DT = 0.002                                   # plant / esmini step (ECU task stays 10 ms, radar 50 ms)
PHASES = (0.0, 0.01, 0.02, 0.03, 0.04)       # radar sampling phase within its 50 ms frame
LATENCIES = (0.0, 0.1, 0.2)                  # extra sensor latency [s]   (sensitivity, project choice)
TAUS = (0.12, 0.25, 0.40)                    # brake time constant [s]    (sensitivity, project choice)

ORIGINAL = AEB.AebConfig()                   # upstream defaults, commit 27da004
# Proposed values = the candidate selected by scripts/param_search.py (rule in its docstring,
# full candidate table in results/param_search.csv).  Every other AebConfig field keeps its upstream value.
PROPOSED = replace(ORIGINAL, ttc_fcw_s=3.0, ttc_pb_s=2.0, ttc_fb_s=1.3, partial_decel_mps2=6.0)
CONFIGS = {"original": ORIGINAL, "proposed": PROPOSED}


def config(aeb=ORIGINAL, latency=0.0, tau=0.12, phase=0.0, noise=(0.0, 0.0), seed=0, dt=DT) -> RunConfig:
    return RunConfig(aeb=aeb, dt=dt,
                     sensor=SensorParams(latency_s=latency, phase_s=phase, range_noise_sd_m=noise[0],
                                         rate_noise_sd_mps=noise[1], seed=seed),
                     actuator=ActuatorParams(tau_s=tau))


def sweep(case_ids, aeb, latency, tau, kind, lib, workers, phases=PHASES, noise=(0.0, 0.0), seeds=(0,),
          dt=DT) -> dict[str, list[RunResult]]:
    jobs = [(c, config(aeb, latency, tau, ph, noise, sd, dt), False, None)
            for c in case_ids for ph in phases for sd in seeds]
    out = defaultdict(list)
    for r in run_many(jobs, kind, lib, workers):
        out[r.case_id].append(r)
    return dict(out)


def worst(runs: list[RunResult]) -> RunResult:
    """Worst run of a phase sweep: highest relative impact speed, then smallest min gap."""
    return max(runs, key=lambda r: (r.v_rel_impact_kph, -r.min_gap_m))


def summarize(group: dict[str, list[RunResult]]) -> dict:
    """R152 verdicts (worst over runs) and NCAP colours (worst over runs) for one condition."""
    r152, ncap = {}, {}
    for cid, runs in group.items():
        tp = POINTS[cid]
        if tp.protocol not in ("R152", "NCAP"):
            continue
        w = worst(runs)
        if tp.protocol == "R152":
            vs = [r152_verdict(r, tp) for r in runs]
            leads = [v["lead_s"] for v in vs if v["lead_s"] is not None]
            r152[cid] = {"v_rel_impact_max": w.v_rel_impact_kph,
                         "v_rel_impact_min": min(r.v_rel_impact_kph for r in runs),
                         "contacts": sum(r.contact for r in runs), "runs": len(runs),
                         "limit": vs[0]["limit_kph"],
                         "impact_ok": all(v["impact_ok"] for v in vs),
                         "failed_runs": sum(not v["impact_ok"] for v in vs),
                         "lead_min": min(leads) if leads else None, "lead_ok": all(v["lead_ok"] for v in vs),
                         "demand_ok": all(v["demand_ok"] for v in vs),
                         "min_gap_min": min(r.min_gap_m for r in runs)}
        else:
            c = ncap_cell(w, tp)
            ncap[cid] = {"v_rel_impact_max": w.v_rel_impact_ncap_kph,
                         "v_rel_impact_min": min(r.v_rel_impact_ncap_kph for r in runs),
                         "colour": c["colour"], "standard": c["standard"],
                         "late_contact_any": any(ncap_cell(r, tp)["late_contact"] for r in runs),
                         "min_gap_min": min(r.min_gap_m for r in runs), "lead_min": min(
                             (r.fcw_lead_s for r in runs if r.fcw_lead_s is not None), default=None)}
    score = ncap_simplified_score([(worst(runs), POINTS[cid]) for cid, runs in group.items()
                                   if POINTS[cid].protocol == "NCAP"]) if ncap else {}
    return {"r152": r152, "ncap": ncap, "score": score}


def r152_ok(summary: dict) -> bool:
    return all(v["impact_ok"] and v["lead_ok"] and v["demand_ok"] for v in summary["r152"].values())


def ncap_total(summary: dict) -> float:
    s = summary["score"]
    return sum(s[f]["estimate"] for f in ("CCRs", "CCRm", "CCRb") if f in s)


def with_params(base, **kw):
    return replace(base, **kw)


__all__ = ["DT", "PHASES", "LATENCIES", "TAUS", "ORIGINAL", "PROPOSED", "CONFIGS", "config", "sweep", "worst",
           "summarize", "r152_ok", "ncap_total", "with_params", "NCAP_SUBSCORE"]
