"""Deepening experiments (wet road, friction-aware thresholds, SOTIF triggering conditions).

W1  wet road: RoadMu in {0.9, 0.7, 0.5} (achievable decel = min(9.0, 10*mu) m/s2), original and proposed
    (dry-tuned) AebConfig, all R152 + Euro NCAP test points, 5 radar sampling phases (worst reported)
W2  friction-aware schedule: for mu = 0.7 and 0.5 choose ttc_pb_s / ttc_fb_s (ttc_fcw_s = ttc_pb_s + 1.0,
    ttc_fb_s < ttc_pb_s, partial 6 m/s2) with the same kind of rule as scripts/param_search.py:
    constraint = the 6 official R152 points pass all three checks at that mu; objective = simplified NCAP
    estimate at that mu; tie-break = later intervention (smaller ttc_pb_s, then smaller ttc_fb_s)
W3  before / after on the wet grid (dry-tuned proposed vs friction-aware set at the matching mu)
S1  phantom stationary object ("ghost target") on a free road, 60 / 100 km/h, ghost TTC {2.8, 1.8, 1.2} s,
    visible {0.05, 0.15, 0.30} s, for each parameter set -> unintended warning / braking / speed loss
S2  rain-degraded sensing: detection range {200, 60, 40} m (dry road), and 40 m on mu = 0.5

The friction coefficients, the detection ranges and the ghost-target parameters are PROJECT ASSUMPTIONS;
R152 5.2.1.4 only requires its impact-speed table on a dry road (condition (a)(i)), and Euro NCAP tests on
PBC >= 0.9 - every wet / rain verdict below is in-model and informative, not a compliance statement.

Run:  python -I scripts/run_deepening.py --esmini-lib <...>/esminiLib.dll [--workers 10]
"""
import argparse
import csv
import itertools
import json
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aebtest.closed_loop import RunConfig  # noqa: E402
from aebtest.experiments import (CONFIGS, DT, PHASES, ncap_total, r152_ok, summarize, worst)  # noqa: E402
from aebtest.models import ActuatorParams, GhostTarget, SensorParams, friction_limit_mps2  # noqa: E402
from aebtest.regulations import ncap_points, r152_points  # noqa: E402
from aebtest.runner import run_many  # noqa: E402
from aebtest.scenarios import regulatory_points, sotif_points  # noqa: E402

RES = ROOT / "results"
MUS = (0.9, 0.7, 0.5)
RANGES = (200.0, 60.0, 40.0)
GHOST_TTCS = (2.8, 1.8, 1.2)
GHOST_DURATIONS = (0.05, 0.15, 0.30)
GHOST_T_START = 3.0


def cfg(aeb, phase=0.0, max_range=200.0, ghost=None) -> RunConfig:
    return RunConfig(aeb=aeb, dt=DT, sensor=SensorParams(phase_s=phase, max_range_m=max_range, ghost=ghost),
                     actuator=ActuatorParams())


def grid_run(case_ids, aeb, mu, kind, lib, W, max_range=200.0):
    jobs = [(c, cfg(aeb, ph, max_range), False, None, {"RoadMu": f"{mu}"}) for c in case_ids for ph in PHASES]
    group = {}
    for r in run_many(jobs, kind, lib, W):
        group.setdefault(r.case_id, []).append(r)
    return group


def late(group):
    lr = [r for runs in group.values() for r in runs if r.contact and not r.contact_ncap]
    return len(lr), round(max((r.v_rel_impact_kph for r in lr), default=0.0), 2)


def rows_of(s, group, label, mu, extra=None):
    out = []
    for c, v in {**s["r152"], **s["ncap"]}.items():
        out.append({"config": label, "road_mu": mu, **(extra or {}), "case_id": c,
                    "v_rel_impact_worst_kph": round(v["v_rel_impact_max"], 2),
                    "verdict": (("pass" if v["impact_ok"] else "FAIL") if c.startswith("R152") else v["colour"]),
                    "all_r152_checks": (v["impact_ok"] and v["lead_ok"] and v["demand_ok"]) if c.startswith("R152") else "",
                    "fcw_lead_min_s": None if v["lead_min"] is None else round(v["lead_min"], 3),
                    "min_gap_m": round(v["min_gap_min"], 3)})
    return out


def summary_row(s, group, label, mu, extra=None):
    n_late, late_max = late(group)
    return {"config": label, "road_mu": mu, **(extra or {}),
            "r152_impact_fail_points": sum(not v["impact_ok"] for c, v in s["r152"].items()),
            "r152_any_fail_points": sum(not (v["impact_ok"] and v["lead_ok"] and v["demand_ok"]) for v in s["r152"].values()),
            "r152_fail_list": " ".join(c for c, v in s["r152"].items() if not v["impact_ok"]),
            **{f"ncap_{f}": s["score"][f]["estimate"] for f in ("CCRs", "CCRm", "CCRb")},
            "ncap_total": round(ncap_total(s), 2), "red_cells": sum(v["colour"] == "red" for v in s["ncap"].values()),
            "late_contact_runs": n_late, "late_contact_max_kph": late_max}


def write_csv(path, rows):
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esmini-lib")
    ap.add_argument("--backend", default="esmini", choices=["esmini", "python"])
    ap.add_argument("--workers", type=int, default=10)
    a = ap.parse_args()
    kind, lib, W = a.backend, a.esmini_lib, a.workers
    t0 = time.time()
    reg_ids = [p.case_id for p in regulatory_points()]
    off_ids = [p.case_id for p in r152_points(sweep=False)]
    ncap_ids = [p.case_id for p in ncap_points()]
    out = {"mus": MUS, "decel_limit_mps2": {mu: min(9.0, friction_limit_mps2(mu)) for mu in MUS}}

    # ------------------------------------------------------------ W1 wet road, existing parameter sets
    wet_rows, wet_sum = [], []
    for label in ("original", "proposed"):
        for mu in MUS:
            g = grid_run(reg_ids, CONFIGS[label], mu, kind, lib, W)
            s = summarize(g)
            wet_rows += rows_of(s, g, label, mu)
            wet_sum.append(summary_row(s, g, label, mu))
            print(f"[W1] {label} mu={mu}: R152 impact fails {wet_sum[-1]['r152_impact_fail_points']}, "
                  f"NCAP {wet_sum[-1]['ncap_total']}", flush=True)

    # ------------------------------------------------------------ W2 friction-aware schedule search
    search, chosen = [], {}
    for mu in (0.7, 0.5):
        cands = []
        for pb, fb in itertools.product((2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2), (1.3, 1.5, 1.7, 1.9, 2.1, 2.3)):
            if fb >= pb:
                continue
            aeb = replace(CONFIGS["proposed"], ttc_pb_s=pb, ttc_fcw_s=round(pb + 1.0, 2), ttc_fb_s=fb)
            ok = r152_ok(summarize(grid_run(off_ids, aeb, mu, kind, lib, W)))
            gn = grid_run(ncap_ids, aeb, mu, kind, lib, W)
            sn = summarize(gn)
            n_late, late_max = late(gn)
            row = {"road_mu": mu, "ttc_fcw_s": aeb.ttc_fcw_s, "ttc_pb_s": pb, "ttc_fb_s": fb, "r152_ok": ok,
                   "ncap_total": round(ncap_total(sn), 2), "late_contact_runs": n_late, "late_contact_max_kph": late_max}
            search.append(row)
            cands.append((row, aeb))
        feas = [(r, aeb) for r, aeb in cands if r["r152_ok"]]
        best_r, best_aeb = max(feas, key=lambda x: (x[0]["ncap_total"], -x[0]["ttc_pb_s"], -x[0]["ttc_fb_s"]))
        chosen[mu] = best_aeb
        print(f"[W2] mu={mu}: chosen {best_r}", flush=True)
    write_csv(RES / "wet_schedule_search.csv", search)
    out["friction_aware"] = {str(mu): {k: getattr(aeb, k) for k in ("ttc_fcw_s", "ttc_pb_s", "ttc_fb_s",
                                                                      "partial_decel_mps2", "full_decel_mps2")}
                             for mu, aeb in chosen.items()}

    # ------------------------------------------------------------ W3 before / after on the wet grid
    for mu, aeb in chosen.items():
        g = grid_run(reg_ids, aeb, mu, kind, lib, W)
        s = summarize(g)
        wet_rows += rows_of(s, g, f"friction_aware_{mu}", mu)
        wet_sum.append(summary_row(s, g, f"friction_aware_{mu}", mu))
        print(f"[W3] friction-aware {mu} on mu={mu}: R152 impact fails {wet_sum[-1]['r152_impact_fail_points']}, "
              f"NCAP {wet_sum[-1]['ncap_total']}", flush=True)
    # cost side: the wet set used on a dry road (mu over-estimated as low) - still evaluated on the grid
    g = grid_run(reg_ids, chosen[0.5], 0.9, kind, lib, W)
    s = summarize(g)
    wet_rows += rows_of(s, g, "friction_aware_0.5", 0.9)
    wet_sum.append(summary_row(s, g, "friction_aware_0.5", 0.9))
    write_csv(RES / "wet_results.csv", wet_rows)
    write_csv(RES / "wet_summary.csv", wet_sum)

    # ------------------------------------------------------------ S1 phantom object (ghost target)
    sets = [("original", CONFIGS["original"], 0.9), ("proposed", CONFIGS["proposed"], 0.9),
            ("friction_aware_0.5", chosen[0.5], 0.9), ("friction_aware_0.5", chosen[0.5], 0.5)]
    ghost_rows = []
    jobs, meta = [], []
    for (label, aeb, mu), p, ttc0, dur in itertools.product(sets, sotif_points(), GHOST_TTCS, GHOST_DURATIONS):
        jobs.append((p.case_id, cfg(aeb, 0.0, ghost=GhostTarget(GHOST_T_START, dur, ttc0)), False, None,
                     {"RoadMu": f"{mu}"}))
        meta.append((label, mu, p, ttc0, dur))
    for (label, mu, p, ttc0, dur), r in zip(meta, run_many(jobs, kind, lib, W)):
        ghost_rows.append({"config": label, "road_mu": mu, "case_id": p.case_id, "v_ego_kph": p.v_ego_kph,
                           "ghost_ttc_s": ttc0, "ghost_visible_s": dur,
                           "fcw": r.t_fcw is not None, "max_decel_request_mps2": r.max_decel_req,
                           "peak_decel_mps2": round(r.peak_decel_mps2, 2),
                           "speed_loss_kph": round(p.v_ego_kph - r.v_min_kph, 2),
                           "brake_time_s": round(r.brake_time_s, 3), "states": r.states})
    write_csv(RES / "sotif_ghost.csv", ghost_rows)
    print(f"[S1] {len(ghost_rows)} phantom-object runs", flush=True)

    # ------------------------------------------------------------ S2 rain-degraded detection range
    rng_rows, rng_sum = [], []
    for label in ("original", "proposed"):
        for R in RANGES:
            g = grid_run(reg_ids, CONFIGS[label], 0.9, kind, lib, W, max_range=R)
            s = summarize(g)
            rng_rows += rows_of(s, g, label, 0.9, {"max_range_m": R})
            rng_sum.append(summary_row(s, g, label, 0.9, {"max_range_m": R}))
            print(f"[S2] {label} range={R}: R152 any-fail {rng_sum[-1]['r152_any_fail_points']}, NCAP {rng_sum[-1]['ncap_total']}",
                  flush=True)
    for R in (200.0, 40.0):
        g = grid_run(reg_ids, chosen[0.5], 0.5, kind, lib, W, max_range=R)
        s = summarize(g)
        rng_rows += rows_of(s, g, "friction_aware_0.5", 0.5, {"max_range_m": R})
        rng_sum.append(summary_row(s, g, "friction_aware_0.5", 0.5, {"max_range_m": R}))
        print(f"[S2] friction-aware 0.5 on mu 0.5, range={R}: R152 any-fail {rng_sum[-1]['r152_any_fail_points']}, "
              f"NCAP {rng_sum[-1]['ncap_total']}", flush=True)
    write_csv(RES / "sotif_range.csv", rng_rows)
    write_csv(RES / "sotif_range_summary.csv", rng_sum)

    out["runtime_s"] = round(time.time() - t0, 1)
    (RES / "deepening_summary.json").write_text(json.dumps(out, indent=2, ensure_ascii=False, default=str),
                                                encoding="utf-8")
    print(f"done in {out['runtime_s']} s")


if __name__ == "__main__":
    main()
