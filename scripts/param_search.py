"""Small, explicit search for the proposed AEB parameter change (results/param_search.csv).

Only AebConfig VALUES change - the upstream state-machine code is untouched.
Candidates:  partial_decel_mps2 in {5, 6}   (R152 5.2.1.2: demand >= 5.0 m/s2; original 4.0)
             ttc_pb_s in {1.6, 1.8, 2.0},  ttc_fcw_s = ttc_pb_s + 1.0 (keeps the original 1.0 s FCW lead,
             margin over R152 5.2.1.1's 0.8 s),  ttc_fb_s in {0.9, 1.1, 1.3, 1.5}
Constraint:  all 6 official R152 M1 test points pass (impact speed, warning lead, demand) in every radar
             sampling phase at the stress corner latency 0.2 s + brake tau 0.4 s AND at the baseline.
Objective:   simplified Euro NCAP Standard-Range estimate (CCRs+CCRm+CCRb, worst phase) summed over the
             baseline (0 s, 0.12 s) and a moderate-stress condition (0.1 s, 0.25 s).
Tie-break:   the later intervention (smaller ttc_pb_s, then smaller ttc_fb_s) - earlier braking costs
             false activations in real traffic, which this simulation cannot measure.
Also recorded (not used for selection): CCRb runs that touch the target AFTER the Euro NCAP end
condition V_VUT = V_target ("late contact", found while verifying the first chosen candidate).

Run:  python -I scripts/param_search.py --esmini-lib <...>/esminiLib.dll [--workers 10]
"""
import argparse
import csv
import itertools
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aebtest.experiments import ORIGINAL, ncap_total, r152_ok, summarize, sweep, with_params  # noqa: E402
from aebtest.regulations import ncap_points, r152_points  # noqa: E402


def evaluate(aeb, kind, lib, workers):
    off = [p.case_id for p in r152_points(sweep=False)]
    ncap = [p.case_id for p in ncap_points()]
    ok = all(r152_ok(summarize(sweep(off, aeb, lat, tau, kind, lib, workers))) for lat, tau in ((0.0, 0.12), (0.2, 0.4)))
    g0 = sweep(ncap, aeb, 0.0, 0.12, kind, lib, workers)
    late = [r.v_rel_impact_kph for runs in g0.values() for r in runs if r.contact and not r.contact_ncap]
    s0 = summarize(g0)
    s1 = summarize(sweep(ncap, aeb, 0.1, 0.25, kind, lib, workers))
    return ok, s0, s1, late


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esmini-lib", required=False)
    ap.add_argument("--backend", default="esmini", choices=["esmini", "python"])
    ap.add_argument("--workers", type=int, default=10)
    a = ap.parse_args()
    rows = []
    cands = [("original", ORIGINAL)]
    for pd, pb, fb in itertools.product((5.0, 6.0), (1.6, 1.8, 2.0), (0.9, 1.1, 1.3, 1.5)):
        cands.append((f"pd{pd:g}_pb{pb:g}_fb{fb:g}",
                      with_params(ORIGINAL, partial_decel_mps2=pd, ttc_pb_s=pb, ttc_fcw_s=round(pb + 1.0, 2), ttc_fb_s=fb)))
    t0 = time.time()
    for name, aeb in cands:
        ok, s0, s1, late = evaluate(aeb, a.backend, a.esmini_lib, a.workers)
        row = {"name": name, "ttc_fcw_s": aeb.ttc_fcw_s, "ttc_pb_s": aeb.ttc_pb_s, "ttc_fb_s": aeb.ttc_fb_s,
               "partial_decel_mps2": aeb.partial_decel_mps2, "r152_constraint_ok": ok,
               "ncap_est_baseline": round(ncap_total(s0), 2), "ncap_est_stress": round(ncap_total(s1), 2),
               **{f"{f}_baseline": s0["score"][f]["estimate"] for f in ("CCRs", "CCRm", "CCRb")},
               **{f"{f}_stress": s1["score"][f]["estimate"] for f in ("CCRs", "CCRm", "CCRb")},
               "ccrs_gate_ok": s0["score"]["ccrs_low_speed_gate_ok"] and s1["score"]["ccrs_low_speed_gate_ok"],
               # recorded for transparency only - NOT part of the selection rule (see report.md 5.4):
               "late_contact_runs_baseline": len(late),
               "late_contact_max_kph": round(max(late), 2) if late else 0.0}
        row["objective"] = round(row["ncap_est_baseline"] + row["ncap_est_stress"], 2)
        rows.append(row)
        print(f"{time.time()-t0:6.0f}s {name:20s} R152ok={ok!s:5} NCAP base={row['ncap_est_baseline']:.2f} "
              f"stress={row['ncap_est_stress']:.2f}", flush=True)
    out = ROOT / "results" / "param_search.csv"
    out.parent.mkdir(exist_ok=True)
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    feas = [r for r in rows[1:] if r["r152_constraint_ok"] and r["ccrs_gate_ok"]]
    best = max(feas, key=lambda r: (r["objective"], -r["ttc_pb_s"], -r["ttc_fb_s"])) if feas else None
    print("chosen:", best)


if __name__ == "__main__":
    main()
