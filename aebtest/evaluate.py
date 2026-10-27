"""Turn RunResults into R152 verdicts and Euro NCAP colours (only the criteria a 1-D simulation can see)."""
from __future__ import annotations

from collections import defaultdict

from .closed_loop import RunResult
from .regulations import (NCAP_CCRS_FULL_AVOID_UP_TO_KPH, NCAP_STANDARD_50PCT, NCAP_STANDARD_POINTS,
                          NCAP_SUBSCORE, R152_MIN_BRAKE_DEMAND_MPS2, R152_WARNING_LEAD_S, TestPoint,
                          ncap_colour, r152_max_rel_impact_kph)


def r152_verdict(r: RunResult, tp: TestPoint) -> dict:
    """R152 5.2.1.4 impact speed, 5.2.1.1 warning lead, 5.2.1.2 demand >= 5 m/s2 (literal reading)."""
    limit = min(r152_max_rel_impact_kph(tp.v_rel_kph, m) for m in tp.masses)
    impact_ok = r.v_rel_impact_kph <= limit + 1e-9
    lead = r.fcw_lead_s
    lead_ok = lead is not None and lead >= R152_WARNING_LEAD_S - 1e-9
    demand_ok = r.max_decel_req >= R152_MIN_BRAKE_DEMAND_MPS2 - 1e-9
    return {"limit_kph": limit, "impact_ok": impact_ok, "lead_s": lead, "lead_ok": lead_ok,
            "demand_ok": demand_ok, "all_ok": impact_ok and lead_ok and demand_ok}


def ncap_cell(r: RunResult, tp: TestPoint) -> dict:
    colour = ncap_colour(tp.v_ego_kph, r.v_rel_impact_ncap_kph)
    standard = int(tp.v_ego_kph) in NCAP_STANDARD_50PCT[tp.family]
    return {"colour": colour, "standard": standard, "subscore": NCAP_SUBSCORE[colour] if standard else None,
            "late_contact": r.contact and not r.contact_ncap}


def ncap_simplified_score(results: list[tuple[RunResult, TestPoint]]) -> dict:
    """Simplified Standard-Range estimate per scenario (Euro NCAP 5.3.1 formula, see report.md 3.5).

    ONLY the 50 % impact-location cell of each speed row is simulated; the estimate assumes the other
    four Standard-Range impact locations of that row get the same colour.  Extended range, robustness
    layers and the verification-test scaling (5.3.4) are not included.  This is not a Euro NCAP score.
    """
    by = defaultdict(list)
    for r, tp in results:
        c = ncap_cell(r, tp)
        if c["standard"]:
            by[tp.family].append((tp.v_ego_kph, c["subscore"], r.v_rel_impact_ncap_kph))
    out = {}
    for fam, cells in by.items():
        mean = sum(s for _, s, _ in cells) / len(cells)
        out[fam] = {"cells": len(cells), "mean_subscore": mean,
                    "estimate": round(mean * NCAP_STANDARD_POINTS[fam], 2), "max": NCAP_STANDARD_POINTS[fam]}
    gate = [v for fam, cells in by.items() if fam == "CCRs" for v, _, vi in cells
            if v <= NCAP_CCRS_FULL_AVOID_UP_TO_KPH and vi > 0]
    out["ccrs_low_speed_gate_ok"] = not gate
    return out
