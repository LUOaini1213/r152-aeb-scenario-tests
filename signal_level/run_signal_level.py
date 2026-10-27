"""Run every signal-level experiment and write results/ (CSV, JSON, Markdown tables).

  E1 equivalence  P5 direct loop vs transparent bus vs DBC bus; 49 regulatory points x 5 phases x 2 parameter sets
  E2 frame loss   i.i.d. loss {0,1,5,10,20} % on every frame; 6 official R152 points x 5 phases x 20 seeds;
                  degradation variants U (baseline) and HR
  E3 burst loss   bus-wide burst {100,200,500} ms starting at (fault-free brake onset + offset), offset -0.5..+0.5 s;
                  6 official points x 5 phases; variants U, H, R, HR (sl_experiments.VARIANTS)
  E4 latency      {0,10,20,50} ms on every frame (+ 10 ms with 0..10 ms jitter, 3 seeds); 19 R152 points x 5 phases
  E5 E2E          (a) CRC vs range-check detection of 1/2/3/4/8-bit errors per message (no closed loop)
                  (b) one RADAR_TGT frame with one flipped signal bit, every 100 ms of the approach, R152-S60/-M60,
                      E2E on vs off (with and without the DBC range check)
  E6 fault demos  measured detection / reaction / recovery times for the fault table; time series for figure 3

Run from the P5 folder:  python -B -I signal_level/run_signal_level.py [--workers 3] [--quick] [--only e1,e2]
All data are SYNTHETIC (simulation of a toy model); see README.md for the assumptions.
"""
import argparse
import csv
import itertools
import json
import random
import sys
import time
from collections import defaultdict
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from signal_level.e2e import P01Protector, p01_crc, p01_read_counter  # noqa: E402
from signal_level.paths import RESULTS  # noqa: E402
from signal_level.sl_experiments import (BURST_DURATIONS, BURST_OFFSETS, CONFIGS, E2E_BITS, HOLD_S,  # noqa: E402
                                         LATENCIES, LOSS_SEEDS, LOSSES, OFFICIAL, PHASES, REGULATORY, SWEEP,
                                         VARIANTS, Job, run_job, run_rows)
from signal_level.sl_loop import BusSetup, load_db  # noqa: E402
from signal_level.supervision import RxConfig, p01_config  # noqa: E402
from signal_level.vbus import BusFaults, Corruption, flip_bits  # noqa: E402

from aebtest.closed_loop import S  # noqa: E402
from aebtest.regulations import R152_MAX_FAILED_RUN_SHARE, ncap_colour  # noqa: E402
from aebtest.runner import POINTS  # noqa: E402

CMP_KEYS = ("contact", "v_rel_impact_kph", "v_rel_impact_ncap_kph", "t_fcw", "t_brake", "t_full", "max_decel_req",
            "min_gap_m", "states", "t_contact", "brake_time_s", "v_min_kph", "end_reason")
THRESH = {"FCW": "ttc_fcw_s", "PARTIAL_BRAKE": "ttc_pb_s", "FULL_BRAKE": "ttc_fb_s"}
SUMMARY: dict = {"synthetic": True, "note": "toy-model simulation; communication parameters are design assumptions"}


def write_csv(name: str, rows: list[dict]) -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(RESULTS / name, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print(f"  wrote results/{name} ({len(rows)} rows)", flush=True)


def merge(rows, params):
    return [{**p, **r} for p, r in zip(params, rows)]


def r2(x, nd=2):
    return None if x is None else round(x, nd)


# ======================================================================================== E1
def e1(W, quick):
    ids = OFFICIAL if quick else REGULATORY
    phases = (0.0, 0.04) if quick else PHASES
    loops = {"direct": None, "transparent": BusSetup(mode="transparent"), "dbc": BusSetup()}
    jobs = [Job(c, n, ph, s, tag=lp) for c in ids for n in CONFIGS for ph in phases for lp, s in loops.items()]
    rows = run_rows(jobs, W)
    by = {(r["case_id"], r["config"], r["phase"], r["tag"]): r for r in rows}
    comp = []
    for c, n, ph in [(c, n, ph) for c in ids for n in CONFIGS for ph in phases]:
        d, t, q = (by[(c, n, ph, k)] for k in ("direct", "transparent", "dbc"))
        row = {"case_id": c, "config": n, "phase": ph, "v_direct": d["v_rel_impact_kph"],
               "v_transparent": t["v_rel_impact_kph"], "v_dbc": q["v_rel_impact_kph"],
               "transparent_identical": all(t[k] == d[k] for k in CMP_KEYS),
               "dbc_identical": all(q[k] == d[k] for k in CMP_KEYS),
               "dbc_dv_kph": round(q["v_rel_impact_kph"] - d["v_rel_impact_kph"], 4),
               "dbc_dt_fcw_s": r2(None if None in (q["t_fcw"], d["t_fcw"]) else q["t_fcw"] - d["t_fcw"], 3),
               "dbc_dt_brake_s": r2(None if None in (q["t_brake"], d["t_brake"]) else q["t_brake"] - d["t_brake"], 3),
               "dbc_dmin_gap_m": round(q["min_gap_m"] - d["min_gap_m"], 4), "contact_direct": d["contact"],
               "contact_dbc": q["contact"]}
        if "impact_ok" in d:
            row["r152_verdict_flip"] = any(q[k] != d[k] for k in ("impact_ok", "lead_ok", "demand_ok"))
        else:
            v_ego = POINTS[c].v_ego_kph
            row.update(ncap_colour_direct=ncap_colour(v_ego, d["v_rel_impact_ncap_kph"]),
                       ncap_colour_dbc=ncap_colour(v_ego, q["v_rel_impact_ncap_kph"]))
            row["ncap_colour_flip"] = row["ncap_colour_direct"] != row["ncap_colour_dbc"]
        comp.append(row)
    write_csv("e1_equivalence_runs.csv", comp)
    summ = []
    for mode in ("transparent", "dbc"):
        for proto in ("R152", "NCAP"):
            sel = [r for r in comp if r["case_id"].startswith(proto)]
            if not sel:
                continue
            ident = [r for r in sel if r[f"{mode}_identical"]]
            dv = [abs(r[f"v_{mode}"] - r["v_direct"]) for r in sel]
            row = {"mode": mode, "protocol": proto, "runs": len(sel), "identical_runs": len(ident),
                   "max_abs_dv_kph": round(max(dv), 4)}
            if mode == "dbc":
                row.update(contact_flips=sum(r["contact_direct"] != r["contact_dbc"] for r in sel),
                           r152_verdict_flips=sum(bool(r.get("r152_verdict_flip")) for r in sel),
                           max_abs_dt_brake_s=max((abs(r["dbc_dt_brake_s"]) for r in sel
                                                   if r["dbc_dt_brake_s"] is not None), default=0.0),
                           max_abs_dmin_gap_m=max(abs(r["dbc_dmin_gap_m"]) for r in sel),
                           ncap_colour_flips=sum(bool(r.get("ncap_colour_flip")) for r in sel))
            summ.append(row)
    # worst over phases per case (what P5 reports)
    worst = []
    for c in ids:
        for n in CONFIGS:
            sel = [r for r in comp if r["case_id"] == c and r["config"] == n]
            worst.append({"case_id": c, "config": n, "worst_direct": max(r["v_direct"] for r in sel),
                          "worst_dbc": max(r["v_dbc"] for r in sel)})
    summ.append({"mode": "dbc", "protocol": "worst-over-phases", "runs": len(worst),
                 "identical_runs": sum(abs(w["worst_direct"] - w["worst_dbc"]) < 1e-9 for w in worst),
                 "max_abs_dv_kph": round(max(abs(w["worst_direct"] - w["worst_dbc"]) for w in worst), 4)})
    write_csv("e1_equivalence_summary.csv", summ)
    # explain every DBC difference: first ECU task where the state differs, TTC float vs quantized
    expl = []
    for r in comp:
        if r["dbc_identical"]:
            continue
        d = run_job(Job(r["case_id"], r["config"], r["phase"], None, record=True))
        q = run_job(Job(r["case_id"], r["config"], r["phase"], BusSetup(), record=True))
        sd, sq = d.series, q.series
        n = min(len(sd["t"]), len(sq["t"]))
        # skip the start-up difference (bus: OFF until the first radar frame arrives, P5: STANDBY at t = 0)
        i = next((k for k in range(n) if sd["state"][k] != sq["state"][k] and int(S.OFF) not in
                  (sd["state"][k], sq["state"][k])), None)
        e = {"case_id": r["case_id"], "config": r["config"], "phase": r["phase"], "dv_kph": r["dbc_dv_kph"],
             "dt_fcw_s": r["dbc_dt_fcw_s"], "dt_brake_s": r["dbc_dt_brake_s"], "dmin_gap_m": r["dbc_dmin_gap_m"],
             "startup_only_off_vs_standby": i is None}
        if i is not None:
            name = S(max(sd["state"][i], sq["state"][i])).name
            cfg = CONFIGS[r["config"]]
            escalation = None not in (sd["ttc_meas"][i], sq["ttc_meas"][i]) and name in THRESH
            e.update(t_first_state_diff=round(sd["t"][i], 3), state_direct=S(sd["state"][i]).name,
                     state_dbc=S(sq["state"][i]).name, ttc_true=r2(sd["ttc_true"][i], 5),
                     closing_speed_mps=r2(sd["v_ego"][i] - sd["v_tgt"][i], 3),
                     ttc_direct_float=r2(sd["ttc_meas"][i], 6), ttc_dbc_quantized=r2(sq["ttc_meas"][i], 6),
                     kind="escalation threshold" if escalation else
                     "release: closing speed <= MIN_CLOSING_MPS 0.1 m/s (TTC undefined) one frame earlier/later",
                     threshold_s=getattr(cfg, THRESH[name]) if escalation else None)
        expl.append(e)
    write_csv("e1_quantization_explained.csv", expl)
    SUMMARY["e1"] = {"summary": summ, "explained_differences": expl}


# ======================================================================================== E2
def e2(W, quick):
    seeds = range(2 if quick else LOSS_SEEDS)
    variants = ("U", "HR")
    jobs, params = [], []
    for var in variants:
        ov, hold = VARIANTS[var]
        for n in CONFIGS:
            for loss in LOSSES:
                for c in OFFICIAL:
                    for ph in PHASES:
                        for sd in (seeds if loss > 0 else (0,)):
                            jobs.append(Job(c, n, ph, BusSetup(faults=BusFaults(loss_prob=loss, seed=sd),
                                                               hold_on_fault_s=hold), aeb_overrides=ov))
                            params.append({"variant": var, "loss": loss, "seed": sd})
    rows = merge(run_rows(jobs, W), params)
    write_csv("e2_loss_runs.csv", rows)
    summ, by_case = [], []
    for var in variants:
        for n in CONFIGS:
            ref = {c: max(r["v_rel_impact_kph"] for r in rows if r["variant"] == var and r["config"] == n
                          and r["loss"] == 0.0 and r["case_id"] == c) for c in OFFICIAL}
            for loss in LOSSES:
                sel = [r for r in rows if r["variant"] == var and r["config"] == n and r["loss"] == loss]
                fails = [r for r in sel if not r["impact_ok"]]
                anyf = [r for r in sel if not r["all_ok"]]
                w = max(sel, key=lambda r: r["v_rel_impact_kph"])
                summ.append({"variant": var, "config": n, "loss": loss, "runs": len(sel),
                             "impact_fail_runs": len(fails), "impact_fail_share": round(len(fails) / len(sel), 4),
                             "any_fail_runs": len(anyf), "lead_fail_runs": sum(not r["lead_ok"] for r in sel),
                             "demand_fail_runs": sum(not r["demand_ok"] for r in sel),
                             "fault_runs": sum(r["t_fault"] is not None for r in sel),
                             "telltale_runs": sum(r["telltale_time_s"] > 0 for r in sel),
                             "worst_impact_kph": w["v_rel_impact_kph"], "worst_case": w["case_id"],
                             "worst_case_limit_kph": w["limit_kph"],
                             "r152_pass_all_runs": not anyf,
                             "share_vs_6_10_1_context": "<=" if len(fails) / len(sel) <= R152_MAX_FAILED_RUN_SHARE
                             else ">"})
                for c in OFFICIAL:
                    sc = [r for r in sel if r["case_id"] == c]
                    wv = max(r["v_rel_impact_kph"] for r in sc)
                    by_case.append({"variant": var, "config": n, "loss": loss, "case_id": c,
                                    "limit_kph": sc[0]["limit_kph"], "worst_impact_kph": wv,
                                    "fault_free_worst_kph": ref[c], "delta_kph": round(wv - ref[c], 3),
                                    "margin_to_limit_kph": round(sc[0]["limit_kph"] - wv, 3),
                                    "impact_fail_runs": sum(not r["impact_ok"] for r in sc), "runs": len(sc),
                                    "lead_fail_runs": sum(not r["lead_ok"] for r in sc)})
    write_csv("e2_loss_summary.csv", summ)
    write_csv("e2_loss_by_case.csv", by_case)
    SUMMARY["e2"] = {"summary": summ}


# ======================================================================================== E3
def reference_onsets(W):
    refs = run_rows([Job(c, n, ph, BusSetup(), tag="ref") for c in OFFICIAL for n in CONFIGS for ph in PHASES], W)
    return {(r["case_id"], r["config"], r["phase"]): r for r in refs}


def e3(W, quick):
    ref = reference_onsets(W)
    offsets = (-0.2, 0.0, 0.2) if quick else BURST_OFFSETS
    jobs, params = [], []
    for var, (ov, hold) in VARIANTS.items():
        for n in CONFIGS:
            for c in OFFICIAL:
                for ph in PHASES:
                    tb = ref[(c, n, ph)]["t_brake"]
                    for dur in BURST_DURATIONS:
                        for off in offsets:
                            t0 = round(tb + off, 6)
                            jobs.append(Job(c, n, ph, BusSetup(faults=BusFaults(bursts=((t0, dur),)),
                                                               hold_on_fault_s=hold), aeb_overrides=ov))
                            params.append({"variant": var, "duration_s": dur, "offset_s": off, "burst_start_s": t0,
                                           "ref_t_brake": tb, "ref_v": ref[(c, n, ph)]["v_rel_impact_kph"]})
    rows = merge(run_rows(jobs, W), params)
    write_csv("e3_burst_runs.csv", rows)
    summ, by_dur = [], []
    for var in VARIANTS:
        for n in CONFIGS:
            for dur in BURST_DURATIONS:
                seld = [r for r in rows if r["variant"] == var and r["config"] == n and r["duration_s"] == dur]
                for off in offsets:
                    sel = [r for r in seld if r["offset_s"] == off]
                    w = max(sel, key=lambda r: r["v_rel_impact_kph"])
                    summ.append({"variant": var, "config": n, "duration_s": dur, "offset_s": off, "runs": len(sel),
                                 "impact_fail_runs": sum(not r["impact_ok"] for r in sel),
                                 "lead_fail_runs": sum(not r["lead_ok"] for r in sel),
                                 "demand_fail_runs": sum(not r["demand_ok"] for r in sel),
                                 "any_fail_runs": sum(not r["all_ok"] for r in sel),
                                 "worst_impact_kph": w["v_rel_impact_kph"], "worst_case": w["case_id"],
                                 "worst_limit_kph": w["limit_kph"],
                                 "mean_impact_kph": round(sum(r["v_rel_impact_kph"] for r in sel) / len(sel), 3)})
                fail_offs = sorted({r["offset_s"] for r in seld if not r["impact_ok"]})
                w = max(seld, key=lambda r: r["v_rel_impact_kph"])
                by_dur.append({"variant": var, "config": n, "duration_s": dur, "runs": len(seld),
                               "impact_fail_runs": sum(not r["impact_ok"] for r in seld),
                               "lead_fail_runs": sum(not r["lead_ok"] for r in seld),
                               "demand_fail_runs": sum(not r["demand_ok"] for r in seld),
                               "any_fail_runs": sum(not r["all_ok"] for r in seld),
                               "offsets_with_impact_fail": " ".join(f"{o:+.1f}" for o in fail_offs),
                               "worst_impact_kph": w["v_rel_impact_kph"], "worst_case": w["case_id"],
                               "worst_offset_s": w["offset_s"], "worst_limit_kph": w["limit_kph"],
                               "fail_at_offset_0": sum(not r["impact_ok"] for r in seld if r["offset_s"] == 0.0)})
    write_csv("e3_burst_summary.csv", summ)
    write_csv("e3_burst_by_duration.csv", by_dur)
    SUMMARY["e3"] = {"by_duration": by_dur}


# ======================================================================================== E4
def e4(W, quick):
    ids = OFFICIAL if quick else SWEEP
    conds = [(lat, 0.0, 0) for lat in LATENCIES] + [(0.010, 0.010, sd) for sd in range(3)]
    jobs, params = [], []
    for n in CONFIGS:
        for lat, jit, sd in conds:
            for c in ids:
                for ph in PHASES:
                    jobs.append(Job(c, n, ph, BusSetup(faults=BusFaults(latency_s=lat, jitter_s=jit, seed=sd))))
                    params.append({"latency_s": lat, "jitter_s": jit, "seed": sd})
    rows = merge(run_rows(jobs, W), params)
    write_csv("e4_latency_runs.csv", rows)
    base = {(r["config"], r["case_id"], r["phase"]): r for r in rows if r["latency_s"] == 0.0}
    summ = []
    for n in CONFIGS:
        for lat, jit in sorted({(r["latency_s"], r["jitter_s"]) for r in rows}):
            sel = [r for r in rows if r["config"] == n and r["latency_s"] == lat and r["jitter_s"] == jit]
            pts = defaultdict(list)
            for r in sel:
                pts[r["case_id"]].append(r)
            dtb = [r["t_brake"] - base[(n, r["case_id"], r["phase"])]["t_brake"] for r in sel
                   if r["t_brake"] is not None and base[(n, r["case_id"], r["phase"])]["t_brake"] is not None]
            dv = [max(x["v_rel_impact_kph"] for x in v) - max(base[(n, c, x["phase"])]["v_rel_impact_kph"] for x in v)
                  for c, v in pts.items()]
            summ.append({"config": n, "latency_s": lat, "jitter_s": jit, "points": len(pts), "runs": len(sel),
                         "impact_fail_points": sum(any(not x["impact_ok"] for x in v) for v in pts.values()),
                         "lead_fail_points": sum(any(not x["lead_ok"] for x in v) for v in pts.values()),
                         "demand_fail_points": sum(any(not x["demand_ok"] for x in v) for v in pts.values()),
                         "max_worst_dv_kph": round(max(dv), 3), "mean_dt_brake_s": round(sum(dtb) / len(dtb), 4),
                         "max_dt_brake_s": round(max(dtb), 4),
                         "min_gap_min_m": round(min(r["min_gap_m"] for r in sel), 3),
                         "worst_impact_kph": max(r["v_rel_impact_kph"] for r in sel),
                         "fault_runs": sum(r["t_fault"] is not None for r in sel)})
    write_csv("e4_latency_summary.csv", summ)
    SUMMARY["e4"] = {"summary": summ}


# ======================================================================================== E5a
def _detected(cfg, data):
    rx = p01_read_counter(cfg, data)
    return rx >= 15 or data[cfg.crc_offset // 8] != p01_crc(cfg, data, rx)


def e5a(quick):
    db = load_db()
    rng = random.Random(2026)
    n_pay, n_rand = (3, 300) if quick else (20, 2000)
    out = []
    for m in db.messages:
        cfg = p01_config(m, RxConfig())
        prot = P01Protector(cfg)
        sigs = [s for s in m.signals if not s.name.endswith(("_CRC", "_AliveCtr"))]
        for w in (1, 2, 3, 4, 8):
            tot = changed = crc_det = range_det_changed = crc_miss = 0
            n_p = n_pay if w != 3 else max(1, n_pay // 4)          # 3-bit: exhaustive on fewer payloads
            for _ in range(n_p):
                vals = {s.name: rng.uniform(s.minimum, s.maximum) for s in sigs}
                data = bytearray(m.encode(vals))
                prot.protect(data)
                ok = m.decode(bytes(data))
                pats = itertools.combinations(range(64), w) if w <= 3 else \
                    (tuple(rng.sample(range(64), w)) for _ in range(n_rand))
                for bits in pats:
                    bad = flip_bits(bytes(data), bits)
                    tot += 1
                    det = _detected(cfg, bad)
                    crc_det += det
                    crc_miss += not det
                    phys = m.decode(bad)
                    if any(phys[s.name] != ok[s.name] for s in sigs):
                        changed += 1
                        range_det_changed += bool(m.out_of_range(phys))
            out.append({"message": m.name, "flipped_bits": w, "patterns": tot,
                        "method": "exhaustive" if w <= 3 else f"random {n_rand} per payload", "payloads": n_p,
                        "crc_detected": crc_det, "crc_undetected": crc_miss,
                        "crc_detection_rate": round(crc_det / tot, 6), "payload_changed": changed,
                        "range_check_detected_of_changed": range_det_changed,
                        "range_check_rate_of_changed": round(range_det_changed / changed, 4) if changed else None})
    write_csv("e5_crc_detection.csv", out)
    SUMMARY["e5a"] = out


# ======================================================================================== E5b
def classify(r, ref):
    if r["t_brake"] is not None and (ref["t_brake"] is None or r["t_brake"] < ref["t_brake"] - 0.2 - 1e-9):
        return "early_brake"
    if r["v_rel_impact_kph"] > ref["v_rel_impact_kph"] + 0.5 or (ref["all_ok"] and not r["all_ok"]):
        return "late_or_missed_brake"
    if r["t_fcw"] is not None and ref["t_fcw"] is not None and r["t_fcw"] < ref["t_fcw"] - 0.2 - 1e-9:
        return "early_warning_only"
    return "no_effect"


def e5b(W, quick):
    cases = ("R152-S60", "R152-M60")
    refs = {(r["case_id"], r["config"]): r for r in
            run_rows([Job(c, n, 0.0, BusSetup(), tag="ref") for c in cases for n in CONFIGS], 1)}
    variants = {"E2E": (RxConfig(), tuple(E2E_BITS)),
                "noE2E": (RxConfig(e2e=False), tuple(E2E_BITS)),
                "noE2E-noRange": (RxConfig(e2e=False, plausibility=False), ("TgtRelSpd",))}
    jobs, params = [], []
    step = 0.4 if quick else 0.1
    for c in cases:
        for n in CONFIGS:
            tb = refs[(c, n)]["t_brake"]
            times = [round(2.0 + step * i, 3) for i in range(int((tb + 1.5 - 2.0) / step) + 1)]
            for var, (rx, sigs) in variants.items():
                for sig in sigs:
                    for bit_index, bit in enumerate(E2E_BITS[sig]):
                        for t_inj in times:
                            hit = Corruption("RADAR_TGT", t_inj, (bit,))
                            jobs.append(Job(c, n, 0.0, BusSetup(faults=BusFaults(targeted=(hit,)), rx=rx)))
                            params.append({"variant": var, "signal": sig, "raw_bit": bit_index, "dbc_bit": bit,
                                           "t_inj": t_inj})
    rows = merge(run_rows(jobs, W), params)
    for r in rows:
        ref = refs[(r["case_id"], r["config"])]
        r["effect"] = classify(r, ref) if r["frames_corrupted"] else "not_injected"
        r["early_brake_s"] = r2(ref["t_brake"] - r["t_brake"], 3) if r["t_brake"] is not None else None
        r["dv_kph"] = round(r["v_rel_impact_kph"] - ref["v_rel_impact_kph"], 3)
    rows = [r for r in rows if r["effect"] != "not_injected"]
    write_csv("e5_corruption_runs.csv", rows)
    summ = []
    for var in variants:
        for n in CONFIGS:
            for c in cases:
                for sig in variants[var][1]:
                    sel = [r for r in rows if r["variant"] == var and r["config"] == n and r["case_id"] == c
                           and r["signal"] == sig]
                    if not sel:
                        continue
                    eff = defaultdict(int)
                    for r in sel:
                        eff[r["effect"]] += 1
                    early = [r for r in sel if r["effect"] == "early_brake"]
                    summ.append({"variant": var, "config": n, "case_id": c, "signal": sig, "runs": len(sel),
                                 "corrupted_frames_accepted": sum(r["corrupted_accepted"] > 0 for r in sel),
                                 "early_brake_runs": eff["early_brake"],
                                 "late_or_missed_runs": eff["late_or_missed_brake"],
                                 "early_warning_only_runs": eff["early_warning_only"],
                                 "no_effect_runs": eff["no_effect"],
                                 "r152_impact_fail_runs": sum(not r["impact_ok"] for r in sel),
                                 "r152_lead_fail_runs": sum(not r["lead_ok"] for r in sel),
                                 "r152_fail_runs": sum(not r["all_ok"] for r in sel),
                                 "max_early_brake_s": max((r["early_brake_s"] for r in early), default=0.0),
                                 "max_ttc_at_early_brake_s": max((r["ttc_brake"] or 0.0 for r in early), default=0.0),
                                 "max_dv_kph": max(r["dv_kph"] for r in sel)})
    write_csv("e5_corruption_summary.csv", summ)
    SUMMARY["e5b"] = {"summary": summ, "refs": {f"{c}|{n}": {k: refs[(c, n)][k] for k in
                                                              ("t_fcw", "t_brake", "v_rel_impact_kph", "all_ok")}
                                                for c, n in refs}}


# ======================================================================================== E6
def _faults(r):
    out = []
    for tok in (r.input_faults or "").split():
        mon, rest = tok.split(":", 1)
        kind, t = rest.split("@")
        out.append((mon, kind, float(t)))
    return out


def e6(W, quick):
    case, cfgn = "R152-M60", "proposed"
    ref = run_job(Job(case, cfgn, 0.0, BusSetup()))
    tb = ref.t_brake
    RAD, SPD = "RADAR_TGT", "EGO_SPEED"
    dist9 = E2E_BITS["TgtDist"][9]
    demos = [
        ("F1", "RADAR_TGT: 2 frames lost (100 ms)", BusFaults(bursts=((2.0, 0.1),), burst_msgs=(RAD,)), 2.0, 0.1, 0.0),
        ("F2", "RADAR_TGT: lost 500 ms (timeout)", BusFaults(bursts=((2.0, 0.5),), burst_msgs=(RAD,)), 2.0, 0.5, 0.0),
        ("F3", "EGO_SPEED: lost 500 ms (timeout)", BusFaults(bursts=((2.0, 0.5),), burst_msgs=(SPD,)), 2.0, 0.5, 0.0),
        ("F4", "RADAR_TGT: 1 corrupted frame (CRC)", BusFaults(targeted=(Corruption(RAD, 2.0, (dist9,)),)),
         2.0, 0.05, 0.0),
        ("F5", "RADAR_TGT: 2 consecutive corrupted frames", BusFaults(targeted=(Corruption(RAD, 2.0, (dist9,), 2),)),
         2.0, 0.1, 0.0),
        ("F6", "RADAR_TGT: stuck sender (same frame repeated 300 ms)", BusFaults(freeze=((RAD, 2.0, 0.3),)),
         2.0, 0.3, 0.0),
        ("F7", "AEB_CMD: AEB ECU bus-off 300 ms during braking", BusFaults(bus_off=(("AEB_ECU", tb + 0.2, 0.3),)),
         tb + 0.2, 0.3, 0.0),
        ("F8", "bus-wide burst 200 ms before braking", BusFaults(bursts=((2.0, 0.2),)), 2.0, 0.2, 0.0),
        ("F9", "bus-wide burst 200 ms during braking (baseline)", BusFaults(bursts=((tb + 0.2, 0.2),)), tb + 0.2, 0.2,
         0.0),
        ("F10", "bus-wide burst 200 ms during braking (hold variant H)", BusFaults(bursts=((tb + 0.2, 0.2),)),
         tb + 0.2, 0.2, HOLD_S),
    ]
    rows = []
    for fid, desc, faults, t0, dur, hold in demos:
        r = run_job(Job(case, cfgn, 0.0, BusSetup(faults=faults, hold_on_fault_s=hold), record=True))
        s = r.series
        fl = _faults(r)
        t_det = min((t for _, _, t in fl), default=None)
        off = None
        if r.t_telltale is not None:
            off = next((t for t, tt in zip(s["t"], s["telltale"]) if t > t0 + dur and not tt), None)
        rows.append({"id": fid, "fault": desc, "t_start": round(t0, 3), "duration_s": dur,
                     "first_detection": " ".join(f"{m}:{k}@{t:.3f}" for m, k, t in fl[:4]) or "none (frame-level only)",
                     "detect_delay_s": r2(t_det - t0, 3) if t_det is not None else None,
                     "t_aeb_fault": r.t_fault, "telltale_delay_s": r2(r.t_telltale - t0, 3) if r.t_telltale else None,
                     "telltale_off_after_end_s": r2(off - (t0 + dur), 3) if off is not None else None,
                     "brake_lost_s": round(r.brake_lost_s, 3), "hold_s": round(r.hold_time_s, 3),
                     "rx_status_aeb_radar": json.dumps(r.rx_status.get("aeb<RADAR_TGT", {}), sort_keys=True),
                     "v_rel_impact_kph": round(r.v_rel_impact_kph, 2), "fcw_lead_s": r2(r.fcw_lead_s, 3),
                     "ref_v_rel_impact_kph": round(ref.v_rel_impact_kph, 2)})
    write_csv("e6_fault_reaction.csv", rows)
    SUMMARY["e6"] = {"case": case, "config": cfgn, "ref_t_brake": tb, "rows": rows}
    # time series for figure 3: 200 ms burst at the brake onset of R152-S60 (proposed), baseline vs HR
    ref60 = run_job(Job("R152-S60", "proposed", 0.0, BusSetup()))
    for var in ("U", "HR"):
        ov, hold = VARIANTS[var]
        r = run_job(Job("R152-S60", "proposed", 0.0, BusSetup(faults=BusFaults(bursts=((ref60.t_brake, 0.2),)),
                                                                hold_on_fault_s=hold), aeb_overrides=ov, record=True))
        s = r.series
        keep = [i for i, t in enumerate(s["t"]) if abs(t / 0.01 - round(t / 0.01)) < 1e-6]
        write_csv(f"e6_timeline_S60_burst200_{var}.csv",
                  [{k: (r2(s[k][i], 4) if isinstance(s[k][i], float) else s[k][i]) for k in s} for i in keep])
        SUMMARY["e6"][f"timeline_{var}"] = {"burst_start": ref60.t_brake, "impact_kph": round(r.v_rel_impact_kph, 2)}
    rf = run_job(Job("R152-S60", "proposed", 0.0, BusSetup(), record=True)).series
    keep = [i for i, t in enumerate(rf["t"]) if abs(t / 0.01 - round(t / 0.01)) < 1e-6]
    write_csv("e6_timeline_S60_faultfree.csv",
              [{k: (r2(rf[k][i], 4) if isinstance(rf[k][i], float) else rf[k][i]) for k in rf} for i in keep])


# ======================================================================================== main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--quick", action="store_true", help="reduced grids (smoke test)")
    ap.add_argument("--only", default="e1,e2,e3,e4,e5a,e5b,e6")
    a = ap.parse_args()
    t0 = time.time()
    steps = {"e1": lambda: e1(a.workers, a.quick), "e2": lambda: e2(a.workers, a.quick),
             "e3": lambda: e3(a.workers, a.quick), "e4": lambda: e4(a.workers, a.quick),
             "e5a": lambda: e5a(a.quick), "e5b": lambda: e5b(a.workers, a.quick), "e6": lambda: e6(a.workers, a.quick)}
    summary_path = RESULTS / "summary_signal_level.json"
    if summary_path.exists():                     # keep the results of steps that are not re-run
        SUMMARY.update(json.loads(summary_path.read_text(encoding="utf-8")))
    for name in a.only.split(","):
        ts = time.time()
        print(f"[{name}] start", flush=True)
        steps[name]()
        SUMMARY.setdefault("runtime_s", {})[name] = round(time.time() - ts, 1)
        print(f"[{name}] done in {time.time() - ts:.0f} s", flush=True)
        summary_path.write_text(json.dumps(SUMMARY, indent=1, ensure_ascii=False, default=str), encoding="utf-8")
    SUMMARY["quick"] = a.quick
    SUMMARY["max_workers"] = a.workers
    summary_path.write_text(json.dumps(SUMMARY, indent=1, ensure_ascii=False, default=str), encoding="utf-8")
    print(f"all done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
