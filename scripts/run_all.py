"""Run every experiment of the project and write results/ (CSV, JSON, Markdown tables).

  A  main matrix: all R152 + Euro NCAP test points, original vs proposed AebConfig, baseline sensor/brake,
     5 radar sampling phases each (worst phase is reported); phase-0 runs keep a time series
  B  cross-check: the same matrix on the pure-Python backend must give identical results
  C  numerical check: dt = 1 ms vs the 2 ms used everywhere (Python backend)
  D  sensitivity: sensor latency {0, 100, 200} ms x brake time constant {0.12, 0.25, 0.40} s
  E  noise: Gaussian range / range-rate noise, 20 seeds per point
  F  late-contact what-if (proposed values + v_min_kph or brake_min_dwell_s variants), see report.md 5.4

Run:  python -I scripts/run_all.py --esmini-lib <...>/esmini/bin/esminiLib.dll [--workers 10]
      python -I scripts/run_all.py --backend python          # fallback without esmini
Then: python -I scripts/make_plots.py
"""
import argparse
import csv
import gzip
import json
import shutil
import sys
import time
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aebtest.experiments import (CONFIGS, DT, LATENCIES, PHASES, TAUS, config, ncap_total,  # noqa: E402
                                 r152_ok, summarize, sweep, with_params, worst)
from aebtest.regulations import ncap_points, r152_points  # noqa: E402
from aebtest.runner import POINTS, run_many, write_series  # noqa: E402
from aebtest.scenarios import regulatory_points  # noqa: E402

RES = ROOT / "results"
ESMINI_LOG_CASES = {"NCAP-CCRs-80", "R152-S60"}   # phase 0, baseline: keep esmini's own CSV/.dat/.log
NOISE = (0.15, 0.15)                              # range [m], range rate [m/s] - project assumption
NOISE_SEEDS = 20
STRESS = (0.1, 0.25)                              # moderate stress condition used in the tables


def fmt(v, nd=1):
    return "-" if v is None else f"{v:.{nd}f}"


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def late_contacts(runs) -> dict:
    """Runs that touch the target only AFTER the Euro NCAP end condition V_VUT = V_target."""
    late = [r for r in runs if r.contact and not r.contact_ncap]
    return {"runs": len(late), "of_runs": len(runs),
            "max_kph": round(max((r.v_rel_impact_kph for r in late), default=0.0), 2),
            "cases": sorted({r.case_id for r in late})}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esmini-lib")
    ap.add_argument("--backend", default="esmini", choices=["esmini", "python"])
    ap.add_argument("--workers", type=int, default=10)
    a = ap.parse_args()
    kind, lib, W = a.backend, a.esmini_lib, a.workers
    t_start = time.time()
    all_ids = [p.case_id for p in regulatory_points()]
    off_ids = [p.case_id for p in r152_points(sweep=False)]
    ncap_ids = [p.case_id for p in ncap_points()]
    summary = {"backend": kind, "dt_s": DT, "phases_s": PHASES, "configs": {n: asdict(c) for n, c in CONFIGS.items()}}

    # ---------------------------------------------------------------- A main matrix
    if (RES / "esmini_logs").exists():
        shutil.rmtree(RES / "esmini_logs")
    main_sum, main_runs = {}, {}
    for name, aeb in CONFIGS.items():
        jobs = [(cid, config(aeb, 0.0, 0.12, ph), ph == 0.0,
                 f"{name}_{cid}" if (kind == "esmini" and ph == 0.0 and cid in ESMINI_LOG_CASES) else None)
                for cid in all_ids for ph in PHASES]
        runs = run_many(jobs, kind, lib, W)
        main_runs[name] = runs
        group = {}
        for r in runs:
            group.setdefault(r.case_id, []).append(r)
        main_sum[name] = summarize(group)
        tdir = RES / "timeseries" / name
        tdir.mkdir(parents=True, exist_ok=True)
        for r in runs:
            if r.series:
                every = max(1, round(0.01 / DT))           # store at 100 Hz (Euro NCAP 1.5: >= 100 Hz)
                r.series = {k: v[::every] + ([v[-1]] if (len(v) - 1) % every else []) for k, v in r.series.items()}
                write_series(r, tdir / f"{r.case_id}.csv")
        print(f"[A] {name}: {len(runs)} runs, {time.time() - t_start:.0f}s", flush=True)
    for f in (RES / "esmini_logs").glob("*.csv") if (RES / "esmini_logs").exists() else []:
        with open(f, "rb") as src, gzip.open(f.with_suffix(".csv.gz"), "wb") as dst:
            shutil.copyfileobj(src, dst)
        f.unlink()

    r152_rows, ncap_rows = [], []
    for cid in all_ids:
        tp = POINTS[cid]
        for name in CONFIGS:
            s = main_sum[name]
            if tp.protocol == "R152":
                v = s["r152"][cid]
                r152_rows.append({"case_id": cid, "config": name, "label": tp.label, "v_ego_kph": tp.v_ego_kph,
                                  "v_target_kph": tp.v_target_kph, "v_rel_kph": tp.v_rel_kph,
                                  "masses": "+".join(tp.masses), "limit_kph": v["limit"],
                                  "v_rel_impact_worst_kph": round(v["v_rel_impact_max"], 2),
                                  "v_rel_impact_best_kph": round(v["v_rel_impact_min"], 2),
                                  "contact_runs": f"{v['contacts']}/{v['runs']}", "impact_ok": v["impact_ok"],
                                  "fcw_lead_min_s": None if v["lead_min"] is None else round(v["lead_min"], 3),
                                  "lead_ok": v["lead_ok"], "demand_ge_5_ok": v["demand_ok"],
                                  "min_gap_m": round(v["min_gap_min"], 3),
                                  "all_ok": v["impact_ok"] and v["lead_ok"] and v["demand_ok"]})
            else:
                v = s["ncap"][cid]
                ncap_rows.append({"case_id": cid, "config": name, "family": tp.family, "function": tp.label,
                                  "v_ego_kph": tp.v_ego_kph, "v_target_kph": tp.v_target_kph,
                                  "range_50pct": "standard" if v["standard"] else "extended",
                                  "v_rel_impact_worst_kph": round(v["v_rel_impact_max"], 2),
                                  "v_rel_impact_best_kph": round(v["v_rel_impact_min"], 2),
                                  "colour": v["colour"], "min_gap_m": round(v["min_gap_min"], 3),
                                  "fcw_lead_min_s": None if v["lead_min"] is None else round(v["lead_min"], 3),
                                  "late_contact_after_ncap_end": v["late_contact_any"]})
    write_csv(RES / "r152_results.csv", r152_rows)
    write_csv(RES / "ncap_results.csv", ncap_rows)
    summary["main"] = {n: {"r152_all_ok": r152_ok(main_sum[n]),
                           "r152_failures": {c: {k: v for k, v in d.items()} for c, d in main_sum[n]["r152"].items()
                                             if not (d["impact_ok"] and d["lead_ok"] and d["demand_ok"])},
                           "ncap_score": main_sum[n]["score"], "ncap_total": round(ncap_total(main_sum[n]), 2),
                           "late_contacts": late_contacts(main_runs[n])}
                       for n in CONFIGS}

    # ---------------------------------------------------------------- B backend cross-check
    if kind == "esmini":
        diffs = []
        for name, aeb in CONFIGS.items():
            jobs = [(cid, config(aeb, 0.0, 0.12, ph), False, None) for cid in all_ids for ph in PHASES]
            py = run_many(jobs, "python", None, W)
            es = {(r.case_id, i % len(PHASES)): r for i, r in enumerate(main_runs[name])}
            for i, r in enumerate(py):
                e = es[(r.case_id, i % len(PHASES))]
                diffs.append({"case": f"{r.case_id}@phase{PHASES[i % len(PHASES)]:.2f}",
                              "impact": abs(e.v_rel_impact_kph - r.v_rel_impact_kph),
                              "gap": abs(e.min_gap_m - r.min_gap_m) if not (e.contact or r.contact) else 0.0,
                              "states": e.states == r.states, "contact": e.contact == r.contact,
                              "bbox": e.backend_collision == e.contact})
        summary["crosscheck_esmini_vs_python"] = {
            "runs": len(diffs), "max_abs_dv_impact_kph": max(d["impact"] for d in diffs),
            "max_abs_dmin_gap_m": max(d["gap"] for d in diffs),
            "same_state_sequence": sum(d["states"] for d in diffs), "same_contact": sum(d["contact"] for d in diffs),
            "esmini_bbox_collision_equals_gap_le_0": sum(d["bbox"] for d in diffs),
            "runs_with_min_gap_diff_gt_1mm": sorted({d["case"] for d in diffs if d["gap"] > 1e-3})}
        print(f"[B] {summary['crosscheck_esmini_vs_python']}", flush=True)

    # ---------------------------------------------------------------- C dt check (Python backend)
    fine = sweep(all_ids, CONFIGS["original"], 0.0, 0.12, "python", None, W, dt=0.001)
    coarse = sweep(all_ids, CONFIGS["original"], 0.0, 0.12, "python", None, W, dt=DT)
    dts = sorted(((abs(worst(fine[c]).v_rel_impact_kph - worst(coarse[c]).v_rel_impact_kph), c) for c in all_ids),
                 reverse=True)
    flips = [c for c in all_ids if any(r.contact for r in fine[c]) != any(r.contact for r in coarse[c])]
    summary["dt_check"] = {"dt_fine_s": 0.001, "dt_used_s": DT, "max_abs_dv_worst_kph": round(dts[0][0], 2),
                           "case": dts[0][1], "contact_flips_any_phase": flips}
    print(f"[C] {summary['dt_check']}", flush=True)

    # ---------------------------------------------------------------- D sensitivity
    sens_rows, sens_sum = [], []
    for name, aeb in CONFIGS.items():
        for lat in LATENCIES:
            for tau in TAUS:
                s = summarize(sweep(off_ids + ncap_ids, aeb, lat, tau, kind, lib, W))
                fails = [c for c, v in s["r152"].items() if not v["impact_ok"]]
                fails_all = [c for c, v in s["r152"].items() if not (v["impact_ok"] and v["lead_ok"] and v["demand_ok"])]
                sens_sum.append({"config": name, "latency_s": lat, "tau_s": tau,
                                 "r152_impact_fail_points": len(fails), "r152_any_fail_points": len(fails_all),
                                 "r152_impact_fail_list": " ".join(fails),
                                 **{f"ncap_{f}": s["score"][f]["estimate"] for f in ("CCRs", "CCRm", "CCRb")},
                                 "ncap_total": round(ncap_total(s), 2),
                                 "ccrs_gate_ok": s["score"]["ccrs_low_speed_gate_ok"],
                                 "red_cells": sum(v["colour"] == "red" for v in s["ncap"].values())})
                for c, v in {**s["r152"], **s["ncap"]}.items():
                    sens_rows.append({"config": name, "latency_s": lat, "tau_s": tau, "case_id": c,
                                      "v_rel_impact_worst_kph": round(v["v_rel_impact_max"], 2),
                                      "verdict": ("pass" if v["impact_ok"] else "FAIL") if c.startswith("R152")
                                      else v["colour"]})
                print(f"[D] {name} lat={lat} tau={tau}: R152 impact fails {len(fails)}, NCAP {ncap_total(s):.2f}",
                      flush=True)
    write_csv(RES / "sensitivity.csv", sens_rows)
    write_csv(RES / "sensitivity_summary.csv", sens_sum)

    # ---------------------------------------------------------------- E noise
    noise_rows = []
    for name, aeb in CONFIGS.items():
        jobs = [(cid, config(aeb, 0.0, 0.12, PHASES[sd % len(PHASES)], NOISE, sd), False, None)
                for cid in off_ids + ncap_ids for sd in range(NOISE_SEEDS)]
        group = {}
        for r in run_many(jobs, kind, lib, W):
            group.setdefault(r.case_id, []).append(r)
        s = summarize(group)
        r152_runs = sum(v["runs"] for v in s["r152"].values())
        r152_failed = sum(v["failed_runs"] for v in s["r152"].values())
        for c, v in {**s["r152"], **s["ncap"]}.items():
            noise_rows.append({"config": name, "case_id": c, "runs": NOISE_SEEDS,
                               "v_rel_impact_worst_kph": round(v["v_rel_impact_max"], 2),
                               "v_rel_impact_best_kph": round(v["v_rel_impact_min"], 2),
                               "verdict": (f"{v['failed_runs']}/{v['runs']} failed" if c.startswith("R152")
                                           else v["colour"])})
        summary.setdefault("noise", {})[name] = {"sd_range_m": NOISE[0], "sd_rate_mps": NOISE[1],
                                                "seeds": NOISE_SEEDS, "r152_failed_runs": r152_failed,
                                                "r152_runs": r152_runs, "ncap_total": round(ncap_total(s), 2),
                                                "ncap_score": s["score"]}
        print(f"[E] {name}: R152 failed runs {r152_failed}/{r152_runs}, NCAP {ncap_total(s):.2f}", flush=True)
    write_csv(RES / "noise.csv", noise_rows)

    # ---------------------------------------------------------------- F late-contact what-if
    whatif = []
    variants = [("proposed", {})] + [(f"proposed + v_min_kph={v:g}", {"v_min_kph": v}) for v in (4.0, 2.0, 0.6)] + \
               [(f"proposed + brake_min_dwell_s={d:g}", {"brake_min_dwell_s": d}) for d in (1.0, 2.0, 3.0)]
    for label, kw in variants:
        aeb = with_params(CONFIGS["proposed"], **kw)
        jobs = [(cid, config(aeb, 0.0, 0.12, ph), False, None) for cid in all_ids for ph in PHASES]
        runs = run_many(jobs, kind, lib, W)
        group = {}
        for r in runs:
            group.setdefault(r.case_id, []).append(r)
        s_ = summarize(group)
        lc = late_contacts(runs)
        whatif.append({"variant": label, "r152_all_ok": r152_ok(s_), "ncap_total": round(ncap_total(s_), 2),
                       "late_contact_runs": lc["runs"], "late_contact_max_kph": lc["max_kph"],
                       "late_contact_cases": " ".join(lc["cases"])})
        print(f"[F] {label}: late contacts {lc['runs']} (max {lc['max_kph']} km/h)", flush=True)
    write_csv(RES / "late_contact_whatif.csv", whatif)
    summary["late_contact_whatif"] = whatif

    summary["runtime_s"] = round(time.time() - t_start, 1)
    (RES / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
    write_tables(r152_rows, ncap_rows, sens_sum, summary)
    print(f"done in {summary['runtime_s']} s")


def write_tables(r152_rows, ncap_rows, sens_sum, summary):
    by = {(r["case_id"], r["config"]): r for r in r152_rows}
    L = ["<!-- generated by scripts/run_all.py - do not edit by hand -->", "",
         "### 表 1  UN R152 官方试验点（M1，6.4 / 6.5），基线条件，5 个雷达采样相位取最差", "",
         "| 试验点 | 自车/目标 km/h | 允许相对碰撞速度 km/h | 原参数：最差碰撞速度 / 判定 | 新参数：最差碰撞速度 / 判定 | 原参数 FCW 提前量 s（≥0.8） | 新参数 FCW 提前量 s | 制动需求≥5 m/s²（原/新） |",
         "|---|---|---:|---|---|---:|---:|---|"]
    for p in r152_points(sweep=False):
        o, n = by[(p.case_id, "original")], by[(p.case_id, "proposed")]
        L.append(f"| {p.case_id} | {p.v_ego_kph:g}/{p.v_target_kph:g} | {o['limit_kph']:g} | "
                 f"{o['v_rel_impact_worst_kph']:.1f} / {'通过' if o['impact_ok'] else '不通过'} | "
                 f"{n['v_rel_impact_worst_kph']:.1f} / {'通过' if n['impact_ok'] else '不通过'} | "
                 f"{fmt(o['fcw_lead_min_s'], 2)} | {fmt(n['fcw_lead_min_s'], 2)} | "
                 f"{'是' if o['demand_ge_5_ok'] else '否'}/{'是' if n['demand_ge_5_ok'] else '否'} |")
    L += ["", "### 表 2  UN R152 扩展扫描（5.2.1.4 表中的其他相对速度，按整备质量一列判定）", "",
          "| 试验点 | 自车/目标 km/h | 允许 km/h | 原参数最差 km/h | 新参数最差 km/h | 原参数制动需求≥5 | 新参数制动需求≥5 |",
          "|---|---|---:|---:|---:|---|---|"]
    for p in r152_points(sweep=True):
        if p.label != "sweep":
            continue
        o, n = by[(p.case_id, "original")], by[(p.case_id, "proposed")]
        L.append(f"| {p.case_id} | {p.v_ego_kph:g}/{p.v_target_kph:g} | {o['limit_kph']:g} | "
                 f"{o['v_rel_impact_worst_kph']:.1f}{'' if o['impact_ok'] else ' ✗'} | "
                 f"{n['v_rel_impact_worst_kph']:.1f}{'' if n['impact_ok'] else ' ✗'} | "
                 f"{'是' if o['demand_ge_5_ok'] else '否'} | {'是' if n['demand_ge_5_ok'] else '否'} |")
    nb = {(r["case_id"], r["config"]): r for r in ncap_rows}
    zh = {"green": "绿", "yellow": "黄", "orange": "橙", "brown": "棕", "red": "红"}
    L += ["", "### 表 3  Euro NCAP CCR 网格（50 % 撞击位置），基线条件，5 个相位取最差的相对碰撞速度 km/h 与颜色", "",
          "| 场景 | VUT/GVT km/h | 范围 | 功能列 | 原参数 | 新参数 |", "|---|---|---|---|---|---|"]
    for p in ncap_points():
        o, n = nb[(p.case_id, "original")], nb[(p.case_id, "proposed")]
        L.append(f"| {p.family} | {p.v_ego_kph:g}/{p.v_target_kph:g} | {'标准' if o['range_50pct'] == 'standard' else '扩展'} | "
                 f"{p.label.split(',')[0]} | {o['v_rel_impact_worst_kph']:.1f} {zh[o['colour']]} | "
                 f"{n['v_rel_impact_worst_kph']:.1f} {zh[n['colour']]} |")
    L += ["", "### 表 4  简化 Euro NCAP 标准范围估计（只含 50 % 撞击位置，见 report.md 3.5 的假设）", "",
          "| 条件 | 参数 | CCRs（满分 1.2） | CCRm（2.4） | CCRb（1.6） | 合计（5.2） | CCRs ≤20 km/h 全避免 |",
          "|---|---|---:|---:|---:|---:|---|"]
    for row in sens_sum:
        if (row["latency_s"], row["tau_s"]) in ((0.0, 0.12), STRESS):
            cond = "基线（0 ms，τ=0.12 s）" if row["latency_s"] == 0.0 else "中等压力（100 ms，τ=0.25 s）"
            L.append(f"| {cond} | {'原' if row['config'] == 'original' else '新'} | {row['ncap_CCRs']:.2f} | "
                     f"{row['ncap_CCRm']:.2f} | {row['ncap_CCRb']:.2f} | {row['ncap_total']:.2f} | "
                     f"{'是' if row['ccrs_gate_ok'] else '否'} |")
    L += ["", "### 表 5  敏感性：传感器附加时延 × 制动一阶时间常数 τ（死区时间固定 80 ms）", "",
          "| 参数 | 时延 s | τ s | R152 官方点碰撞速度不通过数（/6） | R152 任一条不通过数（/6） | NCAP 简化估计合计（/5.2） | 红格数（/30） |",
          "|---|---:|---:|---:|---:|---:|---:|"]
    for row in sens_sum:
        L.append(f"| {'原' if row['config'] == 'original' else '新'} | {row['latency_s']:g} | {row['tau_s']:g} | "
                 f"{row['r152_impact_fail_points']} | {row['r152_any_fail_points']} | {row['ncap_total']:.2f} | {row['red_cells']} |")
    if "noise" in summary:
        L += ["", f"### 表 6  噪声：距离 σ={NOISE[0]} m、距离变化率 σ={NOISE[1]} m/s，每点 {NOISE_SEEDS} 个种子", "",
              "| 参数 | R152 官方点失败次数 / 总次数 | NCAP 简化估计合计（最差种子） |", "|---|---|---:|"]
        for n, v in summary["noise"].items():
            L.append(f"| {'原' if n == 'original' else '新'} | {v['r152_failed_runs']}/{v['r152_runs']} | {v['ncap_total']:.2f} |")
    L += ["", "### 表 7  Euro NCAP 结束条件之后的二次接触（CCRb：先减到与目标同速、随后松开制动、再碰到已停下的目标）", "",
          "| 参数 | 二次接触运行数（/全部 49 点×5 相位） | 最高相对碰撞速度 km/h | 涉及场景 |", "|---|---:|---:|---|"]
    for n, v in summary["main"].items():
        lc = v["late_contacts"]
        L.append(f"| {'原' if n == 'original' else '新'} | {lc['runs']} | {lc['max_kph']:.1f} | {' '.join(lc['cases']) or '-'} |")
    if "late_contact_whatif" in summary:
        L += ["", "### 表 8  只改一个参数能否消除二次接触（在新参数基础上，基线条件）", "",
              "| 变体 | R152 全部通过 | NCAP 简化估计合计 | 二次接触运行数 | 最高 km/h |", "|---|---|---:|---:|---:|"]
        for w in summary["late_contact_whatif"]:
            L.append(f"| {w['variant']} | {'是' if w['r152_all_ok'] else '否'} | {w['ncap_total']:.2f} | "
                     f"{w['late_contact_runs']} | {w['late_contact_max_kph']:.1f} |")
    (RES / "tables.md").write_text("\n".join(L) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
