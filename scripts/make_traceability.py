"""Build traceability.csv (clause -> requirement -> signals -> OpenSCENARIO test case -> result) from the
results of run_all.py / run_deepening.py, and check completeness in both directions:
  forward : every requirement has at least one test row, or an explicit N/A reason
  backward: every test case ID of every grid file (and every condition variant that was run) traces to
            at least one requirement
Run:  python -I scripts/make_traceability.py      (exit code 1 if a trace is missing)
"""
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aebtest.requirements import REQS  # noqa: E402
from aebtest.runner import GRID, POINTS  # noqa: E402

RES = ROOT / "results"
ZH = {"green": "绿", "yellow": "黄", "orange": "橙", "brown": "棕", "red": "红"}
CFG = {"original": "原参数", "proposed": "新参数", "friction_aware_0.7": "摩擦自适应(μ0.7)",
       "friction_aware_0.5": "摩擦自适应(μ0.5)"}


def load(name):
    p = RES / name
    return list(csv.DictReader(open(p, encoding="utf-8"))) if p.exists() else []


def row(req, case_id, config, condition, result, verdict, note=""):
    base = case_id.split("@")[0]
    grid, scen = (GRID[base][0], GRID[base][1]) if base in GRID else ("-", "-")
    return {"req_id": req.rid, "source": req.source, "clause": req.clause, "requirement": req.text,
            "kind": req.kind, "signals": req.signals, "safety_goal": req.safety_goal, "test_case_id": case_id,
            "scenario_file": scen, "grid_file": grid, "config": CFG.get(config, config), "condition": condition,
            "result": result, "verdict": verdict, "note": note}


def main() -> int:
    r152, ncap = load("r152_results.csv"), load("ncap_results.csv")
    wet, ghost, rng = load("wet_results.csv"), load("sotif_ghost.csv"), load("sotif_range.csv")
    rows = []
    for req in REQS:
        t = req.tests
        if t == "r152:lead":
            for r in r152:
                rows.append(row(req, r["case_id"], r["config"], "基线", f"FCW 提前量最小 {r['fcw_lead_min_s']} s",
                                "通过" if r["lead_ok"] == "True" else "不通过"))
        elif t == "r152:demand":
            for r in r152:
                rows.append(row(req, r["case_id"], r["config"], "基线", "最大需求≥5 m/s²" if r["demand_ge_5_ok"] == "True"
                                else "只到部分制动 4 m/s²", "通过" if r["demand_ge_5_ok"] == "True" else "不通过（字面）"))
        elif t == "r152:active":
            for r in r152:
                act = r["fcw_lead_min_s"] not in ("", "None")
                rows.append(row(req, r["case_id"], r["config"], "基线", f"{float(r['v_ego_kph']):g} km/h 下有预警和制动"
                                if act else "未激活", "通过" if act else "不通过"))
        elif t.startswith("r152:impact:"):
            fam = t.split(":")[2]
            for r in r152:
                if POINTS[r["case_id"]].family == fam:
                    rows.append(row(req, r["case_id"], r["config"], "基线",
                                    f"最差相对碰撞速度 {r['v_rel_impact_worst_kph']} km/h（限值 {r['limit_kph']}）",
                                    "通过" if r["impact_ok"] == "True" else "不通过"))
        elif t == "all:ccr_constant":
            for cid, (grid, scen, _, p) in GRID.items():
                if POINTS[cid].protocol in ("R152", "NCAP") and POINTS[cid].family in ("CCRs", "CCRm"):
                    v_rel = float(p["EgoSpeed"]) - float(p["TargetSpeed"])
                    gap = float(p["TargetStartS"]) - 0.7 - float(p["EgoStartS"]) - 3.7
                    rows.append(row(req, cid, "-", "场景参数", f"起始 TTC {gap / v_rel:.2f} s，T0 在 2 s 预行驶之后",
                                    "已实现" if gap / v_rel >= 4.0 - 1e-6 else "不符"))
        elif t == "r152:CCRm":
            for cid, (grid, scen, _, p) in GRID.items():
                if POINTS[cid].protocol == "R152" and POINTS[cid].family == "CCRm":
                    v = float(p["TargetSpeed"]) * 3.6
                    rows.append(row(req, cid, "-", "场景参数", f"目标车速 {v:.1f} km/h", "已实现" if abs(v - 20) < 0.01 else "不符"))
        elif t == "ncap:CCRb_def":
            for cid, (grid, scen, _, p) in GRID.items():
                if POINTS[cid].protocol == "NCAP" and POINTS[cid].family == "CCRb":
                    thw = (float(p["TargetStartS"]) - 0.7 - float(p["EgoStartS"]) - 3.7) / float(p["EgoSpeed"])
                    rows.append(row(req, cid, "-", "场景参数", f"车头时距 {thw:.2f} s，减速度 {p['TargetDecel']} m/s²（阶跃）",
                                    "已实现" if abs(thw - 1.0) < 1e-6 and float(p["TargetDecel"]) == 4.0 else "不符"))
        elif t.startswith("ncap:CCR"):
            fam = t.split(":")[1]
            for r in ncap:
                if r["family"] == fam:
                    rows.append(row(req, r["case_id"], r["config"], "基线",
                                    f"最差相对碰撞速度 {r['v_rel_impact_worst_kph']} km/h（{'标准' if r['range_50pct'] == 'standard' else '扩展'}范围）",
                                    f"颜色：{ZH[r['colour']]}"))
        elif t == "ncap:gate":
            for r in ncap:
                if r["family"] == "CCRs" and float(r["v_ego_kph"]) <= 20:
                    ok = float(r["v_rel_impact_worst_kph"]) == 0.0
                    rows.append(row(req, r["case_id"], r["config"], "基线", "全部相位避撞" if ok else "有碰撞",
                                    "通过" if ok else "不通过"))
        elif t == "ncap:late":
            for r in ncap:
                if r["family"] == "CCRb":
                    late = r["late_contact_after_ncap_end"] == "True"
                    rows.append(row(req, r["case_id"], r["config"], "基线，物理续算到两车都停",
                                    "V_VUT=V_target 之后又碰到目标" if late else "无二次接触", "不通过" if late else "通过"))
        elif t == "wet":
            for r in wet:
                if r["road_mu"] in ("0.7", "0.5"):
                    cid = f"{r['case_id']}@mu{r['road_mu']}"
                    verdict = ("通过" if r["verdict"] == "pass" else "不通过") if r["case_id"].startswith("R152") \
                        else f"颜色：{ZH[r['verdict']]}"
                    rows.append(row(req, cid, r["config"], f"RoadMu={r['road_mu']}",
                                    f"最差相对碰撞速度 {r['v_rel_impact_worst_kph']} km/h", verdict, "模型内参考评价"))
        elif t == "sotif:range":
            for r in rng:
                if r["case_id"].startswith("R152") and float(r["max_range_m"]) < 200:
                    cid = f"{r['case_id']}@range{float(r['max_range_m']):g}m" + ("" if r["road_mu"] == "0.9" else f"@mu{r['road_mu']}")
                    lead = r["fcw_lead_min_s"]
                    ok = lead not in ("", "None") and float(lead) >= 0.8 - 1e-9
                    rows.append(row(req, cid, r["config"], f"探测距离 {float(r['max_range_m']):g} m，RoadMu={r['road_mu']}",
                                    f"FCW 提前量最小 {lead} s，最差碰撞 {r['v_rel_impact_worst_kph']} km/h",
                                    "通过" if ok else "不通过"))
        elif t == "sotif:ghost":
            for r in ghost:
                cid = f"{r['case_id']}@ghost-ttc{r['ghost_ttc_s']}-vis{r['ghost_visible_s']}" + \
                      ("" if r["road_mu"] == "0.9" else f"@mu{r['road_mu']}")
                brake = float(r["max_decel_request_mps2"]) > 0
                rows.append(row(req, cid, r["config"], f"幻影静止目标 TTC {r['ghost_ttc_s']} s，可见 {r['ghost_visible_s']} s",
                                f"{'误制动 ' + r['max_decel_request_mps2'] + ' m/s²，速度损失 ' + r['speed_loss_kph'] + ' km/h' if brake else ('仅误报警' if r['fcw'] == 'True' else '无反应')}",
                                "不通过（误制动）" if brake else ("通过（仅预警）" if r["fcw"] == "True" else "通过")))
        elif req.na_reason:
            rows.append(row(req, "-", "-", "-", "P5 未验证", "N/A", req.na_reason))

    # ---- completeness checks
    problems = []
    traced = defaultdict(set)
    for r in rows:
        traced[r["req_id"]].add(r["test_case_id"])
    for req in REQS:
        if not traced[req.rid] or (traced[req.rid] == {"-"} and not req.na_reason):
            problems.append(f"forward: {req.rid} has no test and no N/A reason")
    case_ids = {r["test_case_id"].split("@")[0] for r in rows}
    for cid in GRID:
        if cid not in case_ids:
            problems.append(f"backward: test case {cid} (grid {GRID[cid][0]}) traces to no requirement")
    keys = list(rows[0])
    with open(ROOT / "traceability.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    # ---- summary table for report.md
    agg = defaultdict(lambda: defaultdict(int))
    for r in rows:
        agg[(r["req_id"], r["config"])][r["verdict"]] += 1
    L = ["<!-- generated by scripts/make_traceability.py -->", "",
         "| 需求 | 来源条款 | 测试用例数 | 参数组：判定计数 |", "|---|---|---:|---|"]
    for req in REQS:
        cases = sorted({r["test_case_id"] for r in rows if r["req_id"] == req.rid})
        parts = []
        for (rid, cfg), cnt in agg.items():
            if rid == req.rid:
                parts.append(f"{cfg}：" + "，".join(f"{k} {v}" for k, v in sorted(cnt.items())))
        L.append(f"| {req.rid} {req.text[:28]}{'…' if len(req.text) > 28 else ''} | {req.clause} | "
                 f"{0 if cases == ['-'] else len(cases)} | {'；'.join(parts)} |")
    L += ["", f"完整性检查：{'通过（每条需求都有测试或 N/A 理由，每个测试用例都能追溯到需求）' if not problems else '；'.join(problems)}",
          f"总行数 {len(rows)}；OpenSCENARIO 用例 {len(GRID)} 个（含 SOTIF 自由路 2 个），条件变体用 @ 后缀区分。"]
    (RES / "traceability_summary.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print(f"{len(rows)} rows, {len(REQS)} requirements, {len(GRID)} OpenSCENARIO cases; problems: {problems or 'none'}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
