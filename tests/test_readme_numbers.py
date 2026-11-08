"""Every headline number on the README first screen is recomputed from the committed results and must appear
in README.md exactly as rendered here.  A change to a results file OR to the README text makes this fail.
Run:  python -I -m pytest -q tests/test_readme_numbers.py"""
import csv
import importlib.util
import json
from pathlib import Path

import pytest

from aebtest.experiments import PROPOSED
from aebtest.regulations import r152_points

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"
SL = ROOT / "signal_level" / "results"
README = (ROOT / "README.md").read_text(encoding="utf-8")


def rows(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def pick(rs, **kw):
    out = [r for r in rs if all(r[k] == v for k, v in kw.items())]
    assert len(out) == 1, (kw, len(out))
    return out[0]


def must_contain(*snippets):
    missing = [s for s in snippets if s not in README]
    assert not missing, f"README.md no longer matches the committed results: {missing}"


SUMMARY = json.loads((RES / "summary.json").read_text(encoding="utf-8"))
R152 = rows(RES / "r152_results.csv")


def test_r152_official_points_and_lead():
    official = {p.case_id for p in r152_points(sweep=False)}
    for cfg in ("original", "proposed"):
        off = [r for r in R152 if r["config"] == cfg and r["case_id"] in official]
        assert len(off) == 6 and all(r["impact_ok"] == "True" for r in off)
        lead = min(float(r["fcw_lead_min_s"]) for r in off)
        assert f"{lead:.2f}" == "1.00"
    must_contain("| R152 官方 6 点碰撞速度 | 全部在限值内 | 全部在限值内 |",
                 "| R152 5.2.1.1 预警提前量（≥ 0.8 s） | 1.00 s | 1.00 s |")


def test_r152_demand_literal():
    o = [r for r in R152 if r["config"] == "original"]
    p = [r for r in R152 if r["config"] == "proposed"]
    o_fail = sum(r["demand_ge_5_ok"] == "False" for r in o)
    p_fail = sum(r["demand_ge_5_ok"] == "False" for r in p)
    assert p_fail == 0
    must_contain(f"| R152 5.2.1.2 制动需求 ≥ 5 m/s²（字面） | {len(o)} 点中 {o_fail} 点不满足 | 全部满足 |",
                 f"not the literal 5 m/s² braking-demand requirement ({o_fail} of {len(o)} points)")


def test_stress_corner_and_ncap_baseline():
    sens = rows(RES / "sensitivity_summary.csv")
    so = pick(sens, config="original", latency_s="0.2", tau_s="0.4")
    sp = pick(sens, config="proposed", latency_s="0.2", tau_s="0.4")
    must_contain(f"| R152 压力角（+0.2 s 时延，τ 0.4 s） | 6 个官方点中 {so['r152_impact_fail_points']} 个超限 | "
                 f"{sp['r152_impact_fail_points']} 个 |")
    no = SUMMARY["main"]["original"]["ncap_total"]
    np_ = SUMMARY["main"]["proposed"]["ncap_total"]
    for cfg, v in (("original", no), ("proposed", np_)):
        assert float(pick(sens, config=cfg, latency_s="0.0", tau_s="0.12")["ncap_total"]) == v
    must_contain(f"| Euro NCAP 简化估计（只含 50 % 撞击位置，满分 5.2） | {no:.2f} | {np_:.2f} |",
                 f"from {no:.2f} to {np_:.2f} (of 5.2)")


def test_late_contacts():
    lo = SUMMARY["main"]["original"]["late_contacts"]
    lp = SUMMARY["main"]["proposed"]["late_contacts"]
    first = rows(RES / "late_contact_whatif.csv")[0]
    assert first["variant"] == "proposed" and int(first["late_contact_runs"]) == lp["runs"]
    must_contain(f"| 规程结束（V_VUT = V_target）之后的二次接触 | {lo['runs']} | "
                 f"{lp['of_runs']} 次中 {lp['runs']} 次，最高 {lp['max_kph']:.1f} km/h |",
                 f"{lp['runs']} of {lp['of_runs']} low-speed contacts after the NCAP end condition "
                 f"(max {lp['max_kph']:.1f} km/h)")


def test_ghost_target():
    g = rows(RES / "sotif_ghost.csv")
    o = pick(g, config="original", road_mu="0.9", case_id="SOTIF-FREE-60", ghost_ttc_s="1.8", ghost_visible_s="0.05")
    p = pick(g, config="proposed", road_mu="0.9", case_id="SOTIF-FREE-60", ghost_ttc_s="1.8", ghost_visible_s="0.05")
    assert o["fcw"] == "True" and float(o["max_decel_request_mps2"]) == 0.0      # warning only
    must_contain(f"| 1 帧幻影目标（TTC 1.8 s） | 只报警 | {float(p['max_decel_request_mps2']):g} m/s² 误制动，"
                 f"速度损失 {float(p['speed_loss_kph']):.1f} km/h |",
                 f"into a {float(p['max_decel_request_mps2']):g} m/s² false brake")


def test_wet_road():
    w = rows(RES / "wet_summary.csv")
    o = float(pick(w, config="original", road_mu="0.5")["ncap_total"])
    p = float(pick(w, config="proposed", road_mu="0.5")["ncap_total"])
    fa = float(pick(w, config="friction_aware_0.5", road_mu="0.5")["ncap_total"])
    must_contain(f"| 湿路 μ = 0.5 的简化估计（模型内） | {o:.2f} | {p:.2f}（摩擦自适应参数 {fa:.2f}） |")


def test_rain_range_40m():
    r = rows(RES / "sotif_range.csv")
    o60 = float(pick(r, config="original", road_mu="0.9", max_range_m="40.0", case_id="R152-S60")["fcw_lead_min_s"])
    p60 = float(pick(r, config="proposed", road_mu="0.9", max_range_m="40.0", case_id="R152-S60")["fcw_lead_min_s"])
    p55 = float(pick(r, config="proposed", road_mu="0.9", max_range_m="40.0", case_id="R152-S55")["fcw_lead_min_s"])
    assert p60 < 0.8 and p55 < 0.8 <= o60
    must_contain(f"| 雨天量程 40 m 时的 R152 预警提前量 | S60 恰好 {o60:.2f} s | S60 / S55 不足（{p60:g} / {p55:g} s） |")


def test_conservative_alternative():
    c = pick(rows(RES / "param_search.csv"), name="pd5_pb1.6_fb0.9")
    assert c["r152_constraint_ok"] == "True" and int(c["late_contact_runs_baseline"]) == 0
    must_contain(f"没有二次接触（NCAP 估计 {float(c['ncap_est_baseline']):.2f}）")


def _param_search():
    spec = importlib.util.spec_from_file_location("param_search", ROOT / "scripts" / "param_search.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_param_selection_reproduces_proposed():
    conv = {"r152_constraint_ok": lambda s: s == "True", "ccrs_gate_ok": lambda s: s == "True", "name": str}
    rs = [{k: conv.get(k, float)(v) for k, v in r.items()} for r in rows(RES / "param_search.csv")]
    best = _param_search().select(rs)
    assert (best["ttc_fcw_s"], best["ttc_pb_s"], best["ttc_fb_s"], best["partial_decel_mps2"]) == \
        (PROPOSED.ttc_fcw_s, PROPOSED.ttc_pb_s, PROPOSED.ttc_fb_s, PROPOSED.partial_decel_mps2)
    feas = [r for r in rs if r["name"] != "original" and r["r152_constraint_ok"] and r["ccrs_gate_ok"]]
    tied = [r for r in feas if r["objective"] == best["objective"]]
    must_contain(f"目标值在 {len(tied)} 组候选上并列 {best['objective']:.2f}")


def test_backend_crosscheck_and_dt():
    assert SUMMARY["backend"] == "esmini"
    c = SUMMARY["crosscheck_esmini_vs_python"]
    assert c["runs"] == c["same_state_sequence"] == c["same_contact"] == 490
    assert c["max_abs_dv_impact_kph"] < 1e-6
    assert c["runs_with_min_gap_diff_gt_1mm"] == ["R152-S45@phase0.00"]
    d = SUMMARY["dt_check"]
    must_contain(f"在 {c['runs']} 次运行中状态序列、是否碰撞、碰撞速度（差 < 1e-6 km/h）都一致",
                 f"最小车距在 R152-S45 一处差 {c['max_abs_dmin_gap_m']:.3f} m",
                 f"最差碰撞速度最多差 {d['max_abs_dv_worst_kph']:g} km/h")


def test_signal_level_table():
    e1 = rows(SL / "e1_equivalence_summary.csv")
    tr = [r for r in e1 if r["mode"] == "transparent"]
    n_tr, id_tr = sum(int(r["runs"]) for r in tr), sum(int(r["identical_runs"]) for r in tr)
    dbc = [r for r in e1 if r["mode"] == "dbc" and r["protocol"] in ("R152", "NCAP")]
    diffs = sum(int(r["runs"]) - int(r["identical_runs"]) for r in dbc)
    r152_flips = int(pick(e1, mode="dbc", protocol="R152")["r152_verdict_flips"])
    assert len(rows(SL / "e1_quantization_explained.csv")) == diffs
    must_contain(f"透明模式 {id_tr}/{n_tr} 完全一致；DBC 量化后 R152 判定 {r152_flips} 翻转，{diffs} 处差异都能用量化解释")

    e3 = rows(SL / "e3_burst_summary.csv")
    u = [r for r in e3 if r["variant"] == "U" and r["config"] == "proposed" and r["duration_s"] == "0.1"]
    rr = [r for r in e3 if r["variant"] == "R" and r["config"] == "proposed" and r["duration_s"] == "0.1"]
    fail, runs = sum(int(r["impact_fail_runs"]) for r in u), sum(int(r["runs"]) for r in u)
    worst = max(float(r["worst_impact_kph"]) for r in u)
    assert sum(int(r["impact_fail_runs"]) for r in rr) == 0
    must_contain(f"新参数 {runs} 个组合中 {fail} 个超限（最差 {worst:.1f} km/h，限值 35）",
                 f"pushes {fail} of {runs} runs over the limit")

    e5 = rows(SL / "e5_corruption_summary.csv")
    no = sum(int(r["early_brake_runs"]) for r in e5 if r["variant"] == "noE2E")
    yes = sum(int(r["early_brake_runs"]) for r in e5 if r["variant"] == "E2E")
    must_contain(f"无 E2E：{no} 次误制动 / 提前制动；有 E2E：{yes} 次")

    e4 = rows(SL / "e4_latency_summary.csv")
    lat = [r for r in e4 if r["jitter_s"] == "0.0" and r["latency_s"] in ("0.01", "0.02", "0.05")]
    dts = sorted({round(float(r["max_dt_brake_s"]) * 1000) for r in lat})
    assert all(r["impact_fail_points"] == "0" for r in lat)
    must_contain("附加时延 10 / 20 / 50 ms | 制动起点推迟 " + " / ".join(str(x) for x in dts) + " ms，两套参数都不超限")


@pytest.mark.parametrize("fig", ["results/plots/fig1_r152_impact.png"])
def test_readme_figures_exist(fig):
    assert fig in README and (ROOT / fig).is_file()
