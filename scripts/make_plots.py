"""Draw every figure in results/plots/ from the CSV results (no simulation here).

Run:  python -I scripts/make_plots.py
Palette: reference categorical slots 1-3 (validated with the dataviz validator, light surface): blue = original
values, orange = proposed values, aqua = friction-aware values; marker shape is a second identity channel.
Euro NCAP grid colours are the protocol's own colour bands (Figure 5-1), each cell is also labelled.
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import to_rgb  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RES, OUT = ROOT / "results", ROOT / "results" / "plots"
SURF, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
C = {"original": "#2a78d6", "proposed": "#eb6834", "friction_aware": "#1baf7a"}
M = {"original": "o", "proposed": "s", "friction_aware": "^"}
NAME = {"original": "原参数（上游默认）", "proposed": "新参数（干路调参）", "friction_aware": "摩擦自适应参数"}
NCAP_RGB = {"green": "#00b050", "yellow": "#ffff00", "orange": "#ffc000", "brown": "#7f6a00", "red": "#ff0000"}
ZH = {"green": "绿", "yellow": "黄", "orange": "橙", "brown": "棕", "red": "红"}
STATE = ["OFF", "STANDBY", "FCW", "PARTIAL", "FULL", "FAULT"]

plt.rcParams.update({
    "font.family": ["Microsoft YaHei", "Segoe UI", "DejaVu Sans"], "axes.unicode_minus": False,
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": AXIS, "axes.linewidth": 0.8, "axes.labelcolor": INK2, "axes.titlecolor": INK,
    "axes.titlesize": 11, "axes.titleweight": "bold", "axes.labelsize": 9.5,
    "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelsize": 8.5, "ytick.labelsize": 8.5,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "grid.linestyle": "-",
    "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False, "legend.fontsize": 8.5,
    "lines.linewidth": 2.0, "lines.solid_capstyle": "round", "lines.solid_joinstyle": "round",
})


def load(name):
    return list(csv.DictReader(open(RES / name, encoding="utf-8")))


def series(name, cfg):
    rows = list(csv.DictReader(open(RES / "timeseries" / cfg / f"{name}.csv", encoding="utf-8")))
    return {k: [float(r[k]) if r[k] != "" else float("nan") for r in rows] for k in rows[0]}


def line(ax, x, y, key, label=None, **kw):
    ax.plot(x, y, color=C[key], marker=M[key], markersize=7, markeredgecolor=SURF, markeredgewidth=1.6,
            label=label or NAME[key], **kw)


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / name, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print("wrote results/plots/" + name)


def r152_limit_steps(vmax=60):
    import aebtest.regulations as R
    xs, ys = [], []
    prev = 0
    for v, (mm, ro) in sorted(R.R152_M1_MAX_REL_IMPACT_KPH.items()):
        xs += [prev, v]
        ys += [ro, ro]
        prev = v
    return xs, ys


# ------------------------------------------------------------------------------------------ fig 1
def fig1():
    r152, sens = load("r152_results.csv"), load("sensitivity.csv")
    xs, ys = r152_limit_steps()
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 3.9), sharey=True)
    for ax, title in zip(axes, ("基线：无附加时延，制动 τ = 0.12 s", "压力角：附加时延 0.2 s，制动 τ = 0.40 s")):
        ax.plot(xs, ys, color=INK, linewidth=1.2, drawstyle="default", label="R152 允许值（M1，整备质量）")
        for cfg in ("original", "proposed"):
            if "基线" in title:
                pts = sorted((float(r["v_ego_kph"]), float(r["v_rel_impact_worst_kph"])) for r in r152
                             if r["config"] == cfg and r["case_id"].startswith("R152-S"))
            else:
                pts = sorted((float(r["case_id"].split("-S")[1]), float(r["v_rel_impact_worst_kph"])) for r in sens
                             if r["config"] == cfg and r["latency_s"] == "0.2" and r["tau_s"] == "0.4"
                             and r["case_id"].startswith("R152-S"))
            line(ax, [p[0] for p in pts], [p[1] for p in pts], cfg,
                 linestyle="-" if "基线" in title else "none")
            for v, imp in pts:
                if cfg == "original" and imp > 0:
                    ax.annotate(f"{imp:.1f}", (v, imp), textcoords="offset points", xytext=(-14, 7), fontsize=8, color=INK2)
        ax.set_title(title, loc="left")
        ax.set_xlabel("自车车速（静止目标），km/h")
        ax.set_xlim(8, 62)
    axes[0].set_ylabel("最差相对碰撞速度，km/h")
    axes[1].text(10, 24, "只跑了 4 个官方速度点（20/40/42/60）\n原参数在 40、42 km/h 超出“不允许碰撞”，\n60 km/h 超出 35 km/h 限值",
                 fontsize=8.5, color=INK2)
    axes[0].legend(loc="upper left")
    fig.suptitle("图 1  UN R152 静止目标：碰撞速度与允许值", x=0.01, ha="left", fontsize=12, color=INK, fontweight="bold")
    save(fig, "fig1_r152_impact.png")


# ------------------------------------------------------------------------------------------ fig 2 / 6 grids
def ncap_grid(ax, rows, title):
    fams, speeds = ["CCRs", "CCRm", "CCRb"], list(range(10, 131, 10))
    for i, fam in enumerate(fams):
        for r in rows:
            if r["family"] != fam:
                continue
            j = speeds.index(int(float(r["v_ego_kph"])))
            col = r.get("colour") or r.get("verdict")
            v = float(r["v_rel_impact_worst_kph"])
            ext = r.get("range_50pct", "standard") == "extended" or (fam == "CCRb" and float(r["v_ego_kph"]) >= 90)
            ax.add_patch(plt.Rectangle((j + 0.04, i + 0.04), 0.92, 0.92, facecolor=NCAP_RGB[col], edgecolor=SURF, lw=2))
            rgb = to_rgb(NCAP_RGB[col])
            txt = "white" if (0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2]) < 0.5 else INK
            ax.text(j + 0.5, i + 0.5, ("0" if v == 0 else f"{v:.0f}") + ("*" if ext else ""), ha="center", va="center",
                    fontsize=8, color=txt)
    ax.set_xlim(0, len(speeds))
    ax.set_ylim(len(fams), 0)
    ax.set_xticks([k + 0.5 for k in range(len(speeds))], [str(s) for s in speeds])
    ax.set_yticks([k + 0.5 for k in range(len(fams))], fams)
    ax.grid(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_visible(False)
    ax.tick_params(length=0)
    ax.set_title(title, loc="left")


def colour_legend(fig, y=0.0):
    hs = [plt.Rectangle((0, 0), 1, 1, color=NCAP_RGB[c]) for c in NCAP_RGB]
    hs.append(plt.Rectangle((0, 0), 1, 1, facecolor=SURF, edgecolor=SURF))
    fig.legend(hs, [f"{ZH[c]}" for c in NCAP_RGB] + ["* = 扩展范围（其余为标准范围）"], loc="lower center", ncol=6,
               bbox_to_anchor=(0.5, y), fontsize=8.5)


def fig2():
    rows = load("ncap_results.csv")
    fig, axes = plt.subplots(2, 1, figsize=(10.5, 4.6))
    ncap_grid(axes[0], [r for r in rows if r["config"] == "original"], "原参数（上游默认）")
    ncap_grid(axes[1], [r for r in rows if r["config"] == "proposed"], "新参数（TTC 3.0 / 2.0 / 1.3 s，部分制动 6 m/s²）")
    axes[1].set_xlabel("VUT 车速，km/h（格内数字 = 5 个采样相位中最差的相对碰撞速度，km/h）")
    fig.suptitle("图 2  Euro NCAP CCR 网格（50 % 撞击位置，基线条件）", x=0.01, ha="left", fontsize=12, color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0.07, 1, 0.97))
    colour_legend(fig)
    save(fig, "fig2_ncap_grid.png")


def fig6():
    wet = load("wet_results.csv")
    fams = {"CCRs": "CCRs", "CCRm": "CCRm", "CCRb": "CCRb"}
    fig, axes = plt.subplots(2, 3, figsize=(11.5, 6.2), sharey=True)
    for i, mu in enumerate(("0.7", "0.5")):
        for j, fam in enumerate(fams):
            ax = axes[i, j]
            for key, cfg in (("original", "original"), ("proposed", "proposed"), ("friction_aware", f"friction_aware_{mu}")):
                pts = sorted((float(r["case_id"].rsplit("-", 1)[1]), float(r["v_rel_impact_worst_kph"])) for r in wet
                             if r["config"] == cfg and r["road_mu"] == mu and r["case_id"].startswith(f"NCAP-{fam}-"))
                line(ax, [p[0] for p in pts], [p[1] for p in pts], key)
            ax.set_title(f"μ = {mu}  ·  {fam}", loc="left")
            if i == 1:
                ax.set_xlabel("VUT 车速，km/h")
        axes[i, 0].set_ylabel("最差相对碰撞速度，km/h")
    axes[0, 0].legend(loc="upper left")
    fig.suptitle("图 6  湿滑路面（模型内）：干路调参 vs 摩擦自适应参数，Euro NCAP 网格", x=0.01, ha="left",
                 fontsize=12, color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    save(fig, "fig6_wet_before_after.png")


# ------------------------------------------------------------------------------------------ fig 3 / 4 time series
def ts_panels(case, title, fname, annotate_late=False, xlabel="仿真时间，s（T0 = TTC 4 s，在 2 s 处）"):
    fig, axes = plt.subplots(4, 1, figsize=(10.5, 8.2), sharex=True, gridspec_kw={"height_ratios": [1.2, 1, 0.8, 1]})
    for cfg in ("original", "proposed"):
        s = series(case, cfg)
        t = s["t"]
        axes[0].plot(t, [v * 3.6 for v in s["v_ego"]], color=C[cfg], label=f"自车 · {NAME[cfg]}")
        axes[1].plot(t, s["gap"], color=C[cfg], label=NAME[cfg])
        axes[2].step(t, s["state"], where="post", color=C[cfg], label=NAME[cfg])
        axes[3].plot(t, s["a_act"], color=C[cfg], label=f"实际减速度 · {NAME[cfg]}")
        axes[3].plot(t, s["a_req"], color=C[cfg], linewidth=1.0, alpha=0.6, label=f"请求 · {NAME[cfg]}")
        if annotate_late and cfg == "proposed":
            ve, vt = s["v_ego"], s["v_tgt"]
            k_end = next((k for k in range(1, len(t)) if t[k] > 3.01 and ve[k] <= vt[k] + 1e-9), None)
            if k_end:
                for ax in axes:
                    ax.axvline(t[k_end], color=MUTED, linewidth=1.0)
                axes[0].annotate("V_VUT = V_target\nEuro NCAP 试验到此结束（判绿）", (t[k_end], ve[k_end] * 3.6),
                                 xytext=(10, 25), textcoords="offset points", fontsize=8.5, color=INK2)
            axes[1].annotate("目标停稳后本车以低速\n蹭上（二次接触）", (t[-1], 0), xytext=(-150, 25),
                             textcoords="offset points", fontsize=8.5, color=INK2,
                             arrowprops={"arrowstyle": "-", "color": MUTED})
    s = max((series(case, c) for c in ("original", "proposed")), key=lambda d: len(d["t"]))
    axes[0].plot(s["t"], [v * 3.6 for v in s["v_tgt"]], color=INK, linewidth=1.2, label="目标车")
    axes[0].set_ylabel("车速，km/h")
    axes[1].set_ylabel("间距，m")
    axes[1].set_ylim(bottom=0)
    axes[2].set_yticks(range(6), STATE)
    axes[2].set_ylim(-0.4, 4.6)
    axes[2].set_ylabel("AEB 状态")
    axes[3].set_ylabel("减速度，m/s²")
    axes[3].set_xlabel(xlabel)
    axes[0].legend(loc="upper right", ncol=1)
    axes[3].legend(loc="upper left", ncol=2)
    axes[0].set_title(title, loc="left")
    save(fig, fname)


# ------------------------------------------------------------------------------------------ fig 5 sensitivity
def fig5():
    rows = load("sensitivity_summary.csv")
    lats, taus = ["0", "0.1", "0.2"], ["0.12", "0.25", "0.4"]
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 3.9))
    ramp = matplotlib.colors.LinearSegmentedColormap.from_list("blue", ["#cde2fb", "#2a78d6", "#0d366b"])
    for ax, cfg in zip(axes, ("original", "proposed")):
        grid = [[0.0] * 3 for _ in range(3)]
        fails = [[0] * 3 for _ in range(3)]
        for r in rows:
            if r["config"] == cfg:
                i, j = lats.index(f"{float(r['latency_s']):g}"), taus.index(f"{float(r['tau_s']):g}")
                grid[i][j] = float(r["ncap_total"])
                fails[i][j] = int(r["r152_impact_fail_points"])
        ax.imshow(grid, cmap=ramp, vmin=0, vmax=5.2)
        for i in range(3):
            for j in range(3):
                txt = "white" if grid[i][j] > 3.2 else INK
                ax.text(j, i - 0.12, f"{grid[i][j]:.2f}", ha="center", va="center", fontsize=11, color=txt, fontweight="bold")
                ax.text(j, i + 0.22, f"R152 不通过 {fails[i][j]}/6", ha="center", va="center", fontsize=8, color=txt)
        ax.set_xticks(range(3), [f"τ {t} s" for t in taus])
        ax.set_yticks(range(3), [f"时延 {l} s" for l in lats])
        ax.grid(False)
        ax.set_title(NAME[cfg], loc="left")
    fig.suptitle("图 5  敏感性：NCAP 简化估计（满分 5.2）与 R152 官方点碰撞速度不通过数", x=0.01, ha="left",
                 fontsize=12, color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    save(fig, "fig5_sensitivity.png")


# ------------------------------------------------------------------------------------------ fig 7 SOTIF
def fig7():
    gh, rng = load("sotif_ghost.csv"), load("sotif_range.csv")
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.0))
    ax = axes[0]
    sets = [("original", "original", "0.9"), ("proposed", "proposed", "0.9"), ("friction_aware", "friction_aware_0.5", "0.9")]
    ttcs = ["2.8", "1.8", "1.2"]
    w = 0.26
    for k, (key, cfg, mu) in enumerate(sets):
        vals = [float(next(r["speed_loss_kph"] for r in gh if r["config"] == cfg and r["road_mu"] == mu
                           and r["case_id"] == "SOTIF-FREE-100" and r["ghost_ttc_s"] == t and r["ghost_visible_s"] == "0.05"))
                for t in ttcs]
        xs = [i + (k - 1) * w for i in range(3)]
        ax.bar(xs, vals, width=w * 0.9, color=C[key], label=NAME[key], edgecolor=SURF, linewidth=2)
        for x, v in zip(xs, vals):
            ax.text(x, max(v, 0) + 0.3, f"{(v if v > 0.05 else 0):.0f}", ha="center", fontsize=8, color=INK2)
    ax.set_xticks(range(3), [f"幻影 TTC {t} s" for t in ttcs])
    ax.set_ylabel("误制动造成的速度损失，km/h")
    ax.set_title("(a) 只出现 1 帧（50 ms）的幻影静止目标，100 km/h，干路", loc="left")
    ax.legend(loc="upper left", labels=[NAME["original"], NAME["proposed"], NAME["friction_aware"] + "（μ0.5 档）"])
    ax = axes[1]
    ranges = ["200", "60", "40"]
    groups = (("original", "original", "0.9", "（干路）"), ("proposed", "proposed", "0.9", "（干路）"),
              ("friction_aware", "friction_aware_0.5", "0.5", "（μ0.5 路面）"))
    for k, (key, cfg, mu, tag) in enumerate(groups):
        for i, R in enumerate(ranges):
            hit = [r for r in rng if r["config"] == cfg and r["road_mu"] == mu and r["case_id"] == "R152-S60"
                   and f"{float(r['max_range_m']):g}" == R]
            if not hit:
                continue
            v = float(hit[0]["fcw_lead_min_s"])
            x = i + (k - 1) * w
            ax.bar(x, v, width=w * 0.9, color=C[key], edgecolor=SURF, linewidth=2,
                   label=NAME[key] + tag if i == 0 else None)
            ax.text(x, v + 0.02, f"{v:.1f}", ha="center", fontsize=8, color=INK2)
    ax.axhline(0.8, color=INK, linewidth=1.2, label="R152 5.2.1.1 要求 ≥ 0.8 s")
    ax.set_xticks(range(3), [f"探测距离 {R} m" for R in ranges])
    ax.set_ylim(0, 1.45)
    ax.set_ylabel("R152-S60 最小 FCW 提前量，s")
    ax.set_title("(b) 雨天探测距离缩短（假设值）→ 预警提前量，60 km/h 静止目标", loc="left")
    ax.legend(loc="upper right", fontsize=8)
    fig.suptitle("图 7  SOTIF 触发条件：阈值提前的两个代价", x=0.01, ha="left", fontsize=12, color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    save(fig, "fig7_sotif_tradeoffs.png")


if __name__ == "__main__":
    sys.path.insert(0, str(ROOT))
    fig1()
    fig2()
    ts_panels("NCAP-CCRs-80", "图 3  NCAP-CCRs-80（静止目标 80 km/h）：原参数 39 km/h 相撞，新参数在 4.8 m 外停住",
              "fig3_timeseries_ccrs80.png")
    ts_panels("NCAP-CCRb-60", "图 4  NCAP-CCRb-60：新参数在 NCAP 判定中为绿，但松开制动后又碰到已停下的目标",
              "fig4_ccrb60_late_contact.png", annotate_late=True,
              xlabel="仿真时间，s（目标 3 s 开始以 4 m/s² 减速；T0 = 减速开始前 1 s = 2 s）")
    fig5()
    fig6()
    fig7()
