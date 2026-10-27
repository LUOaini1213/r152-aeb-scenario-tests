"""Figures for the signal-level study (reads results/*.csv written by run_signal_level.py).

  fig_sl1_equivalence.png   fault-free bus vs P5 direct loop; why quantization moves a threshold by one frame
  fig_sl2_bus_faults.png    frame loss, burst loss at the critical moment, added latency
  fig_sl3_timeline_e2e.png  200 ms burst timeline (baseline vs mitigation) and one corrupted RADAR_TGT frame

Palette: reference categorical slots 1-3 (#2a78d6, #eb6834, #1baf7a), validated with the dataviz
validator (light surface #fcfcfb); slot 3 is below 3:1 contrast, so every series also has a legend /
direct label and every value is in the CSV tables.
Run:  python -B -I signal_level/make_figures.py
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402

sys.dont_write_bytecode = True
RES = Path(__file__).resolve().parent / "results"

C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED, GRID, AXIS, SURF = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
NEUTRAL = "#e9e8e3"
plt.rcParams.update({
    "font.family": "sans-serif", "font.sans-serif": ["Segoe UI", "DejaVu Sans", "Arial"], "font.size": 9,
    "axes.edgecolor": AXIS, "axes.linewidth": 0.8, "axes.labelcolor": INK2, "axes.titlesize": 10,
    "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.grid": True, "grid.color": GRID,
    "grid.linewidth": 0.6, "grid.linestyle": "-", "xtick.color": MUTED, "ytick.color": MUTED,
    "xtick.labelcolor": INK2, "ytick.labelcolor": INK2, "figure.facecolor": SURF, "axes.facecolor": SURF,
    "savefig.facecolor": SURF, "legend.frameon": False, "legend.fontsize": 8, "lines.linewidth": 2.0,
    "axes.spines.top": False, "axes.spines.right": False})


def load(name):
    with open(RES / name, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def fnum(x):
    return None if x in ("", "None", None) else float(x)


def truth(x):
    return x == "True"


def note(fig, text):
    fig.text(0.01, 0.005, text, color=MUTED, fontsize=7, ha="left", va="bottom")


# ================================================================================ figure 1
def fig1():
    runs = load("e1_equivalence_runs.csv")
    allx = load("e1_quantization_explained.csv")
    expl = [e for e in allx if e.get("threshold_s") not in ("", None) and e.get("ttc_dbc_quantized") not in ("", None)
            and e.get("ttc_direct_float") not in ("", None)]
    n_release = len(allx) - len(expl)
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.4), gridspec_kw={"width_ratios": [1, 1.35]})
    n = len(runs)

    def category(r):
        if truth(r["dbc_identical"]):
            return 0
        shift = fnum(r["dbc_dt_fcw_s"]) or fnum(r["dbc_dt_brake_s"]) or 0.0
        return 1 if shift > 1e-9 else (2 if shift < -1e-9 else 3)

    cats = ["identical to P5", "warning/brake 1 radar frame (50 ms) later",
            "warning/brake 1 radar frame (50 ms) earlier", "a later transition shifted by 1 frame"]
    trans_ident = sum(truth(r["transparent_identical"]) for r in runs)
    counts = [sum(category(r) == k for r in runs) for k in range(len(cats))]
    y = list(range(len(cats)))[::-1]
    a.barh(y, counts, color=[C1, C2, C2, C3], height=0.55)
    for yy, c in zip(y, counts):
        a.text(c + n * 0.01, yy, f"{c}", va="center", color=INK, fontsize=9)
    a.set_yticks(y, cats)
    a.set_xlim(0, n * 1.6)
    a.set_xlabel(f"runs (of {n}: 49 points x 5 radar phases x 2 parameter sets)")
    r152 = [r for r in runs if r["case_id"].startswith("R152")]
    ncap = [r for r in runs if r["case_id"].startswith("NCAP")]
    flips = sum(truth(r.get("r152_verdict_flip", "False")) for r in r152)
    cflips = sum(truth(r.get("ncap_colour_flip", "False")) for r in ncap)
    r152_same_v = sum(abs(fnum(r["dbc_dv_kph"])) < 1e-9 for r in r152)
    dv_ncap = max(abs(fnum(r["dbc_dv_kph"])) for r in ncap)
    a.set_title("DBC-quantized bus vs P5 direct loop (fault-free)")
    a.text(0.98, 0.40, f"transparent bus (no quantization):\n  {trans_ident}/{n} runs identical\n"
                       f"DBC bus, R152 ({len(r152)} runs):\n  impact speed unchanged {r152_same_v}/{len(r152)}\n"
                       f"  verdict changes {flips}\n"
                       f"DBC bus, Euro NCAP ({len(ncap)} runs):\n  max |impact change| {dv_ncap:.2f} km/h\n"
                       f"  colour changes {cflips}",
           transform=a.transAxes, ha="right", va="center", color=INK2, fontsize=7.5, linespacing=1.3)
    a.grid(axis="y", visible=False)
    # panel b: at the first diverging ECU task the threshold lies between the float and the quantized TTC
    groups = (("FCW", "warning threshold", C1), ("PARTIAL_BRAKE", "partial-brake threshold", C2),
              ("FULL_BRAKE", "full-brake threshold", C3))
    order = {"OFF": 0, "STANDBY": 1, "FCW": 2, "PARTIAL_BRAKE": 3, "FULL_BRAKE": 4, "FAULT": 5}
    lim = 32.0                                            # ms; points beyond are clipped to the frame and counted
    clipped = 0
    for st, lab, col in groups:
        pts = [((float(e["ttc_direct_float"]) - float(e["threshold_s"])) * 1000,
                (float(e["ttc_dbc_quantized"]) - float(e["threshold_s"])) * 1000) for e in expl
               if max(e["state_direct"], e["state_dbc"], key=order.get) == st]
        if pts:
            clipped += sum(abs(x) > lim or abs(yv) > lim for x, yv in pts)
            xs = [max(-lim, min(lim, x)) for x, _ in pts]
            ys = [max(-lim, min(lim, yv)) for _, yv in pts]
            b.scatter(xs, ys, s=30, color=col, edgecolor=SURF, linewidth=1.2, label=f"{lab} ({len(pts)})", zorder=3,
                      clip_on=False)
    b.axhline(0, color=INK2, lw=1.0)
    b.axvline(0, color=INK2, lw=1.0)
    b.plot([-lim, lim], [-lim, lim], color=AXIS, lw=1.0, ls="--", zorder=1)
    b.set_xlim(-lim, lim)
    b.set_ylim(-lim, lim)
    b.text(-lim * 0.95, lim * 0.95, "float: below threshold (acts now)\nquantized: not yet (one frame later)",
           color=INK2, fontsize=7, va="top")
    b.text(lim * 0.95, -lim * 0.95, "float: not yet\nquantized: below threshold (acts now)", color=INK2, fontsize=7,
           ha="right", va="bottom")
    b.text(lim * 0.95, lim * 0.95, f"+ {n_release} runs: brake RELEASE one frame\nearlier/later (closing speed crosses\n"
                                   f"0.1 m/s = 2 LSB of TgtRelSpd)" + (f"\n{clipped} point(s) beyond +-{lim:.0f} ms "
                                                                      "drawn on the frame" if clipped else ""),
           color=INK2, fontsize=7, ha="right", va="top")
    b.set_ylabel("TTC from DBC-decoded signals minus threshold [ms]")
    b.set_xlabel("TTC from float signals (P5 direct) minus threshold [ms]")
    b.set_title(f"Why: float and quantized TTC straddle the threshold ({len(expl)} runs)")
    b.legend(loc="lower left")
    note(fig, "SYNTHETIC simulation. TgtDist 0.05 m, TgtRelSpd 0.05 m/s (design assumptions). "
              "Source: results/e1_equivalence_runs.csv, e1_quantization_explained.csv")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(RES / "fig_sl1_equivalence.png", dpi=150)
    plt.close(fig)


# ================================================================================ figure 2
def fig2():
    fig, (a, b, c) = plt.subplots(1, 3, figsize=(13.5, 4.3))
    # (a) frame loss
    loss = load("e2_loss_summary.csv")
    for cfg, col in (("original", C1), ("proposed", C2)):
        for var, ls, mk in (("U", "-", "o"), ("HR", "--", "s")):
            sel = sorted((float(r["loss"]) * 100, float(r["impact_fail_share"]) * 100) for r in loss
                         if r["config"] == cfg and r["variant"] == var)
            xs, ys = zip(*sel)
            a.plot(xs, ys, ls=ls, color=col, marker=mk, ms=5, mec=SURF, mew=1.2,
                   label=f"{cfg} thresholds, {'baseline' if var == 'U' else 'hold + 0.2 s heal'}")
            if var == "U":
                a.text(xs[-1] + 0.4, ys[-1], f"{ys[-1]:.0f}%", color=INK2, fontsize=8, va="center")
    a.set_xlabel("i.i.d. frame loss on every message [%]")
    a.set_ylabel("runs exceeding the R152 5.2.1.4 impact limit [%]")
    a.set_title("Random frame loss (6 official points x 5 phases x 20 seeds)")
    a.set_xlim(-0.5, 23.5)
    a.legend(loc="upper left")
    # (b) bursts at the critical moment, proposed thresholds
    burst = load("e3_burst_summary.csv")
    for dur, col in (("0.1", C1), ("0.2", C2), ("0.5", C3)):
        for var, ls, mk in (("U", "-", "o"), ("HR", "--", "s")):
            sel = sorted((float(r["offset_s"]), int(r["impact_fail_runs"])) for r in burst
                         if r["config"] == "proposed" and r["variant"] == var and r["duration_s"] == dur)
            xs, ys = zip(*sel)
            b.plot(xs, ys, ls=ls, color=col, marker=mk, ms=4, mec=SURF, mew=1.0,
                   label=f"{int(float(dur) * 1000)} ms, {'baseline' if var == 'U' else 'hold + 0.2 s heal'}")
    b.axvline(0, color=INK2, lw=1.0, ls="--")
    b.text(-0.02, 30.5, "fault-free brake onset", color=INK2, fontsize=8, va="top", ha="right")
    b.set_xlabel("burst start minus fault-free brake onset [s]")
    b.set_ylabel("(point, phase) combinations failing the impact limit (of 30)")
    b.set_title("Bus-wide burst at the critical moment (proposed thresholds)")
    b.set_ylim(-1, 31)
    b.legend(loc="lower left", bbox_to_anchor=(0.0, 0.07), fontsize=7)
    b.text(0.02, 0.98, "all 'hold + 0.2 s heal'\nlines stay at 0", transform=b.transAxes, ha="left",
           va="top", color=INK2, fontsize=7)
    # (c) latency
    lat = load("e4_latency_summary.csv")
    for cfg, col in (("original", C1), ("proposed", C2)):
        sel = sorted((float(r["latency_s"]) * 1000, float(r["worst_impact_kph"])) for r in lat
                     if r["config"] == cfg and float(r["jitter_s"]) == 0.0)
        xs, ys = zip(*sel)
        c.plot(xs, ys, color=col, marker="o", ms=5, mec=SURF, mew=1.2, label=f"{cfg} thresholds")
        c.text(xs[-1] + 1.5, ys[-1], f"{ys[-1]:.1f}", color=INK2, fontsize=8, va="center")
    c.axhline(35.0, color=INK2, lw=1.0, ls="--")
    c.text(1, 35.6, "R152 limit at 60 km/h (35 km/h)", color=INK2, fontsize=8)
    c.set_xlabel("added latency on every frame [ms] (radar -> AEB and AEB -> brake)")
    c.set_ylabel("worst relative impact speed over 19 points x 5 phases [km/h]")
    c.set_title("Added bus latency")
    c.set_xlim(-2, 58)
    c.set_ylim(-1, 40)
    c.legend(loc="center left")
    note(fig, "SYNTHETIC simulation, P5 toy vehicle model. Baseline = upstream FAULT reaction (brake released, 1.0 s heal); "
              "'hold + 0.2 s heal' = mitigation variant HR. Source: results/e2_*, e3_*, e4_*.csv")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(RES / "fig_sl2_bus_faults.png", dpi=150)
    plt.close(fig)


# ================================================================================ figure 3
def fig3():
    fig = plt.figure(figsize=(13, 8.2))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1], height_ratios=[1, 1], hspace=0.55, wspace=0.22)
    a, b = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[1, 0])
    series = {"fault-free": load("e6_timeline_S60_faultfree.csv"), "200 ms burst, baseline": load(
        "e6_timeline_S60_burst200_U.csv"), "200 ms burst, hold + 0.2 s heal": load("e6_timeline_S60_burst200_HR.csv")}
    import json
    summ = json.loads((RES / "summary_signal_level.json").read_text(encoding="utf-8"))["e6"]
    t0 = float(summ["timeline_U"]["burst_start"])
    for (name, rows), col in zip(series.items(), (MUTED, C2, C1)):
        t = [float(r["t"]) for r in rows]
        a.plot(t, [float(r["v_ego"]) * 3.6 for r in rows], color=col, label=name, lw=2.0 if col != MUTED else 1.6)
        b.plot(t, [float(r["a_req"]) for r in rows], color=col, label=name, lw=2.0 if col != MUTED else 1.6)
    for ax in (a, b):
        ax.axvspan(t0, t0 + 0.2, color=NEUTRAL, zorder=0)
        ax.set_xlim(t0 - 1.2, t0 + 2.6)
    rows_u, rows_h = series["200 ms burst, baseline"], series["200 ms burst, hold + 0.2 s heal"]
    tel_u = [float(r["t"]) for r in rows_u if r["telltale"] == "1"]
    tel_h = [float(r["t"]) for r in rows_h if r["telltale"] == "1"]
    if tel_u and tel_h:
        a.annotate(f"telltale on {tel_u[0]:.2f}-{tel_u[-1]:.2f} s (baseline: 1.0 s heal)\n"
                   f"telltale on {tel_h[0]:.2f}-{tel_h[-1]:.2f} s (0.2 s heal)",
                   xy=(tel_u[0], 60), xytext=(t0 - 1.15, 40), color=INK2, fontsize=8,
                   arrowprops=dict(arrowstyle="-", color=AXIS))
    a.text(t0 + 0.1, 62.5, "burst", color=INK2, fontsize=8, ha="center")
    a.set_ylim(-2, 66)
    a.set_ylabel("ego speed [km/h]")
    a.set_title(f"R152-S60, proposed: 200 ms bus burst at the brake onset (impact: baseline "
                f"{summ['timeline_U']['impact_kph']:.1f} km/h, hold + 0.2 s heal {summ['timeline_HR']['impact_kph']:.1f} km/h)",
                fontsize=9)
    a.legend(loc="lower left")
    b.set_ylabel("deceleration executed by the brake ECU [m/s2]")
    b.set_xlabel("time [s]")
    # (c) heatmap: one corrupted TgtDist bit, E2E off, R152-M60 proposed
    runs = load("e5_corruption_runs.csv")
    c = fig.add_subplot(gs[0, 1])
    sel = [r for r in runs if r["variant"] == "noE2E" and r["signal"] == "TgtDist" and r["case_id"] == "R152-M60"
           and r["config"] == "proposed"]
    times = sorted({float(r["t_inj"]) for r in sel})
    code = {"no_effect": 0, "early_warning_only": 1, "early_brake": 2, "late_or_missed_brake": 3}
    grid = [[0] * len(times) for _ in range(13)]
    for r in sel:
        grid[int(r["raw_bit"])][times.index(float(r["t_inj"]))] = code[r["effect"]]
    cmap = ListedColormap([NEUTRAL, C3, C1, C2])
    c.imshow(grid, aspect="auto", origin="lower", cmap=cmap, vmin=-0.5, vmax=3.5, interpolation="nearest",
             extent=(times[0] - 0.05, times[-1] + 0.05, -0.5, 12.5))
    c.set_yticks(range(13), [f"{k} ({0.05 * 2 ** k:g} m)" for k in range(13)], fontsize=7)
    c.set_xlabel("time of the corrupted RADAR_TGT frame [s]")
    c.set_ylabel("flipped TgtDist raw bit (weight)")
    c.grid(False)
    ref = summ.get("ref_t_brake")
    if ref:
        c.axvline(float(ref), color=INK, lw=1.0, ls="--")
        c.text(float(ref) + 0.03, 12.2, "fault-free\nbrake onset", color=INK, fontsize=7, va="top")
    handles = [plt.Rectangle((0, 0), 1, 1, color=col) for col in (C1, C2, C3, NEUTRAL)]
    c.legend(handles, ["phantom / early braking", "late / missed braking", "early warning only", "no effect"],
             loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=4, fontsize=7)
    c.set_title("One flipped TgtDist bit, WITHOUT E2E (R152-M60, proposed)")
    # (d) phantom braking per signal, E2E off vs on, all cases/configs
    d = fig.add_subplot(gs[1, 1])
    sigs = ("TgtValid", "TgtQuality", "TgtDist", "TgtRelSpd")
    agg = defaultdict(lambda: [0, 0, 0])
    late = defaultdict(list)
    for r in runs:
        k = (r["variant"], r["signal"])
        agg[k][0] += r["effect"] == "early_brake"
        agg[k][2] += 1
        if r["effect"] == "late_or_missed_brake" and r["variant"] in ("E2E", "noE2E"):
            late[r["variant"]].append(float(r["dv_kph"]))
    x = range(len(sigs))
    w = 0.38
    for j, (var, col, lab) in enumerate((("noE2E", C1, "without E2E (DBC range check on)"),
                                         ("E2E", C2, "with E2E (CRC + counter)"))):
        vals = [agg[(var, s)][0] for s in sigs]
        xs = [i + (j - 0.5) * w for i in x]
        d.bar(xs, vals, width=w - 0.04, color=col, label=lab)
        for xx, v, s in zip(xs, vals, sigs):
            d.text(xx, v + 1.5, f"{v}/{agg[(var, s)][2]}", ha="center", color=INK, fontsize=7)
    d.set_xticks(list(x), sigs)
    d.set_ylabel("runs with phantom / early braking")
    d.set_title("One RADAR_TGT frame, one flipped bit (R152-S60/M60, both parameter sets)")
    d.legend(loc="upper left")
    d.grid(axis="x", visible=False)
    d.text(0.02, 0.80, f"late / missed braking (not in the bars):\n"
                       f"  without E2E {len(late['noE2E'])} runs, max +{max(late['noE2E'], default=0):.1f} km/h\n"
                       f"  with E2E {len(late['E2E'])} runs, max +{max(late['E2E'], default=0):.1f} km/h: the rejected\n"
                       f"  frame was the decisive one in a zero-margin run\n"
                       f"  (S60 original) -> E2E turns corruption into a lost frame",
           transform=d.transAxes, color=INK2, fontsize=7, va="top")
    note(fig, "SYNTHETIC simulation. Left: results/e6_timeline_*.csv. Right: results/e5_corruption_runs.csv "
              "(one frame, one bit, every 100 ms from T0 = TTC 4 s).")
    fig.savefig(RES / "fig_sl3_timeline_e2e.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    fig1()
    fig2()
    fig3()
    print("figures written to", RES.name)
