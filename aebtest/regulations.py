"""Regulatory test points and pass criteria - every number carries its source.

Sources actually read for this project (quotes in report.md section 2):

R152  UN Regulation No 152 (AEBS for M1/N1), Official Journal of the EU, OJ L 2024/2497 (27.9.2024),
      https://eur-lex.europa.eu/eli/reg/2024/2497/oj/eng - "Incorporating all valid text up to:
      Supplement 3 to the 02 series of amendments - Date of entry into force: 24 September 2023".
      EUR-Lex itself states that only the original UN/ECE texts are authentic.  Supplements adopted after
      Supplement 3 to the 02 series were NOT checked (未核实).

NCAP  Euro NCAP "Crash Avoidance - Frontal Collisions Protocol", Version 1.2, July 2026,
      "Implementation January 2026", linked as the current protocol on https://www.euroncap.com/crash-avoidance/
      PDF: https://cdn.euroncap.com/cars/assets/Euro_NCAP_Protocol_Crash_Avoidance_Frontal_Collisions_v1_2_220d87e28a.pdf
      (SHA-256 of the copy read: b99c59d10707fca6b9fb537ca439887c049552ea7babdc234d9f22286c814236).
      Page numbers below are the printed page numbers.

Anything not in these two documents is a PROJECT CHOICE and is labelled as such where it is used.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

R152_SRC = "UN R152 (OJ L 2024/2497, up to Suppl. 3 to 02 series)"
NCAP_SRC = "Euro NCAP CA Frontal Collisions v1.2 (Jul 2026)"

# ============================================================== UN R152 ====
# 5.2.1.4, table "Maximum relative Impact Speed (km/h) for M1 vehicle":
#   relative speed (stationary / moving target) -> (maximum mass, mass in running order)
R152_M1_MAX_REL_IMPACT_KPH: dict[int, tuple[float, float]] = {
    10: (0.0, 0.0), 15: (0.0, 0.0), 20: (0.0, 0.0), 25: (0.0, 0.0), 30: (0.0, 0.0), 35: (0.0, 0.0),
    40: (0.0, 0.0), 42: (10.0, 0.0), 45: (15.0, 15.0), 50: (25.0, 25.0), 55: (30.0, 30.0), 60: (35.0, 35.0),
}
# 5.2.1.1: warning "shall be triggered at the latest 0.8 seconds before the start of emergency braking"
R152_WARNING_LEAD_S = 0.8
# 5.2.1.2: "there shall be a braking demand of at least 5.0 m/s2"
R152_MIN_BRAKE_DEMAND_MPS2 = 5.0
# 5.2.1.3: active "at least within the vehicle speed range between 10 km/h and 60 km/h"
R152_SPEED_RANGE_KPH = (10.0, 60.0)
# 6.4 / 6.5: functional part starts at "a TTC of at least 4 seconds"; straight approach >= 2 s before it
R152_TTC_START_S = 4.0
# 6.5: moving target "travelling at 20 km/h (with a tolerance of +0/-2 km/h ...)"
R152_MOVING_TARGET_KPH = 20.0
# 6.10.1: each scenario run twice; failed runs <= 10.0 % of car-to-car runs
R152_MAX_FAILED_RUN_SHARE = 0.10


def r152_max_rel_impact_kph(v_rel_kph: float, mass: str = "running_order") -> float:
    """Allowed relative impact speed for an M1 vehicle (R152 5.2.1.4).

    Footnote (4): "For relative speeds between the listed values (e.g. 53 km/h), the maximum relative
    impact speed (i.e. 30/30 km/h) assigned to the next higher relative speed (i.e. 55 km/h) shall apply."
    """
    col = {"max_mass": 0, "running_order": 1}[mass]
    for v in sorted(R152_M1_MAX_REL_IMPACT_KPH):
        if v_rel_kph <= v + 1e-9:
            return R152_M1_MAX_REL_IMPACT_KPH[v][col]
    raise ValueError(f"relative speed {v_rel_kph} km/h is above the R152 table (60 km/h)")


# ============================================================ Euro NCAP ====
# 3.1.1.1 Car-to-Car Rear grids (p.29).  "Function" column: AEB or FCW.
NCAP_CCRS = [(v, 0.0, "AEB" if v <= 50 else "FCW") for v in (10, 20, 30, 40, 50, 60, 70, 80)]
NCAP_CCRM = [(30, 20.0), (40, 20.0), (50, 20.0), (60, 20.0), (70, 20.0), (80, 20.0),
             (90, 30.0), (100, 40.0), (110, 50.0), (120, 60.0), (130, 70.0)]
NCAP_CCRB_SPEEDS = (30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130)
# p.30: "For CCRb, the Time Headway = 1.0 s, and the target acceleration = -4m/s2 for both Standard Range
# and Extended Range.  The desired deceleration of the GVT shall be reached within 1.0 second (T0 + 2.0s)"
NCAP_CCRB_HEADWAY_S = 1.0
NCAP_CCRB_DECEL_MPS2 = 4.0
# 1.5.1: "Time of test start T0 = TTC 4s, unless stated otherwise ... Braking scenarios: T0 =
# T_Target_deceleration_start - 1s"
NCAP_T0_TTC_S = 4.0
NCAP_CCRB_T0_BEFORE_DECEL_S = 1.0
# Standard / Extended range of the 50 % impact-location cell (the only one a 1-D model can represent).
# Read from the cell shading of the grids on p.29 (legend on p.28: dark grey = Standard Range,
# light grey = Extended Range): every CCRs and CCRm row is Standard at 50 %; CCRb 30-80 km/h Standard,
# CCRb 90-130 km/h all cells Extended.
NCAP_STANDARD_50PCT = {"CCRs": set(range(10, 81, 10)), "CCRm": {30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130},
                       "CCRb": {30, 40, 50, 60, 70, 80}}
# 3.1.1: Standard Range points per scenario (Car-to-Car Rear, total 6.5 incl. extended + robustness)
NCAP_STANDARD_POINTS = {"CCRs": 1.2, "CCRm": 2.4, "CCRb": 1.6}
# 5.3.1: Standard Range sub-score per grid cell by colour
NCAP_SUBSCORE = {"green": 1.00, "yellow": 0.75, "orange": 0.50, "brown": 0.25, "red": 0.00}
# 5.1: "For the AEB CCRs scenario, full avoidance shall be achieved for test speeds up to and including
# 20 km/h for all impact locations within the Standard Range."
NCAP_CCRS_FULL_AVOID_UP_TO_KPH = 20
# 4.2.4: 2 km/h impact-speed tolerance is applied only when checking a manufacturer prediction.
NCAP_VERIFICATION_TOL_KPH = 2.0

# Figure 5-1 "Relative Impact Speed colour band criteria (applicable to Car & PTW Rear ...)" (p.67).
# The figure is a vector graphic; the band edges below were reconstructed from the PDF drawing commands
# (bar x-coordinates calibrated against the axis tick labels 0..80 km/h) - see report.md 2.3.
# Edge convention (which band owns 10, 20, 30 km/h) taken from the worked example in 4.2.4:
# "Green vimpact = 0, Yellow 0 < vimpact <= 10, Orange 10 < vimpact <= 20, Brown 20 < vimpact <= 30, Red 30 < vimpact".
# row (VUT test speed, km/h; the last row is ">=80 km/h") -> [(upper edge inclusive, colour), ...] for v > 0
NCAP_BANDS: dict[int, list[tuple[float, str]]] = {
    10: [(math.inf, "red")],
    20: [(math.inf, "red")],
    30: [(10.0, "brown"), (math.inf, "red")],
    40: [(10.0, "orange"), (20.0, "brown"), (math.inf, "red")],
    50: [(10.0, "yellow"), (20.0, "orange"), (30.0, "brown"), (math.inf, "red")],
    60: [(10.0, "yellow"), (20.0, "orange"), (30.0, "brown"), (math.inf, "red")],
    70: [(10.0, "yellow"), (20.0, "orange"), (30.0, "brown"), (math.inf, "red")],
    80: [(10.0, "yellow"), (20.0, "orange"), (30.0, "brown"), (math.inf, "red")],
}


def ncap_colour(vut_speed_kph: float, v_rel_impact_kph: float) -> str:
    """Colour of one grid cell from the relative impact speed (Figure 5-1, row = VUT test speed)."""
    if v_rel_impact_kph <= 0.0:
        return "green"
    row = min(80, int(round(vut_speed_kph / 10.0)) * 10)
    for edge, colour in NCAP_BANDS[row]:
        if v_rel_impact_kph <= edge + 1e-9:
            return colour
    return "red"


# ====================================================== test matrix ====
@dataclass(frozen=True)
class TestPoint:
    case_id: str
    family: str               # CCRs | CCRm | CCRb  (scenario file)
    protocol: str             # R152 | NCAP
    v_ego_kph: float
    v_target_kph: float
    decel_mps2: float = 0.0   # target deceleration magnitude (CCRb)
    headway_s: float = 0.0    # CCRb initial time headway
    label: str = ""           # e.g. "AEB" / "FCW" cell, "official"/"sweep"
    masses: tuple[str, ...] = field(default=("running_order",))

    @property
    def v_rel_kph(self) -> float:
        return self.v_ego_kph - self.v_target_kph


def r152_points(sweep: bool = False) -> list[TestPoint]:
    """6.4 / 6.5 subject-vehicle test speeds for M1 (official), optionally the 5.2.1.4 table sweep."""
    pts = [
        # 6.4 stationary target: maximum mass 20 / 40 / 60, mass in running order 20 / 42 / 60 km/h
        TestPoint("R152-S20", "CCRs", "R152", 20, 0, label="official", masses=("max_mass", "running_order")),
        TestPoint("R152-S40", "CCRs", "R152", 40, 0, label="official", masses=("max_mass",)),
        TestPoint("R152-S42", "CCRs", "R152", 42, 0, label="official", masses=("running_order",)),
        TestPoint("R152-S60", "CCRs", "R152", 60, 0, label="official", masses=("max_mass", "running_order")),
        # 6.5 moving target at 20 km/h: subject 30 / 60 km/h for both masses
        TestPoint("R152-M30", "CCRm", "R152", 30, R152_MOVING_TARGET_KPH, label="official",
                  masses=("max_mass", "running_order")),
        TestPoint("R152-M60", "CCRm", "R152", 60, R152_MOVING_TARGET_KPH, label="official",
                  masses=("max_mass", "running_order")),
    ]
    if sweep:   # "the technical service may test any other speeds listed in the tables in 5.2.1.4"
        done = {p.v_ego_kph for p in pts if p.family == "CCRs"}
        for v in (10, 15, 20, 25, 30, 35, 40, 42, 45, 50, 55, 60):
            if v not in done:
                pts.append(TestPoint(f"R152-S{v}", "CCRs", "R152", v, 0, label="sweep"))
        done = {p.v_ego_kph for p in pts if p.family == "CCRm"}
        for v in (30, 35, 40, 45, 50, 55, 60):
            if v not in done:
                pts.append(TestPoint(f"R152-M{v}", "CCRm", "R152", v, R152_MOVING_TARGET_KPH, label="sweep"))
    return pts


def ncap_points() -> list[TestPoint]:
    pts = [TestPoint(f"NCAP-CCRs-{v}", "CCRs", "NCAP", v, vt, label=fn) for v, vt, fn in NCAP_CCRS]
    pts += [TestPoint(f"NCAP-CCRm-{v}", "CCRm", "NCAP", v, vt, label="AEB") for v, vt in NCAP_CCRM]
    pts += [TestPoint(f"NCAP-CCRb-{v}", "CCRb", "NCAP", v, v, decel_mps2=NCAP_CCRB_DECEL_MPS2,
                      headway_s=NCAP_CCRB_HEADWAY_S,
                      label="AEB" + ("" if v in NCAP_STANDARD_50PCT["CCRb"] else ",extended"))
            for v in NCAP_CCRB_SPEEDS]
    return pts
