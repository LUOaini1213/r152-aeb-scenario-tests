"""Unit / regression tests.  Run:  python -I -m pytest -q tests
The esmini test runs only when the environment variable ESMINI_LIB points to esminiLib.dll."""
import math
import os

import pytest

from aebtest.aeb_loader import load
from aebtest.closed_loop import RunConfig, run_case
from aebtest.backends import EsminiBackend, PythonBackend
from aebtest.experiments import ORIGINAL, PROPOSED, config
from aebtest.models import ActuatorParams, BrakeActuator, RadarModel, SensorParams
from aebtest.regulations import ncap_colour, r152_max_rel_impact_kph, r152_points, ncap_points
from aebtest.runner import GRID, POINTS
from aebtest.scenarios import SCEN_DIR, read_geometry, scenario_params

AEB = load()


def test_vendored_code_identical():
    import importlib.util
    spec = importlib.util.spec_from_file_location("check_vendor", SCEN_DIR.parent / "scripts" / "check_vendor.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.main() == 0


def test_upstream_ttc_and_table():
    assert AEB.upstream_ttc(10.0, -5.0) == pytest.approx(2.0)
    assert AEB.upstream_ttc(10.0, -0.05) is None          # closing <= MIN_CLOSING_MPS (0.1 m/s)
    assert [t.rid for t in AEB.TRANSITIONS] == [f"R{i:02d}" for i in range(1, 14)]


def test_state_machine_escalation_original_values():
    sm = AEB.StateMachine(AEB.AebConfig())
    seq = []
    for k, ttc in enumerate([None, 3.0, 2.5, 1.5, 0.8]):
        st = sm.step(AEB.Inputs(t=k * 0.01, ttc=ttc, v_ego_kph=50.0, gas_pct=0.0, brake_pressed=False,
                                inputs_ok=True, input_fault=False, ok_since=0.0))
        seq.append((st.name, sm.decel_request()))
    assert seq == [("STANDBY", 0.0), ("STANDBY", 0.0), ("FCW", 0.0), ("PARTIAL_BRAKE", 4.0), ("FULL_BRAKE", 9.0)]


def test_proposed_changes_only_four_values():
    changed = {k for k in ORIGINAL.__dataclass_fields__ if getattr(ORIGINAL, k) != getattr(PROPOSED, k)}
    assert changed == {"ttc_fcw_s", "ttc_pb_s", "ttc_fb_s", "partial_decel_mps2"}


def test_r152_table_and_footnote():
    assert r152_max_rel_impact_kph(42, "running_order") == 0.0
    assert r152_max_rel_impact_kph(42, "max_mass") == 10.0
    assert r152_max_rel_impact_kph(53, "running_order") == 30.0     # footnote (4) example
    assert r152_max_rel_impact_kph(60) == 35.0
    with pytest.raises(ValueError):
        r152_max_rel_impact_kph(61)
    assert {p.case_id for p in r152_points()} == {"R152-S20", "R152-S40", "R152-S42", "R152-S60", "R152-M30", "R152-M60"}


def test_ncap_colour_bands():
    # worked example of Euro NCAP v1.2 4.2.4 (60 km/h row)
    for v, c in [(0, "green"), (5, "yellow"), (10, "yellow"), (10.1, "orange"), (20, "orange"),
                 (25, "brown"), (30, "brown"), (30.1, "red")]:
        assert ncap_colour(60, v) == c
    assert ncap_colour(20, 0.1) == "red" and ncap_colour(30, 5) == "brown" and ncap_colour(30, 11) == "red"
    assert ncap_colour(40, 5) == "orange" and ncap_colour(40, 15) == "brown" and ncap_colour(40, 21) == "red"
    assert ncap_colour(130, 25) == "brown"                              # ">= 80 km/h" row
    assert len(ncap_points()) == 8 + 11 + 11


def test_scenario_params_geometry():
    geo = read_geometry(SCEN_DIR / "CCRs.xosc")
    assert geo.ego_front_m == pytest.approx(3.7) and geo.target_rear_m == pytest.approx(-0.7)
    p = scenario_params(POINTS["NCAP-CCRs-50"], geo)
    assert p["TargetStartS"] == pytest.approx(50 + 3.7 + 6.0 * 50 / 3.6 + 0.7)     # T0 = TTC 4 s + 2 s pre-roll
    b = scenario_params(POINTS["NCAP-CCRb-50"], read_geometry(SCEN_DIR / "CCRb.xosc"))
    assert b["TargetStartS"] - (50 + 3.7 + 0.7) == pytest.approx(1.0 * 50 / 3.6)  # headway 1.0 s
    assert b["DecelStartTime"] == pytest.approx(3.0) and b["TargetDecel"] == 4.0
    assert GRID["NCAP-CCRb-50"][1] == "CCRb.xosc"


def test_models():
    act = BrakeActuator(ActuatorParams(dead_time_s=0.08, tau_s=0.12), 0.01)
    out = [act.step(9.0) for _ in range(30)]
    assert out[7] == 0.0 and out[8] > 0.0                               # 80 ms dead time
    assert out[8 + 12] == pytest.approx(9.0 * (1 - math.exp(-0.13 / 0.12)), rel=1e-6)
    radar = RadarModel(SensorParams(latency_s=0.1, update_period_s=0.05), 0.01)
    seen = [radar.step(k, 100.0 - k, -1.0) for k in range(20)]
    assert seen[9] is None and seen[10] == (100.0, -1.0) and seen[15] == (95.0, -1.0)


def test_python_backend_target_braking():
    be = PythonBackend()
    be.start("CCRb.xosc", GRID["NCAP-CCRb-50"][3], 0.002)
    t_stop = None
    for k in range(5000):
        be.advance(0.002, 0.0, 0.0)
        if t_stop is None and be.target()[1] == 0.0:
            t_stop = be.time()
    assert t_stop == pytest.approx(3.0 + (50 / 3.6) / 4.0, abs=0.004)


def test_closed_loop_regression_python_backend():
    def run(cid, aeb):
        _, scen, _, params = GRID[cid]
        return run_case(POINTS[cid], scen, params, PythonBackend(), config(aeb, 0.0, 0.12, 0.0))
    r = run("NCAP-CCRs-80", ORIGINAL)
    assert r.contact and 37.0 < r.v_rel_impact_kph < 40.5             # upstream README reports 39.2 km/h
    assert not run("NCAP-CCRs-80", PROPOSED).contact
    s = run("R152-S20", ORIGINAL)
    assert not s.contact and s.max_decel_req == 4.0 and s.fcw_lead_s == pytest.approx(1.0, abs=0.011)


@pytest.mark.skipif(not os.environ.get("ESMINI_LIB"), reason="set ESMINI_LIB to esminiLib.dll to run")
def test_esmini_equals_python_backend():
    for cid in ("NCAP-CCRb-70", "R152-M60"):
        _, scen, _, params = GRID[cid]
        cfg = config(ORIGINAL, 0.0, 0.12, 0.0)
        a = run_case(POINTS[cid], scen, params, EsminiBackend(os.environ["ESMINI_LIB"]), cfg)
        b = run_case(POINTS[cid], scen, params, PythonBackend(), cfg)
        assert a.states == b.states and a.v_rel_impact_kph == pytest.approx(b.v_rel_impact_kph, abs=1e-4)
        assert a.backend_collision == a.contact


def test_friction_limit_and_wet_run():
    from aebtest.models import friction_limit_mps2
    assert friction_limit_mps2(0.9) == pytest.approx(9.0) and friction_limit_mps2(0.5) == pytest.approx(5.0)
    _, scen, _, params = GRID["NCAP-CCRs-80"]
    r = run_case(POINTS["NCAP-CCRs-80"], scen, {**params, "RoadMu": "0.5"}, PythonBackend(), config(PROPOSED, 0.0, 0.12, 0.0))
    assert r.road_mu == 0.5 and r.peak_decel_mps2 <= 5.0 + 1e-9 and r.contact      # wet: proposed no longer avoids 80 km/h


def test_ghost_target_original_vs_proposed():
    from aebtest.closed_loop import RunConfig
    from aebtest.experiments import DT
    from aebtest.models import GhostTarget
    _, scen, _, params = GRID["SOTIF-FREE-60"]
    def run(aeb):
        cfg = RunConfig(aeb=aeb, dt=DT, sensor=SensorParams(ghost=GhostTarget(3.0, 0.05, 1.8)))
        return run_case(POINTS["SOTIF-FREE-60"], scen, params, PythonBackend(), cfg)
    o, p = run(ORIGINAL), run(PROPOSED)
    assert o.max_decel_req == 0.0 and o.t_fcw is not None               # one phantom frame at TTC 1.8 s: warning only
    assert p.max_decel_req == 6.0 and 60.0 - p.v_min_kph > 9.0          # proposed brakes on it


def test_traceability_complete():
    import importlib.util
    if not (SCEN_DIR.parent / "results" / "r152_results.csv").exists():
        pytest.skip("run scripts/run_all.py and scripts/run_deepening.py first")
    spec = importlib.util.spec_from_file_location("mt", SCEN_DIR.parent / "scripts" / "make_traceability.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.main() == 0
