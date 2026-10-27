"""Closed loop through the bus: equivalence with P5's direct loop, fault reactions, E2E in the loop."""
import hashlib
import importlib.util

import pytest

from signal_level.paths import P5_ROOT, VENDOR_FILE
from signal_level.sl_experiments import E2E_BITS, OFFICIAL, Job, run_job
from signal_level.sl_loop import BusSetup
from signal_level.supervision import RxConfig
from signal_level.vbus import BusFaults, Corruption

SAME = ("contact", "v_rel_impact_kph", "t_fcw", "t_brake", "t_full", "max_decel_req", "min_gap_m", "states",
        "t_contact", "ttc_brake", "brake_time_s", "peak_decel_mps2", "v_min_kph", "end_reason")


def test_vendored_state_machine_is_unchanged():
    spec = importlib.util.spec_from_file_location("check_vendor", P5_ROOT / "scripts" / "check_vendor.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.main() == 0
    assert hashlib.sha256(VENDOR_FILE.read_bytes()).hexdigest() == \
        "c90dff394375959636cf4ddac63bc53e00e0d8e9466567f73d3c060d5d188c18"


@pytest.mark.parametrize("cid,cfg,phase", [("R152-S60", "original", 0.0), ("R152-S60", "original", 0.04),
                                           ("R152-M30", "proposed", 0.02), ("NCAP-CCRb-50", "original", 0.04),
                                           ("NCAP-CCRs-80", "original", 0.0)])
def test_transparent_bus_reproduces_p5_direct_loop_exactly(cid, cfg, phase):
    d = run_job(Job(cid, cfg, phase))
    b = run_job(Job(cid, cfg, phase, BusSetup(mode="transparent")))
    for k in SAME:
        assert getattr(b, k) == getattr(d, k), k
    assert b.corrupted_accepted == 0 and b.t_fault is None and b.telltale_time_s == 0.0


def test_dbc_quantization_only_moves_a_knife_edge_threshold_by_one_radar_frame():
    # R152-S60, phase 0: true TTC hits 2.6 s (= original ttc_fcw_s) exactly at a radar frame; the float path
    # computes 2.59999..., the quantized one 43.35 m / 16.65 m/s = 2.6036 s -> warning one frame (50 ms) later
    d = run_job(Job("R152-S60", "original", 0.0))
    q = run_job(Job("R152-S60", "original", 0.0, BusSetup(mode="dbc")))
    assert q.t_fcw == pytest.approx(d.t_fcw + 0.05) and q.t_brake == pytest.approx(d.t_brake + 0.05)
    assert q.contact == d.contact and q.states == d.states and q.fcw_lead_s == pytest.approx(d.fcw_lead_s)
    for cid in OFFICIAL:                                    # other phase: identical
        d, q = run_job(Job(cid, "proposed", 0.03)), run_job(Job(cid, "proposed", 0.03, BusSetup()))
        assert (q.contact, q.v_rel_impact_kph, q.t_brake) == pytest.approx((d.contact, d.v_rel_impact_kph, d.t_brake))


def test_radar_burst_timeout_fault_telltale_and_recovery():
    # 500 ms bus-wide burst at t = 2.0 s on R152-M60 (before any warning).
    r = run_job(Job("R152-M60", "proposed", 0.0, BusSetup(faults=BusFaults(bursts=((2.0, 0.5),))), record=True))
    s = r.series
    # EGO_SPEED times out first: last frame 1.98 s, > 3 x 20 ms -> AEB task 2.05 s enters FAULT
    assert r.t_fault == pytest.approx(2.05)
    # the AEB's UNAVAILABLE status cannot cross the dead bus: the HMI detects the missing AEB_CMD itself
    # (last frame 1.99 s, > 3 x 10 ms -> 2.022 s) and switches the telltale on
    assert r.t_telltale == pytest.approx(2.022)
    # recovery: radar frames 2.50 WRONGSEQUENCE, 2.55 SYNC, 2.60/2.65/2.70 valid -> healed 2.70 s,
    # then the upstream fault_recover_s = 1.0 s -> STANDBY and telltale off at ~3.70 s
    off = next(t for t, tt in zip(s["t"], s["telltale"]) if t > 2.5 and not tt)
    assert 3.69 <= off <= 3.72
    for f in ("aeb<RADAR_TGT:timeout", "aeb<EGO_SPEED:timeout", "brake<AEB_CMD:timeout", "hmi<AEB_CMD:timeout"):
        assert f in r.input_faults


def test_e2e_rejects_a_corrupted_distance_that_would_cause_phantom_braking():
    bit = E2E_BITS["TgtDist"][9]                            # raw bit 9 = 25.6 m
    ref = run_job(Job("R152-M60", "original", 0.0, BusSetup()))
    hit = Corruption("RADAR_TGT", 2.5, (bit,))              # true gap 38.9 m -> 13.3 m, TTC 3.5 s -> 1.2 s
    with_e2e = run_job(Job("R152-M60", "original", 0.0, BusSetup(faults=BusFaults(targeted=(hit,)))))
    no_e2e = run_job(Job("R152-M60", "original", 0.0,
                         BusSetup(faults=BusFaults(targeted=(hit,)), rx=RxConfig(e2e=False))))
    assert with_e2e.frames_corrupted == 1 and with_e2e.corrupted_accepted == 0
    assert with_e2e.t_brake == ref.t_brake and with_e2e.v_min_kph == pytest.approx(ref.v_min_kph)
    assert no_e2e.corrupted_accepted == 1 and no_e2e.t_brake == pytest.approx(2.5)   # brakes ~2 s too early
    assert no_e2e.ttc_brake > 3.4


def test_burst_during_braking_baseline_releases_hold_variant_keeps_braking():
    ref = run_job(Job("R152-M60", "proposed", 0.0, BusSetup()))
    burst = BusFaults(bursts=((ref.t_brake + 0.2, 0.2),))
    base = run_job(Job("R152-M60", "proposed", 0.0, BusSetup(faults=burst)))
    hold = run_job(Job("R152-M60", "proposed", 0.0, BusSetup(faults=burst, hold_on_fault_s=2.0)))
    assert not ref.contact and base.contact and base.v_rel_impact_kph > 10.0     # R152-M60 limit is 0 km/h
    assert base.t_fault is not None and base.hold_time_s == 0.0
    assert not hold.contact and hold.hold_time_s > 0.5 and hold.t_fault == base.t_fault   # FAULT + telltale kept
    assert hold.t_telltale == pytest.approx(base.t_telltale)


def test_aeb_ecu_bus_off_during_braking_releases_the_brake():
    ref = run_job(Job("R152-S60", "original", 0.04, BusSetup()))
    r = run_job(Job("R152-S60", "original", 0.04,
                    BusSetup(faults=BusFaults(bus_off=(("AEB_ECU", ref.t_brake + 0.2, 0.3),)))))
    # brake ECU: last AEB_CMD at t_brake + 0.19, timeout > 30 ms -> request dropped 32 ms later.
    # 30 frames are lost = 2 x 15, so the 4-bit counter wraps exactly and the first frame after the bus-off
    # looks like "counter + 1" (status OK, no WRONGSEQUENCE): only the time-based timeout saw the gap.
    # Healing after 3 valid frames -> request executed again 20 ms after the bus-off ends.
    assert r.brake_lost_s == pytest.approx(0.3 + 0.02 + 0.01 - 0.032, abs=0.003)
    assert "WRONGSEQUENCE" not in r.rx_status["brake<AEB_CMD"] and "brake<AEB_CMD:timeout" in r.input_faults
    assert r.telltale_time_s > 0.25 and r.v_rel_impact_kph > ref.v_rel_impact_kph + 5.0
