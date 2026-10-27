"""Signal-level closed loop: P5's ``aebtest/closed_loop.py::_loop`` with a virtual CAN bus inserted.

Everything outside the communication path is P5's code or a line-by-line copy of it: target motion from
the backend (PythonBackend by default, esmini possible), bumper gap, contact interpolation, Euro NCAP end
condition, RadarModel, BrakeActuator, friction limit, point-mass Ego, end-of-test rules.  Lines that differ
from P5 are marked ``# BUS``.

Per plant step (dt = 2 ms):
  radar update instant -> RADAR node encodes RADAR_TGT -> bus          (50 ms, P5 radar update period)
  every 20 ms          -> ESC node encodes EGO_SPEED -> bus             (P5 speed_sample_s)
  bus.deliver(t)       -> AEB ECU: E2E check, plausibility, supervision
  every 10 ms          -> AEB task: TTC, upstream StateMachine.step, AEB_CMD -> bus
  bus.deliver(t)       -> BRAKE ECU / HMI: E2E check, timeout; brake request -> actuator -> plant

Recorded times are SIGNAL-LEVEL times: t_fcw = first instant the HMI shows the warning, t_brake = first
instant the brake ECU applies a request (so bus latency and lost frames are included).  The ECU-internal
times are kept as *_ecu.  With a fault-free, zero-latency bus both are identical to P5's direct loop.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache

from .paths import DBC_PATH, bootstrap

bootstrap()

from aebtest.closed_loop import AEB, RunConfig, RunResult, _true_ttc  # noqa: E402  (P5)
from aebtest.models import BrakeActuator, RadarModel, friction_limit_mps2  # noqa: E402  (P5)

from .dbc import Database, load_dbc  # noqa: E402
from .ecus import (CMD_MSG, RADAR_MSG, SPEED_MSG, AebEcuBus, BrakeEcuBus, HmiBus, TxNode,  # noqa: E402
                   radar_values, speed_values)
from .supervision import RxConfig  # noqa: E402
from .vbus import BusFaults, VirtualBus  # noqa: E402

S = AEB.S


@lru_cache(maxsize=1)
def load_db() -> Database:
    return load_dbc(DBC_PATH)


@dataclass(frozen=True)
class BusSetup:
    mode: str = "dbc"                      # "dbc": DBC quantization; "transparent": frames carry exact floats
    faults: BusFaults = BusFaults()
    rx: RxConfig = RxConfig()              # AEB ECU receiving RADAR_TGT / EGO_SPEED
    rx_out: RxConfig = RxConfig()          # brake ECU and HMI receiving AEB_CMD
    hold_on_fault_s: float = 0.0           # 0 = baseline (FAULT releases the brake); > 0 = mitigation variant:
                                           # AEB ECU keeps an ongoing intervention through an input FAULT and the
                                           # brake ECU keeps the last request through an AEB_CMD timeout

    def label(self) -> str:
        return (self.mode + ("" if self.rx.e2e else "-noE2E") + ("" if self.rx.plausibility else "-noRange")
                + (f"-hold{self.hold_on_fault_s:g}" if self.hold_on_fault_s > 0 else ""))


@dataclass
class BusRunResult(RunResult):
    bus_mode: str = ""
    e2e: bool = True
    t_fcw_ecu: float | None = None
    t_brake_ecu: float | None = None
    max_decel_req_ecu: float = 0.0
    t_fault: float | None = None           # first ECU task in FAULT
    fault_time_s: float = 0.0
    t_telltale: float | None = None        # first instant the HMI failure telltale is on
    telltale_time_s: float = 0.0
    brake_lost_s: float = 0.0              # AEB requests braking but the brake ECU has no healthy AEB_CMD
    hold_time_s: float = 0.0               # mitigation variant: time braking was held through a FAULT
    frames_tx: int = 0
    frames_dropped: int = 0
    frames_corrupted: int = 0
    corrupted_accepted: int = 0            # corrupted frames that a receiver used (only possible without E2E)
    rx_status: dict = field(default_factory=dict)
    input_faults: str = ""


def _check_cycles(db: Database, cfg: RunConfig, radar: RadarModel) -> None:
    """The DBC cycle times must be the periods of P5's models, otherwise timeouts would be inconsistent."""
    pairs = {RADAR_MSG: radar.n_update * cfg.dt, SPEED_MSG: cfg.speed_sample_s, CMD_MSG: cfg.aeb.task_s}
    for name, period in pairs.items():
        if abs(db.get_message_by_name(name).cycle_time_ms / 1000.0 - period) > 1e-9:
            raise ValueError(f"{name}: DBC cycle {db.get_message_by_name(name).cycle_time_ms} ms != model {period} s")


def run_case_bus(tp, scen: str, params: dict, backend, cfg: RunConfig, bus: BusSetup = BusSetup(),
                 record: bool = False) -> BusRunResult:
    backend.start(scen, params, cfg.dt, None)
    try:
        return _loop(tp, params, backend, cfg, bus, record)
    finally:
        backend.close()


def _loop(tp, params, backend, cfg: RunConfig, bus: BusSetup, record: bool) -> BusRunResult:
    dt = cfg.dt
    res = BusRunResult(tp.case_id, tp.family, tp.protocol, tp.label, backend.name, tp.v_ego_kph, tp.v_target_kph,
                       bus_mode=bus.mode, e2e=bus.rx.e2e)
    radar = RadarModel(cfg.sensor, dt)
    act = BrakeActuator(cfg.actuator, dt)
    n_task = max(1, round(cfg.aeb.task_s / dt))
    n_spd = max(1, round(cfg.speed_sample_s / dt))
    db = load_db()                                                                        # BUS
    _check_cycles(db, cfg, radar)                                                         # BUS
    transparent = bus.mode == "transparent"                                               # BUS
    vb = VirtualBus(bus.faults)                                                           # BUS
    ecu = AebEcuBus(AEB, cfg.aeb, db, bus.rx, transparent, bus.hold_on_fault_s)            # BUS
    brake = BrakeEcuBus(db.get_message_by_name(CMD_MSG), bus.rx_out, transparent, bus.hold_on_fault_s)  # BUS
    hmi = HmiBus(db.get_message_by_name(CMD_MSG), bus.rx_out, transparent)                # BUS
    radar_tx = TxNode(db.get_message_by_name(RADAR_MSG), bus.rx)                          # BUS
    esc_tx = TxNode(db.get_message_by_name(SPEED_MSG), bus.rx)                            # BUS
    vb.subscribe(RADAR_MSG, ecu.on_frame)
    vb.subscribe(SPEED_MSG, ecu.on_frame)
    vb.subscribe(CMD_MSG, brake.on_frame)
    vb.subscribe(CMD_MSG, hmi.on_frame)
    sim_end = float(params.get("SimEndTime", 25.0))
    decel_start = float(params["DecelStartTime"]) if tp.family == "CCRb" else None
    res.t0 = (decel_start - 1.0) if decel_start is not None else None
    res.road_mu = float(params.get("RoadMu", 0.9))
    a_fric = friction_limit_mps2(res.road_mu)

    s_e, v_e = backend.ego_s0, backend.ego_v0
    ef, tr = backend.ego_front, backend.target_rear
    res.v_min_kph = v_e * 3.6
    state = ecu.sm.state
    a_req = a_act = a_ecu = 0.0
    prev = None
    t_end = None
    keys = ("t", "gap", "v_ego", "v_tgt", "ttc_true", "ttc_meas", "state", "a_req", "a_req_ecu", "a_act",
            "telltale", "warning", "radar_age", "radar_faulted", "brake_rx_faulted")
    ser = {k: [] for k in keys} if record else None
    k = 0
    while True:
        t = k * dt
        s_t, v_t = backend.target()
        gap = (s_t + tr) - (s_e + ef)

        # ---- outcome bookkeeping (identical to P5) ------------------------------------------------
        if gap <= 0.0:
            g0, ve0, vt0, tp0 = prev if prev else (gap, v_e, v_t, t)
            f = g0 / (g0 - gap) if g0 > 0 else 0.0
            res.contact = True
            res.t_contact = tp0 + f * (t - tp0)
            ve_c, vt_c = ve0 + f * (v_e - ve0), vt0 + f * (v_t - vt0)
            res.v_ego_impact_kph = ve_c * 3.6
            res.v_rel_impact_kph = max(0.0, (ve_c - vt_c) * 3.6)
            res.min_gap_m = 0.0
            if res.t_ncap_end is None or res.t_contact < res.t_ncap_end:
                res.contact_ncap, res.v_rel_impact_ncap_kph = True, res.v_rel_impact_kph
            res.end_reason = "contact"
            break
        res.min_gap_m = min(res.min_gap_m, gap)
        if res.t0 is None and _true_ttc(gap, v_e, v_t) is not None and _true_ttc(gap, v_e, v_t) <= 4.0:
            res.t0 = t
        threat_started = decel_start is None or t > decel_start + 0.5 * dt
        if res.t_ncap_end is None and threat_started and k > 0 and v_e <= v_t + 1e-9:
            res.t_ncap_end = t

        # ---- sensor -> bus -> AEB ECU -> bus -> brake ECU -> actuator ---------------------------- BUS
        meas = radar.step(k, gap, v_t - v_e, v_e)
        if k >= radar.n_phase and (k - radar.n_phase) % radar.n_update == 0:
            vb.send(radar_tx.frame(t, radar_values(meas)))
        if k % n_spd == 0:
            vb.send(esc_tx.frame(t, speed_values(v_e * 3.6)))
        vb.deliver(t)
        if k % n_task == 0:
            frame, state, a_ecu = ecu.task(t)
            vb.send(frame)
            if res.t_fcw_ecu is None and state in AEB.ACTIVE:
                res.t_fcw_ecu = t
            if res.t_brake_ecu is None and a_ecu > 0.0:
                res.t_brake_ecu = t
            res.max_decel_req_ecu = max(res.max_decel_req_ecu, a_ecu)
            if state == S.FAULT and res.t_fault is None:
                res.t_fault = t
        vb.deliver(t)
        a_req = brake.request(t)
        warning, telltale = hmi.update(t)
        if res.t_fcw is None and warning:
            res.t_fcw, res.ttc_fcw = t, _true_ttc(gap, v_e, v_t)
        if res.t_brake is None and a_req > 0.0:
            res.t_brake, res.ttc_brake = t, _true_ttc(gap, v_e, v_t)
        if res.t_brake5 is None and a_req >= 5.0 - 1e-9:
            res.t_brake5 = t
        if res.t_full is None and brake.state() == int(S.FULL_BRAKE):
            res.t_full, res.ttc_full = t, _true_ttc(gap, v_e, v_t)
        res.max_decel_req = max(res.max_decel_req, a_req)
        if ecu.sm.state == S.FAULT:
            res.fault_time_s += dt
        if telltale:
            res.telltale_time_s += dt
            if res.t_telltale is None:
                res.t_telltale = t
        if ecu.last_decel > 0.0 and not brake.rx.usable and not brake.holding:
            res.brake_lost_s += dt
        if ecu.holding or brake.holding:
            res.hold_time_s += dt
        a_act = min(act.step(a_req), a_fric)
        res.peak_decel_mps2 = max(res.peak_decel_mps2, a_act)
        if a_req > 0.0:
            res.brake_time_s += dt
        if record:
            age = ecu.rx[RADAR_MSG].age(t)
            for key, val in (("t", t), ("gap", gap), ("v_ego", v_e), ("v_tgt", v_t),
                             ("ttc_true", _true_ttc(gap, v_e, v_t)), ("ttc_meas", ecu.last_ttc),
                             ("state", int(ecu.sm.state)), ("a_req", a_req), ("a_req_ecu", ecu.last_decel),
                             ("a_act", a_act), ("telltale", int(telltale)), ("warning", int(warning)),
                             ("radar_age", age), ("radar_faulted", int(ecu.rx[RADAR_MSG].faulted)),
                             ("brake_rx_faulted", int(brake.rx.faulted))):
                ser[key].append(val)

        # ---- end of test (identical to P5) ----------------------------------------------------------
        target_can_slow = tp.family == "CCRb" and v_t > 0.0
        if tp.protocol == "SOTIF":
            gh = cfg.sensor.ghost
            if t_end is None:
                t_end = (gh.t_start_s + gh.duration_s + 4.0) if gh else 5.0
                res.end_reason = "fixed duration (SOTIF free road)"
        elif t_end is None and ((v_e <= 0.0 and v_t <= 0.0) or (not target_can_slow and v_e <= v_t + 1e-9 and k > 0)):
            t_end = t + cfg.tail_s
            res.end_reason = "no further closing possible"
        if (t_end is not None and t >= t_end) or t >= sim_end:
            if t_end is None:
                res.end_reason = "timeout"
            break

        # ---- plant (identical to P5) ------------------------------------------------------------------
        prev = (gap, v_e, v_t, t)
        v_new = max(0.0, v_e - a_act * dt)
        if v_new == 0.0 and a_act > 0.0:
            s_new = s_e + v_e * v_e / (2.0 * a_act)
        else:
            s_new = s_e + 0.5 * (v_e + v_new) * dt
        backend.advance(dt, s_new, v_new)
        s_e, v_e = s_new, v_new
        res.v_min_kph = min(res.v_min_kph, v_e * 3.6)
        k += 1

    res.transitions = [(round(tt, 3), a.name, b.name, rid) for tt, a, b, rid in ecu.sm.log]
    res.states = ">".join([S.OFF.name] + [b for _, _, b, _ in res.transitions])
    res.backend_collision = backend.collision if backend.name == "esmini" else None
    res.series = ser
    st = vb.stats                                                                          # BUS
    res.frames_tx = st["tx"]
    res.frames_dropped = st["drop_bus_off"] + st["drop_burst"] + st["drop_random"]
    res.frames_corrupted = st["corrupted"]
    monitors = {**{f"aeb<{n}": m for n, m in ecu.rx.items()}, "brake<AEB_CMD": brake.rx, "hmi<AEB_CMD": hmi.rx}
    res.corrupted_accepted = sum(m.corrupted_accepted for m in monitors.values())
    res.rx_status = {n: dict(m.status_counts) for n, m in monitors.items()}
    res.input_faults = " ".join(f"{n}:{kind}@{tt:.2f}" for n, m in monitors.items() for tt, kind in m.faults)
    return res
