"""One closed-loop test run: world backend (esmini or Python) + sensor model + reused AEB state machine
+ brake actuator + Ego longitudinal plant.

Per step (dt = 10 ms = AEB ECU task period):
  1. read target truth from the backend, compute bumper-to-bumper gap; contact if gap <= 0
  2. radar model -> measured (range, range rate) -> TTC via the upstream analyzer.ttc()
  3. upstream StateMachine.step(Inputs) -> state, decel_request()
  4. actuator (dead time + first-order lag + saturation) -> deceleration, limited by tyre-road friction
     (RoadMu parameter of the scenario, default 0.9; models.friction_limit_mps2)
  5. integrate Ego (point mass, no driver input) and report it to the backend, advance the backend by dt
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .aeb_loader import load
from .models import ActuatorParams, BrakeActuator, RadarModel, SensorParams, friction_limit_mps2
from .regulations import TestPoint

AEB = load()
S = AEB.S
ACTIVE, BRAKING = AEB.ACTIVE, AEB.BRAKING
upstream_ttc = AEB.upstream_ttc


@dataclass(frozen=True)
class RunConfig:
    aeb: object = field(default_factory=lambda: AEB.AebConfig())   # upstream AebConfig instance
    sensor: SensorParams = SensorParams()
    actuator: ActuatorParams = ActuatorParams()
    dt: float = 0.01
    tail_s: float = 0.5           # keep logging this long after the outcome is decided (plots only)
    speed_sample_s: float = 0.02  # ego-speed input refresh (upstream SPEED message cycle)


@dataclass
class RunResult:
    case_id: str
    family: str
    protocol: str
    label: str
    backend: str
    v_ego_kph: float
    v_target_kph: float
    contact: bool = False
    t_contact: float | None = None
    v_rel_impact_kph: float = 0.0
    v_ego_impact_kph: float | None = None
    contact_ncap: bool = False          # contact before the Euro NCAP end condition V_VUT = V_target
    v_rel_impact_ncap_kph: float = 0.0
    t_ncap_end: float | None = None
    min_gap_m: float = float("inf")
    t0: float | None = None             # protocol T0 (TTC 4 s, or CCRb decel start - 1 s)
    t_fcw: float | None = None          # first ECU task with state in FCW/PARTIAL/FULL (warning on)
    t_brake: float | None = None        # first non-zero deceleration request
    t_brake5: float | None = None       # first request >= 5.0 m/s2
    t_full: float | None = None
    ttc_fcw: float | None = None        # TRUE TTC at those instants
    ttc_brake: float | None = None
    ttc_full: float | None = None
    max_decel_req: float = 0.0
    road_mu: float = 0.9
    peak_decel_mps2: float = 0.0        # achieved (friction-limited) deceleration
    v_min_kph: float | None = None      # lowest Ego speed reached (speed loss in no-threat runs)
    brake_time_s: float = 0.0           # total time with a non-zero AEB deceleration request
    states: str = ""
    transitions: list = field(default_factory=list)
    backend_collision: bool | None = None
    end_reason: str = ""
    series: dict | None = None

    @property
    def fcw_lead_s(self) -> float | None:
        if self.t_fcw is None or self.t_brake is None:
            return None
        return self.t_brake - self.t_fcw


def _true_ttc(gap: float, v_e: float, v_t: float) -> float | None:
    closing = v_e - v_t
    return gap / closing if closing > 0.1 and gap > 0 else None


def run_case(tp: TestPoint, scen: str, params: dict[str, str], backend, cfg: RunConfig,
             record: bool = False, log_stem: str | None = None) -> RunResult:
    dt = cfg.dt
    backend.start(scen, params, dt, log_stem)
    try:
        return _loop(tp, params, backend, cfg, record)
    finally:
        backend.close()


def _loop(tp, params, backend, cfg, record) -> RunResult:
    dt = cfg.dt
    res = RunResult(tp.case_id, tp.family, tp.protocol, tp.label, backend.name, tp.v_ego_kph, tp.v_target_kph)
    sm = AEB.StateMachine(cfg.aeb)
    radar = RadarModel(cfg.sensor, dt)
    act = BrakeActuator(cfg.actuator, dt)
    n_task = max(1, round(cfg.aeb.task_s / dt))
    n_spd = max(1, round(cfg.speed_sample_s / dt))
    sim_end = float(params.get("SimEndTime", 25.0))
    decel_start = float(params["DecelStartTime"]) if tp.family == "CCRb" else None
    res.t0 = (decel_start - 1.0) if decel_start is not None else None
    res.road_mu = float(params.get("RoadMu", 0.9))
    a_fric = friction_limit_mps2(res.road_mu)

    s_e, v_e = backend.ego_s0, backend.ego_v0
    ef, tr = backend.ego_front, backend.target_rear
    v_meas_kph = v_e * 3.6
    res.v_min_kph = v_e * 3.6
    state = sm.state
    a_req = a_act = 0.0
    prev = None
    t_end = None
    ser = {k: [] for k in ("t", "gap", "v_ego", "v_tgt", "ttc_true", "ttc_meas", "state", "a_req", "a_act")} \
        if record else None
    k = 0
    while True:
        t = k * dt
        s_t, v_t = backend.target()
        gap = (s_t + tr) - (s_e + ef)

        # ---- outcome bookkeeping --------------------------------------------------------------
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

        # ---- sensor -> AEB function -> actuator -------------------------------------------------
        meas = radar.step(k, gap, v_t - v_e, v_e)
        ttc_meas = upstream_ttc(meas[0], meas[1]) if meas is not None else None
        if k % n_spd == 0:
            v_meas_kph = v_e * 3.6
        if k % n_task == 0:
            state = sm.step(AEB.Inputs(t=t, ttc=ttc_meas, v_ego_kph=v_meas_kph, gas_pct=0.0, brake_pressed=False,
                                       inputs_ok=True, input_fault=False, ok_since=0.0, main_switch=True))
            a_req = sm.decel_request()
            if res.t_fcw is None and state in ACTIVE:
                res.t_fcw, res.ttc_fcw = t, _true_ttc(gap, v_e, v_t)
            if res.t_brake is None and a_req > 0.0:
                res.t_brake, res.ttc_brake = t, _true_ttc(gap, v_e, v_t)
            if res.t_brake5 is None and a_req >= 5.0:
                res.t_brake5 = t
            if res.t_full is None and state == S.FULL_BRAKE:
                res.t_full, res.ttc_full = t, _true_ttc(gap, v_e, v_t)
            res.max_decel_req = max(res.max_decel_req, a_req)
        a_act = min(act.step(a_req), a_fric)
        res.peak_decel_mps2 = max(res.peak_decel_mps2, a_act)
        if a_req > 0.0:
            res.brake_time_s += dt
        if record:
            for key, val in (("t", t), ("gap", gap), ("v_ego", v_e), ("v_tgt", v_t),
                             ("ttc_true", _true_ttc(gap, v_e, v_t)), ("ttc_meas", ttc_meas),
                             ("state", int(state)), ("a_req", a_req), ("a_act", a_act)):
                ser[key].append(val)

        # ---- end of test --------------------------------------------------------------------------
        target_can_slow = tp.family == "CCRb" and v_t > 0.0
        if tp.protocol == "SOTIF":       # free-road phantom-object runs: fixed length after the ghost
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

        # ---- plant ----------------------------------------------------------------------------------
        prev = (gap, v_e, v_t, t)
        v_new = max(0.0, v_e - a_act * dt)
        if v_new == 0.0 and a_act > 0.0:
            s_new = s_e + v_e * v_e / (2.0 * a_act)      # stopped inside the step
        else:
            s_new = s_e + 0.5 * (v_e + v_new) * dt
        backend.advance(dt, s_new, v_new)
        s_e, v_e = s_new, v_new
        res.v_min_kph = min(res.v_min_kph, v_e * 3.6)
        k += 1

    res.transitions = [(round(tt, 3), a.name, b.name, rid) for tt, a, b, rid in sm.log]
    seq = [S.OFF.name] + [b for _, _, b, _ in res.transitions]
    res.states = ">".join(seq)
    res.backend_collision = backend.collision if backend.name == "esmini" else None
    res.series = ser
    return res
