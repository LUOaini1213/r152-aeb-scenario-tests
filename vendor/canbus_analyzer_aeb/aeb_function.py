"""AEB function logic as it would run in the ECU: input supervision + state machine.

Everything in `AebConfig` is a PROJECT DESIGN CHOICE for this synthetic study
(TTC thresholds, speed window, hysteresis, dwell times, timeout counts,
deceleration levels).  None of these values is taken from a vehicle
manufacturer, a supplier or a regulation.

The state machine is table-driven: `TRANSITIONS` is the single source of
truth, it is evaluated top to bottom every ECU task (first matching row wins)
and the README transition table is rendered from it.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable

from .analyzer import ttc as ttc_from_radar
from .dbcutil import load_db, signal_limits, toyota_checksum


class S(IntEnum):
    """Function states.  The integer is what goes into PRE_COLLISION.STATE
    (3-bit field) -- a project encoding, not Toyota's meaning of that field."""
    OFF = 0
    STANDBY = 1
    FCW = 2
    PARTIAL_BRAKE = 3
    FULL_BRAKE = 4
    FAULT = 5


BRAKING = (S.PARTIAL_BRAKE, S.FULL_BRAKE)
ACTIVE = (S.FCW, S.PARTIAL_BRAKE, S.FULL_BRAKE)


@dataclass(frozen=True)
class AebConfig:
    # --- activation thresholds on time-to-collision (constant-velocity TTC
    #     computed from the latest valid LEAD_INFO frame) --------------------
    ttc_fcw_s: float = 2.6        # warning
    ttc_pb_s: float = 1.6         # partial braking
    ttc_fb_s: float = 0.9         # full braking
    ttc_hyst_s: float = 0.3       # de-escalation needs TTC >= threshold + hysteresis
    # --- ego speed window in which the function may activate ---------------
    v_min_kph: float = 8.0
    v_max_kph: float = 130.0
    v_stop_kph: float = 0.5       # brake intervention ends at standstill (no hold)
    # --- driver override --------------------------------------------------
    gas_override_pct: float = 40.0  # accelerator >= this suppresses / aborts
    # --- minimum dwell before de-escalation (anti-chattering) --------------
    fcw_min_dwell_s: float = 0.8
    brake_min_dwell_s: float = 0.5
    # Brake release rule.  False (default): a brake intervention ends only when
    # the target is no longer closing in (TTC undefined) or at standstill.
    # True: the alternative the project started with -- also release when the
    # constant-velocity TTC rises above ttc_pb_s + ttc_hyst_s.  Under braking
    # that TTC grows by itself, so this releases early; kept for the A/B
    # comparison in results/function/.
    brake_release_ttc_hyst: bool = False
    # --- degradation -------------------------------------------------------
    fault_recover_s: float = 1.0  # inputs valid this long before FAULT heals
    # --- actuation request ------------------------------------------------
    partial_decel_mps2: float = 4.0
    full_decel_mps2: float = 9.0
    vehicle_mass_kg: float = 1500.0  # FORCE = -mass * decel (PRE_COLLISION.FORCE in N)
    task_s: float = 0.010         # ECU task period


@dataclass(frozen=True)
class Inputs:
    t: float
    ttc: float | None             # None: no target / not closing
    v_ego_kph: float | None
    gas_pct: float
    brake_pressed: bool
    inputs_ok: bool               # every monitored input received and healthy
    input_fault: bool             # timeout or E2E error on a monitored input
    ok_since: float | None        # time since inputs_ok has been continuously true
    main_switch: bool = True


Guard = Callable[[AebConfig, Inputs, float], bool]


def _armed(c: AebConfig, i: Inputs) -> bool:
    return (i.ttc is not None and i.v_ego_kph is not None
            and c.v_min_kph <= i.v_ego_kph <= c.v_max_kph
            and i.gas_pct < c.gas_override_pct)


def _gone(c: AebConfig, i: Inputs, threshold: float) -> bool:
    return i.ttc is None or i.ttc >= threshold + c.ttc_hyst_s


# guard name -> (predicate(cfg, inputs, dwell_s), human-readable condition)
GUARDS: dict[str, tuple[Guard, str]] = {
    "switch_off": (lambda c, i, d: not i.main_switch, "main switch off"),
    "switch_on_inputs_ok": (lambda c, i, d: i.main_switch and i.inputs_ok,
                            "main switch on AND all inputs received and valid"),
    "switch_on_input_fault": (lambda c, i, d: i.main_switch and i.input_fault,
                              "main switch on AND input fault (see input_fault)"),
    "input_fault": (lambda c, i, d: i.input_fault,
                    "input timeout (> N cycles) OR E2E error (checksum/DLC/range) on SPEED, LEAD_INFO, "
                    "GAS_PEDAL or BRAKE_MODULE"),
    "inputs_healed": (lambda c, i, d: i.inputs_ok and i.ok_since is not None
                      and i.t - i.ok_since >= c.fault_recover_s,
                      "inputs valid continuously >= fault_recover_s"),
    "gas_override": (lambda c, i, d: i.gas_pct >= c.gas_override_pct,
                     "accelerator pedal >= gas_override_pct (driver override)"),
    "brake_pedal": (lambda c, i, d: i.brake_pressed, "driver brake pedal pressed (driver is reacting)"),
    "standstill": (lambda c, i, d: i.v_ego_kph is not None and i.v_ego_kph < c.v_stop_kph,
                   "ego speed < v_stop_kph (standstill reached)"),
    "brake_threat_gone": (lambda c, i, d: d >= c.brake_min_dwell_s and (
                              i.ttc is None or (c.brake_release_ttc_hyst and _gone(c, i, c.ttc_pb_s))),
                          "dwell >= brake_min_dwell_s AND no TTC (no target / not closing); "
                          "[variant brake_release_ttc_hyst: OR TTC >= ttc_pb_s + ttc_hyst_s]"),
    "fcw_threat_gone": (lambda c, i, d: d >= c.fcw_min_dwell_s and _gone(c, i, c.ttc_fcw_s),
                        "dwell >= fcw_min_dwell_s AND (no TTC OR TTC >= ttc_fcw_s + ttc_hyst_s)"),
    "ttc_full": (lambda c, i, d: _armed(c, i) and i.ttc < c.ttc_fb_s,
                 "armed AND TTC < ttc_fb_s"),
    "ttc_partial": (lambda c, i, d: _armed(c, i) and i.ttc < c.ttc_pb_s,
                    "armed AND TTC < ttc_pb_s"),
    "ttc_warn": (lambda c, i, d: _armed(c, i) and not i.brake_pressed and i.ttc < c.ttc_fcw_s,
                 "armed AND brake pedal not pressed AND TTC < ttc_fcw_s"),
}
ARMED_TEXT = ("armed = TTC defined AND v_min_kph <= ego speed <= v_max_kph AND "
              "accelerator < gas_override_pct")


@dataclass(frozen=True)
class Transition:
    rid: str
    sources: tuple[S, ...]
    target: S
    guard: str


_ON = (S.STANDBY, S.FCW, S.PARTIAL_BRAKE, S.FULL_BRAKE)
TRANSITIONS: tuple[Transition, ...] = (
    Transition("R01", _ON + (S.FAULT,), S.OFF, "switch_off"),
    Transition("R02", (S.OFF,), S.FAULT, "switch_on_input_fault"),
    Transition("R03", (S.OFF,), S.STANDBY, "switch_on_inputs_ok"),
    Transition("R04", _ON, S.FAULT, "input_fault"),
    Transition("R05", (S.FAULT,), S.STANDBY, "inputs_healed"),
    Transition("R06", ACTIVE, S.STANDBY, "gas_override"),
    Transition("R07", (S.FCW,), S.STANDBY, "brake_pedal"),
    Transition("R08", BRAKING, S.STANDBY, "standstill"),
    Transition("R09", BRAKING, S.STANDBY, "brake_threat_gone"),
    Transition("R10", (S.FCW,), S.STANDBY, "fcw_threat_gone"),
    Transition("R11", (S.STANDBY, S.FCW, S.PARTIAL_BRAKE), S.FULL_BRAKE, "ttc_full"),
    Transition("R12", (S.STANDBY, S.FCW), S.PARTIAL_BRAKE, "ttc_partial"),
    Transition("R13", (S.STANDBY,), S.FCW, "ttc_warn"),
)


class StateMachine:
    def __init__(self, cfg: AebConfig = AebConfig(), state: S = S.OFF, t0: float = 0.0):
        self.cfg = cfg
        self.state = state
        self.entered = t0
        self.log: list[tuple[float, S, S, str]] = []   # (t, from, to, rule id)

    def step(self, i: Inputs) -> S:
        dwell = i.t - self.entered
        for tr in TRANSITIONS:
            if self.state in tr.sources and GUARDS[tr.guard][0](self.cfg, i, dwell):
                self.log.append((i.t, self.state, tr.target, tr.rid))
                self.state = tr.target
                self.entered = i.t
                break
        return self.state

    def decel_request(self) -> float:
        if self.state == S.FULL_BRAKE:
            return self.cfg.full_decel_mps2
        if self.state == S.PARTIAL_BRAKE:
            return self.cfg.partial_decel_mps2
        return 0.0


# ------------------------------------------------------------ supervision ---
@dataclass(frozen=True)
class MonitorSpec:
    name: str
    period_s: float
    timeout_cycles: int = 3       # fault when no VALID frame for > N cycles
    e2e_max_consecutive: int = 2  # fault after this many consecutive invalid frames


@dataclass
class _MonState:
    last_valid: float | None = None
    consecutive_bad: int = 0
    bad_total: int = 0


# Physical plausibility on the inputs the function uses (DBC [min|max]).
PLAUSIBILITY = {"SPEED": ("SPEED",), "LEAD_INFO": ("LEAD_LONG_DIST", "LEAD_REL_SPEED")}


class Supervisor:
    """Per-input timeout monitoring and E2E-style frame checks.

    E2E here = DLC + Toyota CHECKSUM (where the DBC defines a CHECKSUM signal)
    + DBC range plausibility.  None of the input messages has a rolling
    counter in the opendbc DBC, so no counter check is possible on the inputs.
    """

    def __init__(self, specs: list[MonitorSpec], t0: float = 0.0):
        self.specs = {s.name: s for s in specs}
        self.state = {s.name: _MonState() for s in specs}
        self.t0 = t0
        db = load_db()
        self._msg = {s.name: db.get_message_by_name(s.name) for s in specs}
        self._has_cs = {s.name: any(sig.name == "CHECKSUM" for sig in self._msg[s.name].signals)
                        for s in specs}

    def frame_valid(self, name: str, arbitration_id: int, data: bytes) -> dict | None:
        m = self._msg[name]
        if len(data) != m.length:
            return None
        if self._has_cs[name] and data[-1] != toyota_checksum(arbitration_id, data):
            return None
        dec = m.decode(data, decode_choices=False)
        for sig in PLAUSIBILITY.get(name, ()):
            lo, hi = signal_limits(name, sig)
            if not lo <= float(dec[sig]) <= hi:
                return None
        return dec

    def on_frame(self, name: str, t: float, arbitration_id: int, data: bytes) -> dict | None:
        st = self.state[name]
        dec = self.frame_valid(name, arbitration_id, data)
        if dec is None:
            st.consecutive_bad += 1
            st.bad_total += 1
            return None
        st.consecutive_bad = 0
        st.last_valid = t
        return dec

    def faults(self, t: float) -> list[str]:
        out = []
        for name, spec in self.specs.items():
            st = self.state[name]
            ref = st.last_valid if st.last_valid is not None else self.t0
            if t - ref > spec.timeout_cycles * spec.period_s:
                out.append(f"{name}:timeout")
            if st.consecutive_bad >= spec.e2e_max_consecutive:
                out.append(f"{name}:e2e")
        return out

    def all_received(self) -> bool:
        return all(st.last_valid is not None for st in self.state.values())


# ------------------------------------------------------------------- ECU ----
INPUT_CYCLES_S = {"SPEED": 0.020, "LEAD_INFO": 0.050, "GAS_PEDAL": 0.020, "BRAKE_MODULE": 0.025}
OUTPUT_CYCLES_S = {"PRE_COLLISION": 0.030, "PCS_HUD": 0.030}
RADAR_NO_TARGET_M = 300.0     # project convention: LEAD_LONG_DIST at DBC max = no target


@dataclass
class AebEcu:
    cfg: AebConfig = field(default_factory=AebConfig)
    main_switch: bool = True
    timeout_cycles: int = 3
    e2e_max_consecutive: int = 2

    def __post_init__(self) -> None:
        self.sup = Supervisor([MonitorSpec(n, T, self.timeout_cycles, self.e2e_max_consecutive)
                               for n, T in INPUT_CYCLES_S.items()])
        self.sm = StateMachine(self.cfg)
        db = load_db()
        self._by_id = {db.get_message_by_name(n).frame_id: n for n in INPUT_CYCLES_S}
        self.latest: dict[str, dict] = {}
        self.ok_since: float | None = None
        self.fault_reasons: list[tuple[float, list[str]]] = []

    def on_frame(self, t: float, arbitration_id: int, data: bytes) -> None:
        name = self._by_id.get(arbitration_id)
        if name is None:
            return
        dec = self.sup.on_frame(name, t, arbitration_id, data)
        if dec is not None:
            self.latest[name] = dec

    def inputs(self, t: float) -> Inputs:
        faults = self.sup.faults(t)
        ok = not faults and self.sup.all_received()
        if ok and self.ok_since is None:
            self.ok_since = t
        elif not ok:
            self.ok_since = None
        if faults and (not self.fault_reasons or self.fault_reasons[-1][1] != faults):
            self.fault_reasons.append((t, faults))
        lead, spd = self.latest.get("LEAD_INFO"), self.latest.get("SPEED")
        ttc = None
        if lead is not None and float(lead["LEAD_LONG_DIST"]) < RADAR_NO_TARGET_M:
            ttc = ttc_from_radar(float(lead["LEAD_LONG_DIST"]), float(lead["LEAD_REL_SPEED"]))
        return Inputs(
            t=t, ttc=ttc,
            v_ego_kph=None if spd is None else float(spd["SPEED"]),
            gas_pct=float(self.latest.get("GAS_PEDAL", {}).get("GAS_PEDAL", 0.0)),
            brake_pressed=bool(int(self.latest.get("BRAKE_MODULE", {}).get("BRAKE_PRESSED", 0))),
            inputs_ok=ok, input_fault=bool(faults), ok_since=self.ok_since,
            main_switch=self.main_switch)

    def step(self, t: float) -> S:
        return self.sm.step(self.inputs(t))

    def output_values(self, name: str, counter: int) -> dict[str, float]:
        s = self.sm.state
        if name == "PRE_COLLISION":
            decel = self.sm.decel_request()
            return {"_COUNTER": counter % 256, "STATE": int(s),
                    "PRECOLLISION_ACTIVE": 1 if s in BRAKING else 0,
                    "FORCE": -self.cfg.vehicle_mass_kg * decel}
        # PCS_HUD
        return {"FCW": 1 if s in ACTIVE else 0,
                "PCS_INDICATOR": 2 if s == S.FAULT else (1 if s == S.OFF else 0)}
