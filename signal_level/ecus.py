"""Bus nodes around the vendored AEB state machine.

RADAR / ESC   encode the P5 sensor values into RADAR_TGT / EGO_SPEED, E2E-protect, put them on the bus
AEB_ECU       receive + supervise RADAR_TGT and EGO_SPEED (supervision.py), compute TTC with the upstream
              ``analyzer.ttc``, step the UNCHANGED upstream ``StateMachine`` (loaded by P5's aeb_loader), send
              AEB_CMD every task.  Replaces the upstream CAN shell (``AebEcu`` / ``Supervisor``), which needs
              cantools and the opendbc Toyota DBC and therefore cannot run here.
BRAKE_ECU     receive + supervise AEB_CMD; executes the deceleration request only while AEB_CMD is healthy
HMI           receive + supervise AEB_CMD; warning display and AEB failure telltale

Mapping of supervision faults onto the upstream state machine (design choice of this project):
  inputs_ok   = every input received at least once and none faulted
  input_fault = any input faulted (timeout or E2E)            -> upstream R04: any ON state -> FAULT
  ok_since    = start of the current inputs_ok interval        -> upstream R05: FAULT -> STANDBY after
                                                                  AebConfig.fault_recover_s (1.0 s upstream)
"""
from __future__ import annotations

import math

from .dbc import Database, Message
from .e2e import P01Protector
from .supervision import RxConfig, RxMonitor, p01_config
from .vbus import Frame

RADAR_MSG, SPEED_MSG, CMD_MSG = "RADAR_TGT", "EGO_SPEED", "AEB_CMD"
AVAILABLE, DEGRADED, UNAVAILABLE = 0, 1, 2
TGT_QUALITY_MIN = 2            # design assumption: target used only if TgtValid = 1 and TgtQuality >= 2


class TxNode:
    """Encode physical values with the DBC, E2E-protect the bytes, return the frame."""

    def __init__(self, msg: Message, rx_cfg: RxConfig = RxConfig()):
        self.msg = msg
        self.protector = P01Protector(p01_config(msg, rx_cfg))

    def frame(self, t: float, values: dict) -> Frame:
        data = bytearray(self.msg.encode(values))
        self.protector.protect(data)
        return Frame(self.msg.name, self.msg.frame_id, self.msg.sender, bytes(data), t, phys=dict(values))


def radar_values(meas: tuple[float, float] | None) -> dict:
    """P5 RadarModel output (range, range rate) -> RADAR_TGT signals; None = no target reported."""
    if meas is None:
        return {"TgtValid": 0, "TgtQuality": 0, "TgtDist": 0.0, "TgtRelSpd": 0.0}
    return {"TgtValid": 1, "TgtQuality": 3, "TgtDist": meas[0], "TgtRelSpd": meas[1]}


def speed_values(v_kph: float) -> dict:
    return {"VehSpdValid": 1, "VehSpd": v_kph}


class AebEcuBus:
    def __init__(self, aeb_mod, cfg, db: Database, rx_cfg: RxConfig = RxConfig(), transparent: bool = False,
                 hold_on_fault_s: float = 0.0):
        self.A = aeb_mod
        self.cfg = cfg
        self.sm = aeb_mod.StateMachine(cfg)
        self.rx = {RADAR_MSG: RxMonitor(db.get_message_by_name(RADAR_MSG), rx_cfg, transparent),
                   SPEED_MSG: RxMonitor(db.get_message_by_name(SPEED_MSG), rx_cfg, transparent,
                                        qualifier=lambda p: p["VehSpdValid"] >= 0.5)}
        self.tx = TxNode(db.get_message_by_name(CMD_MSG), rx_cfg)
        self.ok_since: float | None = None
        self.hold_on_fault_s = hold_on_fault_s
        self._hold_until = -math.inf
        self._hold_decel = 0.0
        self.last_decel = 0.0
        self.last_ttc: float | None = None
        self.holding = False

    def on_frame(self, t: float, frame: Frame) -> None:
        self.rx[frame.name].on_frame(t, frame)

    def task(self, t: float):
        A, S = self.A, self.A.S
        for m in self.rx.values():
            m.poll(t)
        radar, spd = self.rx[RADAR_MSG], self.rx[SPEED_MSG]
        faulted = [m for m in self.rx.values() if m.faulted]
        ok = not faulted and all(m.ever_valid for m in self.rx.values())
        if ok and self.ok_since is None:
            self.ok_since = t
        elif not ok:
            self.ok_since = None
        ttc = None
        if radar.usable and radar.data["TgtValid"] >= 0.5 and radar.data["TgtQuality"] >= TGT_QUALITY_MIN:
            ttc = A.upstream_ttc(float(radar.data["TgtDist"]), float(radar.data["TgtRelSpd"]))
        v = float(spd.data["VehSpd"]) if spd.usable else None
        prev = self.sm.state
        state = self.sm.step(A.Inputs(t=t, ttc=ttc, v_ego_kph=v, gas_pct=0.0, brake_pressed=False, inputs_ok=ok,
                                      input_fault=bool(faulted), ok_since=self.ok_since, main_switch=True))
        decel = self.sm.decel_request()
        # optional mitigation (NOT upstream behaviour): an intervention interrupted by an input FAULT is kept
        # at its last level until standstill, until the state machine requests at least as much again, until
        # the upstream release condition holds with healthy inputs (no TTC = target no longer closing), or
        # until hold_on_fault_s has passed
        self.holding = False
        if self.hold_on_fault_s > 0.0:
            if state == S.FAULT and prev in A.BRAKING:
                self._hold_until, self._hold_decel = t + self.hold_on_fault_s, self.last_decel
            if t < self._hold_until - 1e-9:
                released = (v is not None and v < self.cfg.v_stop_kph) or decel >= self._hold_decel \
                    or (ok and ttc is None)
                if released:
                    self._hold_until = -math.inf
                else:
                    decel, self.holding = self._hold_decel, True
        self.last_decel, self.last_ttc = decel, ttc
        reason = (int(radar.fault_kind == "timeout") | (int(radar.fault_kind == "e2e") << 1)
                  | (int(spd.fault_kind == "timeout") << 2) | (int(spd.fault_kind == "e2e") << 3))
        avail = UNAVAILABLE if state == S.FAULT else (
            DEGRADED if any(m.degraded(t) for m in self.rx.values()) else AVAILABLE)
        values = {"AEB_State": int(state), "AEB_BrkReqActive": 1 if decel > 0.0 else 0, "AEB_AccelReq": -decel,
                  "AEB_WarnLvl": 2 if state in A.BRAKING or self.holding else (1 if state in A.ACTIVE else 0),
                  "AEB_Avail": avail, "AEB_FaultReason": reason}
        return self.tx.frame(t, values), state, decel


class BrakeEcuBus:
    """Executes AEB_AccelReq while AEB_CMD is healthy; holds the last valid request until the timeout.

    ``hold_s`` > 0 (mitigation variant, NOT the baseline): if AEB_CMD is lost while a request is active, keep
    executing the last request for up to hold_s after the last healthy frame (fail-operational hold)."""

    def __init__(self, msg: Message, rx_cfg: RxConfig = RxConfig(), transparent: bool = False, hold_s: float = 0.0):
        self.rx = RxMonitor(msg, rx_cfg, transparent)
        self.hold_s = hold_s
        self._last_req, self._last_ok_t = 0.0, -math.inf
        self.holding = False

    def on_frame(self, t: float, frame: Frame) -> None:
        self.rx.on_frame(t, frame)

    def request(self, t: float) -> float:
        self.rx.poll(t)
        self.holding = False
        if self.rx.usable:
            d = self.rx.data
            req = max(0.0, -float(d["AEB_AccelReq"])) if d["AEB_BrkReqActive"] >= 0.5 else 0.0
            self._last_req, self._last_ok_t = req, t
            return req
        if self.hold_s > 0.0 and self._last_req > 0.0 and t - self._last_ok_t <= self.hold_s + 1e-9:
            self.holding = True
            return self._last_req
        return 0.0

    def state(self) -> int | None:
        return int(self.rx.data["AEB_State"]) if self.rx.usable else None


class HmiBus:
    """Warning display and failure telltale.  Lost AEB_CMD (timeout) also switches the telltale on."""

    def __init__(self, msg: Message, rx_cfg: RxConfig = RxConfig(), transparent: bool = False):
        self.rx = RxMonitor(msg, rx_cfg, transparent)

    def on_frame(self, t: float, frame: Frame) -> None:
        self.rx.on_frame(t, frame)

    def update(self, t: float) -> tuple[bool, bool]:
        self.rx.poll(t)
        if self.rx.usable:
            return self.rx.data["AEB_WarnLvl"] >= 1, self.rx.data["AEB_Avail"] >= UNAVAILABLE
        return False, self.rx.faulted
