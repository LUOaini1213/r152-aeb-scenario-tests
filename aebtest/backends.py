"""World backends: who moves the TARGET and keeps the scenario clock.

EsminiBackend  - esmini v3.10.2 through esminiLib (ctypes).  The scenario (.xosc) and the grid values are
                 loaded by esmini; the Ego is handed to esmini's ExternalController (parameter ExtCtrl=true)
                 and its longitudinal state is reported every step with SE_ReportObjectRoadPos/Speed.
                 esmini moves the target according to the storyboard and checks bounding-box collisions.
PythonBackend  - transparent fallback / cross-check: reads the SAME .xosc parameter declarations and grid
                 values and reproduces the storyboard (constant target speed; CCRb: SpeedAction with linear
                 rate starting at DecelStartTime), integrating the target the way esmini does.

The Ego dynamics are NOT in the backend - they are integrated by closed_loop.py for both backends.

esmini cannot open non-ASCII absolute paths on this machine (the project path contains Chinese
characters), so esmini is always started with the scenarios folder as working directory and relative
ASCII file names.
"""
from __future__ import annotations

import ctypes as C
import os
import xml.etree.ElementTree as ET

from .scenarios import SCEN_DIR, read_declared_defaults, read_geometry


class _State(C.Structure):          # SE_ScenarioObjectState, esminiLib.hpp v3.10.2
    _fields_ = [("id", C.c_int), ("model_id", C.c_int), ("ctrl_type", C.c_int), ("timestamp", C.c_double),
                ("x", C.c_double), ("y", C.c_double), ("z", C.c_double),
                ("h", C.c_double), ("p", C.c_double), ("r", C.c_double),
                ("roadId", C.c_uint32), ("junctionId", C.c_uint32), ("t", C.c_double), ("laneId", C.c_int),
                ("laneOffset", C.c_double), ("s", C.c_double), ("speed", C.c_double),
                ("centerOffsetX", C.c_double), ("centerOffsetY", C.c_double), ("centerOffsetZ", C.c_double),
                ("width", C.c_double), ("length", C.c_double), ("height", C.c_double),
                ("objectType", C.c_int), ("objectCategory", C.c_int),
                ("wheel_angle", C.c_double), ("wheel_rot", C.c_double), ("visibilityMask", C.c_int)]


def _declared_types(scen: str) -> dict[str, str]:
    root = ET.parse(SCEN_DIR / scen).getroot()
    return {d.get("name"): d.get("parameterType")
            for d in root.find("ParameterDeclarations").iter("ParameterDeclaration")}


class EsminiBackend:
    name = "esmini"
    _lib = None

    def __init__(self, lib_path: str):
        if EsminiBackend._lib is None:
            lib = C.CDLL(lib_path)
            lib.SE_InitWithArgs.argtypes = [C.c_int, C.POINTER(C.c_char_p)]
            lib.SE_InitWithArgs.restype = C.c_int
            lib.SE_StepDT.argtypes = [C.c_double]
            lib.SE_StepDT.restype = C.c_int
            lib.SE_GetSimulationTime.restype = C.c_double
            lib.SE_GetIdByName.argtypes = [C.c_char_p]
            lib.SE_GetIdByName.restype = C.c_int
            lib.SE_GetObjectState.argtypes = [C.c_int, C.POINTER(_State)]
            lib.SE_GetObjectState.restype = C.c_int
            lib.SE_ReportObjectRoadPos.argtypes = [C.c_int, C.c_uint32, C.c_int, C.c_double, C.c_double]
            lib.SE_ReportObjectRoadPos.restype = C.c_int
            lib.SE_ReportObjectSpeed.argtypes = [C.c_int, C.c_double]
            lib.SE_ReportObjectSpeed.restype = C.c_int
            lib.SE_SetParameterDouble.argtypes = [C.c_char_p, C.c_double]
            lib.SE_SetParameterDouble.restype = C.c_int
            lib.SE_SetParameterBool.argtypes = [C.c_char_p, C.c_bool]
            lib.SE_SetParameterBool.restype = C.c_int
            lib.SE_SetParameterString.argtypes = [C.c_char_p, C.c_char_p]
            lib.SE_SetParameterString.restype = C.c_int
            lib.SE_GetObjectNumberOfCollisions.argtypes = [C.c_int]
            lib.SE_GetObjectNumberOfCollisions.restype = C.c_int
            lib.SE_GetQuitFlag.restype = C.c_int
            lib.SE_Close.restype = None
            EsminiBackend._cb_type = C.CFUNCTYPE(None, C.c_void_p)
            lib.SE_RegisterParameterDeclarationCallback.argtypes = [EsminiBackend._cb_type, C.c_void_p]
            lib.SE_SetOptionPersistent.argtypes = [C.c_char_p]
            lib.SE_SetOptionPersistent(b"disable_stdout")   # keep the console quiet for batch runs
            EsminiBackend._lib = lib
        self.lib = EsminiBackend._lib
        self._st = _State()
        self._cwd = None

    # ------------------------------------------------------------------ run
    def start(self, scen: str, params: dict[str, str], dt: float, log_stem: str | None = None) -> None:
        lib = self.lib
        types = _declared_types(scen)
        keep: list[bytes] = []

        def on_params(_):                    # called by esmini after parsing ParameterDeclarations
            for k, v in params.items():
                kind = types[k]
                if kind == "double":
                    rc = lib.SE_SetParameterDouble(k.encode(), float(v))
                elif kind == "string":
                    keep.append(v.encode())
                    rc = lib.SE_SetParameterString(k.encode(), keep[-1])
                else:
                    raise ValueError(f"parameter type {kind} not handled")
                if rc != 0:
                    raise RuntimeError(f"esmini rejected parameter {k}={v}")
            lib.SE_SetParameterBool(b"ExtCtrl", True)      # hand the Ego to the ExternalController

        self._cb = EsminiBackend._cb_type(on_params)      # keep a reference while esmini may call it
        lib.SE_RegisterParameterDeclarationCallback(self._cb, None)
        args = ["esmini", "--osc", scen, "--headless", "--fixed_timestep", f"{dt}", "--collision",
                "--disable_stdout"]
        if log_stem:
            os.makedirs(SCEN_DIR.parent / "results" / "esmini_logs", exist_ok=True)
            rel = f"../results/esmini_logs/{log_stem}"
            args += ["--csv_logger", f"{rel}.csv", "--record", f"{rel}.dat", "--logfile_path", f"{rel}.log.txt"]
        else:
            args += ["--disable_log"]
        self._cwd = os.getcwd()
        os.chdir(SCEN_DIR)
        argv = (C.c_char_p * len(args))(*[a.encode("ascii") for a in args])
        if lib.SE_InitWithArgs(len(args), argv) != 0:
            os.chdir(self._cwd)
            raise RuntimeError(f"esmini failed to load {scen} with {params.get('CaseId')}")
        self.ego = lib.SE_GetIdByName(b"Ego")
        self.tgt = lib.SE_GetIdByName(b"Target")
        e, t = self._state(self.ego), self._state(self.tgt)
        if e.ctrl_type != 1:
            raise RuntimeError("Ego is not under the ExternalController (ctrl_type != 1)")
        self.ego_front = e.centerOffsetX + e.length / 2.0
        self.target_rear = t.centerOffsetX - t.length / 2.0
        self.ego_s0, self.ego_v0 = e.s, e.speed
        self.collision = False

    def _state(self, oid: int) -> _State:
        self.lib.SE_GetObjectState(oid, C.byref(self._st))
        return _State.from_buffer_copy(self._st)

    def target(self) -> tuple[float, float]:
        t = self._state(self.tgt)
        return t.s, t.speed

    def advance(self, dt: float, ego_s: float, ego_v: float) -> None:
        self.lib.SE_ReportObjectRoadPos(self.ego, 1, -1, 0.0, ego_s)
        self.lib.SE_ReportObjectSpeed(self.ego, ego_v)
        self.lib.SE_StepDT(dt)
        if not self.collision and self.lib.SE_GetObjectNumberOfCollisions(self.ego) > 0:
            self.collision = True

    def time(self) -> float:
        return self.lib.SE_GetSimulationTime()

    def close(self) -> None:
        self.lib.SE_Close()
        if self._cwd:
            os.chdir(self._cwd)
            self._cwd = None


class PythonBackend:
    """Same scenario files, same parameters, target motion reproduced in Python (no esmini)."""
    name = "python"

    def start(self, scen: str, params: dict[str, str], dt: float, log_stem: str | None = None) -> None:
        vals = {**read_declared_defaults(scen), **params}
        geo = read_geometry(SCEN_DIR / scen)
        self.ego_front, self.target_rear = geo.ego_front_m, geo.target_rear_m
        self.ego_s0, self.ego_v0 = float(vals["EgoStartS"]), float(vals["EgoSpeed"])
        self.s, self.v = float(vals["TargetStartS"]), float(vals["TargetSpeed"])
        self.decel = float(vals.get("TargetDecel", 0.0))
        self.t_decel = float(vals.get("DecelStartTime", 1e9))
        self.t = 0.0
        self.collision = False

    def target(self) -> tuple[float, float]:
        return self.s, self.v

    def advance(self, dt: float, ego_s: float, ego_v: float) -> None:
        # esmini v3.10.2 semantics observed in its CSV log: the SimulationTimeCondition is true at the
        # start of the step with t >= DecelStartTime; speed is updated first, then s += v_new * dt.
        if self.decel > 0.0 and self.t >= self.t_decel - 1e-9:
            self.v = max(0.0, self.v - self.decel * dt)
        self.s += self.v * dt
        self.t += dt

    def time(self) -> float:
        return self.t

    def close(self) -> None:
        pass
