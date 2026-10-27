"""Sensor (range / range-rate), brake-actuator and tyre-road friction models.  All parameter values are
PROJECT CHOICES unless a source is named.

Baseline values deliberately copy the upstream can-bus-analyzer closed loop (canbus_analyzer/closed_loop.py
at commit 27da004): LEAD_INFO radar frame every 50 ms, brake dead time 80 ms, first-order time constant
120 ms, plant deceleration limit 9.0 m/s2.  They are toy values, not measurements of any real sensor/brake.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

# Friction-limited braking: a_max = DECEL_PER_MU * mu.  Calibrated so that mu = 0.9 (the peak braking
# coefficient of the R152 / Euro NCAP test surface, R152 2.14 / NCAP 2.1) gives 9.0 m/s2, the mean fully
# developed deceleration a "dry road affording good adhesion" must permit (R152 2.13).  10 m/s2 per unit mu is
# within 2 % of g = 9.81 m/s2, i.e. the textbook point-mass limit a <= mu * g with ideal ABS.
DECEL_PER_MU = 10.0


def friction_limit_mps2(mu: float) -> float:
    return DECEL_PER_MU * mu


@dataclass(frozen=True)
class GhostTarget:
    """A phantom stationary object reported by the radar (SOTIF triggering condition, report.md 8)."""
    t_start_s: float          # first instant the ghost can appear
    duration_s: float         # how long it is reported
    ttc0_s: float             # TTC of the phantom when it appears (range = ttc0 * v_ego)


@dataclass(frozen=True)
class SensorParams:
    update_period_s: float = 0.050     # radar object update, sample-and-hold (upstream LEAD_INFO cycle)
    latency_s: float = 0.0             # extra transport / processing delay of each measurement
    range_noise_sd_m: float = 0.0      # Gaussian noise on range
    rate_noise_sd_mps: float = 0.0     # Gaussian noise on range rate
    max_range_m: float = 200.0         # beyond this: no target reported (reduced in the rain cases)
    phase_s: float = 0.0               # offset of the radar update instants from t = 0 (sampling phase)
    seed: int = 0
    ghost: GhostTarget | None = None


@dataclass(frozen=True)
class ActuatorParams:
    dead_time_s: float = 0.080         # request -> first brake build-up (upstream BRAKE_DEAD_S)
    tau_s: float = 0.120               # first-order deceleration build-up (upstream BRAKE_TAU_S)
    max_decel_mps2: float = 9.0        # plant saturation (upstream EGO_DECEL_MAX)


class RadarModel:
    """Measured (range, range_rate) of the in-lane target.

    The truth is stored every simulation step; at each update instant the value from ``latency_s`` ago is
    taken, Gaussian noise added, and held until the next update (the AEB uses the latest frame, no
    extrapolation - same as the upstream ECU).  range_rate = v_target - v_ego (negative when closing).
    A GhostTarget, while active, replaces the reported object by a phantom stationary object.
    """

    def __init__(self, p: SensorParams, dt: float):
        self.p, self.dt = p, dt
        self.n_update = max(1, round(p.update_period_s / dt))
        self.n_lat = round(p.latency_s / dt)
        self.n_phase = round(p.phase_s / dt)
        self.hist: list[tuple[float, float]] = []
        self.rng = random.Random(p.seed)
        self.out: tuple[float, float] | None = None
        self.ghost_range: float | None = None

    def step(self, k: int, gap_m: float, range_rate_mps: float, v_ego: float = 0.0) -> tuple[float, float] | None:
        self.hist.append((gap_m, range_rate_mps))
        g_on = False
        if self.p.ghost is not None:
            t, gh = k * self.dt, self.p.ghost
            if gh.t_start_s <= t < gh.t_start_s + gh.duration_s:
                if self.ghost_range is None:
                    self.ghost_range = gh.ttc0_s * v_ego
                g_on = True
            if self.ghost_range is not None:
                self.ghost_range -= v_ego * self.dt          # a stationary phantom gets closer
        if k >= self.n_phase and (k - self.n_phase) % self.n_update == 0:
            if g_on:
                self.out = (self.ghost_range, -v_ego)
                return self.out
            j = k - self.n_lat
            if j < 0:
                self.out = None                     # nothing has arrived yet
            else:
                g, rr = self.hist[j]
                if g > self.p.max_range_m:
                    self.out = None
                else:
                    g += self.rng.gauss(0.0, self.p.range_noise_sd_m) if self.p.range_noise_sd_m else 0.0
                    rr += self.rng.gauss(0.0, self.p.rate_noise_sd_mps) if self.p.rate_noise_sd_mps else 0.0
                    self.out = (g, rr)
        return self.out


class BrakeActuator:
    """Requested deceleration -> actual deceleration: pure dead time, then first-order lag, then saturation."""

    def __init__(self, p: ActuatorParams, dt: float):
        self.p, self.dt = p, dt
        self.n_dead = round(p.dead_time_s / dt)
        self.queue: list[float] = [0.0] * self.n_dead
        self.alpha = 1.0 - math.exp(-dt / p.tau_s) if p.tau_s > 0 else 1.0
        self.a = 0.0

    def step(self, a_req: float) -> float:
        self.queue.append(a_req)
        cmd = self.queue.pop(0)
        self.a += (cmd - self.a) * self.alpha
        return min(self.a, self.p.max_decel_mps2)
