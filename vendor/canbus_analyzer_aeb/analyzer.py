"""Decode recorded traffic with cantools and run the checks.

Everything here works only on what the receiver saw on the bus
(timestamp, arbitration id, data bytes) -- never on simulator ground truth.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import can

from .config import (COUNTER_SIGNALS, CYCLE_TIME_S, DROP_FACTOR, EPOCH, JITTER_TOL_FRAC,
                     MIN_CLOSING_MPS, PRESSURE_X, RANGE_CHECKED, STUCK_MIN_S,
                     STUCK_WHEEL_DELTA_KPH, TTC_TRIGGER_S)
from .dbcutil import all_limits, has_checksum, load_db, toyota_checksum


@dataclass(frozen=True)
class RxFrame:
    t: float
    arbitration_id: int
    data: bytes


@dataclass(frozen=True)
class Finding:
    kind: str          # missing | timing | dlc | checksum | range | counter | stuck
    name: str
    t: float           # frame time (for interval findings: end of interval)
    t_start: float = 0.0
    detail: str = ""


@dataclass(frozen=True)
class CycleStats:
    name: str
    nominal_ms: float
    n: int
    mean_ms: float
    p95_ms: float
    max_ms: float
    missing: int
    timing_violations: int


@dataclass(frozen=True)
class Latency:
    trigger_s: float | None
    request_s: float | None
    pressure_s: float | None

    @property
    def to_request_ms(self) -> float | None:
        if self.trigger_s is None or self.request_s is None:
            return None
        return (self.request_s - self.trigger_s) * 1000.0

    @property
    def to_pressure_ms(self) -> float | None:
        if self.trigger_s is None or self.pressure_s is None:
            return None
        return (self.pressure_s - self.trigger_s) * 1000.0


@dataclass
class Analysis:
    n_frames: int
    unknown_ids: dict[int, int]
    stats: list[CycleStats]
    findings: list[Finding]
    latency: Latency
    bus_load_pct: tuple[float, float] = (0.0, 0.0)   # (no stuffing, worst-case stuffing)
    counts: dict[str, int] = field(default_factory=dict)


# ---------------------------------------------------------------- helpers ---
def from_messages(msgs: list[can.Message]) -> list[RxFrame]:
    """Convert to scenario-relative time.  EPOCH + t as a float64 only has
    ~0.24 us resolution, so times are rounded back onto the 1 us grid the
    scenario uses (also the resolution of an ASC log)."""
    return [RxFrame(round(m.timestamp - EPOCH, 6), m.arbitration_id, bytes(m.data)) for m in msgs]


def percentile(values: list[float], q: float) -> float:
    """Linear-interpolated percentile, q in [0, 100] (same as numpy's default)."""
    if not values:
        return math.nan
    xs = sorted(values)
    pos = (len(xs) - 1) * q / 100.0
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (pos - lo)


def ttc(dist: float, rel_speed: float) -> float | None:
    """Time to collision; rel_speed = lead - ego (negative when closing)."""
    closing = -rel_speed
    if closing <= MIN_CLOSING_MPS:
        return None
    return dist / closing


# -------------------------------------------------------------- detectors ---
def cycle_findings(name: str, times: list[float], period: float) -> tuple[CycleStats, list[Finding]]:
    """Inter-arrival statistics, missing frames (gaps) and timing violations."""
    ts = sorted(times)
    deltas = [b - a for a, b in zip(ts, ts[1:])]
    findings: list[Finding] = []
    missing = 0
    violations = 0
    tol = JITTER_TOL_FRAC * period
    for (a, b), d in zip(zip(ts, ts[1:]), deltas):
        if d > DROP_FACTOR * period:
            n_miss = max(1, round(d / period) - 1)
            missing += n_miss
            findings.append(Finding("missing", name, b, a, f"gap {d*1000:.1f} ms ~ {n_miss} frame(s)"))
        elif abs(d - period) > tol:
            violations += 1
            findings.append(Finding("timing", name, b, a, f"delta {d*1000:.2f} ms"))
    ms = [d * 1000.0 for d in deltas]
    stats = CycleStats(
        name=name, nominal_ms=period * 1000.0, n=len(ts),
        mean_ms=sum(ms) / len(ms) if ms else math.nan,
        p95_ms=percentile(ms, 95), max_ms=max(ms) if ms else math.nan,
        missing=missing, timing_violations=violations,
    )
    return stats, findings


def dlc_ok(name: str, data: bytes) -> bool:
    return len(data) == load_db().get_message_by_name(name).length


def checksum_ok(name: str, arbitration_id: int, data: bytes) -> bool:
    if not has_checksum(name):
        return True
    return data[-1] == toyota_checksum(arbitration_id, data)


def range_violations(name: str, decoded: dict[str, float]) -> list[tuple[str, float, float, float]]:
    limits = all_limits()
    out = []
    for sig in RANGE_CHECKED.get(name, ()):
        lo, hi = limits[(name, sig)]
        v = float(decoded[sig])
        if v < lo or v > hi:
            out.append((sig, v, lo, hi))
    return out


def counter_findings(name: str, seq: list[tuple[float, int]], period: float,
                     modulo: int) -> list[Finding]:
    """Rolling counter must advance by 1 per frame (mod `modulo`).

    A jump of k>1 is accepted only if the time since the previous frame is
    consistent with k cycles (i.e. frames were lost, which the gap detector
    reports separately).
    """
    out = []
    for (t0, c0), (t1, c1) in zip(seq, seq[1:]):
        step = (c1 - c0) % modulo
        if step == 1:
            continue
        if step > 1 and round((t1 - t0) / period) == step:
            continue
        out.append(Finding("counter", name, t1, t0, f"counter {c0} -> {c1}"))
    return out


def stuck_findings(speed: list[tuple[float, float]], wheels: list[tuple[float, float]]) -> list[Finding]:
    """SPEED.SPEED unchanged for >= STUCK_MIN_S while mean wheel speed moved."""
    out = []
    i = 0
    while i < len(speed):
        j = i
        while j + 1 < len(speed) and speed[j + 1][1] == speed[i][1]:
            j += 1
        t0, t1 = speed[i][0], speed[j][0]
        if t1 - t0 >= STUCK_MIN_S:
            ws = [w for (t, w) in wheels if t0 <= t <= t1]
            if ws and max(ws) - min(ws) > STUCK_WHEEL_DELTA_KPH:
                out.append(Finding("stuck", "SPEED", t1, t0,
                                   f"SPEED={speed[i][1]:.2f} for {(t1-t0)*1000:.0f} ms, "
                                   f"wheels moved {max(ws)-min(ws):.2f} km/h"))
        i = j + 1
    return out


def measure_latency(lead: list[tuple[float, float, float]],
                    pre_collision: list[tuple[float, int, float]],
                    brake: list[tuple[float, float]]) -> Latency:
    """Trigger = first LEAD_INFO frame whose TTC < threshold.

    request  = first PRE_COLLISION frame at/after the trigger with
               PRECOLLISION_ACTIVE and FORCE < 0 (openpilot's stockAeb rule);
    pressure = first BRAKE_MODULE frame at/after the trigger with
               BRAKE_PRESSURE >= PRESSURE_X.
    """
    trigger = None
    for t, dist, rel in lead:
        v = ttc(dist, rel)
        if v is not None and v < TTC_TRIGGER_S:
            trigger = t
            break
    if trigger is None:
        return Latency(None, None, None)
    request = next((t for t, act, force in pre_collision
                    if t >= trigger and act == 1 and force < -1e-5), None)
    pressure = next((t for t, p in brake if t >= trigger and p >= PRESSURE_X), None)
    return Latency(trigger, request, pressure)


def bus_load(frames: list[RxFrame], duration_s: float, bitrate: int = 500_000) -> tuple[float, float]:
    """Classic CAN 2.0A frame length in bits: 47 + 8*DLC without stuffing;
    worst case adds floor((34 + 8*DLC - 1) / 4) stuff bits."""
    if duration_s <= 0:
        return (0.0, 0.0)
    lo = sum(47 + 8 * len(f.data) for f in frames)
    hi = sum(47 + 8 * len(f.data) + (34 + 8 * len(f.data) - 1) // 4 for f in frames)
    return (100.0 * lo / (bitrate * duration_s), 100.0 * hi / (bitrate * duration_s))


# ----------------------------------------------------------------- driver ---
def analyze(frames: list[RxFrame]) -> Analysis:
    db = load_db()
    known = {db.get_message_by_name(m).frame_id: m for m in CYCLE_TIME_S}
    unknown: dict[int, int] = {}
    times: dict[str, list[float]] = {m: [] for m in CYCLE_TIME_S}
    findings: list[Finding] = []
    valid: dict[str, list[tuple[float, dict]]] = {m: [] for m in CYCLE_TIME_S}
    counter_seq: dict[str, list[tuple[float, int]]] = {m: [] for m in COUNTER_SIGNALS}

    for f in sorted(frames, key=lambda x: x.t):
        name = known.get(f.arbitration_id)
        if name is None:
            unknown[f.arbitration_id] = unknown.get(f.arbitration_id, 0) + 1
            continue
        times[name].append(f.t)
        if not dlc_ok(name, f.data):
            findings.append(Finding("dlc", name, f.t, f.t,
                                    f"DLC {len(f.data)} != {db.get_message_by_name(name).length}"))
            continue
        if not checksum_ok(name, f.arbitration_id, f.data):
            findings.append(Finding("checksum", name, f.t, f.t, f"checksum byte {f.data[-1]:#04x}"))
            continue
        decoded = db.get_message_by_name(name).decode(f.data, decode_choices=False)
        if name in counter_seq:
            sig, _ = COUNTER_SIGNALS[name]
            counter_seq[name].append((f.t, int(decoded[sig])))
        bad = range_violations(name, decoded)
        for sig, v, lo, hi in bad:
            findings.append(Finding("range", name, f.t, f.t, f"{sig}={v:g} outside [{lo:g}, {hi:g}]"))
        if not bad:
            valid[name].append((f.t, decoded))

    stats = []
    for name, period in CYCLE_TIME_S.items():
        st, fs = cycle_findings(name, times[name], period)
        stats.append(st)
        findings.extend(fs)

    for name, seq in counter_seq.items():
        _, modulo = COUNTER_SIGNALS[name]
        findings.extend(counter_findings(name, seq, CYCLE_TIME_S[name], modulo))

    speed = [(t, float(d["SPEED"])) for t, d in valid["SPEED"]]
    wheels = [(t, sum(float(d[f"WHEEL_SPEED_{w}"]) for w in ("FL", "FR", "RL", "RR")) / 4.0)
              for t, d in valid["WHEEL_SPEEDS"]]
    findings.extend(stuck_findings(speed, wheels))

    latency = measure_latency(
        [(t, float(d["LEAD_LONG_DIST"]), float(d["LEAD_REL_SPEED"])) for t, d in valid["LEAD_INFO"]],
        [(t, int(d["PRECOLLISION_ACTIVE"]), float(d["FORCE"])) for t, d in valid["PRE_COLLISION"]],
        [(t, float(d["BRAKE_PRESSURE"])) for t, d in valid["BRAKE_MODULE"]],
    )

    findings.sort(key=lambda x: (x.t, x.kind, x.name))
    counts: dict[str, int] = {}
    for fd in findings:
        counts[fd.kind] = counts.get(fd.kind, 0) + 1
    all_t = [f.t for f in frames]
    duration = (max(all_t) - min(all_t)) if all_t else 0.0
    return Analysis(len(frames), unknown, stats, findings, latency, bus_load(frames, duration), counts)
