"""In-process virtual CAN bus with fault injection.

Not a physical CAN model: no arbitration, no bit timing, no error frames.  A frame sent at t_tx is
delivered to every subscriber of its message at t_rx = t_tx + latency + U(0, jitter), never before an
earlier frame of the same message (FIFO per ID).  The closed loop calls ``deliver(t)`` every plant step,
so delivery is quantized to the step (2 ms in P5).

Faults (all optional, all deterministic for a given seed):
  loss         i.i.d. Bernoulli loss per frame.  The same seed gives nested loss sets for increasing
               probabilities (a frame lost at 5 % is also lost at 10 %), i.e. common random numbers.
  bursts       every frame sent inside [t_start, t_start + duration) is lost (bus-wide disturbance)
  corruption   random bit flips (corrupt_prob per frame, corrupt_bits flips) or targeted flips of given
               bits in given frames.  Stands for errors that the CAN-level CRC does not catch (e.g. inside a
               gateway or ECU memory); it is a fault-injection abstraction, not a bit-error-rate model.
  bus-off      a node (or "*" = all) cannot transmit inside [t_start, t_start + duration) (duration
               includes its recovery time; design assumption).  Simplification: only the node's transmission
               is blocked, its reception is not modelled as interrupted.
  freeze       stuck sender: inside the window the message repeats the last frame sent before it (same
               counter and CRC), i.e. the "repetition" fault that only the alive counter can reveal
"""
from __future__ import annotations

import heapq
import math
import random
from collections import Counter, defaultdict
from dataclasses import dataclass, replace
from typing import Callable


@dataclass
class Frame:
    name: str
    can_id: int
    sender: str
    data: bytes
    t_tx: float
    phys: dict | None = None        # physical values before quantization (used only by the transparent mode)
    t_rx: float = 0.0
    corrupted: bool = False


@dataclass(frozen=True)
class Corruption:
    """Flip ``bits`` (DBC bit numbers: byte = b // 8, bit = b % 8) in the first ``count`` frames of ``msg``
    sent at t >= t_from."""
    msg: str
    t_from: float
    bits: tuple[int, ...]
    count: int = 1


@dataclass(frozen=True)
class BusFaults:
    latency_s: float = 0.0
    jitter_s: float = 0.0
    loss_prob: float = 0.0
    loss_msgs: tuple[str, ...] | None = None          # None = all messages
    bursts: tuple[tuple[float, float], ...] = ()      # (t_start, duration)
    burst_msgs: tuple[str, ...] | None = None
    corrupt_prob: float = 0.0
    corrupt_bits: int = 1
    corrupt_msgs: tuple[str, ...] | None = None
    targeted: tuple[Corruption, ...] = ()
    bus_off: tuple[tuple[str, float, float], ...] = ()  # (node or "*", t_start, duration)
    freeze: tuple[tuple[str, float, float], ...] = ()   # (message, t_start, duration): stuck sender
    seed: int = 0

    def with_(self, **kw) -> "BusFaults":
        return replace(self, **kw)


def flip_bits(data: bytes, bits) -> bytes:
    b = bytearray(data)
    for bit in bits:
        b[bit // 8] ^= 1 << (bit % 8)
    return bytes(b)


def _in(windows, t: float) -> bool:
    return any(t0 - 1e-12 <= t < t0 + d - 1e-12 for t0, d in windows)


class VirtualBus:
    def __init__(self, faults: BusFaults = BusFaults()):
        self.f = faults
        self._rng_loss = random.Random(1_000_003 * faults.seed + 11)
        self._rng_jit = random.Random(1_000_003 * faults.seed + 23)
        self._rng_cor = random.Random(1_000_003 * faults.seed + 37)
        self._queue: list[tuple[float, int, Frame]] = []
        self._last_rx: dict[str, float] = {}
        self._subs: dict[str, list[Callable[[float, Frame], None]]] = defaultdict(list)
        self._seq = 0
        self._targeted_left = [c.count for c in faults.targeted]
        self._last_sent: dict[str, Frame] = {}
        self.stats: Counter = Counter()

    def subscribe(self, msg: str, callback: Callable[[float, Frame], None]) -> None:
        self._subs[msg].append(callback)

    def send(self, frame: Frame) -> bool:
        """Put a frame on the bus.  Returns False if it was dropped (bus-off, burst or random loss)."""
        f, t = self.f, frame.t_tx
        self.stats["tx"] += 1
        if f.freeze:
            if any(m == frame.name and t0 - 1e-12 <= t < t0 + d - 1e-12 for m, t0, d in f.freeze):
                old = self._last_sent.get(frame.name)
                if old is not None:
                    frame = Frame(frame.name, frame.can_id, frame.sender, old.data, t, old.phys)
                    self.stats["frozen"] += 1
            else:
                self._last_sent[frame.name] = frame
        if any(node in ("*", frame.sender) and t0 - 1e-12 <= t < t0 + d - 1e-12 for node, t0, d in f.bus_off):
            self.stats["drop_bus_off"] += 1
            return False
        if f.bursts and (f.burst_msgs is None or frame.name in f.burst_msgs) and _in(f.bursts, t):
            self.stats["drop_burst"] += 1
            return False
        if f.loss_prob > 0.0 and (f.loss_msgs is None or frame.name in f.loss_msgs):
            if self._rng_loss.random() < f.loss_prob:
                self.stats["drop_random"] += 1
                return False
        data, corrupted = frame.data, False
        if f.corrupt_prob > 0.0 and (f.corrupt_msgs is None or frame.name in f.corrupt_msgs):
            if self._rng_cor.random() < f.corrupt_prob:
                data, corrupted = flip_bits(data, self._rng_cor.sample(range(8 * len(data)), f.corrupt_bits)), True
        for i, c in enumerate(f.targeted):
            if c.msg == frame.name and t >= c.t_from - 1e-12 and self._targeted_left[i] > 0:
                data, corrupted = flip_bits(data, c.bits), True
                self._targeted_left[i] -= 1
        if corrupted:
            self.stats["corrupted"] += 1
        t_rx = t + f.latency_s + (self._rng_jit.uniform(0.0, f.jitter_s) if f.jitter_s > 0.0 else 0.0)
        t_rx = max(t_rx, self._last_rx.get(frame.name, -math.inf))      # no overtaking within one ID
        self._last_rx[frame.name] = t_rx
        out = Frame(frame.name, frame.can_id, frame.sender, data, frame.t_tx, frame.phys, t_rx, corrupted)
        self._seq += 1
        heapq.heappush(self._queue, (t_rx, self._seq, out))
        self.stats["queued"] += 1
        return True

    def deliver(self, t: float) -> int:
        """Hand every frame with t_rx <= t to its subscribers (in t_rx order); returns the count."""
        n = 0
        while self._queue and self._queue[0][0] <= t + 1e-9:
            _, _, fr = heapq.heappop(self._queue)
            for cb in self._subs.get(fr.name, ()):
                cb(t, fr)
            n += 1
        self.stats["delivered"] += n
        return n

    def pending(self) -> int:
        return len(self._queue)


__all__ = ["Frame", "Corruption", "BusFaults", "VirtualBus", "flip_bits"]
