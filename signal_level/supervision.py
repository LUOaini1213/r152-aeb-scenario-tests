"""Receive-side supervision of one message: E2E check per frame, plausibility, timeout, fault latch, recovery.

Layers (all parameters are PROJECT DESIGN CHOICES, see README):
  per frame    E2E_P01Check (e2e.py), called once per received frame (NewDataAvailable = TRUE).  Because of
               this event-driven call pattern the NONEWDATA path and the counter-based timeout of Profile 1
               are not used; with event-driven calls SyncCounterInit must be >= 1, otherwise the receiver
               does not re-lock after a counter jump until the 4-bit counter wraps (tests/test_e2e.py).
               Then the DBC [min|max] plausibility and an optional signal qualifier (e.g. VehSpdValid).
               Valid frame = status in {INITIAL, OK, OKSOMELOST} and plausible.
  per message  timeout   : no valid frame for more than timeout_cycles x cycle time
               E2E fault : max_consecutive_invalid invalid frames in a row
               either one latches ``faulted``; healing needs recovery_frames consecutive valid frames.
  usage        while not faulted the last valid value is used ("qualified hold"); ``degraded`` = holding,
               i.e. no valid frame for more than 1.5 cycles but not yet timed out.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from typing import Callable

from .dbc import Message
from .e2e import DATA_VALID, P01Checker, P01Config
from .vbus import Frame


@dataclass(frozen=True)
class RxConfig:
    timeout_cycles: int = 3
    max_consecutive_invalid: int = 2
    recovery_frames: int = 3
    e2e: bool = True                 # False: CRC / counter not checked (the "without E2E" experiments)
    plausibility: bool = True        # DBC [min|max] range check and signal qualifier
    max_delta_counter_init: int = 2  # tolerate up to 2 lost frames between two valid frames (OKSOMELOST)
    sync_counter_init: int = 1       # >= 1 needed with event-driven calls (re-lock after a counter jump)
    max_no_new_or_repeated_data: int = 15


def p01_config(msg: Message, rx: RxConfig) -> P01Config:
    """Profile 1 configuration of a DBC message: variant 1A positions, Data ID from the E2E_DataID attribute."""
    return P01Config(data_id=int(msg.attributes["E2E_DataID"]), data_id_mode="BOTH", counter_offset=8,
                     crc_offset=0, data_length=8 * msg.length, max_delta_counter_init=rx.max_delta_counter_init,
                     max_no_new_or_repeated_data=rx.max_no_new_or_repeated_data,
                     sync_counter_init=rx.sync_counter_init)


class RxMonitor:
    def __init__(self, msg: Message, cfg: RxConfig = RxConfig(), transparent: bool = False,
                 qualifier: Callable[[dict], bool] | None = None, t0: float = 0.0):
        self.msg = msg
        self.cycle_s = msg.cycle_time_ms / 1000.0
        self.cfg = cfg
        self.transparent = transparent
        self.qualifier = qualifier
        self.t0 = t0
        self.checker = P01Checker(p01_config(msg, cfg))
        self.data: dict | None = None
        self.last_valid_t: float | None = None
        self.ever_valid = False
        self.consecutive_valid = 0
        self.consecutive_invalid = 0
        self.faulted = False
        self.fault_kind = ""
        self.faults: list[tuple[float, str]] = []      # (t, kind) of each fault entry
        self.heals: list[float] = []
        self.status_counts: Counter = Counter()
        self.frames = 0
        self.corrupted_seen = 0
        self.corrupted_accepted = 0

    # ------------------------------------------------------------------ per frame
    def on_frame(self, t: float, frame: Frame) -> bool:
        self.frames += 1
        self.corrupted_seen += frame.corrupted
        if len(frame.data) != self.msg.length:
            self.status_counts["WRONGDLC"] += 1
            valid = False
        elif self.cfg.e2e:
            status = self.checker.check(frame.data)
            self.status_counts[status] += 1
            valid = status in DATA_VALID
        else:
            self.status_counts["UNCHECKED"] += 1
            valid = True
        phys = None
        if valid:
            phys = dict(frame.phys) if (self.transparent and frame.phys is not None) else self.msg.decode(frame.data)
            if self.cfg.plausibility:
                if self.msg.out_of_range(phys):
                    self.status_counts["IMPLAUSIBLE"] += 1
                    valid = False
                elif self.qualifier is not None and not self.qualifier(phys):
                    self.status_counts["QUALIFIER"] += 1
                    valid = False
        if valid:
            self.data, self.last_valid_t, self.ever_valid = phys, t, True
            self.consecutive_valid += 1
            self.consecutive_invalid = 0
            self.corrupted_accepted += frame.corrupted
        else:
            self.consecutive_valid = 0
            self.consecutive_invalid += 1
        return valid

    # ------------------------------------------------------------------ per task
    def timed_out(self, t: float) -> bool:
        ref = self.last_valid_t if self.last_valid_t is not None else self.t0
        return t - ref > self.cfg.timeout_cycles * self.cycle_s + 1e-9

    def poll(self, t: float) -> None:
        timeout = self.timed_out(t)
        e2e_bad = self.consecutive_invalid >= self.cfg.max_consecutive_invalid
        if not self.faulted and (timeout or e2e_bad):
            self.faulted, self.fault_kind = True, "timeout" if timeout else "e2e"
            self.faults.append((t, self.fault_kind))
            self.consecutive_valid = 0
        elif self.faulted and not timeout and not e2e_bad and self.consecutive_valid >= self.cfg.recovery_frames:
            self.faulted, self.fault_kind = False, ""
            self.heals.append(t)

    @property
    def usable(self) -> bool:
        return self.ever_valid and not self.faulted

    def age(self, t: float) -> float | None:
        return None if self.last_valid_t is None else t - self.last_valid_t

    def degraded(self, t: float) -> bool:
        a = self.age(t)
        return self.usable and a is not None and a > 1.5 * self.cycle_s + 1e-9
