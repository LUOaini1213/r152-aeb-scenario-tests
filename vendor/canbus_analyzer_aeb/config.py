"""Static configuration: which DBC messages are used, cycle times, limits.

The opendbc DBC carries NO GenMsgCycleTime attributes (cantools reports
cycle_time=None for every message used here), so the nominal cycle times
below are ASSUMPTIONS for this synthetic scenario, not values from the DBC
or from a real vehicle.
"""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DBC_PATH = ROOT / "dbc" / "toyota_nodsu_pt_generated.dbc"
RESULTS_DIR = ROOT / "results"

# Fixed epoch for logical time (2026-01-01T00:00:00Z) so ASC/BLF headers are sane
# and runs are reproducible.  All scenario times are seconds after this epoch.
EPOCH = 1767225600.0

# Nominal cycle time per message [s] -- assumed (see module docstring).
CYCLE_TIME_S: dict[str, float] = {
    "WHEEL_SPEEDS": 0.010,
    "SPEED": 0.020,
    "BRAKE_MODULE": 0.025,
    "ACC_CONTROL": 0.030,
    "PRE_COLLISION": 0.030,
    "LEAD_INFO": 0.050,
}

# Messages whose DBC defines a CHECKSUM signal; opendbc computes it with
# toyota_checksum() (opendbc/car/toyota/toyotacan.py) for every toyota_* DBC.
CHECKSUM_MESSAGES = ("SPEED", "LEAD_INFO", "ACC_CONTROL", "PRE_COLLISION")

# Rolling counter: PRE_COLLISION._COUNTER is 8 bit.  NOTE: in opendbc this field
# is called `_COUNTER` (not `COUNTER`), so openpilot's own parser does not
# validate it; here we *treat* it as a rolling counter in the synthetic traffic.
COUNTER_SIGNALS: dict[str, tuple[str, int]] = {"PRE_COLLISION": ("_COUNTER", 256)}

# Physical plausibility limits used by the range check.  Default is the DBC
# [min|max]; overrides are listed with the reason.
RANGE_OVERRIDES: dict[tuple[str, str], tuple[float, float]] = {
    # DBC gives [0|0] (i.e. unspecified) for wheel speeds.
    **{("WHEEL_SPEEDS", f"WHEEL_SPEED_{w}"): (0.0, 250.0) for w in ("FL", "FR", "RL", "RR")},
    # DBC says [0|255] N, but openpilot's carstate treats FORCE < 0 as stock AEB,
    # so the DBC range is not physically meaningful.  Limit chosen for this scenario.
    ("PRE_COLLISION", "FORCE"): (-10000.0, 0.0),
}
# Signals the range check looks at (others are constant/bit fields here).
RANGE_CHECKED: dict[str, tuple[str, ...]] = {
    "SPEED": ("SPEED",),
    "WHEEL_SPEEDS": ("WHEEL_SPEED_FL", "WHEEL_SPEED_FR", "WHEEL_SPEED_RL", "WHEEL_SPEED_RR"),
    "BRAKE_MODULE": ("BRAKE_PRESSURE",),
    "LEAD_INFO": ("LEAD_LONG_DIST", "LEAD_REL_SPEED"),
    "ACC_CONTROL": ("ACCEL_CMD",),
    "PRE_COLLISION": ("FORCE",),
}

# --- analyzer thresholds -----------------------------------------------------
DROP_FACTOR = 1.5          # gap > 1.5 x nominal cycle => missing frame(s)
JITTER_TOL_FRAC = 0.20     # |delta - T| > 20 % of T => timing violation
TTC_TRIGGER_S = 1.8        # AEB trigger threshold on time-to-collision
MIN_CLOSING_MPS = 0.1      # TTC only defined when closing faster than this
PRESSURE_X = 200           # "brake pressure reached X" (raw BRAKE_PRESSURE units)
STUCK_MIN_S = 0.20         # SPEED unchanged for >= this long ...
STUCK_WHEEL_DELTA_KPH = 1.0  # ... while mean wheel speed moved more than this
