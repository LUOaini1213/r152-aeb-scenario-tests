"""P5 signal level: the R152 closed loop of P5 run through DBC encoding, a virtual CAN bus and E2E checks.

Modules
  dbc          minimal Vector-DBC parser + Intel/Motorola signal codec (written for this project)
  e2e          CRC-8 (SAE J1850 polynomial 0x1D) and AUTOSAR E2E Profile 1 protect/check
  vbus         in-process virtual bus with fault injection (latency, jitter, loss, bursts, corruption, bus-off)
  supervision  per-message receive monitor: E2E status, timeout, recovery after N valid frames
  ecus         radar / ESC / AEB / brake / HMI nodes around the vendored AEB state machine
  sl_loop      the closed loop (mirrors aebtest/closed_loop.py with the bus inserted)
  sl_experiments  experiment definitions shared by run_signal_level.py and the tests

Personal project, AI-assisted.  All communication parameters are design assumptions.
"""
