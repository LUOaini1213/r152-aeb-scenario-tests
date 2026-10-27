"""Virtual bus fault injection and the per-message receive supervision (timeout, E2E fault, recovery)."""
import pytest

from signal_level.dbc import load_dbc
from signal_level.e2e import OK, SYNC, WRONGCRC, WRONGSEQUENCE
from signal_level.ecus import TxNode, radar_values
from signal_level.paths import DBC_PATH
from signal_level.supervision import RxConfig, RxMonitor
from signal_level.vbus import BusFaults, Corruption, Frame, VirtualBus, flip_bits

DB = load_dbc(DBC_PATH)
RADAR = DB.get_message_by_name("RADAR_TGT")


def _frame(name="RADAR_TGT", t=0.0, sender="RADAR", data=bytes(8)):
    return Frame(name, 0x120, sender, data, t)


def _collect(bus, names=("RADAR_TGT", "EGO_SPEED")):
    got = []
    for n in names:
        bus.subscribe(n, lambda t, fr: got.append((t, fr)))
    return got


def test_latency_jitter_fifo_and_quantized_delivery():
    bus = VirtualBus(BusFaults(latency_s=0.010, jitter_s=0.030, seed=1))
    got = _collect(bus)
    for k in range(200):
        bus.send(_frame(t=k * 0.005))
    for k in range(0, 1000):
        bus.deliver(k * 0.002)
    rx = [fr.t_rx for _, fr in got]
    assert len(got) == 200 and rx == sorted(rx)                        # no overtaking within one ID
    assert all(fr.t_rx >= fr.t_tx + 0.010 - 1e-12 for _, fr in got)
    assert all(t >= fr.t_rx - 1e-9 and t - fr.t_rx < 0.002 + 1e-9 for t, fr in got)   # delivered at the next step


def test_random_loss_rate_and_nested_loss_sets():
    def lost(p):
        bus = VirtualBus(BusFaults(loss_prob=p, seed=4))
        return {k for k in range(20000) if not bus.send(_frame(t=k * 0.01))}
    l5, l20 = lost(0.05), lost(0.20)
    assert abs(len(l20) / 20000 - 0.20) < 0.015 and abs(len(l5) / 20000 - 0.05) < 0.008
    assert l5 <= l20                                                   # common random numbers


def test_burst_bus_off_and_message_filters():
    bus = VirtualBus(BusFaults(bursts=((1.0, 0.2),), bus_off=(("ESC", 2.0, 0.1),)))
    sent = {t: bus.send(_frame(t=t)) for t in (0.95, 1.0, 1.19, 1.2, 2.05)}
    assert sent == {0.95: True, 1.0: False, 1.19: False, 1.2: True, 2.05: True}
    assert not bus.send(_frame("EGO_SPEED", 2.05, sender="ESC")) and bus.send(_frame("EGO_SPEED", 2.1, sender="ESC"))
    only_radar = VirtualBus(BusFaults(loss_prob=1.0, loss_msgs=("RADAR_TGT",)))
    assert not only_radar.send(_frame()) and only_radar.send(_frame("EGO_SPEED", sender="ESC"))
    assert bus.stats["drop_burst"] == 2 and bus.stats["drop_bus_off"] == 1


def test_corruption_random_and_targeted():
    bus = VirtualBus(BusFaults(corrupt_prob=1.0, corrupt_bits=3, seed=2))
    got = _collect(bus)
    bus.send(_frame(data=bytes(8)))
    bus.deliver(0.0)
    assert got[0][1].corrupted and sum(bin(b).count("1") for b in got[0][1].data) == 3
    tb = VirtualBus(BusFaults(targeted=(Corruption("RADAR_TGT", 0.1, (20, 63)),)))
    got = _collect(tb)
    for t in (0.05, 0.10, 0.15):
        tb.send(_frame(t=t))
    tb.deliver(1.0)
    assert [fr.corrupted for _, fr in got] == [False, True, False]
    assert got[1][1].data == flip_bits(bytes(8), (20, 63)) == bytes([0, 0, 0x10, 0, 0, 0, 0, 0x80])


def test_frozen_sender_is_caught_only_by_the_alive_counter():
    tx = TxNode(RADAR)
    for e2e_on in (True, False):
        bus = VirtualBus(BusFaults(freeze=(("RADAR_TGT", 0.10, 0.2),)))
        mon = RxMonitor(RADAR, RxConfig(e2e=e2e_on))
        bus.subscribe("RADAR_TGT", mon.on_frame)
        for k in range(8):                       # distance shrinks 1 m per frame; frames at 0.10..0.25 frozen
            t = k * 0.05
            bus.send(tx.frame(t, radar_values((50.0 - k, -20.0))))
            bus.deliver(t)
            mon.poll(t)
            if t == pytest.approx(0.20):
                frozen_state = (mon.faulted, mon.data["TgtDist"])
        assert bus.stats["frozen"] == 4          # the 0.05 s frame (49 m, counter 1) is repeated 4 times
        if e2e_on:                               # REPEATED, REPEATED -> E2E fault at the 2nd repetition
            assert frozen_state == (True, pytest.approx(49.0)) and mon.status_counts["REPEATED"] == 4
        else:                                    # without the counter check the stale distance is used
            assert frozen_state == (False, pytest.approx(49.0)) and not mon.faulted


def _radar_stream(times, tx=None):
    tx = tx or TxNode(RADAR)
    return [tx.frame(t, radar_values((50.0, -5.0))) for t in times]


def test_timeout_boundary_hold_and_degraded():
    mon = RxMonitor(RADAR, RxConfig())
    frames = _radar_stream([k * 0.05 for k in range(10)])
    for fr in frames[:3]:                     # t = 0, 50, 100 ms received; 150, 200 ms lost
        mon.on_frame(fr.t_tx, fr)
    mon.poll(0.25)                            # age 150 ms = 3 cycles: not yet a timeout
    assert not mon.faulted and mon.usable and mon.degraded(0.25) and mon.data["TgtDist"] == pytest.approx(50.0)
    mon.poll(0.252)                           # age 152 ms > 3 x 50 ms
    assert mon.faulted and mon.fault_kind == "timeout" and not mon.usable


def test_recovery_needs_n_consecutive_valid_frames():
    mon = RxMonitor(RADAR, RxConfig(recovery_frames=3, sync_counter_init=1, max_delta_counter_init=2))
    frames = _radar_stream([k * 0.05 for k in range(20)])
    for fr in frames[:3]:
        mon.on_frame(fr.t_tx, fr)
        mon.poll(fr.t_tx)
    mon.poll(0.30)
    assert mon.faulted                        # frames 3..6 lost -> timeout
    statuses = []
    for fr in frames[7:13]:
        mon.on_frame(fr.t_tx, fr)
        mon.poll(fr.t_tx)
        statuses.append((mon.checker.status, mon.faulted))
    # counter jump -> WRONGSEQUENCE, one SYNC frame, then 3 valid frames are needed before healing
    assert statuses == [(WRONGSEQUENCE, True), (SYNC, True), (OK, True), (OK, True), (OK, False), (OK, False)]


def test_e2e_fault_after_two_consecutive_invalid_frames_and_plausibility():
    mon = RxMonitor(RADAR, RxConfig(max_consecutive_invalid=2))
    frames = _radar_stream([k * 0.05 for k in range(8)])
    mon.on_frame(0.0, frames[0])
    bad1 = Frame(**{**frames[1].__dict__, "data": flip_bits(frames[1].data, (30,)), "corrupted": True})
    mon.on_frame(0.05, bad1)
    mon.poll(0.05)
    assert mon.checker.status == WRONGCRC and not mon.faulted and mon.data["TgtDist"] == pytest.approx(50.0)
    bad2 = Frame(**{**frames[2].__dict__, "data": flip_bits(frames[2].data, (31,)), "corrupted": True})
    mon.on_frame(0.10, bad2)
    mon.poll(0.10)
    assert mon.faulted and mon.fault_kind == "e2e" and mon.corrupted_accepted == 0
    # a correctly protected but implausible frame (relative speed -102.4 m/s is outside [-70|70])
    tx = TxNode(RADAR)
    p = RxMonitor(RADAR, RxConfig())
    raw_bad = bytearray(RADAR.encode({}, raw={"TgtValid": 1, "TgtQuality": 3, "TgtDist": 400, "TgtRelSpd": -2048}))
    tx.protector.protect(raw_bad)
    assert not p.on_frame(0.0, Frame("RADAR_TGT", 0x120, "RADAR", bytes(raw_bad), 0.0))
    assert p.status_counts["IMPLAUSIBLE"] == 1
    # without E2E the corrupted frame is used
    noe2e = RxMonitor(RADAR, RxConfig(e2e=False))
    noe2e.on_frame(0.0, bad1)
    assert noe2e.corrupted_accepted == 1
