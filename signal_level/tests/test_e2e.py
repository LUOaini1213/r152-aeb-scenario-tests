"""CRC-8 and E2E Profile 1 against the official AUTOSAR examples, plus the receiver status logic."""
import itertools
import random

import pytest

from signal_level import e2e
from signal_level.dbc import load_dbc
from signal_level.e2e import (INITIAL, INPUT_ERROR, NONEWDATA, OK, OKSOMELOST, REPEATED, SYNC, WRONGCRC,
                              WRONGSEQUENCE, P01Checker, P01Config, P01Protector, crc8_p01, crc8_sae_j1850,
                              p01_crc, p01_read_counter)
from signal_level.paths import DBC_PATH
from signal_level.supervision import RxConfig, p01_config

DB = load_dbc(DBC_PATH)


def test_crc8_sae_j1850_autosar_check_values():
    # AUTOSAR CP R24-11 SWS CRC Library 7.2.1.1 [SWS_Crc_00073] check 4Bh, [SWS_Crc_00074] result table
    assert crc8_sae_j1850(b"123456789") == 0x4B
    table = [("00000000", 0x59), ("F20183", 0x37), ("0FAA0055", 0x79), ("00FF5511", 0xB8),
             ("332255AABBCCDDEEFF", 0xCB), ("926B55", 0x8C), ("FFFFFFFF", 0x74)]
    for data, crc in table:
        assert crc8_sae_j1850(bytes.fromhex(data)) == crc, data
    assert crc8_p01(b"\x00") == 0x00                      # start/XOR 0x00 [PRS_E2E_00070]


@pytest.mark.parametrize("mode,expected", [
    ("BOTH", [(0xCC, 0x00), (0x91, 0x01)]),               # PRS R24-11 Tables 6.6 / 6.7
    ("ALT", [(0xCE, 0x00), (0x02, 0x01)]),                # Tables 6.8 / 6.9
    ("LOW", [(0xCE, 0x00), (0x93, 0x01)]),                # Tables 6.10 / 6.11
    ("NIBBLE", [(0x2A, 0x10), (0x77, 0x11)]),             # Tables 6.12 / 6.13
])
def test_profile1_official_protocol_examples(mode, expected):
    cfg = P01Config(data_id=0x123, data_id_mode=mode, counter_offset=8, crc_offset=0, data_id_nibble_offset=12,
                    data_length=64, max_delta_counter_init=1, max_no_new_or_repeated_data=15, sync_counter_init=0)
    prot = P01Protector(cfg)                               # Table 6.5: counter starts at 0
    for b0, b1 in expected:
        data = bytearray(8)                                # "data equals all zeros"
        prot.protect(data)
        assert list(data) == [b0, b1, 0, 0, 0, 0, 0, 0]


def test_counter_runs_0_to_14_and_skips_15():
    prot = P01Protector(P01Config(data_id=0x1120))
    seen = []
    for _ in range(40):
        d = bytearray(8)
        prot.protect(d)
        seen.append(d[1] & 0x0F)
    assert seen[:16] == list(range(15)) + [0] and 15 not in seen     # [PRS_E2E_00075]


def _frames(cfg, n):
    prot, out = P01Protector(cfg), []
    for k in range(n):
        d = bytearray(8)
        d[3] = k % 256                                     # some payload
        prot.protect(d)
        out.append(bytes(d))
    return out


def test_check_status_sequence():
    cfg = P01Config(data_id=0x123, max_delta_counter_init=1, sync_counter_init=0)
    f = _frames(cfg, 20)
    c = P01Checker(cfg)
    assert c.check(f[0]) == INITIAL
    assert c.check(f[1]) == OK
    assert c.check(f[1]) == REPEATED                       # same counter again (failed alive-counter check)
    assert c.check(f[3]) == OKSOMELOST and c.lost_data == 1
    assert c.check(f[7]) == WRONGSEQUENCE                  # jump of 4 > allowed (failed sequence check)
    bad = bytearray(f[8])
    bad[5] ^= 0x10
    assert c.check(bytes(bad)) == WRONGCRC
    c15 = bytearray(f[9])
    c15[1] |= 0x0F
    assert c.check(bytes(c15)) == INPUT_ERROR              # counter 15 is the error value (Figure 6.58)
    assert c.check(None) == NONEWDATA
    other = P01Checker(P01Config(data_id=0x10B0))          # masquerade: right bytes, wrong Data ID
    assert other.check(f[0]) == WRONGCRC


def test_event_driven_check_needs_sync_counter_init():
    """With one check call per received frame, SyncCounterInit = 0 never re-locks after a counter jump until
    the 4-bit counter wraps; SyncCounterInit = 1 re-locks after one SYNC frame."""
    def invalid_after_gap(sync_init):
        cfg = P01Config(data_id=0x123, max_delta_counter_init=2, sync_counter_init=sync_init)
        f = _frames(cfg, 40)
        c = P01Checker(cfg)
        for k in range(3):
            c.check(f[k])
        statuses = [c.check(f[k]) for k in range(7, 30)]   # frames 3..6 lost
        return statuses.index(OK), statuses[:3]
    n0, first0 = invalid_after_gap(0)
    n1, first1 = invalid_after_gap(1)
    assert n0 >= 10 and first0 == [WRONGSEQUENCE] * 3
    assert n1 == 2 and first1 == [WRONGSEQUENCE, SYNC, OK]


def _detected(cfg, data):
    rx = p01_read_counter(cfg, data)
    return rx >= 15 or data[cfg.crc_offset // 8] != p01_crc(cfg, data, rx)


def test_all_single_and_double_bit_errors_are_detected():
    rng = random.Random(5)
    for m in DB.messages:
        cfg = p01_config(m, RxConfig())
        prot = P01Protector(cfg)
        for _ in range(3):
            data = bytearray(rng.randrange(256) for _ in range(8))
            prot.protect(data)
            for bits in itertools.chain(itertools.combinations(range(64), 1), itertools.combinations(range(64), 2)):
                bad = bytearray(data)
                for b in bits:
                    bad[b // 8] ^= 1 << (b % 8)
                assert _detected(cfg, bad), (m.name, bits)


def test_data_ids_have_unique_crc():
    # PRS R24-11 9.8.2 [PRS_E2E_UC_00072]: double Data ID configuration -> CRC over {low, high} must differ
    crcs = {m.name: crc8_p01(bytes([m.attributes["E2E_DataID"] & 0xFF, m.attributes["E2E_DataID"] >> 8]))
            for m in DB.messages}
    assert len(set(crcs.values())) == len(crcs)


def test_dbc_e2e_signals_sit_where_profile1_writes():
    for m in DB.messages:
        cfg = p01_config(m, RxConfig())
        assert (cfg.crc_offset, cfg.counter_offset, cfg.data_id_mode) == (0, 8, "BOTH")    # variant 1A positions
        prot = P01Protector(cfg)
        crc_sig = next(s.name for s in m.signals if s.name.endswith("_CRC"))
        ctr_sig = next(s.name for s in m.signals if s.name.endswith("_AliveCtr"))
        for k in range(20):
            data = bytearray(m.encode({}))
            prot.protect(data)
            raw = m.decode_raw(bytes(data))
            assert raw[crc_sig] == data[0] and raw[ctr_sig] == k % 15


def test_profile1_config_rejects_invalid_layouts():
    with pytest.raises(ValueError):
        P01Config(data_id=1, counter_offset=6)
    with pytest.raises(ValueError):
        P01Config(data_id=0x1123, data_id_mode="NIBBLE")
    assert e2e.DATA_VALID == {OK, OKSOMELOST, INITIAL}
