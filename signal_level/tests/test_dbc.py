"""DBC parser and signal codec: parsing, hand-checked byte vectors, an independent bit-by-bit reference,
random round trips, boundary and saturation values."""
import random

import pytest

from signal_level.dbc import load_dbc, parse_dbc, round_half_away
from signal_level.paths import DBC_PATH

DB = load_dbc(DBC_PATH)

SYNTH = '''VERSION ""

NS_ :
	CM_
	BA_DEF_

BS_:

BU_: A B

BO_ 100 T1: 8 A
 SG_ M16 : 7|16@0+ (1,0) [0|0] "" B
 SG_ I16 : 16|16@1+ (1,0) [0|0] "" B
 SG_ S4 : 32|4@1- (1,0) [0|0] "" B
 SG_ U4 : 36|4@1+ (1,0) [0|0] "" B
 SG_ M12S : 43|12@0- (0.5,-10) [0|0] "" B

BO_ 2147484672 EXT: 8 A
 SG_ X : 0|8@1+ (1,0) [0|255] "" B

BO_TX_BU_ 100 : A,B;
CM_ SG_ 100 M16 "first line
second line with a ; inside quotes";
BA_ "GenMsgCycleTime" BO_ 100 100;
BA_DEF_ BO_ "GenMsgCycleTime" INT 0 65535;
BA_DEF_DEF_ "GenMsgCycleTime" 0;
VAL_TABLE_ Foo 1 "one" 0 "zero" ;
VAL_ 100 U4 0 "zero" 15 "max" ;
'''


def ref_positions(start, length, little_endian):
    """Independent reference of the DBC bit numbering; index i = raw bit i (LSB first)."""
    if little_endian:
        return [start + i for i in range(length)]
    seq, pos = [], start
    for _ in range(length):              # Motorola: from the MSB, down within a byte, wrap to bit 7 of the next
        seq.append(pos)
        pos = pos + 15 if pos % 8 == 0 else pos - 1
    return seq[::-1]


def ref_encode(signals_raw, dlc):
    data = [0] * dlc
    for sig, raw in signals_raw:
        u = raw & ((1 << sig.length) - 1)
        for i, p in enumerate(ref_positions(sig.start, sig.length, sig.little_endian)):
            if (u >> i) & 1:
                data[p // 8] |= 1 << (p % 8)
    return bytes(data)


# ------------------------------------------------------------------------------------------- parsing
def test_parse_demo_dbc():
    assert DB.version.startswith("P5-signal-level")
    assert DB.nodes == ["RADAR", "ESC", "AEB_ECU", "BRAKE_ECU", "HMI"]
    assert [(m.name, m.frame_id, m.length, m.sender) for m in DB.messages] == [
        ("AEB_CMD", 0xA0, 8, "AEB_ECU"), ("EGO_SPEED", 0xB0, 8, "ESC"), ("RADAR_TGT", 0x120, 8, "RADAR")]
    assert {m.name: m.cycle_time_ms for m in DB.messages} == {"AEB_CMD": 10, "EGO_SPEED": 20, "RADAR_TGT": 50}
    assert {m.name: m.attributes["E2E_DataID"] for m in DB.messages} == {
        "AEB_CMD": 0x10A0, "EGO_SPEED": 0x10B0, "RADAR_TGT": 0x1120}
    assert DB.get_message_by_name("AEB_CMD").attributes["GenMsgSendType"] == "Cyclic"
    assert DB.attributes["BusType"] == "CAN"
    d = DB.get_message_by_name("RADAR_TGT").signal("TgtRelSpd")
    assert (d.start, d.length, d.little_endian, d.signed, d.factor, d.minimum, d.maximum, d.unit) == \
        (35, 12, False, True, 0.05, -70.0, 70.0, "m/s")
    assert DB.get_message_by_name("AEB_CMD").signal("AEB_State").choices[5] == "FAULT"
    assert DB.get_message_by_name("AEB_CMD").signal("AEB_AccelReq").receivers == ["BRAKE_ECU"]
    for m in DB.messages:
        m.check_layout()                                   # no overlapping bits


def test_every_cycle_time_and_resolution_is_labelled_as_design_assumption():
    assert "DESIGN ASSUMPTION" in DB.comment and "SYNTHETIC" in DB.comment
    for m in DB.messages:
        assert "DESIGN ASSUMPTION" in m.comment and f"{m.cycle_time_ms} ms" in m.comment, m.name
        for s in m.signals:
            if s.unit:                                     # physical signals: resolution must be labelled
                assert "DESIGN ASSUMPTION" in s.comment and f"{s.factor:g}" in s.comment, s.name


def test_parser_edge_cases():
    db = parse_dbc(SYNTH)
    t1 = db.get_message_by_name("T1")
    assert t1.cycle_time_ms == 100                         # BA_ before its BA_DEF_ is still applied
    assert t1.signal("M16").comment == "first line\nsecond line with a ; inside quotes"
    assert t1.signal("U4").choices == {0: "zero", 15: "max"}
    ext = db.get_message_by_name("EXT")
    assert ext.is_extended and ext.frame_id == 0x400 and ext.cycle_time_ms == 0     # default from BA_DEF_DEF_


def test_overlap_and_fit_are_detected():
    bad = SYNTH.replace(' SG_ U4 : 36|4@1+', ' SG_ U4 : 34|4@1+')
    with pytest.raises(ValueError, match="overlaps"):
        parse_dbc(bad).get_message_by_name("T1").check_layout()
    with pytest.raises(ValueError, match="does not fit"):
        parse_dbc(SYNTH.replace("BO_ 100 T1: 8 A", "BO_ 100 T1: 6 A"))


# ---------------------------------------------------------------------------- hand-checked vectors
def test_known_vectors_motorola_and_intel():
    # RADAR_TGT (Motorola): 26.67 m -> raw 533 = 0x215 -> byte2 low 5 bits = 0x02, byte3 = 0x15;
    # -16.67 m/s -> raw -333 -> 12-bit 0xEB3 -> byte4 low nibble 0xE, byte5 0xB3; valid=1, quality=3 -> byte1 0xE0
    r = DB.get_message_by_name("RADAR_TGT")
    assert r.encode({"TgtValid": 1, "TgtQuality": 3, "TgtDist": 26.67, "TgtRelSpd": -16.67}).hex() == "00e002150eb30000"
    # AEB_CMD (Intel): -9.00 m/s2 -> raw -900 -> 0xC7C at bits 16..27; WarnLvl 2 at bits 28..29; State 4 at
    # bits 12..14; BrkReqActive at bit 15
    a = DB.get_message_by_name("AEB_CMD")
    assert a.encode({"AEB_AccelReq": -9.0, "AEB_State": 4, "AEB_WarnLvl": 2, "AEB_BrkReqActive": 1}).hex() == \
        "00c07c2c00000000"
    # EGO_SPEED (Intel): 60.00 km/h -> raw 6000 = 0x1770 -> bytes 2,3 = 70 17; VehSpdValid at bit 12
    e = DB.get_message_by_name("EGO_SPEED")
    assert e.encode({"VehSpd": 60.0, "VehSpdValid": 1}).hex() == "0010701700000000"
    # synthetic: Motorola 16 @7, Intel 16 @16, signed/unsigned nibbles, Motorola signed 12 bit with factor/offset
    t1 = parse_dbc(SYNTH).get_message_by_name("T1")
    data = t1.encode({"M16": 0x1234, "I16": 0x1234, "S4": -3, "U4": 9, "M12S": -510.0})
    assert data == bytes([0x12, 0x34, 0x34, 0x12, 0x9D, 0x0C, 0x18, 0x00])
    assert t1.decode(data) == {"M16": 0x1234, "I16": 0x1234, "S4": -3, "U4": 9, "M12S": -510.0}


def test_fast_codec_equals_bitwise_reference():
    rng = random.Random(7)
    msgs = list(DB.messages) + [parse_dbc(SYNTH).get_message_by_name("T1")]
    for m in msgs:
        for _ in range(300):
            raws = {}
            for s in m.signals:
                raws[s.name] = rng.choice([s.raw_min, s.raw_max, 0, rng.randint(s.raw_min, s.raw_max)])
            data = m.encode({}, raw=raws)
            assert data == ref_encode([(m.signal(n), r) for n, r in raws.items()], m.length), m.name
            assert m.decode_raw(data) == raws


# ---------------------------------------------------------------------------- round trips / limits
def test_random_round_trips_within_half_lsb():
    rng = random.Random(3)
    for m in DB.messages:
        for _ in range(500):
            vals = {s.name: rng.uniform(s.minimum, s.maximum) for s in m.signals}
            dec = m.decode(m.encode(vals))
            for s in m.signals:
                assert abs(dec[s.name] - vals[s.name]) <= s.factor / 2 + 1e-9, s.name


def test_boundaries_saturation_and_rounding():
    r = DB.get_message_by_name("RADAR_TGT")
    dist, rel = r.signal("TgtDist"), r.signal("TgtRelSpd")
    assert dist.to_raw(409.55) == (8191, False) and dist.to_raw(0.0) == (0, False)
    assert dist.to_raw(500.0) == (8191, True) and dist.to_raw(-1.0) == (0, True)        # saturate to [min|max]
    assert rel.to_raw(-70.0) == (-1400, False) and rel.to_raw(-80.0) == (-1400, True) and rel.to_raw(80.0) == (1400, True)
    assert r.decode(r.encode({"TgtRelSpd": -70.0}))["TgtRelSpd"] == pytest.approx(-70.0)
    assert dist.to_raw(0.025) == (1, False) and rel.to_raw(-0.025) == (-1, False)       # ties away from zero
    assert dist.to_raw(0.0249) == (0, False)
    assert round_half_away(2.5) == 3 and round_half_away(-2.5) == -3 and round_half_away(-2.4) == -2
    with pytest.raises(ValueError):
        r.encode({"TgtDist": 500.0}, saturate=False)
    a = DB.get_message_by_name("AEB_CMD")
    for decel in (4.0, 5.0, 6.0, 9.0):                    # every request level of the project is exact on the bus
        assert a.decode(a.encode({"AEB_AccelReq": -decel}))["AEB_AccelReq"] == pytest.approx(-decel, abs=1e-12)
    assert a.decode(a.encode({"AEB_AccelReq": -20.0}))["AEB_AccelReq"] == pytest.approx(-16.0)
    # raw-range saturation where the DBC gives no physical range ([0|0])
    t1 = parse_dbc(SYNTH).get_message_by_name("T1")
    assert t1.signal("U4").to_raw(20) == (15, True) and t1.signal("S4").to_raw(-9) == (-8, True)
    assert t1.signal("S4").to_raw(7) == (7, False) and t1.signal("S4").to_raw(8) == (7, True)
    with pytest.raises(ValueError):
        t1.encode({"U4": 16}, saturate=False)
    with pytest.raises(ValueError):
        t1.encode({}, raw={"S4": 8})                        # raw values must be representable
    with pytest.raises(ValueError):
        r.decode(b"\x00" * 7)                               # wrong DLC
