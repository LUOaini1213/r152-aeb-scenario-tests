"""Minimal Vector-DBC parser and signal codec, written for this project (cantools / python-can are not used).

Scope - what the P5 signal-level test needs:
  * statements parsed: VERSION, BU_, BO_, SG_, CM_, BA_DEF_, BA_DEF_DEF_, BA_, VAL_.  Every other statement
    (VAL_TABLE_, SIG_VALTYPE_, BO_TX_BU_, ...) is skipped.  Multiplexer indicators (M / mN) are parsed and
    kept on the signal, but multiplexed decoding is NOT implemented (ValueError on encode/decode).
  * byte order: @1 = Intel (little endian, start bit = LSB), @0 = Motorola (big endian, start bit = MSB in
    the DBC bit numbering where bit 0 is the LSB of byte 0 and bit 7 its MSB; the signal continues towards
    less significant bits and wraps from bit 8k to bit 8(k+1)+7, the so-called sawtooth numbering).
  * value type: + unsigned, - two's-complement signed.  Float signals (SIG_VALTYPE_) are not supported.
  * physical = raw * factor + offset.
  * encode: round half away from zero; with ``saturate=True`` the physical value is clamped to [min|max]
    (when the DBC gives a range, i.e. min < max) and the raw value to the representable range; with
    ``saturate=False`` an out-of-range value raises ValueError.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path

_TOKEN = re.compile(r'"(?:[^"\\]|\\.)*"|[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|[A-Za-z_][A-Za-z0-9_]*|[;:,|@()\[\]]')
_SG = re.compile(r'^\s*SG_\s+(?P<name>\w+)\s*(?P<mux>M|m\d+M?)?\s*:\s*(?P<start>\d+)\s*\|\s*(?P<len>\d+)\s*@\s*'
                 r'(?P<order>[01])\s*(?P<sign>[+-])\s*\(\s*(?P<factor>[^,\s]+)\s*,\s*(?P<offset>[^)\s]+)\s*\)\s*'
                 r'\[\s*(?P<min>[^|\s]+)\s*\|\s*(?P<max>[^\]\s]+)\s*\]\s*"(?P<unit>[^"]*)"\s*(?P<rx>.*)$')
_BO = re.compile(r'^\s*BO_\s+(?P<id>\d+)\s+(?P<name>\w+)\s*:\s*(?P<dlc>\d+)\s+(?P<tx>\w+)')
_SEMI_KEYWORDS = {"CM_", "BA_DEF_", "BA_DEF_DEF_", "BA_", "VAL_", "VAL_TABLE_", "SIG_VALTYPE_", "BO_TX_BU_",
                  "BA_DEF_REL_", "BA_REL_", "BA_DEF_DEF_REL_", "SIG_GROUP_", "EV_", "ENVVAR_DATA_", "SGTYPE_",
                  "SIG_TYPE_REF_", "SIGTYPE_VALTYPE_", "SG_MUL_VAL_", "CAT_DEF_", "CAT_", "FILTER", "BA_SGTYPE_",
                  "BA_DEF_SGTYPE_", "SGTYPE_VAL_"}
EXTENDED_FLAG = 0x80000000


def round_half_away(x: float) -> int:
    """Round to the nearest integer, ties away from zero (Python's round() would round ties to even)."""
    return int(math.floor(x + 0.5)) if x >= 0 else -int(math.floor(-x + 0.5))


def _num(text: str) -> float:
    return float(text)


def _unquote(tok: str) -> str:
    return tok[1:-1].replace('\\"', '"') if tok.startswith('"') else tok


@dataclass
class Signal:
    name: str
    start: int
    length: int
    little_endian: bool             # True: @1 Intel, False: @0 Motorola
    signed: bool
    factor: float
    offset: float
    minimum: float
    maximum: float
    unit: str = ""
    receivers: list[str] = field(default_factory=list)
    multiplex: str | None = None
    comment: str = ""
    choices: dict[int, str] = field(default_factory=dict)
    # filled in by Message: position of the raw value inside the frame integer
    _shift: int = 0
    _mask: int = 0
    _le_int: bool = True            # which integer view of the frame the shift refers to

    # ------------------------------------------------------------------ raw range / scaling
    @property
    def raw_min(self) -> int:
        return -(1 << (self.length - 1)) if self.signed else 0

    @property
    def raw_max(self) -> int:
        return (1 << (self.length - 1)) - 1 if self.signed else (1 << self.length) - 1

    @property
    def has_range(self) -> bool:
        return self.maximum > self.minimum      # DBC convention: [0|0] = no range given

    def to_phys(self, raw: int) -> float:
        return raw * self.factor + self.offset

    def to_raw(self, value: float, saturate: bool = True) -> tuple[int, bool]:
        """Physical -> raw.  Returns (raw, saturated)."""
        if isinstance(value, bool):
            value = int(value)
        if not math.isfinite(value):
            raise ValueError(f"{self.name}: non-finite value {value!r}")
        sat = False
        if self.has_range and not (self.minimum - 1e-9 <= value <= self.maximum + 1e-9):
            if not saturate:
                raise ValueError(f"{self.name}: {value} outside [{self.minimum}|{self.maximum}]")
            value, sat = min(max(value, self.minimum), self.maximum), True
        raw = round_half_away((value - self.offset) / self.factor)
        if raw < self.raw_min or raw > self.raw_max:
            if not saturate:
                raise ValueError(f"{self.name}: raw {raw} outside [{self.raw_min}|{self.raw_max}]")
            raw, sat = min(max(raw, self.raw_min), self.raw_max), True
        return raw, sat

    # ------------------------------------------------------------------ bit positions
    def bit_positions(self) -> list[int]:
        """DBC bit numbers occupied by the signal, from raw bit 0 (LSB) upwards (reference definition)."""
        if self.little_endian:
            return [self.start + i for i in range(self.length)]
        seq, pos = [], self.start                 # Motorola: MSB first along the sawtooth
        for _ in range(self.length):
            seq.append(pos)
            pos = pos + 15 if pos % 8 == 0 else pos - 1
        return seq[::-1]

    def _place(self, dlc: int) -> None:
        self._mask = (1 << self.length) - 1
        if self.little_endian:
            self._le_int, self._shift = True, self.start
        else:
            msb_linear = 8 * (self.start // 8) + (7 - self.start % 8)     # 0 = MSB of byte 0
            lsb_linear = msb_linear + self.length - 1
            self._le_int, self._shift = False, 8 * dlc - 1 - lsb_linear
        top = max(p // 8 for p in self.bit_positions())
        if self._shift < 0 or top >= dlc:
            raise ValueError(f"signal {self.name} does not fit into {dlc} bytes")


@dataclass
class Message:
    frame_id: int
    name: str
    length: int
    sender: str
    signals: list[Signal] = field(default_factory=list)
    is_extended: bool = False
    comment: str = ""
    attributes: dict[str, object] = field(default_factory=dict)
    _by_name: dict[str, Signal] = field(default_factory=dict, repr=False)

    def finalize(self) -> None:
        self._by_name = {s.name: s for s in self.signals}
        for s in self.signals:
            s._place(self.length)

    def signal(self, name: str) -> Signal:
        return self._by_name[name]

    def check_layout(self) -> None:
        """Raise ValueError if two signals share a bit (multiplexed signals are not supported)."""
        used: dict[int, str] = {}
        for s in self.signals:
            for b in s.bit_positions():
                if b in used:
                    raise ValueError(f"{self.name}: {s.name} overlaps {used[b]} at bit {b}")
                used[b] = s.name

    # ------------------------------------------------------------------ codec
    def encode(self, values: dict[str, float], saturate: bool = True, raw: dict[str, int] | None = None) -> bytes:
        """Physical values (and optionally raw values) -> frame bytes.  Missing signals are sent as raw 0."""
        le = be = 0
        for s in self.signals:
            if s.multiplex:
                raise ValueError("multiplexed signals are not supported by this minimal codec")
            if raw is not None and s.name in raw:
                r = int(raw[s.name])
                if not s.raw_min <= r <= s.raw_max:
                    raise ValueError(f"{s.name}: raw {r} not representable in {s.length} bits")
            elif s.name in values:
                r, _ = s.to_raw(values[s.name], saturate)
            else:
                continue
            bits = (r & s._mask) << s._shift
            if s._le_int:
                le |= bits
            else:
                be |= bits
        return bytes(a | b for a, b in zip(le.to_bytes(self.length, "little"), be.to_bytes(self.length, "big")))

    def decode_raw(self, data: bytes) -> dict[str, int]:
        if len(data) != self.length:
            raise ValueError(f"{self.name}: DLC {len(data)} != {self.length}")
        le, be = int.from_bytes(data, "little"), int.from_bytes(data, "big")
        out = {}
        for s in self.signals:
            r = ((le if s._le_int else be) >> s._shift) & s._mask
            if s.signed and r >> (s.length - 1):
                r -= 1 << s.length
            out[s.name] = r
        return out

    def decode(self, data: bytes) -> dict[str, float]:
        return {n: self._by_name[n].to_phys(r) for n, r in self.decode_raw(data).items()}

    def out_of_range(self, phys: dict[str, float]) -> list[str]:
        """Signals whose decoded physical value violates the DBC [min|max] (receiver plausibility check)."""
        return [n for n, v in phys.items() if self._by_name[n].has_range
                and not (self._by_name[n].minimum - 1e-9 <= v <= self._by_name[n].maximum + 1e-9)]

    @property
    def cycle_time_ms(self) -> int | None:
        v = self.attributes.get("GenMsgCycleTime")
        return int(v) if v is not None else None


@dataclass
class AttributeDefinition:
    name: str
    object_type: str               # "" (network), BU_, BO_, SG_, EV_
    value_type: str                # INT, HEX, FLOAT, STRING, ENUM
    params: list = field(default_factory=list)
    default: object = None


@dataclass
class Database:
    version: str = ""
    nodes: list[str] = field(default_factory=list)
    messages: list[Message] = field(default_factory=list)
    comment: str = ""
    node_comments: dict[str, str] = field(default_factory=dict)
    attribute_definitions: dict[str, AttributeDefinition] = field(default_factory=dict)
    attributes: dict[str, object] = field(default_factory=dict)     # network attributes

    def get_message_by_name(self, name: str) -> Message:
        for m in self.messages:
            if m.name == name:
                return m
        raise KeyError(name)

    def get_message_by_id(self, frame_id: int) -> Message:
        for m in self.messages:
            if m.frame_id == frame_id:
                return m
        raise KeyError(frame_id)


# ====================================================================== parser
def _terminated(text: str) -> bool:
    in_str, esc = False, False
    for ch in text:
        if esc:
            esc = False
        elif ch == "\\" and in_str:
            esc = True
        elif ch == '"':
            in_str = not in_str
        elif ch == ";" and not in_str:
            return True
    return False


def _attr_value(defn: AttributeDefinition | None, tok: str):
    if tok.startswith('"'):
        return _unquote(tok)
    if defn is not None and defn.value_type in ("INT", "HEX", "ENUM"):
        v = int(float(tok))
        if defn.value_type == "ENUM" and 0 <= v < len(defn.params):
            return defn.params[v]
        return v
    return float(tok)


def parse_dbc(text: str) -> Database:
    db = Database()
    lines = text.splitlines()
    by_id: dict[int, Message] = {}
    cur: Message | None = None
    pending_attrs: list[list[str]] = []
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        first = stripped.split(None, 1)[0] if stripped else ""
        kw = first.rstrip(":")
        if not stripped:
            pass
        elif kw == "VERSION":
            m = re.search(r'"([^"]*)"', stripped)
            db.version = m.group(1) if m else ""
        elif kw == "NS_":
            while i + 1 < len(lines) and (not lines[i + 1].strip() or lines[i + 1][:1] in " \t"):
                i += 1
        elif kw == "BS_":
            pass
        elif kw == "BU_":
            db.nodes = stripped.split(":", 1)[1].split()
        elif kw == "BO_":
            m = _BO.match(line)
            if not m:
                raise ValueError(f"line {i + 1}: cannot parse BO_: {line!r}")
            raw_id = int(m.group("id"))
            cur = Message(frame_id=raw_id & 0x1FFFFFFF if raw_id & EXTENDED_FLAG else raw_id, name=m.group("name"),
                          length=int(m.group("dlc")), sender=m.group("tx"), is_extended=bool(raw_id & EXTENDED_FLAG))
            db.messages.append(cur)
            by_id[raw_id] = cur
        elif kw == "SG_":
            m = _SG.match(line)
            if not m or cur is None:
                raise ValueError(f"line {i + 1}: cannot parse SG_: {line!r}")
            rx = [r for r in re.split(r"[,\s]+", m.group("rx").strip()) if r]
            cur.signals.append(Signal(
                name=m.group("name"), start=int(m.group("start")), length=int(m.group("len")),
                little_endian=m.group("order") == "1", signed=m.group("sign") == "-",
                factor=_num(m.group("factor")), offset=_num(m.group("offset")),
                minimum=_num(m.group("min")), maximum=_num(m.group("max")), unit=m.group("unit"),
                receivers=rx, multiplex=m.group("mux")))
        elif kw in _SEMI_KEYWORDS:
            buf = line
            while not _terminated(buf) and i + 1 < len(lines):
                i += 1
                buf += "\n" + lines[i]
            toks = _TOKEN.findall(buf)
            _statement(db, by_id, toks, pending_attrs)
        i += 1
    for m in db.messages:
        m.finalize()
    for toks in pending_attrs:          # BA_ lines may precede their BA_DEF_ in hand-written files
        _apply_attribute(db, by_id, toks)
    for name, d in db.attribute_definitions.items():
        if d.object_type == "BO_" and d.default is not None:
            for m in db.messages:
                m.attributes.setdefault(name, d.default)
        if d.object_type == "" and d.default is not None:
            db.attributes.setdefault(name, d.default)
    return db


def _statement(db: Database, by_id: dict[int, Message], toks: list[str], pending: list[list[str]]) -> None:
    kw = toks[0]
    if kw == "CM_":
        if toks[1].startswith('"'):
            db.comment = _unquote(toks[1])
        elif toks[1] == "BU_":
            db.node_comments[toks[2]] = _unquote(toks[3])
        elif toks[1] == "BO_":
            by_id[int(toks[2])].comment = _unquote(toks[3])
        elif toks[1] == "SG_":
            msg = by_id[int(toks[2])]
            for s in msg.signals:
                if s.name == toks[3]:
                    s.comment = _unquote(toks[4])
    elif kw == "BA_DEF_":
        j = 1
        obj = ""
        if toks[j] in ("BU_", "BO_", "SG_", "EV_"):
            obj, j = toks[j], j + 1
        name, vtype = _unquote(toks[j]), toks[j + 1]
        rest = [t for t in toks[j + 2:] if t not in (",", ";")]
        params = [_unquote(t) for t in rest] if vtype == "ENUM" else [float(t) for t in rest]
        db.attribute_definitions[name] = AttributeDefinition(name, obj, vtype, params)
    elif kw == "BA_DEF_DEF_":
        name = _unquote(toks[1])
        d = db.attribute_definitions.get(name)
        if d is not None:
            tok = toks[2]
            d.default = _unquote(tok) if tok.startswith('"') else _attr_value(d, tok)
    elif kw == "BA_":
        pending.append(toks)
    elif kw == "VAL_":
        if len(toks) > 2 and re.fullmatch(r"\d+", toks[1]):
            msg = by_id.get(int(toks[1]))
            if msg is not None:
                sig = next((s for s in msg.signals if s.name == toks[2]), None)
                pairs = [t for t in toks[3:] if t != ";"]
                if sig is not None:
                    sig.choices = {int(float(pairs[k])): _unquote(pairs[k + 1]) for k in range(0, len(pairs) - 1, 2)}


def _apply_attribute(db: Database, by_id: dict[int, Message], toks: list[str]) -> None:
    name = _unquote(toks[1])
    d = db.attribute_definitions.get(name)
    if toks[2] == "BO_":
        by_id[int(toks[3])].attributes[name] = _attr_value(d, toks[4])
    elif toks[2] == "SG_":
        msg = by_id[int(toks[3])]
        msg.attributes[f"{toks[4]}.{name}"] = _attr_value(d, toks[5])
    elif toks[2] in ("BU_", "EV_"):
        db.attributes[f"{toks[3]}.{name}"] = _attr_value(d, toks[4])
    else:
        db.attributes[name] = _attr_value(d, toks[2])


def load_dbc(path: str | Path) -> Database:
    return parse_dbc(Path(path).read_text(encoding="utf-8"))
