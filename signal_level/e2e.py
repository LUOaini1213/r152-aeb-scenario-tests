"""CRC-8 (SAE J1850 polynomial 0x1D) and AUTOSAR E2E Profile 1 protect / check.

Sources actually read (official AUTOSAR PDFs from autosar.org, text extracted locally):
  [PRS]  AUTOSAR FO R24-11 "E2E Protocol Specification", Document ID 849 (AUTOSAR_FO_PRS_E2EProtocol)
         6.5      "Profile 1 is a Legacy Profile ... New Projects shall use Profile 11."
         6.5      [PRS_E2E_00865] counter 4 bit 0..14; Data ID 16 bit, included in the CRC
         6.5.1.1  [PRS_E2E_00075] counter starts at 0, +1 per send request, after 14 restarts at 0 (0xF skipped)
                  [PRS_E2E_00076] receiver cases: no new data, repeated, +1, +more within limit, +more than allowed
         6.5.1.2  [PRS_E2E_00163] Data ID inclusion modes BOTH / ALT / LOW / NIBBLE
         6.5.1.3  [PRS_E2E_00070] polynomial 0x1D (x8+x4+x3+x2+1), start value and XOR value 0x00
                  [PRS_E2E_00082] CRC over the Data ID byte(s) first, then all transmitted bytes except the CRC byte
                  [PRS_E2E_00640] NIBBLE: low byte of Data ID, then a 0x00 byte, then the user data
         6.5.2.1  Figure 6.54 E2E_P01Protect; 6.5.2.2 Figures 6.55/6.56 CRC over Data ID and data
         6.5.3.1  Figures 6.58, 6.60-6.63 E2E_P01Check
         6.5.4    [PRS_E2E_00866] protect state, [PRS_E2E_00867] check state, [PRS_E2E_00868] status values,
                  [PRS_E2E_00869] configuration
         6.5.5    Tables 6.4-6.13 protocol examples (reproduced by tests/test_e2e.py)
         6.1      variant 1A: CRC starts at bit 0, counter starts at bit 8
  [CRC]  AUTOSAR CP R24-11 "Specification of CRC Routines", Document ID 16 (AUTOSAR_CP_SWS_CRCLibrary)
         7.2.1.1  [SWS_Crc_00073] CRC8 SAE-J1850: width 8, polynomial 1Dh, initial value FFh, not reflected,
                  XOR value FFh, check 4Bh; [SWS_Crc_00074] table of check values (reproduced by the tests)

What is and is not claimed
  * The CRC computation, counter handling and the E2E_P01Check status logic follow the PRS flowcharts listed
    above and reproduce the official examples.  The way this project CALLS the check (once per received
    frame, NewDataAvailable = TRUE), the configuration values and the time-based timeout supervision on top
    (supervision.py) are project design.  No AUTOSAR stack, no conformance test suite: the complete scheme is
    called "Profile-1-like" and is NOT claimed conformant.
"""
from __future__ import annotations

from dataclasses import dataclass

CRC8_SAE_J1850_POLY = 0x1D


def _table(poly: int) -> tuple[int, ...]:
    out = []
    for byte in range(256):
        c = byte
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        out.append(c)
    return tuple(out)


_T1D = _table(CRC8_SAE_J1850_POLY)


def crc8_sae_j1850(data: bytes | bytearray, start: int = 0xFF, xor_out: int = 0xFF) -> int:
    """MSB-first CRC-8, polynomial 0x1D.  Defaults = AUTOSAR Crc_CalculateCRC8 [SWS_Crc_00073]."""
    crc = start & 0xFF
    for b in data:
        crc = _T1D[crc ^ b]
    return crc ^ xor_out


def crc8_p01(data: bytes | bytearray) -> int:
    """CRC as used by E2E Profile 1: polynomial 0x1D, start value 0x00, XOR value 0x00 [PRS_E2E_00070]."""
    return crc8_sae_j1850(data, start=0x00, xor_out=0x00)


# ================================================================ Profile 1
DATAID_MODES = ("BOTH", "ALT", "LOW", "NIBBLE")      # [PRS_E2E_00163]


@dataclass(frozen=True)
class P01Config:
    """[PRS_E2E_00869] members.  Defaults = variant 1A positions (6.1) and the example configuration of
    Table 6.4 except where this project needs other values (set by the caller, see supervision.py)."""
    data_id: int
    data_id_mode: str = "BOTH"
    counter_offset: int = 8          # bit offset; multiple of 4; offset % 8 == 0 -> low nibble (Figure 6.54)
    crc_offset: int = 0              # bit offset; multiple of 8
    data_id_nibble_offset: int = 12
    data_length: int = 64            # bits, multiple of 8, <= 256
    max_delta_counter_init: int = 1
    max_no_new_or_repeated_data: int = 15
    sync_counter_init: int = 0

    def __post_init__(self):
        if self.data_id_mode not in DATAID_MODES:
            raise ValueError(self.data_id_mode)
        if self.counter_offset % 4 or self.crc_offset % 8 or self.data_length % 8 or self.data_length > 256:
            raise ValueError("offsets / length violate PRS_E2E_00869")
        if self.data_id_mode == "NIBBLE" and self.data_id >> 12:
            raise ValueError("NIBBLE mode: high nibble of the Data ID must be 0 [PRS_E2E_00306]")
        # LOW mode: only the low byte enters the CRC.  [PRS_E2E_00169] asks for a 0x00 high byte, but the
        # official examples (Table 6.4: Data ID 0x123 for every mode) do not, so it is not enforced here.


def _nibble(data: bytes | bytearray, bit_offset: int) -> int:
    b = data[bit_offset // 8]
    return b & 0x0F if bit_offset % 8 == 0 else (b >> 4) & 0x0F


def _set_nibble(data: bytearray, bit_offset: int, value: int) -> None:
    i = bit_offset // 8
    if bit_offset % 8 == 0:
        data[i] = (data[i] & 0xF0) | (value & 0x0F)
    else:
        data[i] = (data[i] & 0x0F) | ((value << 4) & 0xF0)


def p01_read_counter(cfg: P01Config, data: bytes | bytearray) -> int:
    return _nibble(data, cfg.counter_offset)


def p01_data_id_bytes(cfg: P01Config, counter: int) -> bytes:
    """Bytes of the Data ID that enter the CRC (Figure 6.56 getDataIDCRC)."""
    lo, hi = cfg.data_id & 0xFF, (cfg.data_id >> 8) & 0xFF
    if cfg.data_id_mode == "BOTH":
        return bytes((lo, hi))
    if cfg.data_id_mode == "ALT":
        return bytes((lo,)) if counter % 2 == 0 else bytes((hi,))
    if cfg.data_id_mode == "LOW":
        return bytes((lo,))
    return bytes((lo, 0x00))                         # NIBBLE [PRS_E2E_00640]


def p01_crc(cfg: P01Config, data: bytes | bytearray, counter: int) -> int:
    """Figure 6.55: CRC over Data ID, then the bytes before and after the CRC byte, start/XOR 0x00."""
    c = cfg.crc_offset // 8
    n = cfg.data_length // 8
    return crc8_p01(p01_data_id_bytes(cfg, counter) + bytes(data[:c]) + bytes(data[c + 1:n]))


class P01Protector:
    """E2E_P01Protect (Figure 6.54) with its state [PRS_E2E_00866]."""

    def __init__(self, cfg: P01Config):
        self.cfg = cfg
        self.counter = 0

    def protect(self, data: bytearray) -> None:
        cfg = self.cfg
        _set_nibble(data, cfg.counter_offset, self.counter)
        if cfg.data_id_mode == "NIBBLE":
            _set_nibble(data, cfg.data_id_nibble_offset, (cfg.data_id >> 8) & 0x0F)
        data[cfg.crc_offset // 8] = p01_crc(cfg, data, self.counter)
        self.counter = (self.counter + 1) % 15         # 0xF skipped [PRS_E2E_00075]


# status values [PRS_E2E_00868]; INPUT_ERROR = return code E2E_E_INPUTERR_WRONG (received counter 15, Figure 6.58)
OK, NONEWDATA, WRONGCRC, SYNC, INITIAL, REPEATED, OKSOMELOST, WRONGSEQUENCE = (
    "OK", "NONEWDATA", "WRONGCRC", "SYNC", "INITIAL", "REPEATED", "OKSOMELOST", "WRONGSEQUENCE")
INPUT_ERROR = "INPUTERR_WRONG"
DATA_VALID = frozenset({OK, OKSOMELOST, INITIAL})    # project choice: statuses whose data the receiver uses


class P01Checker:
    """E2E_P01Check (Figures 6.58, 6.60-6.63) with the state members of [PRS_E2E_00867]."""

    def __init__(self, cfg: P01Config):
        self.cfg = cfg
        self.last_valid_counter = 0
        self.max_delta_counter = cfg.max_delta_counter_init
        self.wait_for_first_data = True
        self.lost_data = 0
        self.status = NONEWDATA
        self.sync_counter = 0
        self.no_new_or_repeated_data = 0

    def check(self, data: bytes | bytearray | None) -> str:
        """``data=None`` means NewDataAvailable = FALSE."""
        cfg = self.cfg
        self.max_delta_counter = min(self.max_delta_counter + 1, 14)
        if data is None:
            self._no_new_or_repeated()
            self.status = NONEWDATA
            return self.status
        rx = p01_read_counter(cfg, data)
        if rx >= 15:
            return INPUT_ERROR                          # state not updated (Figure 6.58)
        valid = data[cfg.crc_offset // 8] == p01_crc(cfg, data, rx)
        if cfg.data_id_mode == "NIBBLE":
            valid = valid and _nibble(data, cfg.data_id_nibble_offset) == (cfg.data_id >> 8) & 0x0F
        if not valid:
            self.status = WRONGCRC
        elif self.wait_for_first_data:
            self.wait_for_first_data = False
            self.max_delta_counter = cfg.max_delta_counter_init
            self.last_valid_counter = rx
            self.status = INITIAL
        else:
            self.status = self._process_counter(rx)
        return self.status

    def _process_counter(self, rx: int) -> str:                       # Figure 6.60
        last = self.last_valid_counter
        delta = rx - last if rx >= last else 15 + rx - last
        if delta == 0:
            self._no_new_or_repeated()
            return REPEATED
        if delta > self.max_delta_counter:
            self._wrong_sequence(rx)
            return WRONGSEQUENCE
        return self._ok_and_some_lost(rx, delta)

    def _no_new_or_repeated(self) -> None:                             # Figure 6.61
        if self.no_new_or_repeated_data < 14:
            self.no_new_or_repeated_data += 1

    def _wrong_sequence(self, rx: int) -> None:                        # Figure 6.62
        self.no_new_or_repeated_data = 0
        self.sync_counter = self.cfg.sync_counter_init
        if self.sync_counter > 0:
            self.max_delta_counter = self.cfg.max_delta_counter_init
            self.last_valid_counter = rx

    def _ok_and_some_lost(self, rx: int, delta: int) -> str:           # Figure 6.63
        self.max_delta_counter = self.cfg.max_delta_counter_init
        self.last_valid_counter = rx
        self.lost_data = delta - 1
        if self.no_new_or_repeated_data > self.cfg.max_no_new_or_repeated_data:
            self.sync_counter = self.cfg.sync_counter_init
            self.no_new_or_repeated_data = 0
            return SYNC
        if self.sync_counter > 0:
            self.sync_counter -= 1
            self.no_new_or_repeated_data = 0
            return SYNC
        self.no_new_or_repeated_data = 0
        return OK if delta == 1 else OKSOMELOST
