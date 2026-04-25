# =============================================================================
# protocol.py — Packet builder, checksum verifier, and parser
# MCT333/MCT344 — Mechatronic Systems Design, Spring 2026, Ain Shams University
# Modular Omni-Wheel Mobile Manipulator
# =============================================================================
#
# This file contains ONLY generic byte-level logic.
# It has no knowledge of the Base MCU, Arm MCU, or any robot-specific messages.
# All robot-specific packet builders live in base_uart.py and arm_uart.py.
#
# Packet format:
#   [ 0xFF | MSG_ID | DATA_LENGTH | DATA BYTES... | CHECKSUM ]
#
#   START_BYTE   : always 0xFF — signals beginning of packet
#   MSG_ID       : 1 byte — identifies the message type (from config.py)
#   DATA_LENGTH  : 1 byte — number of data bytes that follow (0 if no payload)
#   DATA BYTES   : 0 to N bytes — the actual payload
#   CHECKSUM     : 1 byte — sum of MSG_ID + DATA_LENGTH + all DATA bytes,
#                           masked to 8 bits (& 0xFF)
#
# Multi-byte value encoding:
#   Floats cannot fit in one byte, so they are scaled and split across two bytes.
#   Encode: multiply float by 1000 → integer → split into high byte + low byte
#   Decode: combine high byte + low byte → integer → divide by 1000 → float
#
#   Signed   : values that can be negative (velocities, coordinates)
#              uses two's complement via struct.pack('>h', ...)
#   Unsigned : values that are always positive (distances from ToF sensors)
#              uses simple bit-shift
#
# =============================================================================

import struct
from config import START_BYTE


# =============================================================================
# Encoding helpers — convert float values to/from scaled two-byte integers
# =============================================================================

def encode_signed(value: float, scale: int = 1000) -> tuple[int, int]:
    """
    Encode a signed float into two bytes (high, low) using two's complement.

    Use for values that can be negative: velocities (vx, vy, omega),
    coordinates (x, y, z), angles (roll, pitch, yaw).

    How it works:
        -1.234  x 1000  =  -1234  ->  struct.pack('>h', -1234)  ->  (high, low)

    Args:
        value : signed float to encode
        scale : multiply before converting to int — default 1000 gives 3 decimal
                places of precision (e.g. 1.234 m/s -> 1234)

    Returns:
        (high_byte, low_byte) — each is an int from 0x00 to 0xFF

    Range with default scale:
        +/-32.767 (since signed 16-bit holds +/-32767, divide by 1000)
    """
    int_val = int(round(value * scale))
    packed  = struct.pack('>h', int_val)  # '>h' = big-endian signed 16-bit
    return packed[0], packed[1]


def decode_signed(high: int, low: int, scale: int = 1000) -> float:
    """
    Decode two bytes (high, low) back into a signed float.

    Use this to decode any value that was encoded with encode_signed.

    Args:
        high  : high byte (0x00-0xFF)
        low   : low byte  (0x00-0xFF)
        scale : divide by this to recover float — must match encode_signed scale

    Returns:
        Reconstructed signed float
    """
    packed  = bytes([high, low])
    int_val = struct.unpack('>h', packed)[0]  # '>h' = big-endian signed 16-bit
    return int_val / scale


def encode_unsigned(value: float, scale: int = 1000) -> tuple[int, int]:
    """
    Encode a non-negative float into two bytes (high, low).

    Use for values that are always zero or positive: distances from ToF sensors,
    box counts, battery levels.

    How it works:
        0.385  x 1000  =  385  ->  high = (385 >> 8) = 1,  low = (385 & 0xFF) = 129
        -> (0x01, 0x81)

    Args:
        value : non-negative float to encode
        scale : multiply before converting to int — default 1000

    Returns:
        (high_byte, low_byte) — each is an int from 0x00 to 0xFF

    Raises:
        ValueError if the scaled value is negative or exceeds 65535 (0xFFFF)

    Range with default scale:
        0 to 65.535 (since unsigned 16-bit holds 0-65535, divide by 1000)
    """
    int_val = int(round(value * scale))
    if int_val < 0 or int_val > 0xFFFF:
        raise ValueError(
            f"encode_unsigned: value {value} out of range after scaling "
            f"(scaled={int_val}, must be 0-65535)"
        )
    high = (int_val >> 8) & 0xFF
    low  =  int_val       & 0xFF
    return high, low


def decode_unsigned(high: int, low: int, scale: int = 1000) -> float:
    """
    Decode two bytes (high, low) back into a non-negative float.

    Use this to decode any value that was encoded with encode_unsigned.

    Args:
        high  : high byte (0x00-0xFF)
        low   : low byte  (0x00-0xFF)
        scale : divide by this to recover float — must match encode_unsigned scale

    Returns:
        Reconstructed non-negative float
    """
    int_val = (high << 8) | low
    return int_val / scale


# =============================================================================
# Checksum
# =============================================================================

def compute_checksum(msg_id: int, data: bytes) -> int:
    """
    Compute packet checksum.

    Checksum covers: MSG_ID + DATA_LENGTH + all DATA bytes.
    Result is masked to 8 bits so it always fits in one byte.

    Args:
        msg_id : the MSG_ID byte of the packet
        data   : the payload bytes of the packet (b'' if no payload)

    Returns:
        Single checksum byte as int (0x00-0xFF)

    Example:
        MSG_ID=0x01, data=b''
            -> checksum = (0x01 + 0x00) & 0xFF = 0x01

        MSG_ID=0x04, data=b'\\x03\\xe8\\x00\\x00\\x00\\x00'  (vx=1.0, vy=0, omega=0)
            -> checksum = (0x04 + 0x06 + 0x03 + 0xe8 + 0x00*4) & 0xFF = 0xF5
    """
    total = msg_id + len(data)
    for byte in data:
        total += byte
    return total & 0xFF


def verify_checksum(msg_id: int, data: bytes, received_checksum: int) -> bool:
    """
    Verify the checksum of a received packet.

    Args:
        msg_id            : MSG_ID byte from the received packet
        data              : payload bytes from the received packet
        received_checksum : CHECKSUM byte from the end of the received packet

    Returns:
        True if the packet is intact, False if it is corrupted
    """
    return compute_checksum(msg_id, data) == received_checksum


# =============================================================================
# Packet builder
# =============================================================================

def build_packet(msg_id: int, data: bytes = b'') -> bytes:
    """
    Assemble a complete UART packet ready to write to serial.

    Packet layout:
        [ START_BYTE(0xFF) | MSG_ID | DATA_LENGTH | DATA... | CHECKSUM ]

    Args:
        msg_id : message type ID from config.py (e.g. MSG_HEARTBEAT_PING)
        data   : payload as bytes — use b'' for messages with no payload

    Returns:
        Complete packet as a bytes object

    Examples:
        # Heartbeat ping — no payload
        build_packet(0x01)
        -> b'\\xff\\x01\\x00\\x01'
           [  0xFF  |  0x01  |  0x00  |  0x01  ]
            START     MSG_ID   LEN=0    CHECKSUM

        # Velocity command — vx=1.0 m/s, vy=0.0, omega=0.0
        vx_h, vx_l = encode_signed(1.0)   # -> (0x03, 0xE8)
        data = bytes([vx_h, vx_l, 0x00, 0x00, 0x00, 0x00])
        build_packet(0x04, data)
        -> b'\\xff\\x04\\x06\\x03\\xe8\\x00\\x00\\x00\\x00\\xf5'
    """
    checksum = compute_checksum(msg_id, data)
    return bytes([START_BYTE, msg_id, len(data)]) + data + bytes([checksum])


# =============================================================================
# Packet parser
# =============================================================================

class ParseResult:
    """
    Return type of parse_packet().

    Attributes:
        success : True if the packet was valid and fully parsed
        msg_id  : MSG_ID byte extracted from the packet (None if failed)
        data    : payload bytes extracted from the packet (b'' if no payload)
        error   : human-readable description of what went wrong (None if success)

    Usage:
        result = parse_packet(raw_bytes)
        if result.success:
            handle_message(result.msg_id, result.data)
        else:
            log_warning(f"Bad packet: {result.error}")
    """
    def __init__(self, success: bool, msg_id: int = None,
                 data: bytes = b'', error: str = None):
        self.success = success
        self.msg_id  = msg_id
        self.data    = data
        self.error   = error

    def __repr__(self):
        if self.success:
            return (f"ParseResult(OK  "
                    f"msg_id=0x{self.msg_id:02X}  "
                    f"data=[{self.data.hex(' ')}])")
        return f"ParseResult(FAIL  error='{self.error}')"


def parse_packet(raw: bytes) -> ParseResult:
    """
    Parse and validate a complete raw byte sequence as one packet.

    Called by the UART listener after PacketReader has assembled a full packet.
    Does NOT handle stream parsing — that is PacketReader's job.

    Checks performed (in order):
        1. Minimum length — at least 4 bytes required
        2. Start byte    — first byte must be 0xFF
        3. Length match  — actual length must match DATA_LENGTH field
        4. Checksum      — computed checksum must match received checksum

    Args:
        raw : complete byte sequence from START_BYTE through CHECKSUM

    Returns:
        ParseResult — always check .success before using .msg_id and .data

    Minimum valid packet: 4 bytes
        [ 0xFF | MSG_ID | 0x00 | CHECKSUM ]  (no payload)
    """
    # ── 1. Minimum length ─────────────────────────────────────────────────────
    if len(raw) < 4:
        return ParseResult(False,
            error=f"Packet too short: {len(raw)} byte(s), need at least 4")

    # ── 2. Start byte ─────────────────────────────────────────────────────────
    if raw[0] != START_BYTE:
        return ParseResult(False,
            error=f"Bad start byte: 0x{raw[0]:02X} (expected 0xFF)")

    msg_id      = raw[1]
    data_length = raw[2]

    # ── 3. Length consistency ─────────────────────────────────────────────────
    # Total expected = START(1) + MSG_ID(1) + DATA_LEN(1) + DATA(N) + CHECKSUM(1)
    expected_total = 3 + data_length + 1
    if len(raw) != expected_total:
        return ParseResult(False,
            error=(f"Length mismatch: DATA_LENGTH field says {data_length} payload bytes "
                   f"-> expect {expected_total} bytes total, got {len(raw)}"))

    data              = raw[3 : 3 + data_length]
    received_checksum = raw[-1]

    # ── 4. Checksum ───────────────────────────────────────────────────────────
    if not verify_checksum(msg_id, data, received_checksum):
        expected = compute_checksum(msg_id, data)
        return ParseResult(False,
            error=(f"Checksum mismatch: "
                   f"received=0x{received_checksum:02X}, "
                   f"expected=0x{expected:02X}"))

    return ParseResult(True, msg_id=msg_id, data=data)


# =============================================================================
# PacketReader — stateful stream parser for the UART listener thread
# =============================================================================

class PacketReader:
    """
    Stateful byte-by-byte stream parser.

    Why this is needed:
        UART delivers a continuous stream of bytes with no built-in packet
        boundaries. When the listener calls serial.read(1) it gets one byte
        with no context about where in a packet that byte belongs.
        PacketReader tracks position internally so the listener just feeds
        bytes in and gets complete packets out — no framing logic needed
        in the listener itself.

    Internal state machine:
        WAIT_START  -> discard all bytes until 0xFF is seen
        WAIT_MSG_ID -> next byte is MSG_ID
        WAIT_LENGTH -> next byte is DATA_LENGTH
        WAIT_DATA   -> collect DATA_LENGTH payload bytes one by one
        WAIT_CHKSUM -> next byte is CHECKSUM -> emit complete packet

    Usage (inside the UART listener thread):
        reader = PacketReader()

        while running:
            byte = serial_port.read(1)
            if byte:
                raw_packet = reader.feed(byte[0])
                if raw_packet is not None:
                    result = parse_packet(raw_packet)
                    if result.success:
                        handle_message(result.msg_id, result.data)
                    else:
                        log_warning(f"Bad packet: {result.error}")
    """

    _WAIT_START  = 0
    _WAIT_MSG_ID = 1
    _WAIT_LENGTH = 2
    _WAIT_DATA   = 3
    _WAIT_CHKSUM = 4

    def __init__(self):
        self.reset()

    def reset(self):
        """
        Reset internal state to WAIT_START.
        Called automatically after emitting a complete packet.
        Can also be called manually to recover from a corrupted stream.
        """
        self._state       = self._WAIT_START
        self._buffer      = []
        self._data_length = 0
        self._data_count  = 0

    def feed(self, byte: int) -> bytes | None:
        """
        Feed one byte into the parser.

        Args:
            byte : integer value of the incoming byte (0-255)

        Returns:
            Complete raw packet as bytes when a full packet has been assembled.
            None on every other call — packet still in progress.

        The returned bytes should be passed directly to parse_packet().
        """
        if self._state == self._WAIT_START:
            if byte == START_BYTE:
                self._buffer = [byte]
                self._state  = self._WAIT_MSG_ID
            # any byte that is not 0xFF is silently discarded

        elif self._state == self._WAIT_MSG_ID:
            self._buffer.append(byte)
            self._state = self._WAIT_LENGTH

        elif self._state == self._WAIT_LENGTH:
            self._data_length = byte
            self._data_count  = 0
            self._buffer.append(byte)
            # skip WAIT_DATA entirely if there is no payload
            self._state = self._WAIT_DATA if self._data_length > 0 else self._WAIT_CHKSUM

        elif self._state == self._WAIT_DATA:
            self._buffer.append(byte)
            self._data_count += 1
            if self._data_count == self._data_length:
                self._state = self._WAIT_CHKSUM

        elif self._state == self._WAIT_CHKSUM:
            self._buffer.append(byte)
            packet = bytes(self._buffer)
            self.reset()     # ready for the next packet immediately
            return packet    # hand off to parse_packet()

        return None