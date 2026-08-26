"""Encode and decode Robstride CAN frames (29-bit ID + 8-byte payload)."""

from __future__ import annotations

import struct
from dataclasses import dataclass

from solarian_robstride.protocol import CommType, MotorParams


def float_to_uint16(value: float, x_min: float, x_max: float) -> int:
    if x_max == x_min:
        return 0
    bounded = min(x_max, max(x_min, float(value)))
    return int(round((bounded - x_min) * 65535.0 / (x_max - x_min)))


def uint16_to_float(raw: int, x_min: float, x_max: float) -> float:
    return float(raw) * (x_max - x_min) / 65535.0 + x_min


def radians_to_raw(position_rad: float, params: MotorParams) -> int:
    return float_to_uint16(position_rad, params.p_min, params.p_max)


def raw_to_radians(raw: float, params: MotorParams) -> float:
    return uint16_to_float(int(round(raw)), params.p_min, params.p_max)


def build_can_id(comm_type: int, data_area2: int, target_id: int) -> int:
    """29-bit extended CAN ID (no SocketCAN EFF flag)."""
    return ((int(comm_type) & 0x1F) << 24) | ((int(data_area2) & 0xFFFF) << 8) | (
        int(target_id) & 0xFF
    )


def parse_can_id(can_id: int) -> tuple[int, int, int]:
    """Return ``(comm_type, data_area2, motor_id)`` from a 29-bit ID."""
    can_id = int(can_id) & 0x1FFFFFFF
    comm_type = (can_id >> 24) & 0x1F
    data_area2 = (can_id >> 8) & 0xFFFF
    motor_id = can_id & 0xFF
    return comm_type, data_area2, motor_id


def encode_at_frame(can_id: int, data: bytes) -> bytes:
    """CH340 AT serial packet: ``AT | (id<<3|0x4) BE | dlc | payload | \\r\\n``."""
    payload = bytes(data[:8]).ljust(8, b"\x00")
    encoded_id = ((int(can_id) & 0x1FFFFFFF) << 3) | 0x4
    return b"AT" + encoded_id.to_bytes(4, "big") + bytes([len(payload)]) + payload + b"\r\n"


def decode_at_frame(buf: bytes) -> tuple[int, bytes, int] | None:
    """Parse one AT frame at the start of ``buf``.

    Returns ``(can_id, data, total_len)`` or ``None`` if incomplete / invalid.
    """
    if len(buf) < 8:
        return None
    if buf[0] != ord("A") or buf[1] != ord("T"):
        return None
    data_len = buf[6]
    total_len = 7 + data_len + 2
    if len(buf) < total_len:
        return None
    if buf[total_len - 2] != 0x0D or buf[total_len - 1] != 0x0A:
        return None
    raw_id = int.from_bytes(buf[2:6], "big")
    can_id = (raw_id >> 3) & 0x1FFFFFFF
    data = bytes(buf[7 : 7 + data_len])
    return can_id, data, total_len


def extract_at_frames(buf: bytearray) -> list[tuple[int, bytes]]:
    """Pop every complete AT frame from the front of ``buf``."""
    frames: list[tuple[int, bytes]] = []
    while True:
        start = buf.find(b"AT")
        if start < 0:
            buf.clear()
            break
        if start > 0:
            del buf[:start]
        parsed = decode_at_frame(buf)
        if parsed is None:
            if len(buf) > 64:
                del buf[0]
                continue
            break
        can_id, data, total_len = parsed
        frames.append((can_id, data))
        del buf[:total_len]
    return frames


@dataclass
class MotorFeedback:
    motor_id: int
    position: float
    velocity: float
    torque: float
    temperature: float
    position_raw: int
    mode_state: int = 0
    fault_code: int = 0


def parse_feedback(can_id: int, data: bytes, params: MotorParams) -> MotorFeedback | None:
    comm_type, data_area2, motor_id = parse_can_id(can_id)
    if comm_type != CommType.FEEDBACK or len(data) < 8:
        return None
    pos_raw, vel_raw, torque_raw, temp_raw = struct.unpack(">HHHH", data[:8])
    return MotorFeedback(
        motor_id=motor_id,
        position=uint16_to_float(pos_raw, params.p_min, params.p_max),
        velocity=uint16_to_float(vel_raw, params.v_min, params.v_max),
        torque=uint16_to_float(torque_raw, params.t_min, params.t_max),
        temperature=temp_raw / 10.0,
        position_raw=pos_raw,
        mode_state=(data_area2 >> 14) & 0x03,
        fault_code=(data_area2 >> 8) & 0x3F,
    )


def encode_enable(host_can_id: int, motor_id: int) -> tuple[int, bytes]:
    return build_can_id(CommType.ENABLE, host_can_id, motor_id), bytes(8)


def encode_disable(host_can_id: int, motor_id: int, clear_fault: bool = False) -> tuple[int, bytes]:
    data = bytes([1 if clear_fault else 0]) + bytes(7)
    return build_can_id(CommType.DISABLE, host_can_id, motor_id), data


def encode_set_zero(host_can_id: int, motor_id: int) -> tuple[int, bytes]:
    return build_can_id(CommType.SET_ZERO, host_can_id, motor_id), bytes([1]) + bytes(7)


def encode_write_param_float(host_can_id: int, motor_id: int, param_index: int, value: float) -> tuple[int, bytes]:
    data = struct.pack("<HBx", int(param_index) & 0xFFFF, 0) + struct.pack("<f", float(value))
    return build_can_id(CommType.WRITE_PARAM, host_can_id, motor_id), data


def encode_write_param_int(host_can_id: int, motor_id: int, param_index: int, value: int) -> tuple[int, bytes]:
    data = struct.pack("<HBx", int(param_index) & 0xFFFF, 0) + struct.pack("<I", int(value) & 0xFFFFFFFF)
    return build_can_id(CommType.WRITE_PARAM, host_can_id, motor_id), data

