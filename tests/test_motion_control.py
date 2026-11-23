"""MIT (type 1) control on the Robstride bus, against simulated motors."""

from __future__ import annotations

import math
import struct
from types import SimpleNamespace

import pytest

from solarian.core.calibration import MotorCalibration
from solarian.core.data import PosData
from solarian_robstride import bus as bus_module
from solarian_robstride.bus import MAX_POSITION_AGE_S, RobstrideBus
from solarian_robstride.can_codec import (
    build_can_id,
    encode_motion_control,
    parse_can_id,
    radians_to_raw,
    uint16_to_float,
)
from solarian_robstride.protocol import (
    HOST_CAN_ID,
    MODE_MOTOR,
    CommType,
    ParamIndex,
    params_for_model,
)

RS00 = params_for_model("rs00")
#: One raw tick of position, in rad.
TICK = (RS00.p_max - RS00.p_min) / 65535


class SimulatedMotors:
    """Answers every frame at once, as motors that track their MIT target exactly."""

    def __init__(self, positions: dict[int, float]):
        self.positions = dict(positions)
        self.modes = {motor_id: 0 for motor_id in positions}
        self.sent: list[tuple[int, bytes]] = []
        self.bus: RobstrideBus
        self.is_open = True

    def send(self, can_id: int, data: bytes) -> None:
        comm_type, _, motor_id = parse_can_id(can_id)
        self.sent.append((can_id, bytes(data)))
        if comm_type == CommType.ENABLE:
            self.modes[motor_id] = MODE_MOTOR
        elif comm_type == CommType.DISABLE:
            self.modes[motor_id] = 0
        elif comm_type == CommType.MOTION_CONTROL:
            self.positions[motor_id] = decode_mit(can_id, data)["position"]
        elif comm_type == CommType.READ_PARAM:
            (index,) = struct.unpack_from("<H", data, 0)
            if index == ParamIndex.MECH_POS:
                reply = struct.pack("<HH", index, 0) + struct.pack("<f", self.positions[motor_id])
                self.bus._on_frame(build_can_id(CommType.READ_PARAM, motor_id, HOST_CAN_ID), reply)
            return
        area2 = (self.modes[motor_id] << 14) | motor_id
        feedback = struct.pack(
            ">HHHH", radians_to_raw(self.positions[motor_id], RS00), 0x8000, 0x8000, 300
        )
        self.bus._on_frame(build_can_id(CommType.FEEDBACK, area2, HOST_CAN_ID), feedback)

    def frames(self, comm_type: int, motor_id: int) -> list[tuple[int, bytes]]:
        return [
            (can_id, data) for can_id, data in self.sent
            if parse_can_id(can_id)[0] == comm_type and parse_can_id(can_id)[2] == motor_id
        ]

    def mit(self, motor_id: int) -> list[dict[str, float]]:
        return [decode_mit(c, d) for c, d in self.frames(CommType.MOTION_CONTROL, motor_id)]

    def param_writes(self, motor_id: int, index: int) -> list[float]:
        return [
            struct.unpack_from("<f", data, 4)[0]
            for _, data in self.frames(CommType.WRITE_PARAM, motor_id)
            if struct.unpack_from("<H", data, 0)[0] == index
        ]


def decode_mit(can_id: int, data: bytes) -> dict[str, float]:
    _, torque_raw, _ = parse_can_id(can_id)
    pos, vel, kp, kd = struct.unpack(">HHHH", data)
    return {
        "position": uint16_to_float(pos, RS00.p_min, RS00.p_max),
        "velocity": uint16_to_float(vel, RS00.v_min, RS00.v_max),
        "kp": uint16_to_float(kp, RS00.kp_min, RS00.kp_max),
        "kd": uint16_to_float(kd, RS00.kd_min, RS00.kd_max),
        "torque": uint16_to_float(torque_raw, RS00.t_min, RS00.t_max),
    }


@pytest.fixture
def clock(monkeypatch):
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr(bus_module.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(bus_module.time, "sleep", lambda s: None)
    return clock


def make_bus(positions: dict[int, float], drive_modes: dict[int, int] | None = None):
    """A bus of RS00 motors named ``j<id>``, normalized -100..100 over -pi..pi."""
    components = {
        f"j{motor_id}": SimpleNamespace(
            cfg=SimpleNamespace(id=motor_id, model="rs00", mode="range_m100_100")
        )
        for motor_id in positions
    }
    calibration = {
        f"j{motor_id}": MotorCalibration(
            id=motor_id,
            drive_mode=(drive_modes or {}).get(motor_id, 0),
            homing_offset=0,
            range_min=radians_to_raw(-math.pi, RS00),
            range_max=radians_to_raw(math.pi, RS00),
        )
        for motor_id in positions
    }
    bus = RobstrideBus("/dev/null", components, calibration)
    motors = SimulatedMotors(positions)
    motors.bus = bus
    bus._transport = motors
    return bus, motors


def write_rad(bus: RobstrideBus, targets: dict[str, float]) -> None:
    bus.write(
        {name: PosData(value=radians_to_raw(rad, RS00)) for name, rad in targets.items()},
        normalize=False,
    )


def test_encode_motion_control_packs_every_field():
    can_id, data = encode_motion_control(3, 1.25, -2.0, 80.0, 4.0, 3.5, RS00)
    comm_type, _, motor_id = parse_can_id(can_id)
    assert (comm_type, motor_id) == (CommType.MOTION_CONTROL, 3)
    fields = decode_mit(can_id, data)
    assert fields["position"] == pytest.approx(1.25, abs=1e-3)
    assert fields["velocity"] == pytest.approx(-2.0, abs=2e-3)
    assert fields["kp"] == pytest.approx(80.0, abs=1e-2)
    assert fields["kd"] == pytest.approx(4.0, abs=1e-3)
    assert fields["torque"] == pytest.approx(3.5, abs=1e-3)


def test_encode_motion_control_clamps_to_the_motor_range():
    can_id, data = encode_motion_control(1, 0.0, 0.0, 900.0, 0.0, -50.0, RS00)
    fields = decode_mit(can_id, data)
    assert fields["kp"] == pytest.approx(RS00.kp_max)
    assert fields["torque"] == pytest.approx(RS00.t_min)


def test_enable_holds_the_present_pose_with_gravity(clock):
    bus, motors = make_bus({1: 0.3, 2: -0.2})
    seen = []

    def feedforward(normalized):
        seen.append(dict(normalized))
        return {"j1": 2.0, "j2": -1.0}

    bus.use_motion_control({"j1": (80.0, 4.0), "j2": (60.0, 3.0)}, feedforward)
    bus.enable_torque(["j1", "j2"])

    (hold,) = motors.mit(1)
    assert hold["position"] == pytest.approx(0.3, abs=TICK)
    assert hold["velocity"] == pytest.approx(0.0, abs=2e-3)
    assert (hold["kp"], hold["kd"]) == (pytest.approx(80.0, abs=1e-2), pytest.approx(4.0, abs=1e-3))
    assert hold["torque"] == pytest.approx(2.0, abs=1e-3)
    assert motors.mit(2)[0]["torque"] == pytest.approx(-1.0, abs=1e-3)
    assert motors.param_writes(1, ParamIndex.LOC_REF) == []
    # The callback sees the whole arm, normalized, even while one motor enables.
    assert seen[0]["j1"] == pytest.approx(0.3 / math.pi * 100, abs=0.01)
    assert seen[0]["j2"] == pytest.approx(-0.2 / math.pi * 100, abs=0.01)


def test_velocity_feedforward_follows_consecutive_targets(clock):
    bus, motors = make_bus({1: 0.0})
    bus.use_motion_control({"j1": (80.0, 4.0)})
    bus.enable_torque(["j1"])
    clock.now += 0.01
    write_rad(bus, {"j1": 0.01})
    clock.now += 0.01
    write_rad(bus, {"j1": 0.02})
    first, second = motors.mit(1)[1:]
    assert second["velocity"] == pytest.approx(1.0, abs=0.01)
    assert first["velocity"] == pytest.approx(1.0, abs=0.01)


def test_a_gap_zeroes_the_velocity_feedforward(clock):
    bus, motors = make_bus({1: 0.0})
    bus.use_motion_control({"j1": (80.0, 4.0)})
    bus.enable_torque(["j1"])
    clock.now += MAX_POSITION_AGE_S * 4
    write_rad(bus, {"j1": 0.01})
    assert motors.mit(1)[-1]["velocity"] == pytest.approx(0.0, abs=2e-3)


def test_speed_cap_bounds_each_step(clock):
    bus, motors = make_bus({1: 0.0})
    bus.use_motion_control({"j1": (80.0, 4.0)})
    bus.set_speed_cap(["j1"], 1.0)
    bus.enable_torque(["j1"])
    clock.now += 0.01
    write_rad(bus, {"j1": 0.5})
    step = motors.mit(1)[-1]
    assert step["position"] == pytest.approx(0.01, abs=TICK)
    assert step["velocity"] == pytest.approx(1.0, abs=0.01)
    assert motors.param_writes(1, ParamIndex.LIMIT_SPD) == []


def test_speed_cap_after_a_gap_starts_from_the_measured_position(clock):
    bus, motors = make_bus({1: 0.0})
    bus.use_motion_control({"j1": (80.0, 4.0)})
    bus.set_speed_cap(["j1"], 1.0)
    bus.enable_torque(["j1"])
    motors.positions[1] = 0.2
    clock.now += 1.0
    write_rad(bus, {"j1": 2.0})
    assert motors.mit(1)[-1]["position"] == pytest.approx(
        0.2 + MAX_POSITION_AGE_S, abs=2 * TICK
    )


def test_drive_mode_flips_the_feedforward_torque(clock):
    bus, motors = make_bus({1: 0.0}, drive_modes={1: 1})
    bus.use_motion_control({"j1": (80.0, 4.0)}, lambda normalized: {"j1": 2.5})
    bus.enable_torque(["j1"])
    assert motors.mit(1)[0]["torque"] == pytest.approx(-2.5, abs=1e-3)


def test_csp_motors_keep_loc_ref_and_limit_spd(clock):
    bus, motors = make_bus({1: 0.0, 2: 0.0})
    bus.use_motion_control({"j1": (80.0, 4.0)})
    bus.set_speed_cap(["j1", "j2"], 1.5)
    bus.enable_torque(["j1", "j2"])
    clock.now += 0.01
    write_rad(bus, {"j1": 0.01, "j2": 0.01})
    assert motors.mit(2) == []
    assert motors.param_writes(2, ParamIndex.LIMIT_SPD) == [pytest.approx(1.5)]
    assert motors.param_writes(2, ParamIndex.LOC_REF)[-1] == pytest.approx(0.01, abs=TICK)
    assert len(motors.mit(1)) == 2


def test_disable_forgets_the_last_target(clock):
    bus, motors = make_bus({1: 0.0})
    bus.use_motion_control({"j1": (80.0, 4.0)})
    bus.enable_torque(["j1"])
    bus.disable_torque(["j1"])
    motors.positions[1] = 0.4
    clock.now += 1.0
    bus.enable_torque(["j1"])
    hold = motors.mit(1)[-1]
    assert hold["position"] == pytest.approx(0.4, abs=TICK)
    assert hold["velocity"] == pytest.approx(0.0, abs=2e-3)
