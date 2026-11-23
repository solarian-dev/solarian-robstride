"""
Robstride Actuator Bus

Bus implementation for Robstride motors on a CH340 USB-CAN adapter.

A Robstride motor only speaks when spoken to: a type 2 feedback frame answers
enable, disable, a parameter write and a few others, and nothing arrives while
the host is silent. So a position is only as fresh as the last command, and a
limp or idle motor's position is read on demand with a type 17 read of
``mech_pos``, which changes nothing on the motor.
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from typing import Any, Callable, Dict

from solarian.core.actuator_bus import ActuatorBus
from solarian.core.calibration import MotorCalibration
from solarian.core.component import Component
from solarian.core.data import Data, PosData, SpeedData, TemperatureData, TorqueEnableData

from solarian_robstride.can_codec import (
    MotorFeedback,
    encode_disable,
    encode_enable,
    encode_get_device_id,
    encode_read_param,
    encode_write_param_float,
    encode_write_param_int,
    parse_can_id,
    parse_feedback,
    parse_param_read,
    radians_to_raw,
    raw_to_radians,
    reply_motor_id,
)
from solarian_robstride.protocol import (
    CONFIG_PARAM_MAP,
    HOST_CAN_ID,
    MODE_MOTOR,
    PROTOCOL_CPR,
    CommType,
    MotorParams,
    ParamIndex,
    RunMode,
    describe_faults,
    params_for_model,
)
from solarian_robstride.transport import Ch340Transport

logger = logging.getLogger(__name__)

#: A position older than this (s) is re-read before it is returned. A control
#: loop writing ``loc_ref`` every tick gets a reply each time and stays under it.
MAX_POSITION_AGE_S = 0.05
#: How long (s) one round of position reads waits for its replies.
READ_REPLY_TIMEOUT_S = 0.05
READ_ATTEMPTS = 3
#: How long (s) ``connect`` waits for every motor to answer each disable.
CONNECT_REPLY_TIMEOUT_S = 0.5


class RobstrideBus(ActuatorBus):
    """Actuator bus for Robstride motors on one USB-CAN serial port."""

    def __init__(
        self,
        port: str,
        components: Dict[str, Component],
        calibration: Dict[str, MotorCalibration] | None = None,
    ):
        super().__init__(port, components, calibration)
        self.host_can_id = HOST_CAN_ID
        self._transport: Ch340Transport | None = None
        self._lock = threading.Lock()
        self._feedbacks: Dict[int, MotorFeedback] = {}
        #: motor id -> time.monotonic() its last feedback frame arrived
        self._feedback_times: Dict[int, float] = {}
        #: motor id -> (raw position, time.monotonic() it arrived)
        self._positions: Dict[int, tuple[int, float]] = {}
        #: motor id -> (run mode read back, time.monotonic() it arrived)
        self._run_modes: Dict[int, tuple[int, float]] = {}
        #: motor id -> supply voltage (V) it last reported
        self._vbus: Dict[int, float] = {}
        self._fault_codes: Dict[int, int] = {}
        self._params: Dict[str, MotorParams] = {
            name: params_for_model(component.cfg.model)
            for name, component in components.items()
        }
        self._id_to_name: Dict[int, str] = {
            int(component.cfg.id): name for name, component in components.items()
        }
        self._enabled: set[str] = set()
        if not self.calibration:
            self.calibration = self._default_calibration()

    def _default_calibration(self) -> dict[str, MotorCalibration]:
        return {
            name: MotorCalibration(
                id=int(component.cfg.id),
                drive_mode=0,
                homing_offset=0,
                range_min=0,
                range_max=PROTOCOL_CPR - 1,
            )
            for name, component in self.components.items()
        }

    def _motor_id(self, name: str) -> int:
        cfg = self.components[name].cfg
        if cfg is None:
            raise ValueError(f"Component {name} has no configuration")
        return int(cfg.id)

    def _send(self, can_id: int, data: bytes) -> None:
        if self._transport is None:
            raise ConnectionError(f"Robstride bus on {self.port} is not connected")
        self._transport.send(can_id, data)

    def _detach(self) -> None:
        if self._transport is not None:
            self._transport.detach(self._on_frame)
            self._transport = None

    def _on_frame(self, can_id: int, data: bytes) -> None:
        comm_type, _, _ = parse_can_id(can_id)
        motor_id = reply_motor_id(can_id)
        name = self._id_to_name.get(motor_id)
        if name is None:
            return
        now = time.monotonic()
        if comm_type == CommType.FEEDBACK:
            fb = parse_feedback(can_id, data, self._params[name])
            if fb is None:
                return
            with self._lock:
                self._feedbacks[motor_id] = fb
                self._feedback_times[motor_id] = now
                self._positions[motor_id] = (fb.position_raw, now)
            self._note_fault(name, motor_id, fb.fault_code)
        elif comm_type == CommType.READ_PARAM:
            read = parse_param_read(can_id, data)
            if read is None:
                return
            index, value = read
            if index == ParamIndex.MECH_POS:
                (position_rad,) = struct.unpack("<f", value)
                raw = radians_to_raw(position_rad, self._params[name])
                with self._lock:
                    self._positions[motor_id] = (raw, now)
            elif index == ParamIndex.RUN_MODE:
                with self._lock:
                    self._run_modes[motor_id] = (value[0], now)
            elif index == ParamIndex.VBUS:
                (volts,) = struct.unpack("<f", value)
                with self._lock:
                    self._vbus[motor_id] = volts
        elif comm_type == CommType.FAULT_FEEDBACK:
            logger.warning(
                "Robstride motor %d (%s) reported fault frame %s", motor_id, name, data.hex()
            )

    def _note_fault(self, name: str, motor_id: int, fault_code: int) -> None:
        if fault_code == self._fault_codes.get(motor_id, 0):
            return
        self._fault_codes[motor_id] = fault_code
        if fault_code:
            logger.warning(
                "Robstride motor %d (%s) fault: %s", motor_id, name, describe_faults(fault_code)
            )
        else:
            logger.info("Robstride motor %d (%s) fault cleared", motor_id, name)

    def _get_feedback(self, name: str, timeout: float = 0.2) -> MotorFeedback:
        """The last type 2 frame. Its position may be old; use :meth:`_read_positions`."""
        motor_id = self._motor_id(name)
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                fb = self._feedbacks.get(motor_id)
            if fb is not None:
                return fb
            if time.monotonic() >= deadline:
                raise ConnectionError(
                    f"No Robstride feedback from motor {motor_id} ({name}) on {self.port}"
                )
            time.sleep(0.005)

    def _position_time(self, name: str) -> float:
        with self._lock:
            sample = self._positions.get(self._motor_id(name))
        return sample[1] if sample is not None else float("-inf")

    def _read_positions(self, component_names: list[str]) -> Dict[str, int]:
        """Raw positions no older than :data:`MAX_POSITION_AGE_S`, polling the stale ones."""
        stale = list(component_names)
        for _ in range(READ_ATTEMPTS):
            asked = time.monotonic()
            stale = [n for n in stale if asked - self._position_time(n) > MAX_POSITION_AGE_S]
            if not stale:
                break
            for name in stale:
                self._send(
                    *encode_read_param(self.host_can_id, self._motor_id(name), ParamIndex.MECH_POS)
                )
            deadline = asked + READ_REPLY_TIMEOUT_S
            while stale and time.monotonic() < deadline:
                time.sleep(0.001)
                stale = [n for n in stale if self._position_time(n) < asked]
            if not stale:
                break
        if stale:
            ids = ", ".join(f"{n} (id {self._motor_id(n)})" for n in stale)
            raise ConnectionError(f"No position from Robstride motor(s) {ids} on {self.port}")
        with self._lock:
            return {n: self._positions[self._motor_id(n)][0] for n in component_names}

    def _write_loc_ref(self, name: str, raw: float) -> None:
        position_rad = raw_to_radians(raw, self._params[name])
        self._send(
            *encode_write_param_float(
                self.host_can_id, self._motor_id(name), ParamIndex.LOC_REF, position_rad
            )
        )

    @property
    def is_connected(self) -> bool:
        return self._transport is not None and self._transport.is_open

    def connect(self):
        with self._lock:
            self._feedbacks.clear()
        self._transport = Ch340Transport.attach(self.port, self._on_frame)
        names = list(self.components.keys())
        # A disable is answered with a feedback frame, so every powered and wired
        # motor reports in here, and run mode may only change while disabled.
        missing = names
        for _ in range(READ_ATTEMPTS):
            for name in missing:
                self._send(*encode_disable(self.host_can_id, self._motor_id(name)))
            deadline = time.monotonic() + CONNECT_REPLY_TIMEOUT_S
            while missing and time.monotonic() < deadline:
                time.sleep(0.01)
                with self._lock:
                    missing = [n for n in missing if self._motor_id(n) not in self._feedbacks]
            if not missing:
                break
        if missing:
            self._detach()
            ids = ", ".join(f"{n} (id {self._motor_id(n)})" for n in missing)
            raise ConnectionError(
                f"Robstride motor(s) {ids} did not answer on {self.port}. Check the motor "
                "power supply, the CAN wiring and terminator, and the ids in the module yaml "
                "(`solarian calibrate-servo` lists the ids that answer)."
            )
        for name in names:
            self._set_run_mode(name, RunMode.POSITION_CSP)

    def disconnect(self):
        if self.is_connected:
            try:
                self.disable_torque(list(self.components.keys()))
            except Exception:
                logger.debug("Disable on disconnect failed", exc_info=True)
        self._detach()
        self._enabled.clear()

    def read(
        self,
        data_type: type[Data],
        component_names: list[str],
        normalize: bool = True,
    ) -> Dict[str, Any]:
        if data_type is PosData:
            values: Dict[str, Any] = {
                n: float(raw) for n, raw in self._read_positions(component_names).items()
            }
            return self._normalize(values) if normalize else values
        values = {}
        for name in component_names:
            fb = self._get_feedback(name)
            if data_type is SpeedData:
                values[name] = float(fb.velocity)
            elif data_type is TemperatureData:
                values[name] = float(fb.temperature)
            else:
                raise ValueError(f"Unsupported data type: {data_type}")
        return values

    def write(self, values: Dict[str, Data], normalize: bool = True):
        by_class: dict[type[Data], dict[str, Any]] = {}
        for name, value in values.items():
            by_class.setdefault(value.__class__, {})[name] = value.value

        if PosData in by_class:
            raw = dict(by_class[PosData])
            if normalize:
                raw = self._unnormalize(raw)
            for name, raw_pos in raw.items():
                self._write_loc_ref(name, raw_pos)

        if TorqueEnableData in by_class:
            for name, enabled in by_class[TorqueEnableData].items():
                if enabled:
                    self.enable_torque([name])
                else:
                    self.disable_torque([name])

        if SpeedData in by_class:
            for name, speed in by_class[SpeedData].items():
                can_id, data = encode_write_param_float(
                    self.host_can_id,
                    self._motor_id(name),
                    ParamIndex.LIMIT_SPD,
                    float(speed),
                )
                self._send(can_id, data)

    def counts_per_revolution(self, name: str) -> int | None:
        del name
        return PROTOCOL_CPR

    def raw_to_radians(self, name: str, raw: float) -> float | None:
        return raw_to_radians(raw, self._params[name])

    def is_calibrated(self) -> bool:
        if not self.calibration:
            return False
        return all(name in self.calibration for name in self.components)

    def reset_calibration(self, component_names: list[str]):
        if self.calibration is None:
            self.calibration = {}
        for name in component_names:
            cfg = self.components[name].cfg
            self.calibration[name] = MotorCalibration(
                id=int(cfg.id) if cfg is not None else 0,
                drive_mode=0,
                homing_offset=0,
                range_min=0,
                range_max=PROTOCOL_CPR - 1,
            )

    def read_calibration(self) -> dict[str, MotorCalibration]:
        return dict(self.calibration or {})

    def write_calibration(
        self, calibration_dict: dict[str, MotorCalibration]
    ) -> dict[str, MotorCalibration]:
        if self.calibration is None:
            self.calibration = {}
        for name, calibration in calibration_dict.items():
            lo = max(int(calibration.range_min), 0)
            hi = min(int(calibration.range_max), PROTOCOL_CPR - 1)
            calibration.range_min = lo
            calibration.range_max = hi
            self.calibration[name] = calibration
        return calibration_dict

    def set_half_turn_homings(self, component_names: list[str]) -> Dict[str, int]:
        """Software ``center_offset`` so the current pose reads mid-encoder.

        Robstride has no Homing_Offset register. Type 6 set-zero redefines
        mechanical zero and is not used here — unwrap/home persist a tick
        shift that :meth:`_normalize` applies.
        """
        offsets: Dict[str, int] = {}
        mid = (PROTOCOL_CPR - 1) // 2
        readings = self.read(PosData, component_names, normalize=False)
        if self.calibration is None:
            self.calibration = {}
        for name in component_names:
            raw = int(readings[name])
            offset = raw - mid
            entry = self.calibration.get(name)
            if entry is None:
                cfg = self.components[name].cfg
                entry = MotorCalibration(
                    id=int(cfg.id) if cfg is not None else 0,
                    drive_mode=0,
                    homing_offset=offset,
                    range_min=0,
                    range_max=PROTOCOL_CPR - 1,
                )
                self.calibration[name] = entry
            else:
                entry.homing_offset = offset
            offsets[name] = offset
        return offsets

    def drive_to_encoder_center(
        self,
        component_names: list[str],
        *,
        settle_s: float = 3.0,
        tol: int = 40,
    ) -> Dict[str, int]:
        """Drive each rotor to 0 rad, the midpoint of ``p_min..p_max`` (raw CPR/2).

        Robstride reports multi-turn absolute position, so no seam sits inside joint
        travel; this is provisioning convenience (park the output before fitting a
        linkage), not a wrap fix. Only for a free output: 0 rad can be half a turn
        from where the rotor sits.
        """
        mid = PROTOCOL_CPR // 2
        reached: Dict[str, int] = {}
        for name in component_names:
            self._set_run_mode(name, RunMode.POSITION_CSP)
            self.enable_torque([name])
            try:
                self._write_loc_ref(name, mid)
                deadline = time.monotonic() + settle_s
                while time.monotonic() < deadline:
                    if abs(self._read_positions([name])[name] - mid) <= tol:
                        break
                    time.sleep(0.05)
            finally:
                self.disable_torque([name])

            present = int(self._read_positions([name])[name])
            reached[name] = present
            if abs(present - mid) > tol:
                logger.warning(
                    "%s: reached %d, wanted ~%d. Output may be blocked.", name, present, mid
                )
        return reached

    # -- provisioning (no declared components: motors are addressed by CAN ID) --
    #: Highest CAN ID probed when scanning; robot chains use low IDs and each
    #: silent ID costs a full reply timeout.
    SCAN_MAX_ID = 30

    @classmethod
    def available_ports(cls) -> list[str]:
        """Serial ports: the CAN side is reached through a CH340 USB-CAN adapter."""
        import serial.tools.list_ports

        return [p.device for p in serial.tools.list_ports.comports()]

    @classmethod
    def scan_port(cls, port: str, *, reply_timeout: float = 0.05) -> list[dict]:
        """Ping CAN IDs with type 0 (get device ID) and list the responders."""
        transport = Ch340Transport(port)
        seen: dict[int, int] = {}

        def _on_frame(can_id: int, data: bytes) -> None:
            del data
            comm_type, _, _ = parse_can_id(can_id)
            if comm_type == CommType.GET_DEVICE_ID:
                seen.setdefault(reply_motor_id(can_id), comm_type)

        transport.open()
        try:
            transport.start_receive_thread(_on_frame)
            for motor_id in range(1, cls.SCAN_MAX_ID + 1):
                can_id, data = encode_get_device_id(HOST_CAN_ID, motor_id)
                transport.send(can_id, data)
                time.sleep(reply_timeout)
        finally:
            transport.close()

        # The reply carries a MCU UID, not a model number, so the model is unknown
        # until the user names it in the module config.
        return [
            {"id": motor_id, "model": "robstride", "baudrate": None}
            for motor_id in sorted(seen)
        ]

    def start_calibration_mode(self):
        self.disable_torque(list(self.components.keys()))

    def end_calibration_mode(self):
        pass

    def configure(self, params: dict[str, int], component_names: list[str]):
        for name in component_names:
            motor_id = self._motor_id(name)
            for param_name, value in params.items():
                mapping = CONFIG_PARAM_MAP.get(str(param_name).lower())
                if mapping is None:
                    logger.warning("Unknown Robstride configure key %r; skipping", param_name)
                    continue
                index, as_int = mapping
                if as_int:
                    can_id, data = encode_write_param_int(
                        self.host_can_id, motor_id, index, int(value)
                    )
                else:
                    can_id, data = encode_write_param_float(
                        self.host_can_id, motor_id, index, float(value)
                    )
                self._send(can_id, data)

    def enable_torque(self, component_names: list[str]):
        """Energise each motor where it stands.

        In CSP an enabled motor drives to its ``loc_ref`` at up to ``limit_spd``,
        and that target outlives a disable: it is wherever the last session left
        it, or 0 rad after power-up, which can be far from the present pose. So
        the present position is latched as the target on both sides of the
        enable, and enabling holds rather than moves.

        A fault latches until it is cleared, and a motor with one refuses to
        enable long after the cause has gone (an undervoltage from a dip that
        has passed). So a reported fault is cleared first; if its cause is
        still there it comes straight back and the clear fails, naming it.
        """
        to_enable = [n for n in component_names if n not in self._enabled]
        if not to_enable:
            return
        present = self._read_positions(to_enable)
        for name in to_enable:
            motor_id = self._motor_id(name)
            if self._fault_codes.get(motor_id):
                self._clear_fault(name, motor_id)
            self._write_loc_ref(name, present[name])

            def in_motor_mode(asked: float) -> bool:
                with self._lock:
                    fb = self._feedbacks.get(motor_id)
                    at = self._feedback_times.get(motor_id, float("-inf"))
                return at >= asked and fb.mode_state == MODE_MOTOR

            self._send_confirmed(
                name,
                lambda: self._send(*encode_enable(self.host_can_id, motor_id)),
                in_motor_mode,
                "the enable",
            )
            self._write_loc_ref(name, present[name])
            self._enabled.add(name)

    def _clear_fault(self, name: str, motor_id: int) -> None:
        logger.warning(
            "Robstride motor %d (%s): clearing latched fault (%s)",
            motor_id,
            name,
            describe_faults(self._fault_codes[motor_id]),
        )

        def cleared(asked: float) -> bool:
            with self._lock:
                fb = self._feedbacks.get(motor_id)
                at = self._feedback_times.get(motor_id, float("-inf"))
            return at >= asked and fb.fault_code == 0

        self._send_confirmed(
            name,
            lambda: self._send(*encode_disable(self.host_can_id, motor_id, clear_fault=True)),
            cleared,
            "the fault clear",
        )
        self._note_fault(name, motor_id, 0)

    def disable_torque(self, component_names: list[str]):
        """Make each motor limp, confirmed: a lost disable leaves it holding in CSP.

        Every motor is tried before a failure is raised, so one that will not
        answer does not leave the rest energised.
        """
        failures = []
        for name in component_names:
            motor_id = self._motor_id(name)

            def released(asked: float, motor_id=motor_id) -> bool:
                with self._lock:
                    fb = self._feedbacks.get(motor_id)
                    at = self._feedback_times.get(motor_id, float("-inf"))
                return at >= asked and fb.mode_state != MODE_MOTOR

            try:
                self._send_confirmed(
                    name,
                    lambda motor_id=motor_id: self._send(
                        *encode_disable(self.host_can_id, motor_id)
                    ),
                    released,
                    "the disable",
                )
            except ConnectionError as exc:
                failures.append(str(exc))
            else:
                self._enabled.discard(name)
        if failures:
            raise ConnectionError("; ".join(failures))

    def _set_run_mode(self, name: str, mode: RunMode) -> None:
        motor_id = self._motor_id(name)

        def send() -> None:
            self._send(
                *encode_write_param_int(self.host_can_id, motor_id, ParamIndex.RUN_MODE, int(mode))
            )
            self._send(*encode_read_param(self.host_can_id, motor_id, ParamIndex.RUN_MODE))

        def took(asked: float) -> bool:
            with self._lock:
                read = self._run_modes.get(motor_id)
            return read is not None and read[1] >= asked and read[0] == int(mode)

        self._send_confirmed(name, send, took, f"run mode {mode.name}")

    def _send_confirmed(
        self,
        name: str,
        send: Callable[[], None],
        confirmed: Callable[[float], bool],
        what: str,
    ) -> None:
        """Send until the motor confirms, given the time of the latest send.

        Enable, disable and run mode are one frame each and the motor keeps no
        record of a lost one. A lost enable leaves it limp, answering every read
        and ignoring every ``loc_ref``; a lost disable leaves it holding against
        a hand that was told it is free. On a bus that drops the odd frame these
        are checked rather than assumed.
        """
        for _ in range(READ_ATTEMPTS):
            asked = time.monotonic()
            send()
            deadline = asked + READ_REPLY_TIMEOUT_S
            while time.monotonic() < deadline:
                if confirmed(asked):
                    return
                time.sleep(0.001)
        motor_id = self._motor_id(name)
        with self._lock:
            fb = self._feedbacks.get(motor_id)
            self._vbus.pop(motor_id, None)
        faults = describe_faults(fb.fault_code) if fb is not None else ""
        if faults:
            self._send(*encode_read_param(self.host_can_id, motor_id, ParamIndex.VBUS))
            time.sleep(READ_REPLY_TIMEOUT_S)
            with self._lock:
                volts = self._vbus.get(motor_id)
            if volts is not None:
                faults += f" (it measures {volts:.1f} V at its supply)"
        raise ConnectionError(
            f"Robstride motor {motor_id} ({name}) on {self.port} did not confirm {what} "
            f"after {READ_ATTEMPTS} tries"
            + (f"; it reports: {faults}." if faults else ". Check the CAN wiring and terminator.")
        )
