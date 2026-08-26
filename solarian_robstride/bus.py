"""
Robstride Actuator Bus

Bus implementation for Robstride motors on a CH340 USB-CAN adapter.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict

from solarian.core.actuator_bus import ActuatorBus
from solarian.core.calibration import MotorCalibration
from solarian.core.component import Component
from solarian.core.data import Data, PosData, SpeedData, TemperatureData, TorqueEnableData

from solarian_robstride.can_codec import (
    MotorFeedback,
    encode_disable,
    encode_enable,
    encode_set_zero,
    encode_write_param_float,
    encode_write_param_int,
    parse_can_id,
    parse_feedback,
    raw_to_radians,
)
from solarian_robstride.protocol import (
    CONFIG_PARAM_MAP,
    HOST_CAN_ID,
    PROTOCOL_CPR,
    CommType,
    MotorParams,
    ParamIndex,
    RunMode,
    params_for_model,
)
from solarian_robstride.transport import Ch340Transport

logger = logging.getLogger(__name__)


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
        self._transport = Ch340Transport(port)
        self._lock = threading.Lock()
        self._feedbacks: Dict[int, MotorFeedback] = {}
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

    def _params_for(self, name: str) -> MotorParams:
        return self._params[name]

    def _motor_id(self, name: str) -> int:
        cfg = self.components[name].cfg
        if cfg is None:
            raise ValueError(f"Component {name} has no configuration")
        return int(cfg.id)

    def _send(self, can_id: int, data: bytes) -> None:
        self._transport.send(can_id, data)

    def _on_frame(self, can_id: int, data: bytes) -> None:
        comm_type, _, motor_id = parse_can_id(can_id)
        name = self._id_to_name.get(motor_id)
        if name is None:
            return
        if comm_type != CommType.FEEDBACK:
            return
        fb = parse_feedback(can_id, data, self._params_for(name))
        if fb is None:
            return
        with self._lock:
            self._feedbacks[motor_id] = fb

    def _get_feedback(self, name: str, timeout: float = 0.2) -> MotorFeedback:
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

    @property
    def is_connected(self) -> bool:
        return self._transport.is_open

    def connect(self):
        self._transport.open()
        self._transport.start_receive_thread(self._on_frame)
        names = list(self.components.keys())
        for name in names:
            self._set_run_mode(name, RunMode.POSITION_CSP)

    def disconnect(self):
        if self._transport.is_open:
            try:
                self.disable_torque(list(self.components.keys()))
            except Exception:
                logger.debug("Disable on disconnect failed", exc_info=True)
            self._transport.close()
        self._enabled.clear()

    def read(
        self,
        data_type: type[Data],
        component_names: list[str],
        normalize: bool = True,
    ) -> Dict[str, Any]:
        values: Dict[str, Any] = {}
        for name in component_names:
            fb = self._get_feedback(name)
            if data_type is PosData:
                values[name] = float(fb.position_raw)
            elif data_type is SpeedData:
                values[name] = float(fb.velocity)
            elif data_type is TemperatureData:
                values[name] = float(fb.temperature)
            else:
                raise ValueError(f"Unsupported data type: {data_type}")
        if normalize and data_type is PosData:
            return self._normalize(values)
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
                params = self._params_for(name)
                position_rad = raw_to_radians(raw_pos, params)
                can_id, data = encode_write_param_float(
                    self.host_can_id,
                    self._motor_id(name),
                    ParamIndex.LOC_REF,
                    position_rad,
                )
                self._send(can_id, data)

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
        return raw_to_radians(raw, self._params_for(name))

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
        """Set the current mechanical position as zero (Type 6)."""
        offsets: Dict[str, int] = {}
        for name in component_names:
            can_id, data = encode_set_zero(self.host_can_id, self._motor_id(name))
            self._send(can_id, data)
            offsets[name] = 0
        time.sleep(0.05)
        return offsets

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
        for name in component_names:
            can_id, data = encode_enable(self.host_can_id, self._motor_id(name))
            self._send(can_id, data)
            self._enabled.add(name)

    def disable_torque(self, component_names: list[str]):
        for name in component_names:
            can_id, data = encode_disable(self.host_can_id, self._motor_id(name))
            self._send(can_id, data)
            self._enabled.discard(name)

    def _set_run_mode(self, name: str, mode: RunMode) -> None:
        can_id, data = encode_write_param_int(
            self.host_can_id,
            self._motor_id(name),
            ParamIndex.RUN_MODE,
            int(mode),
        )
        self._send(can_id, data)
