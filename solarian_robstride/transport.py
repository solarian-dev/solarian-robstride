"""CH340 / GD32 USB-CAN adapter: AT serial framing at 921600 baud."""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable

import serial

from solarian_robstride.can_codec import encode_at_frame, extract_at_frames
from solarian_robstride.protocol import SERIAL_BAUDRATE

logger = logging.getLogger(__name__)

FrameCallback = Callable[[int, bytes], None]

#: Least time (s) from one frame's write to the next. Sent back to back, the
#: adapter loses the motors' replies, up to every reply from some motors of a
#: six-motor arm; 0.5 ms apart it loses none.
MIN_FRAME_GAP_S = 0.0005


class Ch340Transport:
    """Send/receive 29-bit CAN frames over the Robstride USB-CAN serial adapter."""

    # Two handles on one serial port split its bytes between their readers, so
    # buses sharing a port in one process (an arm and its gripper) share one.
    _shared: dict[str, "Ch340Transport"] = {}
    _shared_lock = threading.Lock()

    def __init__(self, port: str, baudrate: int = SERIAL_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self._serial: serial.Serial | None = None
        self._serial_lock = threading.Lock()
        self._recv_running = False
        self._recv_thread: threading.Thread | None = None
        self._listeners: list[FrameCallback] = []
        self._rx_buf = bytearray()
        self._last_send = float("-inf")

    @classmethod
    def attach(cls, port: str, on_frame: FrameCallback) -> "Ch340Transport":
        """The process's open transport on ``port``; ``on_frame`` gets every frame."""
        with cls._shared_lock:
            transport = cls._shared.get(port)
            if transport is None:
                transport = cls(port)
                transport.open()
                cls._shared[port] = transport
            transport.start_receive_thread(on_frame)
            return transport

    def detach(self, on_frame: FrameCallback) -> None:
        """Stop delivering to ``on_frame``; the last listener out closes the port."""
        with Ch340Transport._shared_lock:
            if on_frame in self._listeners:
                self._listeners.remove(on_frame)
            if self._listeners:
                return
            if Ch340Transport._shared.get(self.port) is self:
                del Ch340Transport._shared[self.port]
        self.close()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def open(self) -> None:
        if self.is_open:
            return
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.2,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._rx_buf.clear()
        logger.info("Opened Robstride USB-CAN adapter on %s @ %d", self.port, self.baudrate)

    def close(self) -> None:
        self.stop_receive_thread()
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                logger.debug("Error closing %s", self.port, exc_info=True)
            self._serial = None
        logger.info("Closed Robstride USB-CAN adapter on %s", self.port)

    def send(self, can_id: int, data: bytes) -> None:
        if self._serial is None or not self._serial.is_open:
            raise ConnectionError(f"Robstride adapter {self.port} is not open")
        frame = encode_at_frame(can_id, data)
        with self._serial_lock:
            wait = self._last_send + MIN_FRAME_GAP_S - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            self._serial.write(frame)
            self._serial.flush()
            self._last_send = time.perf_counter()

    def start_receive_thread(self, on_frame: FrameCallback) -> None:
        self._listeners.append(on_frame)
        if self._recv_running:
            return
        self._recv_running = True
        self._recv_thread = threading.Thread(
            target=self._receive_loop,
            daemon=True,
            name=f"robstride-rx-{self.port}",
        )
        self._recv_thread.start()

    def stop_receive_thread(self) -> None:
        self._recv_running = False
        if self._recv_thread is not None:
            self._recv_thread.join(timeout=1.0)
            self._recv_thread = None
        self._listeners.clear()

    def _receive_loop(self) -> None:
        while self._recv_running:
            try:
                chunk = b""
                with self._serial_lock:
                    ser = self._serial
                    if ser is None or not ser.is_open:
                        break
                    waiting = ser.in_waiting
                    if waiting:
                        chunk = ser.read(waiting)
                if chunk:
                    self._rx_buf.extend(chunk)
                    for can_id, data in extract_at_frames(self._rx_buf):
                        for callback in list(self._listeners):
                            try:
                                callback(can_id, data)
                            except Exception:
                                logger.debug("Frame callback failed", exc_info=True)
                else:
                    time.sleep(0.001)
            except serial.SerialException:
                if self._recv_running:
                    logger.warning("Serial error on %s", self.port, exc_info=True)
                break
            except Exception:
                if self._recv_running:
                    logger.warning("Receive loop error on %s", self.port, exc_info=True)
                time.sleep(0.01)
