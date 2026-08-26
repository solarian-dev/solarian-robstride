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


class Ch340Transport:
    """Send/receive 29-bit CAN frames over the Robstride USB-CAN serial adapter."""

    def __init__(self, port: str, baudrate: int = SERIAL_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self._serial: serial.Serial | None = None
        self._serial_lock = threading.Lock()
        self._recv_running = False
        self._recv_thread: threading.Thread | None = None
        self._on_frame: FrameCallback | None = None
        self._rx_buf = bytearray()

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
            self._serial.write(frame)
            self._serial.flush()

    def start_receive_thread(self, on_frame: FrameCallback) -> None:
        if self._recv_running:
            return
        self._on_frame = on_frame
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
        self._on_frame = None

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
                        callback = self._on_frame
                        if callback is not None:
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
