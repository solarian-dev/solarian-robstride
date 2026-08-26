# solarian-robstride

Robstride servo actuator and CAN bus plugin for the Solarian robotics framework.
Talks to RS00 / EL05 / RS05 motors over the official Robstride USB-CAN adapter
(CH340 + GD32, AT serial framing at 921600 baud).

## Installation

```bash
cd plugins/solarian-robstride
pip install -e .
```

After installation, Solarian discovers `RobstrideServo` / `RobstrideBus` through
the `solarian.actuators` and `solarian.actuator_buses` entry points.

## Usage

```python
from solarian.core.module import Module
from solarian_robstride.actuator import RobstrideServo

class MyArmModule(Module):
    shoulder_pan = RobstrideServo()
    shoulder_lift = RobstrideServo()
```

YAML `type: robstride`, `model: rs00` | `el05` | `rs05`, and `port` pointing at
the serial device (`/dev/cu.wchusbserial*` on macOS, `/dev/ttyUSB0` or
`/dev/ttyCH341USB0` on Linux).

## Adapter

The Seeed / Robstride USB-CAN board is a USB serial device, not SocketCAN.

1. DIP switch 1 OFF (not bootloader). DIP switch 2 ON for the 120 Ω terminator
   if this end of the bus needs it.
2. Install the CH340/CH341 driver if the OS does not already provide it.
3. Set `port` to the serial device. Baud rate is fixed at 921600.

Motors use Robstride private CAN 2.0B extended frames at 1 Mbps. This plugin
drives them in CSP (cyclic sync position) so each Solarian write is a `loc_ref`
target, analogous to Feetech `Goal_Position`.

## Calibration

Raw position is the protocol uint16 (0–65535) that encodes mechanical radians.
`solarian calibrate-module` records that span; `raw_to_radians` converts it
back using the motor model's `p_min` / `p_max`. Set mechanical zero with the
bus Type 6 set-zero command (the adapter GUI “set zero”, or
`set_half_turn_homings`) before sweeping limits.

## Requirements

- Solarian framework (>=0.1.0)
- pyserial (>=3.5)
