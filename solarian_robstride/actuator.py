"""
Robstride Servo Actuator

Actuator class for Robstride motors (RS00, EL05, RS05).
"""

from solarian.core.servo import Servo
from solarian.core.data import PosData, SpeedData, TemperatureData

from solarian_robstride.bus import RobstrideBus


class RobstrideServo(Servo):
    """
    Servo actuator for Robstride CAN motors.

    Declare it as a class attribute inside a Module::

        from solarian_robstride.actuator import RobstrideServo

        class MyArmModule(Module):
            shoulder_pan = RobstrideServo()
            elbow_flex = RobstrideServo()

    The corresponding :class:`RobstrideBus` is used automatically. YAML
    ``model`` must be ``rs00``, ``el05``, or ``rs05``.
    """

    bus_class = RobstrideBus

    pos = PosData()
    speed = SpeedData()
    temperature = TemperatureData()
