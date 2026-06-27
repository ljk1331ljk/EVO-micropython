"""EVO firmware v2 package API.

This module keeps the existing native ``evo`` names available while adding a
Pybricks-inspired package layout for new programs.
"""

try:
    import _evo as _native_evo
    from _evo import *
    _native_all = tuple(name for name in dir(_native_evo) if not name.startswith("_"))
except ImportError:
    _native_all = ()

from .devices import ColorSensor, LineSensor, Motor, Servo, UltrasonicSensor
from .hubs import EvoHub
from .parameters import Button, Color, Direction, Port, Stop
from .robotics import DriveBase, MecanumDrive, MotorPair
from .tools import StopWatch, wait

__all__ = _native_all + (
    "EvoHub",
    "Motor",
    "Servo",
    "ColorSensor",
    "UltrasonicSensor",
    "LineSensor",
    "Button",
    "Color",
    "Direction",
    "Port",
    "Stop",
    "DriveBase",
    "MotorPair",
    "MecanumDrive",
    "wait",
    "StopWatch",
)
