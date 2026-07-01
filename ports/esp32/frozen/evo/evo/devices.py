"""Pybricks-style device wrappers for EVO hardware."""

try:
    import _evo as _native_evo
except ImportError:
    import evo as _native_evo

from EvoColorSensor import EvoColorSensor
from EvoLineLeader import EvoLineLeader
from EvoServo import EvoServo
from EvoTOF import EvoTOF
from .parameters import Direction, Stop


class Motor:
    """Wrapper around the native EvoMotor class."""

    def __init__(self, port, positive_direction=Direction.CLOCKWISE, gears=None,
                 motor_type=None, **kwargs):
        self.port = port
        self.positive_direction = positive_direction
        self.gears = gears

        if motor_type is None:
            motor_type = getattr(_native_evo, "GENERICWITHENCODER", None)

        if motor_type is None:
            self._motor = _native_evo.EvoMotor(port=port, **kwargs)
        else:
            self._motor = _native_evo.EvoMotor(port=port, type=motor_type, **kwargs)

        if positive_direction == Direction.COUNTERCLOCKWISE:
            self._call_if_present("flipEncoderDirection")

    def _call_if_present(self, name, *args):
        method = getattr(self._motor, name, None)
        if method is None:
            return None
        return method(*args)

    def run(self, power):
        return self.dc(power)

    def dc(self, power):
        method = getattr(self._motor, "runPower", None)
        if method is not None:
            return method(power)
        return self._motor.run(power)

    def stop(self):
        return self._call_if_present("coast")

    def brake(self):
        return self._call_if_present("brake")

    def hold(self):
        return self._call_if_present("hold")

    def angle(self):
        method = getattr(self._motor, "getAngle", None)
        if method is not None:
            return method()
        return self._call_if_present("getPosition")

    def reset_angle(self, angle=0):
        method = getattr(self._motor, "resetAngle", None)
        if method is not None:
            result = method()
        else:
            result = self._call_if_present("resetPosition")
        if angle:
            raise NotImplementedError("reset_angle(angle) only supports angle=0")
        return result

    def run_time(self, speed, time_ms, then=Stop.COAST):
        method = getattr(self._motor, "runTime", None)
        if method is None:
            raise NotImplementedError("run_time requires native runTime support")
        return method(speed, time_ms)

    def run_angle(self, speed, rotation_angle, then=Stop.COAST):
        method = getattr(self._motor, "runAngle", None)
        if method is None:
            raise NotImplementedError("run_angle requires native runAngle support")
        return method(speed, rotation_angle)

    def speed(self):
        method = getattr(self._motor, "getSpeedDPS", None)
        if method is not None:
            return method()
        return self._call_if_present("getSpeed")

    def native(self):
        return self._motor

    def deinit(self):
        return self._call_if_present("deinit")


class Servo:
    """Wrapper around EvoServo."""

    def __init__(self, port, *args, **kwargs):
        self._servo = EvoServo(port, *args, **kwargs)

    def angle(self, angle):
        return self._servo.write(angle)

    def write(self, angle):
        return self._servo.write(angle)

    def position(self):
        return self._servo.getPosition()

    def deinit(self):
        return self._servo.deinit()

    def native(self):
        return self._servo


class ColorSensor:
    """Wrapper around EvoColorSensor."""

    def __init__(self, port, *args, **kwargs):
        self._sensor = EvoColorSensor(port, *args, **kwargs)

    def rgb(self):
        return self._sensor.getRGB()

    def hsv(self):
        return self._sensor.getHSV()

    def reflection(self):
        r, g, b = self.rgb()
        return (r + g + b) // 3

    def raw(self):
        return self._sensor.getRawRGB()

    def native(self):
        return self._sensor


class UltrasonicSensor:
    """Distance sensor wrapper around EvoTOF."""

    def __init__(self, port, *args, **kwargs):
        self._sensor = EvoTOF(port, *args, **kwargs)

    def distance(self):
        return self._sensor.getDistance()

    def start_continuous(self, period_ms=0):
        return self._sensor.startContinuous(period_ms)

    def stop_continuous(self):
        return self._sensor.stopContinuous()

    def read_continuous(self):
        return self._sensor.readContinuousMM()

    def native(self):
        return self._sensor


class LineSensor:
    """Line sensor array wrapper around EvoLineLeader."""

    def __init__(self, port, *args, **kwargs):
        self._sensor = EvoLineLeader(port, *args, **kwargs)

    def read(self, index):
        return self._sensor.readPosition(index)

    def read_all(self):
        return self._sensor.readAll()

    def native(self):
        return self._sensor
