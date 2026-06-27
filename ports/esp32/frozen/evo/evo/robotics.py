"""Robot drive helpers for the EVO v2 API."""

try:
    import _evo as _native_evo
except ImportError:
    import evo as _native_evo

from .devices import Motor
from .parameters import Stop


def _native_motor(motor):
    if isinstance(motor, Motor):
        return motor.native()
    return motor


class MotorPair:
    """Wrapper around the native EvoMotorPair."""

    def __init__(self, left_motor, right_motor):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self._pair = _native_evo.EvoMotorPair(
            _native_motor(left_motor),
            _native_motor(right_motor),
        )

    def move(self, speed, steering=0):
        return self._pair.move(speed, steering)

    def dc(self, power, steering=0):
        return self._pair.movePower(power, steering)

    def run_angle(self, speed, rotation_angle, steering=0, then=Stop.COAST):
        return self._pair.moveDegrees(speed, steering, rotation_angle)

    def run_time(self, speed, time_ms, steering=0, then=Stop.COAST):
        return self._pair.moveTime(speed, steering, time_ms)

    def stop(self, then=Stop.COAST):
        return self._pair.stop(then)

    def brake(self):
        return self._pair.brake()

    def hold(self):
        return self._pair.hold()

    def native(self):
        return self._pair


class DriveBase:
    """Educational differential-drive helper backed by MotorPair."""

    def __init__(self, left_motor, right_motor, wheel_diameter=None, axle_track=None):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track
        self._pair = MotorPair(left_motor, right_motor)

    def straight(self, distance, speed=50, then=Stop.COAST):
        return self._pair.run_angle(speed, distance, 0, then)

    def turn(self, angle, speed=50, then=Stop.COAST):
        steering = 100 if angle >= 0 else -100
        return self._pair.run_angle(speed, abs(angle), steering, then)

    def drive(self, speed, turn_rate=0):
        return self._pair.move(speed, turn_rate)

    def stop(self, then=Stop.COAST):
        return self._pair.stop(then)


class MecanumDrive:
    """Wrapper around the native EvoMecanum class."""

    def __init__(self, front_left, front_right, rear_left, rear_right):
        self._drive = _native_evo.EvoMecanum(
            _native_motor(front_left),
            _native_motor(front_right),
            _native_motor(rear_left),
            _native_motor(rear_right),
        )

    def move(self, speed, strafe=0, turn=0):
        return self._drive.move(speed, strafe, turn)

    def dc(self, power, strafe=0, turn=0):
        return self._drive.movePower(power, strafe, turn)

    def stop(self, then=Stop.COAST):
        return self._drive.stop(then)

    def brake(self):
        return self._drive.brake()

    def hold(self):
        return self._drive.hold()

    def native(self):
        return self._drive
