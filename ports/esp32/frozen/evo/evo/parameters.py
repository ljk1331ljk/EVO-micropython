"""Named parameters for the EVO v2 API."""

try:
    from micropython import const
except ImportError:
    def const(value):
        return value

try:
    import pins
except ImportError:
    pins = None

try:
    import _evo as _native_evo
except ImportError:
    _native_evo = None


def _pin(name, default):
    if pins is not None and hasattr(pins, name):
        return getattr(pins, name)
    return const(default)


def _native(name, default):
    if _native_evo is not None and hasattr(_native_evo, name):
        return getattr(_native_evo, name)
    return const(default)


class Port:
    """Motor, sensor, servo, and I2C port names."""

    A = _pin("M1", 1)
    B = _pin("M2", 2)
    C = _pin("M3", 3)
    D = _pin("M4", 4)

    M1 = A
    M2 = B
    M3 = C
    M4 = D

    S1 = _pin("S1", _pin("GPIO1", 10))
    S2 = _pin("S2", _pin("GPIO2", 9))
    S3 = _pin("S3", _pin("GPIO3", 8))
    S4 = _pin("S4", _pin("GPIO4", 3))
    S5 = _pin("S5", _pin("GPIO5", 7))
    S6 = _pin("S6", _pin("GPIO6", 6))
    S7 = _pin("S7", _pin("GPIO7", 5))
    S8 = _pin("S8", _pin("GPIO8", 4))

    I2C1 = _pin("I2C1", 0)
    I2C2 = _pin("I2C2", 1)
    I2C3 = _pin("I2C3", 2)
    I2C4 = _pin("I2C4", 3)
    I2C5 = _pin("I2C5", 4)
    I2C6 = _pin("I2C6", 5)
    I2C7 = _pin("I2C7", 6)
    I2C8 = _pin("I2C8", 7)

    SERVO1 = _pin("SERVO1", 101)
    SERVO2 = _pin("SERVO2", 102)
    SERVO3 = _pin("SERVO3", 103)
    SERVO4 = _pin("SERVO4", 104)
    SERVO5 = _pin("SERVO5", 105)
    SERVO6 = _pin("SERVO6", 106)
    SERVO7 = _pin("SERVO7", 107)
    SERVO8 = _pin("SERVO8", 108)


class Direction:
    """Positive motor direction."""

    CLOCKWISE = const(1)
    COUNTERCLOCKWISE = const(-1)


class Stop:
    """Motor stop behavior."""

    COAST = _native("COAST", 0)
    BRAKE = _native("BRAKE", 1)
    HOLD = _native("HOLD", 2)


class Button:
    """Hub button names."""

    LEFT = const(1)
    CENTER = const(2)
    RIGHT = const(3)
    UP = const(4)
    DOWN = const(5)


class Color:
    """Common color names."""

    BLACK = const(0)
    BLUE = const(1)
    GREEN = const(2)
    YELLOW = const(3)
    RED = const(4)
    WHITE = const(5)
    BROWN = const(6)
    ORANGE = const(7)
    PURPLE = const(8)
    NONE = const(-1)
