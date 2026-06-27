"""Hub-level API for EVO boards."""

try:
    import _evo as _native_evo
except ImportError:
    import evo as _native_evo

try:
    import pins
except ImportError:
    pins = None

try:
    from EvoBattery import EvoBattery
except ImportError:
    EvoBattery = None


class _Battery:
    def __init__(self):
        self._battery = EvoBattery() if EvoBattery is not None else None

    def voltage(self):
        if self._battery is None:
            return None
        return self._battery.getBatteryVoltage()


class _Buttons:
    def __init__(self):
        self._pins = {}
        if pins is None:
            return
        for name in ("BUTTON_PIN", "BUTTON_L_PIN", "BUTTON_C_PIN", "BUTTON_R_PIN"):
            if hasattr(pins, name):
                self._pins[name] = getattr(pins, name)

    def pressed(self):
        result = []
        for name, pin in self._pins.items():
            is_pressed = getattr(_native_evo, "is_pressed", None)
            if is_pressed is not None and is_pressed(pin):
                result.append(name)
        return tuple(result)


class EvoHub:
    """Central access point for board-level features."""

    def __init__(self):
        self.battery = _Battery()
        self.buttons = _Buttons()
        self.display = None

    def name(self):
        get_name = getattr(_native_evo, "get_name", None)
        if get_name is None:
            return "Evo"
        return get_name()

    def firmware_version(self):
        get_version = getattr(_native_evo, "get_firmware_version", None)
        if get_version is None:
            return None
        return get_version()
