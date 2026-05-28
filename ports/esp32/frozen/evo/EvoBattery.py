from micropython import const
from I2CDevice import I2CB

_BQ25887_ADDR = const(0x6A)

_REG_CHARGER_CONTROL_3 = const(0x07)
_REG_ADC_CONTROL       = const(0x15)
_REG_VBAT_ADC_1        = const(0x1D)

class EvoBattery:
    def __init__(self):
        self._i2c = I2CB
        self._channel = 0   # ignored on I2CB, but required by your API

    def _read8(self, reg):
        return self._i2c.readfrom_mem(self._channel, _BQ25887_ADDR, reg, 1)[0]

    def _write8(self, reg, value):
        self._i2c.writeto_mem(self._channel, _BQ25887_ADDR, reg, bytes([value & 0xFF]))

    def _read16(self, reg):
        data = self._i2c.readfrom_mem(self._channel, _BQ25887_ADDR, reg, 2)
        return (data[0] << 8) | data[1]

    def _wdReset(self):
        v = self._read8(_REG_CHARGER_CONTROL_3)
        v = (v & 0b10111111) | 0b01000000
        self._write8(_REG_CHARGER_CONTROL_3, v)

    def _setADC_EN(self, enable=True):
        v = self._read8(_REG_ADC_CONTROL)
        v = v & 0b01111111
        if enable:
            v |= 0b10000000
        self._write8(_REG_ADC_CONTROL, v)

    def getBatteryVoltage(self):
        self._wdReset()
        self._setADC_EN(True)
        raw = self._read16(_REG_VBAT_ADC_1)
        return raw / 1000.0


BATTERY = EvoBattery()

def getBatteryVoltage():
    return BATTERY.getBatteryVoltage()