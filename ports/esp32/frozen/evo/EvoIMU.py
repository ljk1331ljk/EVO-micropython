from micropython import const
from I2CDevice import *
import time
import ustruct
import math


_BNO055_CHIP_ID_ADDR      = const(0x00)
_BNO055_PAGE_ID_ADDR      = const(0x07)
_BNO055_EULER_H_LSB_ADDR  = const(0x1A)
_BNO055_UNIT_SEL_ADDR     = const(0x3B)
_BNO055_OPR_MODE_ADDR     = const(0x3D)
_BNO055_PWR_MODE_ADDR     = const(0x3E)
_BNO055_SYS_TRIGGER_ADDR  = const(0x3F)

_BNO055_ID = const(0xA0)

_POWER_MODE_NORMAL = const(0x00)

_OPERATION_MODE_CONFIG = const(0x00)
_OPERATION_MODE_NDOF   = const(0x0C)


class EvoIMU:
    def __init__(self, channel, i2c=I2CA, address=0x28):
        self.i2c = i2c
        self.channel = channel
        self.address = address

        self._referenceHeading = 0.0
        self._lastRelativeHeading = 0.0
        self._rotationCount = 0
        self._mode = _OPERATION_MODE_CONFIG

        self._init_sensor()

    # -----------------------------
    # Mux channel enforcement
    # -----------------------------
    def _select(self):
        self.i2c.switch_channel(self.channel)

    # -----------------------------
    # Low-level register access
    # -----------------------------
    def _reg8(self, reg, value=None):
        self._select()
        if value is None:
            return self.i2c.readfrom_mem(self.channel, self.address, reg, 1)[0]
        self.i2c.writeto_mem(self.channel, self.address, reg, ustruct.pack("<B", value))

    def _reg16(self, reg, value=None):
        self._select()
        if value is None:
            data = self.i2c.readfrom_mem(self.channel, self.address, reg, 2)
            return ustruct.unpack("<H", data)[0]
        self.i2c.writeto_mem(self.channel, self.address, reg, ustruct.pack("<H", value))

    def _setMode(self, mode):
        self._reg8(_BNO055_OPR_MODE_ADDR, mode)
        self._mode = mode
        time.sleep_ms(30)

    def _readEulerRaw(self):
        self._select()
        data = self.i2c.readfrom_mem(self.channel, self.address, _BNO055_EULER_H_LSB_ADDR, 6)
        return ustruct.unpack("<hhh", data)

    # -----------------------------
    # Initialisation
    # -----------------------------
    def _init_sensor(self):
        time.sleep_ms(700)

        chip_id = self._reg8(_BNO055_CHIP_ID_ADDR)
        if chip_id != _BNO055_ID:
            time.sleep_ms(1000)
            chip_id = self._reg8(_BNO055_CHIP_ID_ADDR)
            if chip_id != _BNO055_ID:
                raise OSError("BNO055 not detected")

        self._setMode(_OPERATION_MODE_CONFIG)

        # Reset sensor
        self._reg8(_BNO055_SYS_TRIGGER_ADDR, 0x20)
        time.sleep_ms(700)

        timeout = time.ticks_ms()
        while self._reg8(_BNO055_CHIP_ID_ADDR) != _BNO055_ID:
            if time.ticks_diff(time.ticks_ms(), timeout) > 2000:
                raise OSError("BNO055 reset timeout")
            time.sleep_ms(10)

        time.sleep_ms(50)

        self._reg8(_BNO055_PWR_MODE_ADDR, _POWER_MODE_NORMAL)
        time.sleep_ms(10)

        self._reg8(_BNO055_PAGE_ID_ADDR, 0)
        self._reg8(_BNO055_UNIT_SEL_ADDR, 0x00)
        self._reg8(_BNO055_SYS_TRIGGER_ADDR, 0x00)
        time.sleep_ms(10)

        self._setMode(_OPERATION_MODE_NDOF)
        time.sleep_ms(20)

    # -----------------------------
    # Requested API
    # -----------------------------
    def resetHeading(self):
        self._referenceHeading = self.getEulerX()
        self._rotationCount = 0
        self._lastRelativeHeading = 0.0

    def getRelativeHeading(self):
        heading = self.getEulerX() - self._referenceHeading

        relativeHeading = math.fmod((heading + 180.0), 360.0) - 180.0
        if relativeHeading <= -180.0:
            relativeHeading += 360.0

        delta = relativeHeading - self._lastRelativeHeading

        if delta >= 180.0:
            self._rotationCount -= 1
        elif delta <= -180.0:
            self._rotationCount += 1

        self._lastRelativeHeading = relativeHeading
        return relativeHeading + (self._rotationCount * 360.0)

    def getEuler(self):
        x, y, z = self._readEulerRaw()
        return (x / 16.0, y / 16.0, z / 16.0)

    def getEulerX(self):
        return self.getEuler()[0]

    def getEulerY(self):
        return self.getEuler()[1]

    def getEulerZ(self):
        return self.getEuler()[2]