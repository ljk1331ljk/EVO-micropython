# EvoColorSensor.py
# TCS34725-style color sensor driver adapted for Evo (I2CDevice + mux channels)
#
# Required API:
#   getRawRed(), getRawGreen(), getRawBlue(), getRawClear()
#   getRed(), getGreen(), getBlue()
#   getRawRGB(), getRGB(), getHSV()
#
# Fixes included:
# 1) Explicitly switches the I2C multiplexer channel BEFORE every sensor access.
# 2) Ensures the sensor is actually ENABLED on startup (PON + AEN). In your earlier version,
#    _active started True and never wrote ENABLE, so the sensor could stay off forever.

from micropython import const
from I2CDevice import *
import time
import ustruct

_COMMAND_BIT = const(0x80)

_REGISTER_ENABLE  = const(0x00)
_REGISTER_ATIME   = const(0x01)
_REGISTER_AILT    = const(0x04)
_REGISTER_AIHT    = const(0x06)
_REGISTER_APERS   = const(0x0C)
_REGISTER_CONTROL = const(0x0F)
_REGISTER_SENSORID = const(0x12)
_REGISTER_STATUS  = const(0x13)

_REGISTER_CDATA = const(0x14)
_REGISTER_RDATA = const(0x16)
_REGISTER_GDATA = const(0x18)
_REGISTER_BDATA = const(0x1A)

_ENABLE_AIEN = const(0x10)
_ENABLE_WEN  = const(0x08)
_ENABLE_AEN  = const(0x02)
_ENABLE_PON  = const(0x01)

_GAINS  = (1, 4, 16, 60)
_CYCLES = (0, 1, 2, 3, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60)


class EvoColorSensor:
    """
    Non-blocking behavior:
    - If data isn't valid yet, returns the last cached reading.
    """

    def __init__(self, channel, i2c=I2CA, address=0x29):
        self.i2c = i2c
        self.channel = channel
        self.address = address

        self._integration_time = 2.4
        self._raw = (0, 0, 0, 0)  # (r,g,b,c)

        # IMPORTANT: start inactive so setActive(True) will actually write ENABLE bits
        self._active = False

        # Apply defaults
        self.setIntegrationTime(2.4)
        self.setGain(60)

        # Power up + enable ADC
        self.setActive(True)

        # Cache sensor id (optional)
        self.sensorId = self.getSensorId()

    # -----------------------------
    # Mux channel enforcement
    # -----------------------------
    def _select(self):
        # Force channel switch before any access (even if I2CDevice also does it internally)
        # This protects you from cases where direct machine.I2C is used elsewhere.
        self.i2c.switch_channel(self.channel)

    # -----------------------------
    # Low-level register access
    # -----------------------------
    def _reg8(self, reg, value=None):
        self._select()
        reg |= _COMMAND_BIT
        if value is None:
            # Hardcoded 0x29 reads kept intentionally
            return self.i2c.readfrom_mem(self.channel, 0x29, reg, 1)[0]
        self.i2c.writeto_mem(self.channel, self.address, reg, ustruct.pack("<B", value))

    def _reg16(self, reg, value=None):
        self._select()
        reg |= _COMMAND_BIT
        if value is None:
            # Hardcoded 0x29 reads kept intentionally
            data = self.i2c.readfrom_mem(self.channel, 0x29, reg, 2)
            return ustruct.unpack("<H", data)[0]
        self.i2c.writeto_mem(self.channel, self.address, reg, ustruct.pack("<H", value))

    # -----------------------------
    # Power / enable
    # -----------------------------
    def setActive(self, enable=True):
        enable = bool(enable)
        if self._active == enable:
            return

        self._active = enable
        en = self._reg8(_REGISTER_ENABLE)

        if enable:
            # Power on then enable RGBC
            self._reg8(_REGISTER_ENABLE, en | _ENABLE_PON)
            time.sleep_ms(3)
            self._reg8(_REGISTER_ENABLE, en | _ENABLE_PON | _ENABLE_AEN)
        else:
            self._reg8(_REGISTER_ENABLE, en & ~(_ENABLE_PON | _ENABLE_AEN))

    def isActive(self):
        return self._active

    # Backward alias
    def active(self, value=None):
        if value is None:
            return self.isActive()
        self.setActive(value)

    # -----------------------------
    # Settings
    # -----------------------------
    def getSensorId(self):
        return self._reg8(_REGISTER_SENSORID)

    def setIntegrationTime(self, ms):
        ms = min(614.4, max(2.4, float(ms)))
        cycles = int(ms / 2.4)
        if cycles < 1:
            cycles = 1
        if cycles > 256:
            cycles = 256
        self._integration_time = cycles * 2.4
        self._reg8(_REGISTER_ATIME, 256 - cycles)
        return self._integration_time

    def getIntegrationTime(self):
        return self._integration_time

    def setGain(self, gain):
        if gain not in _GAINS:
            raise ValueError("gain must be 1, 4, 16 or 60")
        self._reg8(_REGISTER_CONTROL, _GAINS.index(gain))
        return gain

    def getGain(self):
        return _GAINS[self._reg8(_REGISTER_CONTROL)]

    # -----------------------------
    # Read helpers
    # -----------------------------
    def _isValid(self):
        return bool(self._reg8(_REGISTER_STATUS) & 0x01)

    def _refreshRawIfValid(self):
        """
        Refresh cached raw RGBC if sensor says data is valid.
        Non-blocking: if not valid, keep existing cache.
        """
        if not self._isValid():
            return self._raw

        # IMPORTANT: these reads also call _select() via _reg16,
        # so channel is guaranteed set right before each access.
        self._raw = (
            self._reg16(_REGISTER_RDATA),
            self._reg16(_REGISTER_GDATA),
            self._reg16(_REGISTER_BDATA),
            self._reg16(_REGISTER_CDATA),
        )
        return self._raw

    @staticmethod
    def _normalized_rgb(r, g, b, c):
        """
        Convert raw (r,g,b,c) to normalized 0..255 RGB using clear channel scaling.
        """
        if c <= 0:
            return 0, 0, 0

        rr = int((r * 255) / c)
        gg = int((g * 255) / c)
        bb = int((b * 255) / c)

        if rr < 0:
            rr = 0
        elif rr > 255:
            rr = 255

        if gg < 0:
            gg = 0
        elif gg > 255:
            gg = 255

        if bb < 0:
            bb = 0
        elif bb > 255:
            bb = 255

        return rr, gg, bb

    @staticmethod
    def _rgb_to_hsv(r, g, b):
        """
        r,g,b in [0..255] -> (h,s,v)
        h in degrees [0..360), s and v in [0..1]
        """
        rf = r / 255.0
        gf = g / 255.0
        bf = b / 255.0

        cmax = rf
        if gf > cmax:
            cmax = gf
        if bf > cmax:
            cmax = bf

        cmin = rf
        if gf < cmin:
            cmin = gf
        if bf < cmin:
            cmin = bf

        delta = cmax - cmin

        if delta == 0:
            h = 0.0
        elif cmax == rf:
            h = 60.0 * (((gf - bf) / delta) % 6.0)
        elif cmax == gf:
            h = 60.0 * (((bf - rf) / delta) + 2.0)
        else:
            h = 60.0 * (((rf - gf) / delta) + 4.0)

        s = 0.0 if cmax == 0 else (delta / cmax)
        v = cmax

        if h < 0:
            h += 360.0
        elif h >= 360.0:
            h = h % 360.0

        return h, s, v

    # -----------------------------
    # Requested API
    # -----------------------------
    def getRawRed(self):
        r, g, b, c = self._refreshRawIfValid()
        return r

    def getRawGreen(self):
        r, g, b, c = self._refreshRawIfValid()
        return g

    def getRawBlue(self):
        r, g, b, c = self._refreshRawIfValid()
        return b

    def getRawClear(self):
        r, g, b, c = self._refreshRawIfValid()
        return c

    def getRawRGB(self):
        r, g, b, c = self._refreshRawIfValid()
        return (r, g, b)

    def getRed(self):
        r, g, b, c = self._refreshRawIfValid()
        rr, gg, bb = self._normalized_rgb(r, g, b, c)
        return rr

    def getGreen(self):
        r, g, b, c = self._refreshRawIfValid()
        rr, gg, bb = self._normalized_rgb(r, g, b, c)
        return gg

    def getBlue(self):
        r, g, b, c = self._refreshRawIfValid()
        rr, gg, bb = self._normalized_rgb(r, g, b, c)
        return bb

    def getRGB(self):
        r, g, b, c = self._refreshRawIfValid()
        return self._normalized_rgb(r, g, b, c)

    def getHSV(self):
        rr, gg, bb = self.getRGB()
        return self._rgb_to_hsv(rr, gg, bb)

    # -----------------------------
    # Threshold / interrupt (kept)
    # -----------------------------
    def getThreshold(self):
        min_value = self._reg16(_REGISTER_AILT)
        max_value = self._reg16(_REGISTER_AIHT)
        if self._reg8(_REGISTER_ENABLE) & _ENABLE_AIEN:
            cycles = _CYCLES[self._reg8(_REGISTER_APERS) & 0x0F]
        else:
            cycles = -1
        return cycles, min_value, max_value

    def setThreshold(self, cycles=None, min_value=None, max_value=None):
        if min_value is not None:
            self._reg16(_REGISTER_AILT, int(min_value))
        if max_value is not None:
            self._reg16(_REGISTER_AIHT, int(max_value))

        if cycles is not None:
            en = self._reg8(_REGISTER_ENABLE)
            if cycles == -1:
                self._reg8(_REGISTER_ENABLE, en & ~(_ENABLE_AIEN))
            else:
                if cycles not in _CYCLES:
                    raise ValueError("invalid persistence cycles")
                self._reg8(_REGISTER_ENABLE, en | _ENABLE_AIEN)
                self._reg8(_REGISTER_APERS, _CYCLES.index(cycles))

    def interrupt(self, value=None):
        if value is None:
            return bool(self._reg8(_REGISTER_STATUS) & _ENABLE_AIEN)
        if value:
            raise ValueError("interrupt can only be cleared")
        # Clear interrupt special function (device specific)
        self.i2c.writeto(self.channel, self.address, b"\xE6")


# -----------------------------
# Simple test
# -----------------------------
if __name__ == "__main__":
    ls1 = EvoColorSensor(I2C1)
    ls2 = EvoColorSensor(I2C2)

    while True:
        print("LS1 raw:", ls1.getRawRed(), ls1.getRawGreen(), ls1.getRawBlue(), ls1.getRawClear(), "RGB:", ls1.getRGB(), "HSV:", ls1.getHSV())
        print("LS2 raw:", ls2.getRawRed(), ls2.getRawGreen(), ls2.getRawBlue(), ls2.getRawClear(), "RGB:", ls2.getRGB(), "HSV:", ls2.getHSV())
        time.sleep_ms(100)