# EvoLineLeader.py
# Evo Line Leader driver adapted for Evo (I2CDevice + mux channels)
#
# Required API:
#   readPosition(position)
#   setOversamplingRatio(ratio)
#   getOversamplingRatio()
#   readAll()

from micropython import const
from I2CDevice import *


# Registers
SYSTEM_STATUS_REGISTER   = const(0x00)
DATA_CFG_REGISTER        = const(0x02)
OSR_CFG_REGISTER         = const(0x03)
PIN_CFG_REGISTER         = const(0x05)
GPIO_CFG_REGISTER        = const(0x07)
GPO_DRIVE_CFG_REGISTER   = const(0x09)
CHANNEL_SEL_REGISTER     = const(0x11)

# Commands
_CMD_WRITE_REGISTER = const(0x08)
_CMD_READ_REGISTER  = const(0x10)


class EvoLineLeader:
    def __init__(self, channel, i2c=I2CA, address=0x10):
        self.i2c = i2c
        self.channel = channel
        self.address = address
        self._position = None

    # -----------------------------
    # Mux channel enforcement
    # -----------------------------
    def _select(self):
        self.i2c.switch_channel(self.channel)

    # -----------------------------
    # Low-level access
    # -----------------------------
    def _writeto(self, data):
        self._select()
        self.i2c.writeto(self.channel, self.address, data)

    def _readfrom(self, nbytes):
        self._select()
        return self.i2c.readfrom(self.channel, self.address, nbytes)

    def _writeRegister(self, reg, value):
        self._writeto(bytes((_CMD_WRITE_REGISTER, reg, value)))

    def _prepareReadRegister(self, reg):
        self._writeto(bytes((_CMD_READ_REGISTER, reg)))

    # -----------------------------
    # Requested API
    # -----------------------------
    def readPosition(self, position):
        """
        Read ADC value from position 0..7.
        Returned value is inverted: 4095 - raw_value
        """
        if position < 0 or position > 7:
            raise ValueError("position must be 0..7")

        if self._position != position:
            self._writeRegister(CHANNEL_SEL_REGISTER, position)
            self._position = position

        raw = self._readfrom(2)
        if len(raw) != 2:
            return 0

        value = (((raw[0] << 8) | raw[1]) >> 4) & 0x0FFF
        return 4095 - value

    def setOversamplingRatio(self, ratio):
        """
        ratio: 0..7
        """
        if ratio < 0 or ratio > 7:
            raise ValueError("ratio must be 0..7")

        self._writeRegister(OSR_CFG_REGISTER, ratio)

    def getOversamplingRatio(self):
        self._prepareReadRegister(OSR_CFG_REGISTER)
        data = self._readfrom(1)
        if len(data) != 1:
            return 0
        return data[0] & 0x07

    def readAll(self):
        return (
            self.readPosition(0),
            self.readPosition(1),
            self.readPosition(2),
            self.readPosition(3),
            self.readPosition(4),
            self.readPosition(5),
            self.readPosition(6),
            self.readPosition(7),
        )