# EvoPWMDriver.py
# PCA9685 PWM / servo expander driver for Evo
#
# Style aligned with other Evo frozen modules:
# - channel is I2C1 .. I2C8
# - i2c object defaults to I2CA / I2CB style bus object
# - explicit mux channel selection before every access
#
# Main API:
#   begin()
#   reset()
#   sleep()
#   wake()
#   setPWMFreq(freq)
#   getPWMFreq()
#   setPWM(channel, on, off)
#   getPWM(channel)
#   setDuty(channel, duty)
#   getDuty(channel)
#   setPulse(channel, pulse_us)
#   getPulse(channel)
#   setAngle(channel, angle)
#   setChannelPulseRange(channel, min_us, max_us)
#   getChannelPulseRange(channel)
#   setChannelAngleRange(channel, min_angle, max_angle)
#   getChannelAngleRange(channel)
#   stop(channel)
#   allOff()
#   allOn()

from micropython import const
from I2CDevice import *
import time
import ustruct

# ---------------------------------------------------------
# PCA9685 registers
# ---------------------------------------------------------
_MODE1         = const(0x00)
_MODE2         = const(0x01)
_SUBADR1       = const(0x02)
_SUBADR2       = const(0x03)
_SUBADR3       = const(0x04)
_PRESCALE      = const(0xFE)

_LED0_ON_L     = const(0x06)
_LED0_ON_H     = const(0x07)
_LED0_OFF_L    = const(0x08)
_LED0_OFF_H    = const(0x09)

_ALL_LED_ON_L  = const(0xFA)
_ALL_LED_ON_H  = const(0xFB)
_ALL_LED_OFF_L = const(0xFC)
_ALL_LED_OFF_H = const(0xFD)

# MODE1 bits
_RESTART = const(0x80)
_EXTCLK  = const(0x40)
_AI      = const(0x20)
_SLEEP   = const(0x10)
_ALLCALL = const(0x01)

# MODE2 bits
_OUTDRV  = const(0x04)

# defaults
_DEFAULT_ADDR          = const(0x40)
_DEFAULT_FREQ          = const(100)
_OSC_CLOCK             = const(25000000)
_PWM_STEPS             = const(4096)

_DEFAULT_MIN_PULSE_US  = const(500)
_DEFAULT_MAX_PULSE_US  = const(2500)
_DEFAULT_MIN_ANGLE     = const(0)
_DEFAULT_MAX_ANGLE     = const(180)

# public channel constants
CH0  = const(0)
CH1  = const(1)
CH2  = const(2)
CH3  = const(3)
CH4  = const(4)
CH5  = const(5)
CH6  = const(6)
CH7  = const(7)
CH8  = const(8)
CH9  = const(9)
CH10 = const(10)
CH11 = const(11)
CH12 = const(12)
CH13 = const(13)
CH14 = const(14)
CH15 = const(15)


class EvoPWMDriver:
    def __init__(self, channel, i2c=I2CA, address=_DEFAULT_ADDR, freq=_DEFAULT_FREQ):
        self.i2c = i2c
        self.channel = channel
        self.address = address
        self._freq = int(freq)
        self._begun = False

        # Per-channel calibration
        self._min_pulse_us = [_DEFAULT_MIN_PULSE_US] * 16
        self._max_pulse_us = [_DEFAULT_MAX_PULSE_US] * 16
        self._min_angle = [_DEFAULT_MIN_ANGLE] * 16
        self._max_angle = [_DEFAULT_MAX_ANGLE] * 16

    # -----------------------------
    # Mux channel enforcement
    # -----------------------------
    def _select(self):
        self.i2c.switch_channel(self.channel)

    # -----------------------------
    # Validation helpers
    # -----------------------------
    @staticmethod
    def _check_channel(pwm_channel):
        pwm_channel = int(pwm_channel)
        if pwm_channel < 0 or pwm_channel > 15:
            raise ValueError("invalid pwm channel")
        return pwm_channel

    @staticmethod
    def _clamp(v, lo, hi):
        if v < lo:
            return lo
        if v > hi:
            return hi
        return v

    # -----------------------------
    # Low-level register access
    # -----------------------------
    def _reg8(self, reg, value=None):
        self._select()
        if value is None:
            return self.i2c.readfrom_mem(self.channel, self.address, reg, 1)[0]
        self.i2c.writeto_mem(self.channel, self.address, reg, ustruct.pack("<B", value & 0xFF))

    def _reg_bytes(self, reg, n):
        self._select()
        return self.i2c.readfrom_mem(self.channel, self.address, reg, n)

    def _write_bytes(self, reg, data):
        self._select()
        self.i2c.writeto_mem(self.channel, self.address, reg, data)

    # -----------------------------
    # Basic control
    # -----------------------------
    def begin(self):
        if self._begun:
            return True

        self.reset()

        self._reg8(_MODE2, _OUTDRV)
        self._reg8(_MODE1, _AI | _ALLCALL)
        time.sleep_ms(5)

        self.wake()
        self.setPWMFreq(self._freq)
        self.allOff()

        self._begun = True
        return True

    def reset(self):
        self._reg8(_MODE1, _ALLCALL)
        time.sleep_ms(5)

    def sleep(self):
        mode1 = self._reg8(_MODE1)
        self._reg8(_MODE1, mode1 | _SLEEP)
        time.sleep_ms(5)

    def wake(self):
        mode1 = self._reg8(_MODE1)
        self._reg8(_MODE1, mode1 & ~_SLEEP)
        time.sleep_ms(5)

    def isSleeping(self):
        return bool(self._reg8(_MODE1) & _SLEEP)

    def getAddress(self):
        return self.address

    # -----------------------------
    # Frequency
    # -----------------------------
    def setPWMFreq(self, freq):
        freq = int(freq)
        if freq < 1:
            freq = 1
        elif freq > 3500:
            freq = 3500

        self._freq = freq

        prescale = int((_OSC_CLOCK + (freq * _PWM_STEPS // 2)) // (freq * _PWM_STEPS) - 1)
        if prescale < 3:
            prescale = 3
        elif prescale > 255:
            prescale = 255

        oldmode = self._reg8(_MODE1)
        newmode = (oldmode & ~_RESTART) | _SLEEP
        self._reg8(_MODE1, newmode)
        self._reg8(_PRESCALE, prescale)
        self._reg8(_MODE1, oldmode)
        time.sleep_ms(5)
        self._reg8(_MODE1, oldmode | _RESTART | _AI)

        return self._freq

    def getPWMFreq(self):
        return self._freq

    # -----------------------------
    # Per-channel pulse calibration
    # -----------------------------
    def setChannelPulseRange(self, pwm_channel, min_us, max_us):
        pwm_channel = self._check_channel(pwm_channel)
        min_us = int(min_us)
        max_us = int(max_us)

        if min_us < 0 or max_us < 0:
            raise ValueError("pulse must be >= 0")
        if min_us >= max_us:
            raise ValueError("min_us must be < max_us")

        self._min_pulse_us[pwm_channel] = min_us
        self._max_pulse_us[pwm_channel] = max_us

    def getChannelPulseRange(self, pwm_channel):
        pwm_channel = self._check_channel(pwm_channel)
        return (self._min_pulse_us[pwm_channel], self._max_pulse_us[pwm_channel])

    # -----------------------------
    # Per-channel angle calibration
    # -----------------------------
    def setChannelAngleRange(self, pwm_channel, min_angle, max_angle):
        pwm_channel = self._check_channel(pwm_channel)
        min_angle = int(min_angle)
        max_angle = int(max_angle)

        if min_angle >= max_angle:
            raise ValueError("min_angle must be < max_angle")

        self._min_angle[pwm_channel] = min_angle
        self._max_angle[pwm_channel] = max_angle

    def getChannelAngleRange(self, pwm_channel):
        pwm_channel = self._check_channel(pwm_channel)
        return (self._min_angle[pwm_channel], self._max_angle[pwm_channel])

    # -----------------------------
    # Raw PWM control
    # -----------------------------
    def setPWM(self, pwm_channel, on, off):
        pwm_channel = self._check_channel(pwm_channel)

        on = self._clamp(int(on), 0, 4095)
        off = self._clamp(int(off), 0, 4095)

        reg = _LED0_ON_L + (4 * pwm_channel)
        self._write_bytes(reg, ustruct.pack("<HH", on, off))

    def getPWM(self, pwm_channel):
        pwm_channel = self._check_channel(pwm_channel)

        reg = _LED0_ON_L + (4 * pwm_channel)
        data = self._reg_bytes(reg, 4)
        on, off = ustruct.unpack("<HH", data)
        on &= 0x0FFF
        off &= 0x0FFF
        return (on, off)

    def setDuty(self, pwm_channel, duty):
        duty = self._clamp(int(duty), 0, 4095)
        self.setPWM(pwm_channel, 0, duty)

    def getDuty(self, pwm_channel):
        on, off = self.getPWM(pwm_channel)
        return off

    # -----------------------------
    # Servo pulse helpers
    # -----------------------------
    def setPulse(self, pwm_channel, pulse_us):
        pwm_channel = self._check_channel(pwm_channel)

        pulse_us = int(pulse_us)
        if pulse_us < 0:
            pulse_us = 0

        ticks = (pulse_us * self._freq * _PWM_STEPS) // 1000000
        if ticks > 4095:
            ticks = 4095

        self.setPWM(pwm_channel, 0, ticks)

    def getPulse(self, pwm_channel):
        pwm_channel = self._check_channel(pwm_channel)
        duty = self.getDuty(pwm_channel)
        return (duty * 1000000) // (self._freq * _PWM_STEPS)

    def setAngle(self, pwm_channel, angle):
        pwm_channel = self._check_channel(pwm_channel)
        angle = int(angle)

        min_angle = self._min_angle[pwm_channel]
        max_angle = self._max_angle[pwm_channel]
        min_us = self._min_pulse_us[pwm_channel]
        max_us = self._max_pulse_us[pwm_channel]

        if angle < min_angle:
            angle = min_angle
        elif angle > max_angle:
            angle = max_angle

        angle_span = max_angle - min_angle
        pulse_span = max_us - min_us

        if angle_span <= 0:
            pulse = min_us
        else:
            pulse = min_us + ((angle - min_angle) * pulse_span) // angle_span

        self.setPulse(pwm_channel, pulse)

    # -----------------------------
    # Convenience helpers
    # -----------------------------
    def stop(self, pwm_channel):
        self.setPWM(pwm_channel, 0, 0)

    def off(self, pwm_channel):
        self.stop(pwm_channel)

    def allOff(self):
        self._write_bytes(_ALL_LED_ON_L, ustruct.pack("<HH", 0, 0))

    def allOn(self):
        self._write_bytes(_ALL_LED_ON_L, bytes((0, 0x10, 0, 0)))


# -----------------------------
# Simple test
# -----------------------------
if __name__ == "__main__":
    pwm = EvoPWMDriver(I2C1)
    pwm.begin()

    # Example calibration for channel 0
    pwm.setChannelPulseRange(CH0, 600, 2400)
    pwm.setChannelAngleRange(CH0, 0, 180)

    while True:
        pwm.setAngle(CH0, 0)
        print("CH0:", pwm.getChannelPulseRange(CH0), pwm.getChannelAngleRange(CH0), pwm.getPulse(CH0))
        time.sleep_ms(1000)

        pwm.setAngle(CH0, 90)
        print("CH0:", pwm.getChannelPulseRange(CH0), pwm.getChannelAngleRange(CH0), pwm.getPulse(CH0))
        time.sleep_ms(1000)

        pwm.setAngle(CH0, 180)
        print("CH0:", pwm.getChannelPulseRange(CH0), pwm.getChannelAngleRange(CH0), pwm.getPulse(CH0))
        time.sleep_ms(1000)