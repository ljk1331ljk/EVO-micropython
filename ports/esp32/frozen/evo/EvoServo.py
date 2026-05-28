# EvoServo.py
# Frozen module
# Unified pulse convention:
# - All pulse values are stored in microseconds as if using 50Hz servo timing.
# - GPIO mode outputs those pulses directly at 50Hz.
# - SERVO1..SERVO8 use the Evo PWM driver at 100Hz, so pulse widths are halved before output.

from micropython import const
from machine import Pin, PWM
import pins


# === Global Servo Types ===
SG90 = const(0)
GEEK_SERVO_360_GREY = const(1)
GEEK_SERVO_360_ORANGE = const(2)
GEEK_SERVO_CONTINUOUS = const(3)
DEFAULT = const(4)


class EvoServo:
    def __init__(self, port, servo_type=DEFAULT, gpio_freq=50):
        self._type = int(servo_type)
        self._last_pos = 0

        # Stored using standard 50Hz pulse convention (microseconds)
        self._min_p = 500
        self._max_p = 2500
        self._min_r = 0
        self._max_r = 180

        p = int(port)

        self._port = p
        self._pwm = None
        self._pin = None
        self._drv = None
        self._ch = -1

        if p >= 101:
            self._is_gpio = False

            self._ch = p - 101

            if self._ch < 0 or self._ch > 15:
                raise ValueError("invalid servo port")

            self._drv = evo.EVOPWMDriver()
            self._drv.freq(100)

        else:
            if p < 0 or p > 50:
                raise ValueError("invalid gpio pin")

            self._is_gpio = True

            self._gpio_freq = int(gpio_freq)

            if self._gpio_freq <= 0:
                raise ValueError("invalid gpio frequency")

            self._pin = Pin(p, Pin.OUT)

            self._pwm = PWM(self._pin)
            self._pwm.freq(self._gpio_freq)

        #
        # Servo presets
        #
        if self._type == SG90:
            self._min_p = 500
            self._max_p = 2500
            self._min_r = 0
            self._max_r = 180

        elif self._type == GEEK_SERVO_360_GREY:
            self._min_p = 400
            self._max_p = 2200
            self._min_r = 0
            self._max_r = 360

        elif self._type == GEEK_SERVO_360_ORANGE:
            self._min_p = 400
            self._max_p = 1960
            self._min_r = 0
            self._max_r = 360

        elif self._type == GEEK_SERVO_CONTINUOUS:
            self._min_p = 500
            self._max_p = 2500
            self._min_r = -100
            self._max_r = 100

    def setPulse(self, min_pulse, max_pulse):
        self._min_p = int(min_pulse)
        self._max_p = int(max_pulse)

    def getMinPulse(self):
        return self._min_p

    def getMaxPulse(self):
        return self._max_p

    def getPulse(self):
        return (self._min_p, self._max_p)

    def setRange(self, min_range, max_range):
        self._min_r = int(min_range)
        self._max_r = int(max_range)

    def getMinRange(self):
        return self._min_r

    def getMaxRange(self):
        return self._max_r

    def getRange(self):
        return (self._min_r, self._max_r)

    def getPort(self):
        return self._port

    def getType(self):
        return self._type

    def getPosition(self):
        return self._last_pos

    def isGPIO(self):
        return self._is_gpio

    def _write_gpio_us(self, pulse_us):
        if hasattr(self._pwm, "duty_ns"):
            self._pwm.duty_ns(int(pulse_us) * 1000)
            return

        if hasattr(self._pwm, "duty_u16"):
            period_us = 1000000 // self._gpio_freq
            duty = (int(pulse_us) * 65535) // period_us
            if duty < 0:
                duty = 0
            elif duty > 65535:
                duty = 65535
            self._pwm.duty_u16(duty)
            return

        period_us = 1000000 // self._gpio_freq
        duty = (int(pulse_us) * 1023) // period_us
        if duty < 0:
            duty = 0
        elif duty > 1023:
            duty = 1023
        self._pwm.duty(duty)

    def _write_servo_port_us(self, pulse_us):
        # Servo ports run at 100Hz, but stored pulse is based on 50Hz convention.
        # Halve pulse width before sending.
        off = int(pulse_us) // 2
        if off < 0:
            off = 0
        elif off > 4096:
            off = 4096
        self._drv.freq(100)
        self._drv.pwm(self._ch, 0, off)

    def write(self, pos):
        pos = int(pos)
        self._last_pos = pos

        if pos < self._min_r:
            pos = self._min_r
        elif pos > self._max_r:
            pos = self._max_r

        pulse = (pos - self._min_r) * (self._max_p - self._min_p) // (self._max_r - self._min_r) + self._min_p

        if self._is_gpio:
            self._write_gpio_us(pulse)
        else:
            self._write_servo_port_us(pulse)

    def setPWM(self, on, off):
        on = int(on)
        off = int(off)

        if self._is_gpio:
            self._write_gpio_us(off)
            return

        # off is still interpreted as 50Hz-based microseconds for consistency
        self._write_servo_port_us(off)

    def deinit(self):
        if self._pwm is not None:
            self._pwm.deinit()