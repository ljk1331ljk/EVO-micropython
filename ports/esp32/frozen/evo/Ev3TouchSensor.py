# EV3TouchSensor.py
# Frozen MicroPython module for Evo
#
# Based on:
# EV3TouchSensor.cpp from EVO-arduino
# - S1 -> S11
# - S2 -> S21
# - S3 -> S31
# - S4 -> S41
# - pinMode(..., INPUT_PULLDOWN)
# - getButton()
# - waitForPress()
# - waitForRelease()
# - waitForBump()
#
# Arduino reference behavior:
# Constructor maps the sensor port to the first pin of that port and configures
# it as INPUT_PULLDOWN. getButton() returns digitalRead(_pin). The wait methods
# poll until the state changes, then delay for debounce. :contentReference[oaicite:0]{index=0}

from machine import Pin
from pins import *
import time


class EV3TouchSensor:
    def __init__(self, port):
        self._port = port
        self._pin_no = self._resolve_pin(port)

        # Match Arduino INPUT_PULLDOWN as closely as possible.
        try:
            self._pin = Pin(self._pin_no, Pin.IN, Pin.PULL_DOWN)
        except Exception:
            # Some ports/builds may not expose PULL_DOWN cleanly.
            self._pin = Pin(self._pin_no, Pin.IN)

    def _resolve_pin(self, port):
        # Match the Arduino mapping exactly:
        # S1 -> S11, S2 -> S21, S3 -> S31, S4 -> S41. :contentReference[oaicite:1]{index=1}
        if port == S1:
            return S11
        elif port == S2:
            return S21
        elif port == S3:
            return S31
        elif port == S4:
            return S41
        raise ValueError("invalid sensor port")

    def getButton(self):
        # Returns 0 or 1, same idea as digitalRead(_pin). :contentReference[oaicite:2]{index=2}
        return self._pin.value()

    def isPressed(self):
        return self.getButton() == 1

    def isReleased(self):
        return self.getButton() == 0

    def waitForPress(self, debouncems=50):
        # Poll until pressed, then debounce, mirroring the Arduino loop+delay. :contentReference[oaicite:3]{index=3}
        while not self.getButton():
            time.sleep_ms(10)
        time.sleep_ms(debouncems)

    def waitForRelease(self, debouncems=50):
        # Poll until released, then debounce, mirroring the Arduino loop+delay. :contentReference[oaicite:4]{index=4}
        while self.getButton():
            time.sleep_ms(10)
        time.sleep_ms(debouncems)

    def waitForBump(self, debouncems=50):
        # Same sequence as Arduino: press first, then release. :contentReference[oaicite:5]{index=5}
        self.waitForPress(debouncems)
        self.waitForRelease(debouncems)