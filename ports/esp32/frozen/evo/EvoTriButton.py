from machine import Pin, ADC
import time


class EvoTriButton:

    # Thresholds copied from Arduino library
    _button_lows = (0, 600, 1100, 1800)
    _button_highs = (0, 800, 1300, 2100)

    def __init__(self, pin):
        """
        Constructor for TriButton.

        Args:
            pin: GPIO pin number connected to the TriButton analog output.
        """
        self._pin_no = pin
        self._pin = Pin(pin, Pin.IN)
        self._adc = ADC(self._pin)

        # Try to set a predictable ADC width/attenuation on ports that support it.
        # ESP32 MicroPython usually supports these methods.
        try:
            self._adc.atten(ADC.ATTN_11DB)
        except Exception:
            pass

        try:
            self._adc.width(ADC.WIDTH_12BIT)
        except Exception:
            pass

    def _read_raw(self):
        """
        Read raw ADC value.
        Expected to be around 0..4095 on ESP32 with 12-bit ADC.
        """
        return self._adc.read()

    def getButton(self, button=0):
        """
        Gets the state of the selected button.

        Args:
            button:
                0 = Raw ADC value
                1 = Black
                2 = Red
                3 = Blue

        Returns:
            If button == 0:
                raw ADC reading
            Else:
                1 if pressed, 0 if not pressed
        """
        value = self._read_raw()

        if button != 0:
            if button < 0 or button > 3:
                raise ValueError("button must be 0, 1, 2, or 3")

            if self._button_lows[button] <= value <= self._button_highs[button]:
                return 1
            return 0

        return value

    def getButtonPressed(self):
        """
        Gets which button is currently pressed.

        Returns:
            1 = Black
            2 = Red
            3 = Blue
            -1 = none pressed

        Note:
            This follows the Arduino .cpp behavior, which loops 0..3 but
            effectively returns only 1..3 when a threshold matches.
        """
        reading = self._read_raw()
        for i in range(4):
            if self._button_lows[i] <= reading <= self._button_highs[i]:
                return i
        return -1

    def waitForPress(self, button, debouncems=200):
        """
        Wait until the selected button is pressed.

        Args:
            button: 1 = Black, 2 = Red, 3 = Blue
            debouncems: debounce delay after press
        """
        if button not in (1, 2, 3):
            raise ValueError("button must be 1, 2, or 3")

        while self.getButton(button) != 1:
            time.sleep_ms(10)

        time.sleep_ms(debouncems)

    def waitForRelease(self, debouncems=200):
        """
        Wait until all buttons are released.

        The Arduino version waits until raw read returns 4095 exactly. In MicroPython,
        exact full-scale ADC can be unreliable, so this version treats release as
        'no valid button range matched'.
        """
        while self.getButtonPressed() != -1:
            time.sleep_ms(10)

        time.sleep_ms(debouncems)

    def waitForBump(self, button, debouncems=200):
        """
        Wait until the selected button is pressed and then released.

        Args:
            button: 1 = Black, 2 = Red, 3 = Blue
            debouncems: debounce delay after press/release
        """
        self.waitForPress(button, debouncems)
        self.waitForRelease(debouncems)