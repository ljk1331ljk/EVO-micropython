"""Small timing helpers for EVO programs."""

import time


def wait(ms):
    """Wait for ``ms`` milliseconds."""

    return time.sleep_ms(ms)


class StopWatch:
    """Simple millisecond stopwatch."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._start = time.ticks_ms()

    def time(self):
        return time.ticks_diff(time.ticks_ms(), self._start)
