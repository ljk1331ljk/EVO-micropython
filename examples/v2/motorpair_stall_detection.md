# Motor-pair stall detection

Blocking `EvoMotorPair` movements (`moveDegrees` / `runDegrees`, `moveTime`,
`straight`, and `turn`, including IMU modes) stop if either commanded wheel's
raw encoder count has not changed for the stall timeout. The default is 1000 ms.
Each movement starts fresh timers; each observed encoder count change resets
that wheel's timer. A wheel requested at zero power is ignored during pivots.

```python
pair.setStallTimeout(500)  # milliseconds without encoder activity
print(pair.getStallTimeout())  # 500
pair.moveDegrees(1000, 1000, 720)
print(pair.getStalled())  # True if this movement stopped because of a stall
```

On timeout, the movement returns normally (`None`) after applying its selected
stop behavior to both motors, and `isBusy()` becomes false. No exception is
raised. The usual default is brake; per-movement coast/hold overrides also apply.

`getStalled()` reads a stored flag, initially `False`. Starting a blocking movement
resets it to `False`; detecting a stall sets it to `True`. It stays set after
the movement stops, including after explicit stop/brake/coast/hold calls, until
another blocking movement starts or `clearStall()` is called. Reading the flag
does not clear it.

Use `pair.setStallTimeout(0)` to disable detection, for example when using motors
without encoders for timed movements. Negative timeouts raise `ValueError`.
Allow enough time for startup and slow movement when choosing a timeout.

`move`, `moveSpeed`, and `movePower` return immediately. Each call checks encoder
inactivity and sets the flag on timeout, preserving timers and any latched flag
between calls. These commands do not automatically stop on a stall.

Call `clearStall()` immediately before a new run sequence. It clears the flag,
records both current encoder counts, and resets both inactivity timers to now.
It does not reset encoder angles or change motor power. The initial timer window
starts at construction; explicitly clear after idle time or a blocking movement.

```python
pair.clearStall()
while True:
    pair.movePower(1000, 1000)
    if pair.getStalled():
        pair.stop()
        break
```

Detection is polled by run calls, not by a background task or `getStalled()`.
Call run repeatedly to monitor movement; a single run followed only by reads of
`getStalled()` cannot detect a later stall. Encoder changes are observed at each
call, so polling frequency determines detection latency.
