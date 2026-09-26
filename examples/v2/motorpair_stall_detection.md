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
```

On timeout, the movement returns normally (`None`) after applying its selected
stop behavior to both motors, and `isBusy()` becomes false. No exception is
raised. The usual default is brake; per-movement coast/hold overrides also apply.

Use `pair.setStallTimeout(0)` to disable detection, for example when using motors
without encoders for timed movements. Negative timeouts raise `ValueError`.
Allow enough time for startup and slow movement when choosing a timeout.

`move`, `moveSpeed`, and `movePower` return immediately and have no movement loop;
they do not perform stall detection.
