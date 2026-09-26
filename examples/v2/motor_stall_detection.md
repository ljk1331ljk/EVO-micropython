# Individual motor stall detection

`EvoMotor` provides `setStallTimeout(ms)`, `getStallTimeout()`, `getStalled()`,
and `clearStall()`, matching the motor-pair API. The default timeout is 1000 ms;
zero disables detection and negative values raise `ValueError`.

Detection monitors raw encoder counts. Each observed count change resets the
inactivity timer. `clearStall()` clears the latched flag and resets the timer
and encoder baseline without changing the encoder position or motor power.
`getStalled()` only reads the flag; it does not poll or clear it.

`runTime()` and `runAngle()` clear the flag and reset the timer at the start.
A stall exits their movement loop and performs the normal configured stop
behavior (coast by default, or brake/hold). They return `None`, leaving the
flag set for inspection. Explicit brake/coast/hold calls also preserve it.

```python
motor.setStallTimeout(500)
motor.runAngle(180, 360)
print(motor.getStalled())
```

Immediate `run()`, `runPower()`, and `runSpeed()` check on each call and preserve
the timer and flag across calls. They do not automatically stop on a stall.
Clear immediately before each new run sequence, especially after idle time:

```python
motor.clearStall()
while True:
    motor.runSpeed(180)
    if motor.getStalled():
        motor.brake()
        break
```

There is no background monitoring. Call run repeatedly to observe encoder
changes and detect a timeout; polling frequency determines detection latency.
A zero command refreshes the timer without clearing a latched flag.
Allow enough timeout for startup and slow motion. Disable detection for motors
without encoders if using power or timed commands.

Motor-pair stall state and timeout remain independent of individual motor state.
