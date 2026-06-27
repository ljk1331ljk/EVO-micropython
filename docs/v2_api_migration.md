# EVO Firmware v2 API Migration Guide

## Overview

Firmware v2 introduces a Pybricks-inspired `evo` package while keeping the
original EVO modules available. Existing projects can continue to import
modules such as `EvoColorSensor`, `EvoServo`, `EvoTOF`, and `EvoLineLeader`.

New programs can use grouped modules:

```python
from evo.hubs import EvoHub
from evo.devices import Motor, ColorSensor, UltrasonicSensor
from evo.parameters import Port
from evo.robotics import DriveBase
from evo.tools import wait, StopWatch
```

## Benefits

- Consistent class names for motors, sensors, hub features, and robotics.
- Port names are grouped under `Port`, such as `Port.A` and `Port.I2C1`.
- Motor methods use approachable names like `run()`, `dc()`, `stop()`,
  `brake()`, `hold()`, `angle()`, and `reset_angle()`.
- The old API remains available for classroom projects and existing examples.

## Old-to-New Mapping

| Old API | New API |
| --- | --- |
| `from EvoTOF import *` | `from evo.devices import UltrasonicSensor` |
| `sensor = EvoTOF(I2C1)` | `sensor = UltrasonicSensor(Port.I2C1)` |
| `from EvoColorSensor import *` | `from evo.devices import ColorSensor` |
| `sensor = EvoColorSensor(I2C1)` | `sensor = ColorSensor(Port.I2C1)` |
| `from EvoLineLeader import *` | `from evo.devices import LineSensor` |
| `sensor = EvoLineLeader(I2C1)` | `sensor = LineSensor(Port.I2C1)` |
| `from EvoServo import *` | `from evo.devices import Servo` |
| `servo = EvoServo(SERVO1)` | `servo = Servo(Port.SERVO1)` |
| `import evo; motor = evo.EvoMotor(evo.M1)` | `motor = Motor(Port.A)` |
| `motor.runPower(50)` | `motor.dc(50)` |
| `motor.getAngle()` | `motor.angle()` |
| `motor.resetAngle()` | `motor.reset_angle()` |

## Examples

### Motor

```python
from evo.devices import Motor
from evo.parameters import Port
from evo.tools import wait

motor = Motor(Port.A)
motor.run(360)
wait(1000)
motor.brake()
```

### Distance Sensor

```python
from evo.devices import UltrasonicSensor
from evo.parameters import Port

sensor = UltrasonicSensor(Port.I2C1)
print(sensor.distance())
```

### DriveBase

```python
from evo.devices import Motor
from evo.parameters import Port
from evo.robotics import DriveBase

left = Motor(Port.A)
right = Motor(Port.B)
drive = DriveBase(left, right)

drive.straight(360)
drive.turn(90)
drive.stop()
```

## Compatibility Notes

The v2 package wraps the existing EVO implementation. It does not remove or
rename old frozen modules. Programs that use these imports should continue to
work:

```python
from EvoColorSensor import *
from EvoServo import *
from EvoTOF import *
from EvoLineLeader import *
from evo import *
```

Some high-level methods, such as `DriveBase.straight(distance)`, initially map
distance to motor degrees because the firmware does not yet have calibrated
wheel geometry conversion. Future releases can use `wheel_diameter` and
`axle_track` to convert millimeters and degrees into precise wheel rotation.
