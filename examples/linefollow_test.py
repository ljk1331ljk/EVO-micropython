"""Hardware test program for EvoLineTrace.

Default wiring:
  - Left motor:  Port A / M1
  - Right motor: Port B / M2
  - Left sensor: I2C1
  - Right sensor: I2C2

Start with TEST = "readings" and note the values over the black line and white
surface. Then fill in CALIBRATION, tune the PID values, raise the robot so its
wheels are clear, and select one movement test at a time.
"""

from time import sleep_ms

from EvoColorSensor import EvoColorSensor
from EvoLineTrace import BOTH, LEFT, RIGHT, EvoLineTrace
from evo.devices import Motor
from evo.parameters import Port
from evo.robotics import MotorPair


# Select one test. Junction tests continue until a junction is detected.
TEST = "readings"
# TEST = "double_degrees"
# TEST = "single_left_degrees"
# TEST = "single_right_degrees"
# TEST = "double_both_junction"
# TEST = "single_left_right_junction"

# Set to (left_min, left_max, right_min, right_max) after measuring the raw
# clear values. Leave as None to use raw readings without mapping.
CALIBRATION = None
# CALIBRATION = (100, 2000, 120, 2100)

# These are starting values only. Tune them on your robot.
KP = 1.0
KI = 0.0
KD = 0.0

POWER = 700
DEGREES = 360

# When calibrated, the line-edge threshold is normally near the midpoint of
# the mapped 0..500 range. A black junction produces a value at or below the
# junction threshold.
LINE_THRESHOLD = 250
JUNCTION_THRESHOLD = 100

READING_COUNT = 100
READING_INTERVAL_MS = 100
START_DELAY_MS = 3000


left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
drive = MotorPair(left_motor, right_motor)

left_sensor = EvoColorSensor(Port.I2C1)
right_sensor = EvoColorSensor(Port.I2C2)

trace = EvoLineTrace(drive.native(), left_sensor, right_sensor)
trace.setPIDParameters(KP, KI, KD)

if CALIBRATION is not None:
    trace.calibrateColorSensor(*CALIBRATION)


def show_configuration():
    print("Test:", TEST)
    print("PID:", trace.getPIDParameters())
    print("Calibration:", CALIBRATION)
    print("Initial readings:", trace.readCalibratedReadings())


def show_readings():
    print("Move both sensors between the black line and white surface.")
    print("Values are (left, right). Press Ctrl-C to finish early.")
    for _ in range(READING_COUNT):
        print(trace.readCalibratedReadings())
        sleep_ms(READING_INTERVAL_MS)


def countdown():
    print("WARNING: motors will move. Lift the robot or clear the test area.")
    remaining = START_DELAY_MS // 1000
    while remaining > 0:
        print("Starting in", remaining)
        sleep_ms(1000)
        remaining -= 1


def run_selected_test():
    if TEST == "readings":
        show_readings()
        return

    countdown()

    if TEST == "double_degrees":
        trace.doubleLineFollowDegrees(POWER, DEGREES, True)
    elif TEST == "single_left_degrees":
        trace.singleLineFollowDegrees(
            LEFT, POWER, LINE_THRESHOLD, DEGREES, True
        )
    elif TEST == "single_right_degrees":
        trace.singleLineFollowDegrees(
            RIGHT, POWER, LINE_THRESHOLD, DEGREES, True
        )
    elif TEST == "double_both_junction":
        trace.doubleLineFollowJunction(
            POWER, BOTH, JUNCTION_THRESHOLD, True
        )
    elif TEST == "single_left_right_junction":
        # Track using the left sensor and stop when the right sensor reaches
        # the black junction.
        trace.singleLineFollowJunction(
            LEFT,
            POWER,
            LINE_THRESHOLD,
            RIGHT,
            JUNCTION_THRESHOLD,
            True,
        )
    else:
        raise ValueError("unknown TEST: " + TEST)

    print("Movement test complete. Final readings:", trace.readCalibratedReadings())


show_configuration()

try:
    run_selected_test()
except BaseException:
    # Ensure a sensor/I2C error or Ctrl-C cannot leave the motors powered.
    drive.brake()
    raise
finally:
    drive.brake()
