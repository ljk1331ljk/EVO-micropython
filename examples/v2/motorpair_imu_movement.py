"""Basic test program for IMU-assisted MotorPair movement.

Hardware used by this example:
  - Left motor on port A
  - Right motor on port B
  - BNO055 IMU on I2C port 1

Place the robot on the floor with space around it before running the program.
"""

from time import sleep_ms

from EvoIMU import EvoIMU
from evo.devices import Motor
from evo.parameters import Port, Stop
from evo.robotics import MotorPair


STRAIGHT_POWER = 800
STRAIGHT_DEGREES = 720
TURN_POWER = 800
TEST_PAUSE_MS = 1500


left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
imu = EvoIMU(Port.I2C1)

drive = MotorPair(left_motor, right_motor, imu)
drive.useIMU(True)

# Start conservatively. Increase Kp if correction is too weak. Increase Kd in
# small steps if the robot oscillates around its desired heading.
drive.setIMUPD(1.0, 0.0)


def show_heading(label):
    print(label, imu.getEulerX())


try:
    show_heading("Initial raw heading:")
    print("IMU PD gains:", drive.getIMUPD())
    sleep_ms(TEST_PAUSE_MS)

    # Hold the raw heading measured at the start of this movement. The IMU is
    # not reset, and encoder degrees still determine the travel distance.
    print("Test 1: relative straight movement")
    show_heading("Start heading:")
    drive.straight(
        STRAIGHT_POWER,
        STRAIGHT_DEGREES,
        Stop.BRAKE,
        True,
    )
    show_heading("End heading:")
    sleep_ms(TEST_PAUSE_MS)

    # Turn 90 degrees toward increasing raw IMU headings.
    print("Test 2: positive 90-degree IMU turn")
    drive.turn(TURN_POWER, 90, Stop.BRAKE)
    show_heading("Heading after +90 turn:")
    sleep_ms(TEST_PAUSE_MS)

    # Return approximately to the heading before the previous turn.
    print("Test 3: negative 90-degree IMU turn")
    drive.turn(TURN_POWER, -90, Stop.BRAKE)
    show_heading("Heading after -90 turn:")
    sleep_ms(TEST_PAUSE_MS)

    # Absolute mode corrects toward raw IMU heading 0 degrees while driving.
    # Skip this test initially if the robot does not have enough clear space.
    print("Test 4: absolute-heading straight movement toward 0 degrees")
    drive.straight(
        STRAIGHT_POWER,
        STRAIGHT_DEGREES,
        Stop.BRAKE,
        False,
    )
    show_heading("Final heading:")
finally:
    drive.brake()
    print("IMU movement test complete; motors braked.")
