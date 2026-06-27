from evo.devices import Motor
from evo.parameters import Port
from evo.robotics import DriveBase


left = Motor(Port.A)
right = Motor(Port.B)
drive = DriveBase(left, right)

drive.straight(360)
drive.turn(90)
drive.stop()
