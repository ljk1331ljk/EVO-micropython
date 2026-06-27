from evo.devices import Motor
from evo.parameters import Port
from evo.tools import wait


motor = Motor(Port.A)

motor.run(360)
wait(1000)
motor.brake()

print("Angle:", motor.angle())
