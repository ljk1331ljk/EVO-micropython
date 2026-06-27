from evo.devices import UltrasonicSensor
from evo.parameters import Port


sensor = UltrasonicSensor(Port.I2C1)

print("Distance mm:", sensor.distance())
