from evo.devices import ColorSensor
from evo.parameters import Port


sensor = ColorSensor(Port.I2C1)

print("RGB:", sensor.rgb())
print("HSV:", sensor.hsv())
print("Reflection:", sensor.reflection())
