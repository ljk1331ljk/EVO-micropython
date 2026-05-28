# X1pins.py (FROZEN)
from micropython import const
from machine import I2C, Pin

# LEGO Sensor pins
S11 = const(10)
S12 = const(9)
S21 = const(8)
S22 = const(3)
S31 = const(7)
S32 = const(6)
S41 = const(5)
S42 = const(4)

# LEGO Sensor ports
S1 = S11
S2 = S21
S3 = S31
S4 = S41

# GPIO
GPIO1 = const(10)
GPIO2 = const(9)
GPIO3 = const(8)
GPIO4 = const(3)
GPIO5 = const(7)
GPIO6 = const(6)
GPIO7 = const(5)
GPIO8 = const(4)

# Motor Encoder Pins (GPIO numbers)
TACH41 = const(42)
TACH42 = const(41)
TACH31 = const(40)
TACH32 = const(39)
TACH21 = const(38)
TACH22 = const(48)
TACH11 = const(47)
TACH12 = const(21)

# Motor PWM channels (PCA9685 channels 0..15)
MOTOR21 = const(13)
MOTOR22 = const(12)
MOTOR11 = const(15)
MOTOR12 = const(14)
MOTOR41 = const(9)
MOTOR42 = const(8)
MOTOR31 = const(11)
MOTOR32 = const(10)

# === Global Servo Ports ===
SERVO1 = const(101)
SERVO2 = const(102)
SERVO3 = const(103)
SERVO4 = const(104)
SERVO5 = const(105)
SERVO6 = const(106)
SERVO7 = const(107)
SERVO8 = const(108)

# I2C mux channels
I2C1 = const(0)
I2C2 = const(1)
I2C3 = const(2)
I2C4 = const(3)
I2C5 = const(4)
I2C6 = const(5)
I2C7 = const(6)
I2C8 = const(7)

# Other peripherals
BUZZER_PIN = const(11)
BUTTON_PIN = const(14)
NEOPIXEL_PIN = const(14)
SDA0_PIN = const(1)
SCL0_PIN = const(2)
SDA1_PIN = const(12)
SCL1_PIN = const(13)


# I2C device addresses
TCA9548A_ADDR = const(0x70)
SSD1309_ADDR = const(0x3C)
SSD1309_CHANNEL = const(I2C8)
BATTERY_CHARGER_ADDRESS = const(0x6A)
PCA9685PW_ADDRESS = const(0x40)

# ---- Added: fixed motor-control I2C bus object (I2CB) ----
# This is created once at import time, then reused by evo.EVOPWMDriver singleton in C.


# Using I2C bus 1 because your file separates SDA0/SCL0 and SDA1/SCL1.
# Adjust bus_id if your board uses a different bus index.
I2CA = I2C(0, sda=Pin(SDA0_PIN), scl=Pin(SCL0_PIN), freq=400_000)
I2CB = I2C(1, sda=Pin(SDA1_PIN), scl=Pin(SCL1_PIN), freq=400_000)

# Logical motor port constants exposed to users.
# Boards with fewer ports should only define the ports that exist.
M1 = const(1)
M2 = const(2)
M3 = const(3)
M4 = const(4)

MOTOR_PORTS = {
    M1: {
        "pwm1": MOTOR11,
        "pwm2": MOTOR12,
        "tach1": TACH11,
        "tach2": TACH12,
    },
    M2: {
        "pwm1": MOTOR21,
        "pwm2": MOTOR22,
        "tach1": TACH21,
        "tach2": TACH22,
    },
    M3: {
        "pwm1": MOTOR31,
        "pwm2": MOTOR32,
        "tach1": TACH31,
        "tach2": TACH32,
    },
    M4: {
        "pwm1": MOTOR41,
        "pwm2": MOTOR42,
        "tach1": TACH41,
        "tach2": TACH42,
    },
}
