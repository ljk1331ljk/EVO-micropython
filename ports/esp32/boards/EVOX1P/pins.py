# X1pins.py (FROZEN)
from micropython import const
from machine import I2C, Pin

# GPIO
GPIO1A = const(4)
GPIO1B = const(5)
GPIO2A = const(6)
GPIO2B = const(7)

#Servo 
SERVO1 = const(15)
SERVO2 = const(1)
SERVO3 = const(17)
SERVO4 = const(18)

# Motor Encoder Pins (GPIO numbers)
TACH41 = const(42)
TACH42 = const(39)
TACH31 = const(41)
TACH32 = const(40)
TACH21 = const(38)
TACH22 = const(48)
TACH11 = const(47)
TACH12 = const(21)

# Motor PWM channels (PCA9685 channels 0..15)
MOTOR11 = const(12)
MOTOR12 = const(13)
MOTOR21 = const(15)
MOTOR22 = const(14)
MOTOR31 = const(0)
MOTOR32 = const(1)
MOTOR41 = const(2)
MOTOR42 = const(3)

# I2C mux channels
I2C1 = const(0)
I2C2 = const(1)
I2C3 = const(2)
I2C4 = const(3)
I2C5 = const(7)
I2C6 = const(6)
I2C7 = const(5)
I2C8 = const(4)

# Other peripherals
BUZZER_PIN = const(8)
BUTTON_L_PIN = const(11)
BUTTON_C_PIN = const(10)
BUTTON_R_PIN = const(3)
SDA0_PIN = const(1)
SCL0_PIN = const(2)
SDA1_PIN = const(12)
SCL1_PIN = const(13)

# I2C device addresses
TCA9548A_ADDR = const(0x70)
SSD1309_ADDR = const(0x3C)
SSD1309_CHANNEL = const(I2C5)
BATTERY_CHARGER_ADDRESS = const(0x6A)
PCA9685PW_ADDRESS = const(0x40)
NSLEEP_PIN = const(46)
BOOT_LED_PIN = const(45)
SHUTDOWN_PIN = const(14)

# I2C bus objects
I2CA = I2C(0, sda=Pin(SDA0_PIN), scl=Pin(SCL0_PIN), freq=400_000)
I2CB = I2C(1, sda=Pin(SDA1_PIN), scl=Pin(SCL1_PIN), freq=400_000)


# Logical motor port constants exposed to users.
# Boards with fewer ports should only define the ports that exist.
M1 = const(1)
M2 = const(2)
M3 = const(3)
M4 = const(4)


# Board-specific motor port map.
# pwm1/pwm2 are PCA9685 channels.
# tach1/tach2 are ESP32 GPIO numbers for the quadrature encoder.
MOTOR_PORTS = {
    M1: {
        "pwm1": MOTOR12,
        "pwm2": MOTOR11,
        "tach1": TACH11,
        "tach2": TACH12,
    },
    M2: {
        "pwm1": MOTOR22,
        "pwm2": MOTOR21,
        "tach1": TACH21,
        "tach2": TACH22,
    },
    M3: {
        "pwm1": MOTOR32,
        "pwm2": MOTOR31,
        "tach1": TACH31,
        "tach2": TACH32,
    },
    M4: {
        "pwm1": MOTOR42,
        "pwm2": MOTOR41,
        "tach1": TACH41,
        "tach2": TACH42,
    },
}
