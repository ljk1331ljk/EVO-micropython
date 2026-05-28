import micropython
import time
import pins


class I2CDevice:
    _instances = {}

    def __new__(cls, name):
        if name not in ["I2CA", "I2CB"]:
            raise ValueError("Only 'I2CA' and 'I2CB' are allowed.")

        if name in cls._instances:
            return cls._instances[name]

        instance = super().__new__(cls)
        cls._instances[name] = instance
        return instance

    def __init__(self, name):
        if hasattr(self, "_initialized"):
            return

        self._initialized = True
        self.name = name

        if name == "I2CA":
            self.i2c = pins.I2CA
            self.has_multiplexer = True
            self.current_channel = None
            self.multiplexer_address = pins.TCA9548A_ADDR

        elif name == "I2CB":
            self.i2c = pins.I2CB
            self.has_multiplexer = False
            self.current_channel = None

        else:
            raise ValueError("Invalid I2C bus name")

    @micropython.native
    def switch_channel(self, channel):
        if not self.has_multiplexer:
            return

        if self.current_channel != channel:
            self.i2c.writeto(
                self.multiplexer_address,
                bytes([1 << channel])
            )

            self.current_channel = channel

    @micropython.native
    def scan(self):
        return [hex(addr) for addr in self.i2c.scan()]

    @micropython.native
    def scan_channel(self, channel):
        if not self.has_multiplexer:
            return self.scan()

        self.switch_channel(channel)

        return [hex(addr) for addr in self.i2c.scan()]

    @micropython.native
    def writeto(self, channel, address, data):
        if self.has_multiplexer:
            self.switch_channel(channel)

        self.i2c.writeto(address, data)

    @micropython.native
    def readfrom(self, channel, address, nbytes):
        if self.has_multiplexer:
            self.switch_channel(channel)

        return self.i2c.readfrom(address, nbytes)

    @micropython.native
    def readfrom_mem(self, channel, address, memaddr, nbytes):
        if self.has_multiplexer:
            self.switch_channel(channel)

        return self.i2c.readfrom_mem(address, memaddr, nbytes)

    @micropython.native
    def writeto_mem(self, channel, address, memaddr, data):
        if self.has_multiplexer:
            self.switch_channel(channel)

        self.i2c.writeto_mem(address, memaddr, data)


I2CA = I2CDevice("I2CA")
I2CB = I2CDevice("I2CB")


if __name__ == "__main__":

    def example():
        t = time.ticks_us()

        for i in range(8):
            devices = I2CA.scan_channel(i)
            print(f"Devices found on I2CA channel {i}: {devices}")

        print("Devices found on I2CB:", I2CB.scan())

        delta = time.ticks_diff(time.ticks_us(), t)

        print("Function Time = {:6.3f}ms".format(delta / 1000))

    example()