from pins import I2CB


def writeto_mem(addr, memaddr, data):
    I2CB.writeto_mem(addr, memaddr, data)


def readfrom_mem(addr, memaddr, n):
    return I2CB.readfrom_mem(addr, memaddr, n)
