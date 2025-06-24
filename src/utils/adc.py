import smbus
import time

bus: smbus.SMBus = None

CONFIG = [0x40, 0x82]
ADS1115_ADDRESS = 0x48
CONFIG_REG = 0x01
CONVERT_REG = 0x00

def init():
    global bus
    bus = smbus.SMBus(1)

    # Write config register
    bus.write_i2c_block_data(ADS1115_ADDRESS, CONFIG_REG, CONFIG)
    time.sleep(0.01)

def read_bat():
    global bus
    data = bus.read_i2c_block_data(ADS1115_ADDRESS, CONVERT_REG, 2)
    raw = (data[0]<<8) | data[1]
    if raw > 0x7FFF:
        raw -= 0x10000
    
    voltage = (raw / 32768.0) * 6.144

    return voltage*2

