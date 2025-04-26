from utils.position_tracking import PositionTracker
from utils.bin.gyro import Gyro
import utils.movement as move
import RPi.GPIO as GPIO
import time

#tracker = PositionTracker()
gyro = Gyro("/dev/i2c-1", 0x68, 1, 0)

# Initialize RPi GPIO
GPIO.setmode(GPIO.BCM)

move.init()

move.set_angle(30)
while True:
    try:
        time.sleep(0.1)
    except KeyboardInterrupt:
        break

# Cleanup
GPIO.cleanup((5,6,23,24))
gyro.kill()