from utils.position_tracking import PositionTracker
from utils.gyro import Gyro
import utils.movement as move
import RPi.GPIO as GPIO
import time

#tracker = PositionTracker()
gyro = Gyro(0x68, 0)

# Initialize RPi GPIO
#GPIO.setmode(GPIO.BCM)

#move.init()

while True:
    try:
        print(gyro.get_z())
        time.sleep(0.1)
    except KeyboardInterrupt:
        break

# Cleanup
#GPIO.cleanup((5,6,23,24))
gyro.kill()