import utils.movement as move
import RPi.GPIO as GPIO
import time
import math
import utils.image_processing as processing
#from utils.position_tracking import PositionTracker

GPIO.setmode(GPIO.BCM)
try:
    move.init()

    move.turn(-90, 100, 13, 50)
except KeyboardInterrupt:
    pass

move.set_angle(0)
move.set_speed(0)
move.cleanup()
