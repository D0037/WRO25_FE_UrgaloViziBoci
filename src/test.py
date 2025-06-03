import utils.movement as move
import RPi.GPIO as GPIO
import time
import math
import utils.image_processing as processing
#from utils.position_tracking import PositionTracker

GPIO.setmode(GPIO.BCM)

move.init()

move.move(10)
