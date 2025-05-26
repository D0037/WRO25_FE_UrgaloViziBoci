import utils.movement as move
import RPi.GPIO as GPIO
import time
import math
#from utils.position_tracking import PositionTracker

GPIO.setmode(GPIO.BCM)
move.init()

while True:
    try:
        #move.move(100, 10, 10, 30)
        #move.move(-100, 10, 10, 30)
        #move.move(20, 10, 10)
        #move.turn(180, 40, 10, 30)
        #move.move(-20, 10, 10)
        move.turn(-90, 40, 10, 30)
        move.move(-20, 10)
        break
    except KeyboardInterrupt:
        break

#r = move.RelativeCoordinates(1, 1, 90 * (math.pi / 180))
#print(r.get_point(2, 3))


# Cleanup GPIO settings
#tracker.kill()
move.set_angle(0)
move.cleanup()
GPIO.cleanup((5,6,23,24))