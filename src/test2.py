import time
import utils.movement as move

move.init()

try:
    move.move(100, 20)
    #move.move(-100, 20)
except KeyboardInterrupt:
    print("Exiting...")
    move.tracker.kill()
    move.set_speed(0)
    move.cleanup()