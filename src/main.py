import utils.movement as move
import time
import threading
import utils.image_processing as processing
import os
import signal

image_thread = threading.Thread(target=processing.image_thread)
image_thread.start()

move.init()

def go_until_line():
    start_angle = move.tracker.gyro.get_z()
    move.set_speed(8)
    move.set_angle(0)
    
    while not processing.lines:
        error = move.tracker.gyro.get_z() - start_angle
        correction = error * 10

        move.set_angle(correction)
        print("searching for line...")
        time.sleep(0.1)
    move.set_angle(0)
    move.set_speed(0)


try:
    go_until_line()
    time.sleep(99999999)
except KeyboardInterrupt:
    move.set_speed(0)
    move.set_angle(0)

# Cleanup
processing.kill = True
move.cleanup()
os.kill(os.getpid(), signal.SIGTERM)