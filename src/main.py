import utils.movement as move
import time
import threading
import utils.image_processing as processing
import os
import signal

image_thread = threading.Thread(target=processing.image_thread)
image_thread.start()

move.init()

"""def go_until_line():
    start_angle = move.tracker.gyro.get_z()
    #move.set_speed(8)
    move.set_angle(0)
    
    while not processing.lines:
        error = move.tracker.gyro.get_z() - start_angle
        correction = error * 10

        move.set_angle(correction)
        print("searching for line...")
        time.sleep(0.1)
    move.set_angle(0)
    move.set_speed(0)"""


try:
    """x, y, z = 0, 0, 0

    while (not processing.lines):
        time.sleep(0.001)
    pos = processing.start_pos

    if pos == "front_outer": x, y = 40, 70   #1
    if pos == "front_middle": x, y = 60, 50  #2
    if pos == "front_inner": x, y = 80, 30   #3
    if pos == "back_outer": x, y = 61.5, 70  #4
    if pos == "back_middle": x, y = 81.5, 50 #5
    if pos == "back_inner": x, y = 101.5, 30 #6
    print("START_POS", pos)

    #if orient == "blue_upper": z = 90            
    #if orient == "orange_upper": z = -90
    z = -90

    def first(x:float, y:float, z:float):
        move.move(x, 10)
        move.turn(z, y)

        for i in range(11):
            move.move(100, 10)
            move.turn(z, 60)

        move.move(40, 10)
    
    first(x, y, z)"""
    move.turn(90, 100)

except KeyboardInterrupt:
    move.set_speed(0)
    move.set_angle(0)

# Cleanup
processing.kill = True
move.cleanup()
os.kill(os.getpid(), signal.SIGTERM)