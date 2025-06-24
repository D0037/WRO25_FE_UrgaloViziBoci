import utils.movement as move
import time
import threading
import utils.image_processing as processing
import os
import signal

image_thread = threading.Thread(target=processing.image_thread)
image_thread.start()

move.init()

try:
    #processing.set_mode("blocks")
    #while True:
    #    print(processing.get_blocks())
    #    time.sleep(0.5)
    
    x, y, z = 0, 0, 0

    while (not processing.lines):
        time.sleep(0.001)
    pos = processing.start_pos
    processing.kill = True

    if pos == "front_outer": x, y = 40, 70   #1
    if pos == "front_middle": x, y = 60, 50  #2
    if pos == "front_inner": x, y = 50, 60   #3
    if pos == "back_outer": x, y = 61.5, 70  #4
    if pos == "back_middle": x, y = 81.5, 50 #5
    if pos == "back_inner": x, y = 71.5, 60 #6
    print("START_POS", pos)

    z = 0
    if processing.direction: z = 90            
    else: z = -90

    def first(x:float, y:float, z:float):
        move.move(x + 20, 12)
        time.sleep(0.5)
        move.turn(z, y)
        #time.sleep(5)

        for i in range(11):
            if i == 0 and pos.endswith("inner"):
                move.move(110, 12)
                #time.sleep(5)
            else:
                move.move(140, 12)
                #time.sleep(5)
            time.sleep(0.5)
            move.turn(z, 60)
            #time.sleep(5)

        move.move(70, 12)
    
    first(x, y, z)
    #move.move(70)
    #move.turn(-90, 50, forward=-1)

except KeyboardInterrupt:
    move.set_speed(0)
    move.set_angle(0)
    pass

# Cleanup
processing.kill = True
move.cleanup()
os.kill(os.getpid(), signal.SIGTERM)