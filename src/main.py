#code for runing and connecting the whole project
import utils.movement as move
import time
import threading
import utils.image_processing as processing
import os
import signal
import round2

#starting thread
image_thread = threading.Thread(target=processing.image_thread)
image_thread.start()

processing.set_mode("parking")
move.init()

mode = "obstacle"

try:
    #setup for first round
    if mode == "free":
        x, y, z = 0, 0, 0

        #set pos
        while (not processing.lines):
            time.sleep(0.001)
        pos = processing.start_pos
        #processing.kill = True

        #costume moving behaviors
        if pos == "front_outer": x, y, side = 40, 70, 150   #1
        if pos == "front_middle": x, y, side = 55, 50, 155  #2
        if pos == "front_inner": x, y, side = 35, 60, 115   #3
        if pos == "back_outer": x, y, side = 65, 70, 152  #4
        if pos == "back_middle": x, y, side = 90, 50, 155 #5
        if pos == "back_inner": x, y, side = 80, 60, 117.5 #6
        print("START_POS", pos)

        #dealing with directions
        z = 0
        if processing.direction: z = -90            
        else: z = 90

        #moving through the first round chalange
        def first(x:float, y:float, z:float):
            move.move(x + 20, 12)
            move.turn(z, y)
            move.move(side)
            #time.sleep(5)

            for i in range(10):
                move.turn(z, 60)
                move.move(140)
                move.global_angle -= 1.16 * (z / abs(z))
                #time.sleep(5)

            move.turn(z, 60)
            move.move(70, 12)
        
        first(x, y, z)
        #move.move(70)
        #move.turn(-90, 50, forward=-1)
    
    #setup for second round
    elif mode == "obstacle":
        processing.set_mode("parking")
        time.sleep(5)
        d = processing.in_parking_heading
        print(d)
        """for i in range(30):
            move.turn(3 * d, 45, 8)
            move.turn(3 * d, 45, 8, forward=-1)"""
    
        #exiting parking space
        for i in range(3):
            move.set_angle(50*d)
            move.set_speed(8)
            while move.tracker.get_y() < 9:
                pass
            move.set_angle(-50*d)
            move.set_speed(-8)
            while move.tracker.get_y() > 2:
                pass
            move.set_speed(0)

        processing.set_mode("1st_obstacle")
        time.sleep(1)
        block = processing.first_block
        if block == "n":
            move.global_angle = move.tracker.gyro.get_z()
            print(move.tracker.gyro.get_z())
            move.turn(40, 50)
            time.sleep(5)
            move.global_angle = 0
            move.turn(90, 60)
            time.sleep(5)
            move.move(80)



        raise Exception
        with open("block.csv", "w") as f:
            f.write("x,y\n")
        processing.get_blocks()
        round2.rounds(1, 4, 1)

# for ctrl^C to work during data flow
except:
    pass

# Finishing arguments
move.set_speed(0)
move.set_angle(0)

# Cleanup
processing.kill = True
move.cleanup()
os.kill(os.getpid(), signal.SIGTERM)