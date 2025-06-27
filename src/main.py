#code for runing and connecting the whole project
import utils.movement as move
import time
import threading
import utils.image_processing as processing
import os
import signal
import round2
import RPi.GPIO as GPIO


#starting thread
GPIO.setmode(GPIO.BCM)

GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

image_thread = threading.Thread(target=processing.image_thread)
image_thread.start()

do_sleeps = True

def sleep(time):
    if do_sleeps:
        time.sleep(time)

processing.set_mode("parking")
move.init()

mode = "obstacle"

def start(channel):
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
            # Left turns
            if (d == -1):
                block = processing.first_block
                # None
                if block == "n":
                    move.global_angle = move.tracker.gyro.get_z()
                    print(move.tracker.gyro.get_z())
                    move.turn(40, 50)
                    sleep(5)
                    move.global_angle = 0
                    move.turn(-90, 60)
                    sleep(5)
                    move.move(-60)
                
                # Red
                if block == "r":
                    move.global_angle = move.tracker.gyro.get_z()
                    print(move.global_angle)
                    sleep(5)
                    move.turn(40, 50)
                    sleep(5)
                    move.global_angle = 0
                    move.turn(90, 60)
                    sleep(5)
                    move.move(-60)
                
                # Green
                if block == "g":
                    move.global_angle = move.tracker.gyro.get_z()
                    print(move.global_angle)
                    sleep(5)
                    move.turn(-60, 60)
                    sleep(5)
                    move.turn(90, 60)
                    sleep(5)
                    move.global_angle = 0
                    move.move(80)
                    sleep(5)
                    move.turn(-90, 60, forward=-1)
                    sleep(5)
                    move.move(-15)

            # Right turns
            else:
                block = processing.first_block

                # None
                if block == "n":
                    move.global_angle = move.tracker.gyro.get_z()
                    print(move.tracker.gyro.get_z())
                    move.turn(40, 50)
                    sleep(5)
                    move.turn(-40, 50)
                    sleep(5)
                    move.global_angle = 0
                    move.move(120)
                    sleep(5)
                    move.turn(90, 60, forward=-1)
                
                # Red
                elif block == "r":
                    move.global_angle = move.tracker.gyro.get_z()
                    print(move.tracker.gyro.get_z())
                    move.turn(40, 60)
                    sleep(5)
                    move.move(20)
                    sleep(5)
                    move.turn(-40, 60)
                    sleep(5)
                    move.global_angle = 0
                    move.move(150)
                    sleep(5)
                    move.turn(90, 60, forward=-1)
                    sleep(5)
                    move.move(-20)
                    sleep(5)
                 
                # Green
                elif block == "g":
                    move.global_angle = move.tracker.gyro.get_z()
                    print(move.global_angle)
                    move.turn(-60, 60)
                    sleep(5)
                    move.global_angle = 0
                    move.move(20)
                    sleep(5)
                    move.turn(90, 60)
                    sleep(5)
                    move.move(-80)


            processing.set_mode("blocks")
            time.sleep(5)
            round2.rounds(1, 4, d)

    # for ctrl^C to work during data flow
    except:
        pass

GPIO.add_event_detect(17, GPIO.RISING, start, 200)
time.sleep(9999999)

# Finishing arguments
move.set_speed(0)
move.set_angle(0)

# Cleanup
processing.kill = True
move.cleanup()
os.kill(os.getpid(), signal.SIGTERM)