#code for running and connecting the whole project
import utils.movement as move
import time
import threading
import utils.image_processing as processing
import os
import signal
import round2
import RPi.GPIO as GPIO
import config
import utils.sensors as sensors

#starting thread
GPIO.setmode(GPIO.BCM)

sensors.init()

image_thread = threading.Thread(target=processing.image_thread)
image_thread.start()

do_sleeps = True

def sleep(time):
    if do_sleeps:
        time.sleep(time)

#processing.set_mode("parking")
move.init()

mode = "obstacle"

def start(channel):
    try:
        #setup for first round
        if mode == "free":
            processing.set_mode("start_pos")
            move.tracker.gyro.reset()
            move.tracker.reset()
            x, y, z = 0, 0, 0

            #set pos
            while (not processing.lines):
                time.sleep(0.001)
            pos = processing.start_pos
            print("START_POS", pos)
            processing.set_mode("none")
            # processing.kill = True

            # custom moving behaviors
            if pos == "front_outer": x, y, side = 40, 70, 150   #1
            if pos == "front_middle": x, y, side = 57, 50, 155  #2
            if pos == "front_inner": x, y, side = 35, 60, 115   #3
            if pos == "back_outer": x, y, side = 65, 70, 152  #4
            if pos == "back_middle": x, y, side = 92, 50, 155 #5
            if pos == "back_inner": x, y, side = 80, 60, 117.5 #6
            print("START_POS", pos)

            # dealing with directions
            z = 0
            if processing.direction: z = -90            
            else: z = 90

            # moving through the first round challange
            def first(x:float, y:float, z:float):
                move.move(x + 20)
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
            processing.set_mode("start_pos")
            time.sleep(2)
            d = 1
            if processing.direction:
                d = -1
            processing.set_mode("1st_obstacle")
            time.sleep(2)
            processing.set_mode("none")
            block = processing.first_block

            # Left turns
            if (d == -1):
                move.set_angle(40)
                move.set_speed(10)
                time.sleep(0.8)

                move.set_angle(-40)
                time.sleep(0.8)
                
                move.set_speed(10)
                move.set_angle(0)
                time.sleep(0.5)

                move.set_angle(-60)
                move.set_speed(20)
                time.sleep(0.62)

                move.set_angle(0)
                time.sleep(3)

                move.set_speed(-15)
                time.sleep(1.4)

                move.set_angle(0)
                move.set_speed(0)

                for i in range(11):
                    move.set_angle(-60)
                    move.set_speed(20)
                    time.sleep(1.2)

                    move.set_angle(0)
                    time.sleep(3)

                    move.set_speed(-15)
                    time.sleep(1)

                    move.set_angle(-60)
                    move.set_speed(20)
                    time.sleep(0.8)


            # Right turns
            else:
                move.set_speed(10)
                move.set_angle(-20)
                time.sleep(0.5)

                move.set_angle(0)
                time.sleep(2)

                move.set_angle(20)
                time.sleep(0.5)

                move.set_angle(0)
                move.set_speed(20)
                time.sleep(0.2)

                move.set_angle(60)
                time.sleep(0.8)

                move.set_angle(0)
                time.sleep(3)

                for i in range(11):
                    move.set_speed(-15)
                    time.sleep(1)
                    
                    move.set_angle(60)
                    move.set_speed(20)
                    time.sleep(0.8)

                    move.set_angle(0)
                    time.sleep(3)


    # for ctrl^C to work during data flow
    except:
        pass
    finally:
        # Finishing arguments
        move.set_speed(0)        # Finishing arguments
        move.set_speed(0)
        move.set_angle(0)

        # Cleanup
        processing.kill = True
        move.cleanup()
        os.kill
        move.set_angle(0)

        # Cleanup
        processing.kill = True
        move.cleanup()
        os.kill(os.getpid(), signal.SIGTERM)


GPIO.add_event_detect(17, GPIO.RISING, start, 200)
try:
    time.sleep(9999999)
except:
    pass
finally:
    move.set_speed(0)
    move.set_angle(0)
    processing.kill = True
    move.cleanup()
