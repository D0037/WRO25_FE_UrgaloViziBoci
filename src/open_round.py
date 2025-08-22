import utils.movement as move
import utils.sensors as sensors
import utils.image_processing as processing
import time
import threading
import config
from utils.button import Button
import RPi.GPIO as GPIO

def until_dist(dist, sensor_dist, reverse=False):
    real_dist = sensors.tof_front.get_distance()
    print("distance from wall:", real_dist)

    if 50 < real_dist <= sensor_dist and not reverse:
        return True
    if sensor_dist < real_dist and reverse:
        return True
    
    return False

# Positions
posistions = {
    ### ===== LEFT turns =====
    "wall_middle_left":         lambda dist, _, dir:    300 < dist < 700 and dir,
    "wall_outer_left":          lambda dist, _, dir:    dist < 220 and dir,

    ### ===== RIGHT turns =====
    ## BIG walls
    "big_wall_middle_right":     lambda dist, _, dir:            dist < 20 and not dir,
    "big_wall_outer_right":      lambda dist, black_count, dir: 180 < dist < 220 and not dir
        and ((not processing.blue_visible and black_count <= 400) or (processing.x_intersect >= 2000)),

    ## small walls
    "small_wall_inner_right":    lambda dist, black_count, dir:   180 < dist < 220 and not dir
        and ((not processing.blue_visible and black_count >= 400) or (processing.x_intersect < 2000)),
    "small_wall_middle_right":   lambda dist, _, dir:        300 < dist < 500 and not dir,
    "small_wall_outer_right":    lambda dist, _, dir:        600 < dist and not dir,
}

def actions(pos):

    # === RIGHT TURNS ===
    # BIG WALLS
    if pos == "big_wall_middle_right":
        move.move(100, until=move.Until(until_dist, 1000))
        move.turn(90, 60, 14)

    elif pos == "big_wall_outer_right":
        move.move(100, until=move.Until(until_dist, 1000))
        move.turn(90, 60, 14)
    
    # SMALL WALLS
    elif pos == "small_wall_inner_right":
        move.move(100, until=move.Until(until_dist, 1000))
        move.turn(90)
    elif pos == "small_wall_middle_right":
        move.move(100, until=move.Until(until_dist, 1000))
        move.turn(90)
    elif pos == "small_wall_outer_right":
        move.move(100, until=move.Until(until_dist, 1000))
        move.turn(90)

    
    # === LEFT TURNS ===
    elif pos == "wall_inner_left":
        move.move(100, until=move.Until(until_dist, 1100))
        move.turn(-90)
    elif pos == "wall_middle_left":
        #sensors.tof_front.set_roi(8, 4, 90)
        move.turn(35)
        move.turn(-35)
        move.move(-100, until=move.Until(until_dist, 1600, True))
        time.sleep(2)
        move.move(100, until=move.Until(until_dist, 1600))
        move.turn(-90)
    elif pos == "wall_outer_left":
        move.move(100, until=move.Until(until_dist, 1100))
        move.turn(-90)

def start():
    processing.set_mode("heading")
    print(processing.direction)
    time.sleep(2)

    while True:

        found = False
        while (not found):
            side = sensors.tof_side.get_distance()
            front = sensors.tof_front.get_distance()
            back = sensors.tof_back.get_distance()

            print(f"Side: {side}, Front: {front}, Back: {back}, black count: {processing.black_count}, x intersect: {processing.x_intersect}")

            for pos in posistions.keys():
                if posistions[pos](side, processing.black_count, processing.direction):
                    sensors.tof_front.set_distance_mode(2)
                    sensors.tof_front.set_timing_budget_in_ms(20)
                    sensors.tof_front.set_inter_measurement_in_ms(20)
                    
                    #while True:
                     #   print("side: ", sensors.tof_front.get_distance())

                    print(pos)
                    actions(pos)
                    found = True
                    break

        time.sleep(1)

        
        if processing.direction:
            for i in range(8):
                move.move(250, until=move.Until(until_dist, 1500))
                move.turn(-90)
                
            move.set_speed(20)
            time.sleep(1.5)
            
        
        else: 
            for i in range(8):
                move.move(250, until=move.Until(until_dist, 1500))
                move.turn(90)
            
            #move.move(50, until=move.Until(until_dist, 1500))
            move.set_speed(20)
            time.sleep(1.5)
    
        

    
GPIO.setmode(GPIO.BCM)
start_btn = Button(config.BTN_START)

move.init()
sensors.init()

# Wait for start
print("Waiting for button press")
start_btn.wait_until()

image_thread = threading.Thread(target=processing.image_thread)
image_thread.start()

time.sleep(2)

# Reset angle
move.global_angle = move.tracker.get_angle()

start()