import utils.movement as move
import utils.sensors as sensors
import utils.image_processing as processing
import time
import threading
import config
from utils.button import Button

def until_dist(dist, sensor_dist):
    real_dist = sensors.tof_back.get_distance()

    if real_dist >=sensor_dist:
        return True
    
    return False

# Positions
posistions = {
    ### ===== LEFT turns =====
    ## BIG walls
    "big_wall_middle_right":    lambda dist, _, dir:            dist < 10 and dir,
    "big_wall_outer_right":     lambda dist, black_count, dir:  10 < dist < 200 and black_count < 750 and dir,

    ## small walls
    "small_wall_inner_right":   lambda dist, black_count, dir:  10 < dist < 200 and black_count >= 750 and dir,
    "small_wall_middle_right":  lambda dist, _, dir:            200 < dist < 400 and dir,
    "small_wall_outer_right":   lambda dist, _, dir:            400 < dist and dir,

    ### ===== RIGHT turns =====
    ## BIG walls
    "big_wall_middle_left":     lambda dist, _, dir:            dist < 10,
    "big_wall_outer_left":      lambda dist, black_count, dir:  10 < dist < 200 and black_count < 750 and not dir,

    ## small walls
    "small_wall_inner_left":    lambda dist, black_count:   10 < dist < 200 and black_count >= 750,
    "small_wall_middle_left":   lambda dist, _, dir:        200 < dist < 400 and not dir,
    "small_wall_outer_left":    lambda dist, _, dir:        400 < dist and not dir,
}

def actions(pos):

    # === RIGHT TURNS ===
    # BIG WALLS
    if pos == "big_wall_middle_right":
        move.move(100, until=move.Until(until_dist, 600))
        move.turn(90, 60, 14)
        move.move(20)

    elif pos == "big_wall_outer_right":
        move.move(100, until=move.Until(until_dist, 600))
        move.turn(90, 60, 14)
        move.move(40)
    
    # SMALL WALLS
    elif pos == "small_wall_inner_right":
        move.move(100, until=move.Until(until_dist, 600))
        move.turn(90, 60, 14)
    elif pos == "small_wall_middle_right":
        move.move(100, until=move.Until(until_dist, 600))
        move.turn(90, 60, 14)
        move.move(20)
    elif pos == "small_wall_outer_right":
        move.move(100, until=move.Until(until_dist, 600))
        move.turn(90, 60, 14)
        move.move(40)

    
    # === LEFT TURNS ===
    # BIG WALLS
    elif pos == "big_wall_middle_left":
        move.move(100, until=move.Until(until_dist, 600))
        move.turn(-90, 60, 14)
        move.move(20)
    elif pos == "big_wall_outer_left":
        move.move(100, until=move.Until(until_dist, 600))
        move.turn(-90, 60, 14)
        move.move(40)
    
    # SMALL WALLS
    elif pos == "small_wall_inner_left":
        move.move(100, until=move.Until(until_dist, 600))
        move.turn(-90, 60, 14)
    elif pos == "small_wall_middle_left":
        move.move(100, until=move.Until(until_dist, 600))
        move.turn(-90, 60, 14)
        move.move(20)
    elif pos == "small_wall_outer_left":
        move.move(100, until=move.Until(until_dist, 600))
        move.turn(-90, 60, 14)
        move.move(40)

def start():
    processing.set_mode("heading")
    print(processing.direction)
    time.sleep(10)
    processing.set_mode("none")

    side_avg = 0
    front_avg = 0
    back_avg = 0

    for i in range(10):
        side_avg += sensors.tof_side.get_distance()
        front_avg += sensors.tof_front.get_distance()
        back_avg += sensors.tof_back.get_distance()
        time.sleep(0.2)
    
    side_avg /= 10
    front_avg /= 10
    back_avg /= 10

    print(f"Side: {side_avg}, Front: {front_avg}, Back: {back_avg}")

    for pos in posistions.keys():
        if posistions[pos](side_avg, processing.black_count, processing.direction):
            print(pos)
            actions(pos)
    
    if processing.direction:
        for i in range(8):
            move.move(250, until=move.Until(until_dist, 60))
            move.turn(90, 60)
        
        move.move(50)
    
    else: 
        for i in range(8):
            move.move(250, until=move.Until(until_dist, 60))
            move.turn(-90, 60)
        
        move.move(50)
    
start_btn = Button(config.BTN_START)

move.init()
sensors.init()

# Wait for start
start_btn.wait_until()

image_thread = threading.Thread(target=processing.image_thread)
image_thread.start()

time.sleep(2)

# Reset angle
move.global_angle = move.tracker.get_angle()

start()