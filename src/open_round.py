import utils.movement as move
import utils.sensors as sensors
import utils.image_processing as processing
import time
import threading

def until_dist(dist, sensor_dist):
    real_dist = sensors.tof_back.get_distance()

    if real_dist >=sensor_dist:
        return True
    
    return False

def start():
    processing.set_mode("heading")
    print(processing.direction)
    #time.sleep(10)
    processing.set_mode("none")

    if False: #processing.direction:
        pass
    else:
        while True:
            print(f"side: {sensors.tof_side.get_distance()}, front: {sensors.tof_front.get_distance()}, back: {sensors.tof_back.get_distance()}")
            time.sleep(0.2)
    
    move.move(300, 10, move.Until(until_dist, 200))

move.init()
sensors.init()

image_thread = threading.Thread(target=processing.image_thread)
image_thread.start()

start()