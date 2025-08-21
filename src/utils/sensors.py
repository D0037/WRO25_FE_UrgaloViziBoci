import RPi.GPIO as GPIO
import config
import qwiic_vl53l1x
import time


tof_back = qwiic_vl53l1x.QwiicVL53L1X()
tof_front = qwiic_vl53l1x.QwiicVL53L1X()
tof_side = qwiic_vl53l1x.QwiicVL53L1X()

def init():
    # Setting up TOF ranging sensors and changing their I2C addresses to avoid conflict
    # Sensors are enabled one by one
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(config.SHUT_LASER_1, GPIO.OUT)
    GPIO.setup(config.SHUT_LASER_2, GPIO.OUT)
    GPIO.setup(config.SHUT_LASER_3, GPIO.OUT)
    
    # Reset sensors
    GPIO.output(config.SHUT_LASER_1, GPIO.LOW)
    GPIO.output(config.SHUT_LASER_2, GPIO.LOW)
    GPIO.output(config.SHUT_LASER_3, GPIO.LOW)

    time.sleep(0.2)


    GPIO.output(config.SHUT_LASER_1, GPIO.HIGH)
    time.sleep(0.2)
    # --- Configure sensors (address, region of interest, ranging, mode, timing budget) ---
    if not tof_back.init_sensor(0x29): # Check for the succes of the initialization
        raise IOError("Connection with the back tof sensor could not be initialized!")

    # Config back sensor
    tof_back.set_i2c_address(0x2b)
    tof_back.set_inter_measurement_in_ms(100)
    tof_back.set_roi(4, 4, 199)
    tof_back.set_distance_mode(2)
    tof_back.start_ranging()

    time.sleep(0.2)
    GPIO.output(config.SHUT_LASER_2, GPIO.HIGH) # Enable sensor
    time.sleep(0.2)
    if not tof_front.init_sensor(0x29): # Check for success
        raise IOError("Connection with the back tof sensor could not be initialized!")
    
    # Config front sensor
    tof_front.set_i2c_address(0x2c)
    tof_front.set_inter_measurement_in_ms(100)
    tof_front.set_roi(4, 4, 199)
    tof_front.set_distance_mode(2)
    tof_front.start_ranging()

    time.sleep(0.2)
    GPIO.output(config.SHUT_LASER_3, GPIO.HIGH) # Enable sensor
    time.sleep(0.2)
    if not tof_side.init_sensor(0x29): # Check for success
        raise IOError("Connection with the back tof sensor could not be initialized!")
    
    # Config side sensor
    tof_side.set_inter_measurement_in_ms(500)
    tof_side.set_roi(4, 4, 199)
    tof_side.set_distance_mode(1)
    tof_side.start_ranging()

# Mostly used for debugging purposes
def get():
    while True:
        print(f"front: {tof_front.get_distance()}, back: {tof_back.get_distance()}, side: {tof_side.get_distance()}")

# If its run by itself print a whole bunch of data
if __name__ == "__main__":
    init()
    get()