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

    """ ROI Center table
        128,136,144,152,160,168,176,184,  192,200,208,216,224,232,240,248

        129,137,145,153,161,169,177,185,  193,201,209,217,225,233,241,249

        130,138,146,154,162,170,178,186,  194,202,210,218,226,234,242,250

        131,139,147,155,163,171,179,187,  195,203,211,219,227,235,243,251

        132,140,148,156,164,172,180,188,  196,204,212,220,228,236,244,252 
        
        133,141,149,157,165,173,181,189,  197,205,213,221,229,237,245,253 
        
        134,142,150,158,166,174,182,190,  198,206,214,222,230,238,246,254
        
        135,143,151,159,167,175,183,191,  199,207,215,223,231,239,247,255

        
        127,119,111,103,095,087,079,071,  063,055,047,039,031,023,015,007
        
        126,118,110,102,094,086,078,070,  062,054,046,038,030,022,014,006
        
        125,117,109,101,093,085,077,069,  061,053,045,037,029,021,013,005
        
        124,116,108,100,092,084,076,068,  060,052,044,036,028,020,012,004
        
        123,115,107,099,091,083,075,067,  059,051,043,035,027,019,011,003
        
        122,114,106,098,090,082,074,066,  058,050,042,034,026,018,010,002
        
        121,113,105,097,089,081,073,065,  057,049,041,033,025,017,009,001
        
        120,112,104,096,088,080,072,064,  056,048,040,032,024,016,008,000
    """


    GPIO.output(config.SHUT_LASER_1, GPIO.HIGH)
    time.sleep(0.2)
    # --- Configure sensors (address, region of interest, ranging, mode, timing budget) ---
    if not tof_back.init_sensor(0x29): # Check for the succes of the initialization
        raise IOError("Connection with the back tof sensor could not be initialized!")

    # Config back sensor
    tof_back.set_i2c_address(0x2b)
    tof_back.set_inter_measurement_in_ms(200)
    tof_back.set_timing_budget_in_ms(200)
    tof_back.set_roi(16, 4, 197)
    tof_back.set_distance_mode(2)
    tof_back.start_ranging()

    time.sleep(0.2)
    GPIO.output(config.SHUT_LASER_2, GPIO.HIGH) # Enable sensor
    time.sleep(0.2)
    if not tof_front.init_sensor(0x29): # Check for success
        raise IOError("Connection with the front tof sensor could not be initialized!")
    
    # Config front sensor
    tof_front.set_i2c_address(0x2a)
    tof_front.set_inter_measurement_in_ms(200)
    tof_front.set_timing_budget_in_ms(200)
    tof_front.set_roi(8, 4, 195)
    tof_front.set_distance_mode(2)
    tof_front.start_ranging()

    time.sleep(0.2)
    GPIO.output(config.SHUT_LASER_3, GPIO.HIGH) # Enable sensor
    time.sleep(0.2)
    if not tof_side.init_sensor(0x29): # Check for success
        print("fuckk the back sensor!!!")
        raise IOError("Connection with the side tof sensor could not be initialized!")
    
    # Config side sensor
    tof_side.set_inter_measurement_in_ms(200)
    tof_side.set_timing_budget_in_ms(200)
    tof_side.set_roi(16, 4, 195)
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