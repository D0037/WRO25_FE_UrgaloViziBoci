import RPi.GPIO as GPIO
from rpi_hardware_pwm import HardwarePWM
from utils.bin.tracker import PositionTracker
from utils.pid import PID
import time
import math
import utils.adc as adc

# PWM chip (hardware)
servo_pwm_chip = 0

# PWM pins (software)
r_pwm_pin = 5
l_pwm_pin = 6
MOTOR_PWM_FREQ = 1000

# Other motor controller pins
l_en = 23
r_en = 24

servo_pwm: None | HardwarePWM = None
l_pwm: None | GPIO.PWM = None
r_pwm: None | GPIO.PWM = None

tracker = None

radian_conversion = math.pi / 180

global_angle: float = 0

def get_pid(radius: float) -> tuple[float, float, float]:
    p = 30 - (11 / 50) * radius
    i = -0.003 * radius + 0.65
    d = 8

    return p, i, d

class RelativeCoordinates:
    def __init__(self, origin_x, origin_y, angle):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.angle = angle
        self.y_const = 0
        self.x_const = 0

    def get_point(self, x, y):
        x_new = math.cos(self.angle) * (x - self.origin_x) - math.sin(self.angle) * (y - self.origin_y)
        y_new = math.sin(self.angle) * (x - self.origin_x) + math.cos(self.angle) * (y - self.origin_y)

        return x_new + self.x_const, y_new + self.y_const

    def get_point_x(self, x, y):
        return (math.cos(self.angle) * (x - self.origin_x) - math.sin(self.angle) * (y - self.origin_y)) + self.x_const

    def get_point_y(self, x, y):
        return (math.sin(self.angle) * (x - self.origin_x) + math.cos(self.angle) * (y - self.origin_y)) + self.y_const
    
    def set_origin(self, x, y):
        self.x, self.y = x, y
    
    def set_origin_from_relative(self, x, y):
        self.y_const = y
        self.x_const = x

def to_deg(radian):
    return radian * 180 / math.pi


def init():
    global servo_pwm, l_pwm, r_pwm, tracker

    tracker = PositionTracker("/dev/i2c-1", 0x68, 1, 0, 496.5)

    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(r_en, GPIO.OUT)
    GPIO.setup(l_en, GPIO.OUT)
    GPIO.setup(r_pwm_pin, GPIO.OUT)
    GPIO.setup(l_pwm_pin, GPIO.OUT)

    # Setup PWM
    # Hardware PWM for the servo
    servo_pwm = HardwarePWM(pwm_channel=0, hz=50, chip=servo_pwm_chip)
    servo_pwm.start(0)

    # Software PWM for motor controller
    l_pwm = GPIO.PWM(l_pwm_pin, MOTOR_PWM_FREQ)
    r_pwm = GPIO.PWM(r_pwm_pin, MOTOR_PWM_FREQ)
    l_pwm.start(0)
    r_pwm.start(0)

    # Enable motor controller
    GPIO.output(r_en, GPIO.HIGH)
    GPIO.output(l_en, GPIO.HIGH)

    #Initialize ADC
    adc.init()

def set_angle(angle):
    """Set angle of servo relative to forward pointing wheels
        @angle: angle to set servo to (positive to right)
    """
    # Map the angle to a duty cycle (0 to 100)
    duty = ((angle + 90) / 36) + 5
    servo_pwm.change_duty_cycle(max(min(duty, 20), 0))

def set_speed(speed: float):
    bat = 0 #adc.read_bat()
    multiplier = 1 #(1 + math.sqrt(8.4 - bat)) / math.sqrt(8.4 - bat)
    if speed > 0:
        l_pwm.ChangeDutyCycle(0)
        r_pwm.ChangeDutyCycle(speed * multiplier)
    elif speed < 0: 
        r_pwm.ChangeDutyCycle(0)
        l_pwm.ChangeDutyCycle(-speed * multiplier)
    else:
        r_pwm.ChangeDutyCycle(0)
        l_pwm.ChangeDutyCycle(0)

def cleanup():
    servo_pwm.stop()
    l_pwm.stop()
    r_pwm.stop()
    tracker.kill()
    GPIO.cleanup((5,6,23,24))

def move(distance: float, max_speed: float = 10, p: float = 10, a: float = 30):
    """Function to move in a straight line 
        @distance: distance to go
        @max_speed: max speed to reach
        @p: correction constant
    """
    print(f"move {distance}")
    relative_start = RelativeCoordinates(tracker.get_x(), tracker.get_y(), tracker.gyro.get_z() * radian_conversion)
    
    global global_angle
    start_gyro = global_angle
    prev_dist = -1
    prev_angle = tracker.gyro.get_z()

    set_angle(0) # move servo to center

    while abs(relative_start.get_point_y(tracker.get_x(), tracker.get_y())) < abs(distance):
        error = tracker.gyro.get_z() - start_gyro
        if prev_angle != tracker.gyro.get_z():
            prev_angle = tracker.gyro.get_z()
        correction = error * -p

        # Calculate speed
        current_dist = abs(relative_start.get_point_y(tracker.get_x(), tracker.get_y()))
        
        speed = 10

        if current_dist != prev_dist:
            #print("move", speed, error, correction, current_dist, acc_dist)
            prev_dist = current_dist

        set_angle(correction * (distance / abs(distance)))
        set_speed(speed * (distance / abs(distance)))
    
    set_speed(0)

def turn(angle: float, radius: float, max_speed = 10, p=None, i=None, d=None, forward=1):
    """Move in a circle with a radius
        @angle: angle difference after stop from starting point (positive to right)
        @radius: radius of the circle
        @max_speed: max speed to reach
        @p: correction constant
    """
    global global_angle

    relative_start = RelativeCoordinates(tracker.get_x(), tracker.get_y(), tracker.gyro.get_z() * radian_conversion)
    angle_start = global_angle

    global_angle += angle

    #p /= radius / 20
    pid_conf = {
        (60, 1, 1): (17, 0.2, 30),
        (60, -1, 1): (17, 0.2, 30),
    }

    angle_sign = angle / abs(angle)
    set_speed(max_speed)
    start_time = time.time()
    #pid = PID(8, 0.35, 8) # 100
    #pid = PID(19, 0.5, 8) # 50
    #if p == i == d == None:
    #    if list(pid_conf.keys()).count((radius, angle_sign, forward)) == 1:
    #        p, i , d = pid_conf[radius]
    pid = PID(17, 0.3, 8) # 40
    print("turn", angle, radius, forward)

    with open("turn.csv", "w") as f:
        f.write("angle,ratio,correction,rotation_calculated,x_rotation_calculated,y_rotation_calculated,distance_error,rotation_error,x_pos,y_pos,p,i,d,bat,time\n")

    correction = 0
    prev_pos = relative_start.get_point(tracker.get_x(), tracker.get_y())
    d_prev = 0
    p, i, d = 0,0,0

    while abs(tracker.gyro.get_z() - angle_start) < abs(angle):
        x_pos, y_pos = relative_start.get_point(tracker.get_x(), tracker.get_y())
        angle_current = tracker.gyro.get_z() - angle_start

        if prev_pos != (x_pos, y_pos):
            prev_pos = x_pos, y_pos
            distance_from_center = math.sqrt((radius - abs(x_pos))**2 + abs(y_pos)**2)

            def derivative_x(x):
                derived = x * (radius - abs(x)) / ((abs(x)**(3/2) * math.sqrt(2 * radius - abs(x))) + 1e-10)
                return derived

            def derivative_y(y):
                derived = y / (math.sqrt(max(radius**2 - y**2, 0)) + 1e-10)
                return derived

            x_rotation_calculated = 0
            y_rotation_calculated = 0
            x_derived = 0
            y_derived = 0

            # positive angle: left, negative x
            if True:
                y = y_pos
                y_derived = derivative_y(y) * angle_sign * forward
                y_rotation_calculated = to_deg(math.atan(y_derived))
                x = x_pos
                x_derived = derivative_x(abs(x))
                x_rotation_calculated = (90 - to_deg(math.atan(x_derived))) * angle_sign
            
            ratio = (abs(angle_current) + 1e-10) / 90
            rotation_calculated = x_rotation_calculated * ratio + y_rotation_calculated * (1 - ratio)

            set_speed(max_speed * forward)

            if (prev_pos != x_pos, y_pos):
                prev_pos = x_pos, y_pos
                rotation_error = (rotation_calculated - angle_current) * forward # brutálisan megszorozva xdd
                distance_error = (distance_from_center - radius) * angle_sign * forward
                error = rotation_error * 0.3 + distance_error
                correction, p, i, d_new = pid.update(error)
                if d_new != 0:
                    d = d_new
                
                log = f"{angle_current},{ratio},{correction},{rotation_calculated},{x_rotation_calculated},{y_rotation_calculated},{distance_error},{rotation_error},{x_pos},{y_pos},{p},{i},{d},{adc.read_bat()},{time.time() - start_time}"
                with open("turn.csv", "+a") as f:
                    f.write(log + "\n")


        set_angle(correction)
    
    set_speed(0)
    set_angle(0)