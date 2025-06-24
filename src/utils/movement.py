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
        @angle: angle to set servo to
    """
    # Map the angle to a duty cycle (0 to 100)
    duty = ((angle + 90) / 36) + 5
    servo_pwm.change_duty_cycle(max(min(duty, 20), 0))

def set_speed(speed: float):
    if speed > 0:
        l_pwm.ChangeDutyCycle(0)
        r_pwm.ChangeDutyCycle(speed)
    elif speed < 0: 
        r_pwm.ChangeDutyCycle(0)
        l_pwm.ChangeDutyCycle(-speed)
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
    
    start_gyro = tracker.gyro.get_z()
    prev_dist = -1

    set_angle(0) # move servo to center

    while abs(relative_start.get_point_y(tracker.get_x(), tracker.get_y())) < abs(distance):
        error = start_gyro - tracker.gyro.get_z()
        correction = error * -p

        # Calculate speed
        current_dist = abs(relative_start.get_point_y(tracker.get_x(), tracker.get_y()))
        
        speed = 10

        if current_dist != prev_dist:
            #print("move", speed, error, correction, current_dist, acc_dist)
            prev_dist = current_dist

        set_angle(correction * (distance / abs(distance)))
        set_speed(speed * (distance / abs(distance)))
    
    set_speed(-5)

def turn(angle: float, radius: float, max_speed = 12, p=None, i=None, d=None, forward=1):
    """Move in a circle with a radius
        @angle: angle difference after stop from starting point
        @radius: radius of the circle
        @max_speed: max speed to reach
        @p: correction constant
    """
    relative_start = RelativeCoordinates(tracker.get_x(), tracker.get_y(), tracker.gyro.get_z() * radian_conversion)
    angle_start = tracker.gyro.get_z()
    angle_prev = -1

    #p /= radius / 20

    set_speed(max_speed)
    start_time = time.time()
    #pid = PID(8, 0.35, 8) # 100
    #pid = PID(19, 0.5, 8) # 50
    if p == i == d == None:
        p, i, d = get_pid(radius)
        if radius == 60:
            p, i, d = 12, 0.40, 16
    pid = PID(p, i, d) # 40
    print(p, i, d, radius)

    with open("turn.csv", "w") as f:
        f.write("angle,correction,derived,rotát-ion,x_pos,y_pos,p,i,d,bat,time\n")

    correction = 0
    prev_pos = relative_start.get_point(tracker.get_x(), tracker.get_y())
    d_prev = 0
    p, i, d = 0,0,0

    angle_sign = angle / abs(angle)

    while abs(tracker.gyro.get_z() - angle_start) < abs(angle):
        x_pos, y_pos = relative_start.get_point(tracker.get_x(), tracker.get_y())
        angle_current = tracker.gyro.get_z() - angle_start

        if prev_pos != (x_pos, y_pos):
            prev_pos = x_pos, y_pos

            def derivative_x(x):
                derived = x * (radius - abs(x)) / ((abs(x)**(3/2) * math.sqrt(2 * radius - abs(x))) + 1e-10)
                return derived

            def derivative_y(y):
                derived = y / (math.sqrt(max(radius**2 - y**2, 0)) + 1e-10)
                return derived

            rotation = 0
            derived = 0

            # forward
            if forward == 1:
                # positive angle: left, negative x
                if True:
                    if abs(angle_current) < 45:
                        y = y_pos
                        derived = derivative_y(y) * -angle_sign
                        rotation = to_deg(math.atan(derived))
                    else:
                        x = x_pos
                        derived = derivative_x(abs(x))
                        rotation = (90 - to_deg(derived)) * -angle_sign

            # backwards
            elif forward == -1:
                if abs(angle_current) > 45:
                    x = x_pos
                    derived = abs(x) * (radius - abs(x)) / ((abs(x)**(3/2) * math.sqrt(2 * radius - abs(x))) + 1e-10)
                    rotation = (90 - (math.atan(derived) * 180 / math.pi))
                    if x < 0:
                        rotation -= 180
                    rotation *= -1
                else:
                    y = y_pos
                    derived = y / (math.sqrt(max(radius**2 - y**2, 0)) + 1e-10)
                    rotation = (math.atan(derived) * 180 / math.pi * (angle / abs(angle)))

            print(x_pos, y_pos)

            set_speed(max_speed * forward)

            if (prev_pos != x_pos, y_pos):
                prev_pos = x_pos, y_pos
                correction, p, i, d_new = pid.update(angle_current - rotation)
                if d_new != 0:
                    d = d_new
                
                log = f"{angle_current}, {correction}, {derived}, {rotation}, {x_pos}, {y_pos}, {p}, {i}, {d}, {adc.read_bat()}, {time.time() - start_time}"
                with open("turn.csv", "+a") as f:
                    f.write(log + "\n")


        set_angle(correction)
    
    set_speed(0)
    set_angle(0)