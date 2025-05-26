import RPi.GPIO as GPIO
from rpi_hardware_pwm import HardwarePWM
from utils.bin.tracker import PositionTracker
import time
import math

# PWM chip (hardware)
servo_pwm_chip = 0

# PWM pins (software)
r_pwm_pin = 5
l_pwm_pin = 6

# Other motor controller pins
l_en = 23
r_en = 24

servo_pwm: None | HardwarePWM = None
l_pwm: None | GPIO.PWM = None
r_pwm: None | GPIO.PWM = None

tracker: None | PositionTracker = None

radian_conversion = math.pi / 180

class RelativeCoordinates:
    def __init__(self, origin_x, origin_y, angle):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.angle = angle

    def get_point(self, x, y):
        x_new = math.cos(self.angle) * (x - self.origin_x) - math.sin(self.angle) * (y - self.origin_y)
        y_new = math.sin(self.angle) * (x - self.origin_x) + math.cos(self.angle) * (y - self.origin_y)

        return x_new, y_new

    def get_point_x(self, x, y):
        return math.cos(self.angle) * (x - self.origin_x) - math.sin(self.angle) * (y - self.origin_y)

    def get_point_y(self, x, y):
        return math.sin(self.angle) * (x - self.origin_x) + math.cos(self.angle) * (y - self.origin_y)
    
    def set_origin(self, x, y):
        self.x, self.y = x, y
    
    def set_origin_from_relative(self, x, y):
        relative_to_origin = RelativeCoordinates(-self.origin_x, -self.origin_y, -self.angle)
        new_origin_x, new_origin_y = relative_to_origin.get_point(x, y)

        self.origin_x = new_origin_x
        self.origin_y = new_origin_y



def init():
    global servo_pwm, l_pwm, r_pwm, tracker

    tracker = PositionTracker("/dev/i2c-1", 0x68, 1, 0, 496.5)

    # Setup GPIO
    GPIO.setup(r_en, GPIO.OUT)
    GPIO.setup(l_en, GPIO.OUT)
    GPIO.setup(r_pwm_pin, GPIO.OUT)
    GPIO.setup(l_pwm_pin, GPIO.OUT)

    # Setup PWM
    # Hardware PWM for the servo
    servo_pwm = HardwarePWM(pwm_channel=0, hz=50, chip=servo_pwm_chip)
    servo_pwm.start(0)

    # Software PWM for motor controller
    l_pwm = GPIO.PWM(l_pwm_pin, 500)
    r_pwm = GPIO.PWM(r_pwm_pin, 500)
    l_pwm.start(0)
    r_pwm.start(0)

    # Enable motor controller
    GPIO.output(r_en, GPIO.HIGH)
    GPIO.output(l_en, GPIO.HIGH)

def set_angle(angle):
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
    #tracker.kill()

def move(distance: float, max_speed: float = 10, p: float = 2, a: float = 30):
    relative_start = RelativeCoordinates(tracker.get_x(), tracker.get_y(), tracker.gyro.get_z() * radian_conversion)
    
    start_gyro = tracker.gyro.get_z()
    acc_dist = max_speed * max_speed / (2 * a)
    prev_dist = -1
    stage = 0

    set_angle(0) # move servo to center

    while abs(relative_start.get_point_y(tracker.get_x(), tracker.get_y())) < abs(distance):
        error = start_gyro - tracker.gyro.get_z()
        correction = error * -p

        # Calculate speed
        current_dist = abs(relative_start.get_point_y(tracker.get_x(), tracker.get_y()))
        """if current_dist == 0:
            speed = 30
            stage = 1
        elif current_dist < acc_dist:
            speed = math.sqrt(2 * a * current_dist)
            stage = 2
        elif abs(distance) - acc_dist > current_dist and current_dist > acc_dist:
            speed = max_speed
            stage = 3
        elif abs(distance) > current_dist:
            speed = math.sqrt(2 * a * (abs(distance) - current_dist))
            stage = 4"""
        speed = 10

        if current_dist != prev_dist:
            print("move", speed, error, correction, current_dist, acc_dist)
            prev_dist = current_dist

        set_angle(correction * (distance / abs(distance)))
        set_speed(speed * (distance / abs(distance)))
    
    set_speed(-5)

def turn(angle: float, radius: float, max_speed = 10, p = 30):
    relative_start = RelativeCoordinates(tracker.get_x(), tracker.get_y(), tracker.gyro.get_z() * radian_conversion)
    relative_start.set_origin_from_relative(-radius, 0)
    print(relative_start.get_point(tracker.get_x(), tracker.get_y()), tracker.get_x(), tracker.get_y(), tracker.gyro.get_z())
    angle_start = tracker.gyro.get_z()
    angle_prev = -1

    p /= radius / 20

    set_speed(max_speed)

    while abs(tracker.gyro.get_z() - angle_start) < abs(angle):
        x_pos, y_pos = relative_start.get_point(tracker.get_x(), tracker.get_y())
        angle_current = tracker.gyro.get_z() - angle_start

        dist_from_center = math.sqrt(x_pos * x_pos + y_pos * y_pos)
        error = dist_from_center - radius
        if error < 0:
            error /= 5
        correction = error * p

        if angle_prev != angle_current:
            print("turn", angle_current, error, dist_from_center, x_pos, y_pos, p)
            angle_prev = angle_current

        set_angle(correction * (-angle / abs(angle)))
    
    set_speed(0)