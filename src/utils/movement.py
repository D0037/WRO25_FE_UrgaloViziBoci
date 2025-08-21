import RPi.GPIO as GPIO
from rpi_hardware_pwm import HardwarePWM
from utils.bin.tracker import PositionTracker
from utils.pid import PID
import time
import math
import config

# Global variables for hardware control
servo_pwm: None | HardwarePWM = None
l_pwm: None | GPIO.PWM = None
r_pwm: None | GPIO.PWM = None

tracker = None

radian_conversion = math.pi / 180

global_angle: float = 0 # This is the angle of the robot in degrees, relative to the starting position
                        # it prevents accumulating errors (not gyro drift sadly)

# Tranforms coordinates relative to the robot's position and angle
class RelativeCoordinates:
    def __init__(self, origin_x, origin_y, angle):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.angle = angle
        self.y_const = 0
        self.x_const = 0

    # using trigonometry we can calculate the relative x and y values dependante on the incidance
    def get_point(self, x, y):
        x_new = math.cos(self.angle) * (x - self.origin_x) - math.sin(self.angle) * (y - self.origin_y)
        y_new = math.sin(self.angle) * (x - self.origin_x) + math.cos(self.angle) * (y - self.origin_y)

        return x_new + self.x_const, y_new + self.y_const

    # geting specific values from movement
    def get_point_x(self, x, y):
        return (math.cos(self.angle) * (x - self.origin_x) - math.sin(self.angle) * (y - self.origin_y)) + self.x_const

    def get_point_y(self, x, y):
        return (math.sin(self.angle) * (x - self.origin_x) + math.cos(self.angle) * (y - self.origin_y)) + self.y_const
    
    def set_origin(self, x, y):
        self.x, self.y = x, y
    
    def set_origin_from_relative(self, x, y):
        self.y_const = y
        self.x_const = x

# radian to degree
def to_deg(radian):
    return radian * 180 / math.pi

def init():
    """Initialize hardware"""
    global servo_pwm, l_pwm, r_pwm, tracker, adc, global_angle

    # Mouse and gyro based position tracker written in C++ (fairly accurate ((not accurate enough)))
    tracker = PositionTracker(config.MOUSE_CONVERSION_RATIO)
    time.sleep(2)
    global_angle = tracker.get_angle() # Starting angle of the robot (becouse of the magnetometer)

    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(config.MOTOR_R_EN, GPIO.OUT)
    GPIO.setup(config.MOTOR_L_EN, GPIO.OUT)
    GPIO.setup(config.MOTOR_R_PWM, GPIO.OUT)
    GPIO.setup(config.MOTOR_L_PWM, GPIO.OUT)

    # Setup PWM
    # Hardware PWM for the servo
    servo_pwm = HardwarePWM(pwm_channel=0, hz=50, chip=config.SERVO_PWM_CHIP)
    servo_pwm.start(0)

    # Software PWM for motor controller
    l_pwm = GPIO.PWM(config.MOTOR_L_PWM, config.MOTOR_PWM_FREQ)
    r_pwm = GPIO.PWM(config.MOTOR_R_PWM, config.MOTOR_PWM_FREQ)
    l_pwm.start(0)
    r_pwm.start(0)

    # Enable motor controller
    GPIO.output(config.MOTOR_R_EN, GPIO.HIGH)
    GPIO.output(config.MOTOR_L_EN, GPIO.HIGH)

def set_angle(angle):
    """Set angle of servo relative to forward pointing wheels
        @angle: angle to set servo to (positive to right)
    """
    # Map the angle to a duty cycle for the servo
    # The servo expects a duty cycle between 5% and 20%
    duty = ((angle + 90) / 36) + 5
    servo_pwm.change_duty_cycle(max(min(duty, 20), 0))

def set_speed(speed: float):
    """Set speed of the motor py varying PWM duty cycle
        @speed: duty cycle %
    """
    # Activate correct half-bridge with the given speed
    if speed > 0:
        l_pwm.ChangeDutyCycle(0)
        r_pwm.ChangeDutyCycle(speed)
    elif speed < 0: 
        r_pwm.ChangeDutyCycle(0)
        l_pwm.ChangeDutyCycle(-speed)
    else:
        r_pwm.ChangeDutyCycle(0)
        l_pwm.ChangeDutyCycle(0)

# Killing child processes and cleaning up GPIO
def cleanup():
    servo_pwm.stop()
    l_pwm.stop()
    r_pwm.stop()
    tracker.kill()
    GPIO.cleanup((5,6,23,24))

class Until:
    """A class to create a callable that can be used as a condition for movement functions"""
    def __init__(self, func, *args, **kwargs):
        self.func = func
        self.args = args
        self.kwargs = kwargs
    
    def __call__(self, dist):
        return self.func(dist, self.args, self.kwargs)
        

#general function to move in a straight line
def move(distance: float, max_speed: float = 20, until = None, p: float = 10):
    """Function to move in a straight line 
        @distance: distance to go
        @max_speed: max speed to reach
        @p: correction constant
        @until: a function that if it returns true the movement halts
    """
    print(f"move {distance}")
    relative_start = RelativeCoordinates(tracker.get_x(), tracker.get_y(), tracker.get_angle_rad())
    
    global global_angle
    start_gyro = global_angle # Save the starting angle of the robot, also prevents accumulating errors
    prev_angle = tracker.get_angle() # Gyro reported angle in previous cycle

    set_angle(0) # move servo to center

    # Main loop for moving
    while abs(relative_start.get_point_y(tracker.get_x(), tracker.get_y())) < abs(distance):
        error = tracker.get_angle() - start_gyro
        
        # Gyro return values between -180 and 180 degrees
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Skip cycle if there is no new data
        if prev_angle != tracker.get_angle():
            prev_angle = tracker.get_angle()
        else:
           pass

        correction = error * p

        current_dist = abs(relative_start.get_point_y(tracker.get_x(), tracker.get_y()))

        print(f"Distance: {current_dist}")
        
        # until is a callable that can be used to stop the movement or run code on the main thread mid-movement
        if until is not None:
            if until(current_dist):
                break
        
        # Calculate servo angle based on correction and movement direction
        angle = correction * (distance / abs(distance))
        
        # Actually set the speed and angle
        set_angle(angle)
        set_speed(-max_speed * (distance / abs(distance)))
    
    # Stop the motor
    set_speed(0)

def turn(angle: float, radius: float, max_speed = 8, p=None, i=None, d=None, forward=1):
    """Move in a circle with a radius
        @angle: angle difference after stop from starting point (positive to right)
        @radius: radius of the circle
        @max_speed: max speed to reach
        @p: correction constant
    """
    global global_angle

    # Relative coordinates with an origin at the current position and angle
    relative_start = RelativeCoordinates(tracker.get_x(), tracker.get_y(), tracker.get_angle_rad())
    angle_start = global_angle

    # Calculate the new global angle after the turn
    global_angle += angle
    global_angle = (global_angle + 180) % 360 - 180

    angle_sign = angle / abs(angle) # Sign of the angle to determine the direction of the turn (1 for right, -1 for left)
    set_speed(max_speed)
    
    pid = PID(17, 0.3, 8)
    print("turn", angle, radius, forward)

    correction = 0
    angle_current = 0

    #IDK ;)
    while abs(angle_current) < abs(angle):
        x_pos, y_pos = relative_start.get_point(tracker.get_x(), tracker.get_y()) # Get the current position relative to the starting point
        angle_current = tracker.get_angle() - angle_start
        prev_pos = (x_pos, y_pos)
        
        # Gyro return values between -180 and 180 degrees
        if angle_current > 180:
            angle_current -= 360
        elif angle_current < -180:
            angle_current += 360

        """
        WARNING: This is arguably the most complex part of the code, and IS extremely over-thought
            Proceed with caution!
        """
        """
        Calculating where we should be during the turning process
         two methods are used:
            1. Calculate distance from the center of the circle, to keep the robot on its path
            2. Based on position (x and y weighted based on where the vehicle is in the turn) calculate the what the angle should be
               This is done by calculating the derivative of the position and using it to calculate the angle
        """
        if prev_pos != (x_pos, y_pos):
            prev_pos = x_pos, y_pos
            distance_from_center = math.sqrt((radius - abs(x_pos))**2 + abs(y_pos)**2)

            # Derivative based on x position and y position
            def derivative_x(x):
                derived = x * (radius - abs(x)) / ((abs(x)**(3/2) * math.sqrt(2 * radius - abs(x))) + 1e-10) # És ezt miből számolta ki? Matematikus ön, vagy miből számolta ki?
                return derived

            def derivative_y(y):
                derived = y / (math.sqrt(max(radius**2 - y**2, 0)) + 1e-10) # IDK
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
            
            ratio = (abs(angle_current) + 1e-10) / 90 # Calculate weight based on the current angle, so the robot can turn smoothly
            rotation_calculated = x_rotation_calculated * ratio + y_rotation_calculated * (1 - ratio)

            """
               This is a lot simpler. It probably would have worked perfectly.
               But I just couldn't accept the few centimeters of error.
               So the above monstrousity was born.
            """
            # Keep the vehicle on the circle,
            # by trying to minimize the difference between the turning radius and the actual distance from the center
            distance_error = (distance_from_center - radius) * angle_sign * forward # Account for the direction of the turn (left, right, forward, backward)
            rotation_error = (rotation_calculated - angle_current) * forward # Account for the direction of the turn (left, right, forward, backward)

            if (prev_pos != x_pos, y_pos):
                prev_pos = x_pos, y_pos
                error = rotation_error * 0.3 + distance_error # Combine the distance error and rotation error, weighinh them with magic numbers
                correction, p, i, d = pid.update(error) # It returns p, i, d values for debugging purposes (but we don't use them here)
                
        # Control hardware
        set_angle(correction)
        set_speed(max_speed * forward)
    
    # Stop the motor, and reset the servo to center
    set_speed(0)
    set_angle(0)