import RPi.GPIO as GPIO
from rpi_hardware_pwm import HardwarePWM

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

def init():
    global servo_pwm, l_pwm, r_pwm

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
    servo_pwm.change_duty_cycle(duty)

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