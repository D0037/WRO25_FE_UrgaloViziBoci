import RPi.GPIO as GPIO
import time

class LED():
    def __init__(self, pin: int):
        self.pin = pin
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

    def on(self):
        GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        GPIO.output(self.pin, GPIO.LOW)
    
    def toggle(self):
        current_state = GPIO.input(self.pin)
        GPIO.output(self.pin, not current_state)

    def blink(self, freq, duty_cycle, duration):
        "Can also be used for brightness control via PWM"
        pwm = GPIO.PWM(self.pin, freq)
        pwm.start(duty_cycle)

        time.sleep(duration)
        pwm.stop()
        GPIO.output(self.pin, GPIO.LOW)