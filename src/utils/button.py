import RPi.GPIO as GPIO
import time

# Just a simple button class for GPIO buttons
class Button:
    def __init__(self, pin: int, callback = None):
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Pull up, becouse the button is connected to ground
        GPIO.add_event_detect(pin, GPIO.FALLING, callback=self._pressed, bouncetime=200) # Add callback for falling edge
        self.callback = callback
        self.count = 0
    
    def _pressed(self, idk):
        if self.callback is not None:
            self.callback()
        self.count += 1
    
    def get_count(self):
        return self.count
    
    def wait_until(self):
        while self.count == 0:
            time.sleep(0.1)
        