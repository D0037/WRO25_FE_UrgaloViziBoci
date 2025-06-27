# code for starting the whole program with a puss of the START buton
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def foo(channel):
    print("Button pressed!")

GPIO.add_event_detect(17, GPIO.RISING, callback=foo, bouncetime=500)

try:
    while True:
        time.sleep(1)  # Keep program running
except KeyboardInterrupt:
    print("Exiting...")
finally:
    GPIO.cleanup()