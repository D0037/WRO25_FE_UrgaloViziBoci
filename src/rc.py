import utils.movement as move
import RPi.GPIO as GPIO
import requests

GPIO.setmode(GPIO.BCM)
move.init()

while True:
    try:
        response = requests.get("http://10.42.0.1:8000/")
        data = response.json()
        move.set_angle(float(data["s"]) * 1.5)
        print(data["s"])
        motor = float(data["m"]*1.1)
        move.set_speed(motor)
    except KeyboardInterrupt:
        break

# Cleanup GPIO settings
GPIO.cleanup((5,6,23,24))
