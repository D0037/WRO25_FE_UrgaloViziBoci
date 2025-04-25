import utils.movement as move
import RPi.GPIO as GPIO
import requests

GPIO.setmode(GPIO.BCM)
move.init()

while True:
    try:
        response = requests.get("http://192.168.0.191:8000/")
        data = response.json()
        move.set_angle(float(data["s"]))
        motor = float(data["m"])
        move.set_speed(motor)
    except KeyboardInterrupt:
        break

# Cleanup GPIO settings
GPIO.cleanup((5,6,23,24))
