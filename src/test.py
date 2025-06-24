import utils.movement as move
import RPi.GPIO as GPIO
import time
import math
#import utils.image_processing as processing
import traceback

GPIO.setmode(GPIO.BCM)
try:
    move.init()

    while True:
        s = input("input: ")
        op = s.split()[0]
        if op == "turn":
            f = 1
            if len(s.split()) == 4:
                f = int(s.split()[3])
            try:
                move.turn(float(s.split()[1]), float(s.split()[2]), 10, forward=f)
            except Exception as e:
                print(f"WTF: {e}")
                traceback.print_exc()
                move.set_speed(0)
                move.set_angle(0)
        elif op == "move":
            speed = 12
            if len(s.split()) == 3:
                move.move(float(s.split()[1]), float(s.split()[2]))
            else:
                move.move(float(s.split()[1]))
        elif op == "pid":
            [_, angle, radius, p, i, d] = s.split()
            with open("pid.txt", "+a") as f:
                f.write(f"r: {radius}, p: {p}, i: {i}, d:{d}\n")
            try:
                move.turn(float(angle), float(radius), p=float(p), i=float(i), d=float(d))
            except KeyboardInterrupt:
                move.set_angle(0)
                move.set_speed(0)
            # 
except KeyboardInterrupt:
    pass

move.set_angle(0)
move.set_speed(0)
move.cleanup()