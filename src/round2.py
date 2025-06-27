# code for the second rounds movement
import utils.movement as move
import utils.image_processing as processing
import time

#processing.get_blocks() -> ["r", "g", "n"]
#green left, red right

#Exiting parking spot
def exit_parking_spot():
    for i in range(3):
        move.turn(-30, 50)
        move.turn(30, 50, forward = -1)
    block = processing.get_block()

    #checking for block in the first segment
    if block != "g":
        move.move(70)
        move.turn(-90, 60, forward = -1)
        move.move(70)
        move.turn(90, 30)
        array = processing.get_blocks()
        move.move(-29)

    elif block != "r":
        move.turn(90, 80)
        move.move(50)
        move.turn(45, 90, forward = -1)
        array = processing.get_blocks()
        move.turn(45, 90, forward = -1)
    
#Going 3 rounds
def rounds(n, m, d):
    array = processing.get_blocks()
    for j in range(n):
        for i in range(m):
            if d == -1:
                for i in range(3):
                    if array[i] == "r":
                        array[i] == "g"
                    elif array[i] == "g":
                        array[i] == "r"
                    
            print(array)
            time.sleep(10)

            #algorithm for all greens segment
            if array.count("r") == 0 and array.count("g") > 0:
                print("all green")
                move.turn(-60 * d, 50)
                move.turn(60 * d, 50)
                move.move(92.5)
                move.turn(90 * d, 60)
                move.move(-70)
                array = processing.get_blocks()
                continue
            
            #algorithm for all reds segment
            if array.count("g") == 0 and array.count("r") > 0:
                print("all red")
                move.turn(12 * d, 50)
                move.turn(-12 * d, 50)
                move.move(275.5)
                move.turn(90 * d, 50, p=None, i=None, d=None, forward=-1)
                move.move(-30.5)
                array = processing.get_blocks()
                continue

            #algorithm for firs block red then second green segment
            if array[0] == "r" and array[2] == "g":
                print("red and green")
                move.turn(14 * d, 50)
                move.turn(-14 * d, 50)
                move.move(37.5)
                move.turn(-45 * d, 55)
                move.move(45)
                move.turn(45 * d, 55)
                move.move(11)
                move.turn(90 * d, 60)
                move.move(12)
                move.turn(-22.5 * d, 50, forward=-1)
                move.turn(22.5 * d, 50, forward=-1)
                move.move(-8)
                move.set_speed(-3)
                time.sleep(0.5)
                array = processing.get_blocks()
                continue

            #algorithm for first block green then second red segment
            if array[0] == "g" and array[2] == "r":
                print("green and red")
                move.turn(-60 * d, 50)
                move.turn(60 * d, 50)
                move.move(7.5)
                move.turn(45 * d, 50)
                move.move(45)
                move.turn(-45 * d, 50)
                move.move(90)
                move.turn(90 * d, 50, forward=-1)
                move.move(-30)
                array = processing.get_blocks()
                continue
            
            # Again in parking segment
            if i == 3:

                # Left turns
                blocks = processing.blocks
                if d == 1:
                    if blocks[0] == "r":
                        move.turn(12, 60)
                        move.turn(-12, 60)
                        move.move(15)
                        move.turn(-90, 60)
                        move.move(20)

                    elif blocks[0] == "g":
                        move.turn(-40, 60)
                        move.turn(40, 60)
                        move.move(20)
                        move.turn(-45, 60)
                        move.turn(-45, 60, forward=-1)
                        move.move(20)

                    else:
                        move.move(80)
                        move.turn(-90, 60)
                        move.move(20)
                    
                # Right turns
                elif d == -1:
                    
                    
# Entering parking space
