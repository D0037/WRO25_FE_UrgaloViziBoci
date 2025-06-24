import cv2
import numpy as np
import utils.stream as stream
import time
import threading
import copy
import math

# Flags
lines = False
kill = False
new_frame = None
blocks = ["n", "n", "n"]
mode = "start_pos" # start_pos, blocks

def tan(x: float) -> float:
    return math.tan(math.pi * x / 180)

def get_blocks():
    return blocks

def set_mode(new_mode: str):
    """Set image processing mode
        @mode: start_pos, blocks
    """
    global mode
    mode = new_mode

def cnt_middle(cnt):
    M = cv2.moments(cnt)

    if M["m00"] != 0:  # Avoid division by zero
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
    else:
        cx, cy = 0, 0  # Or handle differently
    
    return cx, cy
    

positions = {
    "front_inner":  lambda x, y: y > tan(10) * x + 375 and y > tan(10) * -x + 330,

    "front_middle": lambda x, y: y > tan(10) * x + 375 and tan(10) * -x + 250 < y < tan(10) * -x + 330,

    "front_outer":  lambda x, y: y > tan(10) * x + 375 and y < tan(10) * -x + 250,

    "back_inner":   lambda x, y: y < tan(10) * x + 375 and y > tan(8) * -x + 292.69,

    "back_middle":  lambda x, y: y < tan(10) * x + 375 and tan(6) * -x + 270 < y < tan(8) * -x + 292.69,

    "back_outer":   lambda x, y: y < tan(10) * x + 375 and y < tan(6) * -x + 270,
}

block_pos = [
    lambda y: 220 < y, # front
    lambda y: 202 <= y < 220, # middle
    lambda y: y < 202, # back
]

"""positions = <
    "front_middle": lambda x, y: y > 300 and math.sqrt(3) * x + 300 - (math.sqrt(3) / 2) * 2200 < y < math.sqrt(3) * x + 300 - (math.sqrt(3) / 2) * 1760,
    "front_inner":  lambda x, y: y > 300 and y > math.sqrt(3) * x + 300 - (math.sqrt(3) / 2) * 1760,
    "front_outer":  lambda x, y: y > 300 and y < math.sqrt(3) * x + 300 - (math.sqrt(3) / 2) * 2200,
    "back_outer":   lambda x, y: y < 300 and x > 850,
    "back_middle":  lambda x, y: y < 300 and 700 < x < 850,
    "back_inner":   lambda x, y: y < 300 and x < 700,
}"""

start_pos = None
direction = True # True: orange is further, False: blue is further
height, width = 0, 0
obstacle = True

def image_thread():
    print("Image thread started")
    cap = cv2.VideoCapture(0)
    stream.init()
 
    x_min_0 = -500
    y_min_0 = 400
    time_prev = time.time()
    _, frame = cap.read()
    global height, width
    height, width, _ = frame.shape
    with open("values.csv", "w") as f:
        f.write("x,y\n")
    while not kill:
        _, frame = cap.read()
        frame = cv2.GaussianBlur(frame, (5, 5), 1.4)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        orange_line = [None, None, None, None]
        blue_line = [None, None, None, None]

        lower_green = np.array([50, 100, 50])
        upper_green = np.array([80, 255, 255])

        lower_blue = np.array([90, 35, 20])
        upper_blue = np.array([170, 255, 255])

        lower_orange = np.array([0, 50, 100])
        upper_orange = np.array([10, 255, 255])

        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lower_black = np.array([0])
        upper_black = np.array([65])
        black_mask = cv2.inRange(gray, lower_black, upper_black)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 2))
        if mode == "start_pos":
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
        black_mask = cv2.dilate(black_mask, kernel, iterations=2)
        stream.show("black", black_mask)
        
        bottom_black_row = black_mask.shape[0] - np.argmax(np.flipud(black_mask), axis=0) - 1
        for x in range(frame.shape[1]):
            frame[:bottom_black_row[x], x] = 0
            hsv[:bottom_black_row[x], x] = 0

        edges = cv2.Canny(frame, threshold1=100, threshold2=200)
        stream.show("edges", edges)

        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

        blue = np.zeros(blue_mask.shape, dtype=np.uint8)
        orange = np.zeros(orange_mask.shape, dtype=np.uint8)

        if mode == "start_pos":
            blue_cnts: list | None = None
            blue_cnts, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if len(blue_cnts) > 0:
                filtered_cnt = sorted(blue_cnts, key=cv2.contourArea, reverse=True)
                filtered_cnt = filtered_cnt[:round(90 / (cv2.contourArea(filtered_cnt[0]) + 1)) + 2]
                cv2.drawContours(blue, filtered_cnt, -1, 255, -1)
            
                points = np.vstack(filtered_cnt).squeeze()
                if len(points) > 2:
                    [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

                    left_y = int((-x0 * vy / vx) + y0)
                    right_y = int(((blue_mask.shape[1] - x0) * vy / vx) + y0)
                    start = (0, left_y)
                    end = (blue_mask.shape[1]-1, right_y) 

                    blue_line[0], blue_line[1] = start
                    blue_line[2], blue_line[3] = end    

                    cv2.line(frame, start, end, (255, 0, 0), 2)
        
            orange_cnts, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            red_block = np.zeros(orange_mask.shape, dtype=np.uint8)
            if len(orange_cnts) > 0:
                filtered_cnt = sorted(orange_cnts, key=cv2.contourArea, reverse=True)
                filtered_cnt = filtered_cnt[:round(90 / (cv2.contourArea(filtered_cnt[0]) + 1)) + 1]
                cv2.drawContours(orange, filtered_cnt, -1, 255, -1)
                
                points = np.vstack(orange_cnts).squeeze()
                if len(points) > 2:
                    [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

                    left_y = int((-x0 * vy / vx) + y0)
                    right_y = int(((orange_mask.shape[1] - x0) * vy / vx) + y0)
                    start = (0, left_y)
                    end = (orange_mask.shape[1]-1, right_y)

                    orange_line[0], orange_line[1] = start
                    orange_line[2], orange_line[3] = end

                    cv2.line(frame, start, end, (0, 0, 255), 2)

            orange_slope = 0
            blue_slope = 0

            if orange_line[0] is not None and blue_line[0] is not None:
                if orange_line[0] - orange_line[2] != 0:
                    orange_slope = (orange_line[1] - orange_line[3]) / (orange_line[0] - orange_line[2])
                if blue_line[0] - blue_line[2] != 0:
                    blue_slope = (blue_line[1] - blue_line[3]) / (blue_line[0] - blue_line[2])
                
                x_intersect = None
                y_intersect = None
                if (blue_slope - orange_slope) != 0:
                    x_intersect = round((orange_line[1] - blue_line[1]) / (blue_slope - orange_slope))
                    y_intersect = round(blue_slope * x_intersect + blue_line[1])
                
                if x_intersect is not None and y_intersect is not None:
                    global lines
                    lines = True

                    for key in positions.keys():
                        global start_pos, direction
                        direction = abs(orange_slope) > abs(blue_slope)
                        if not direction:
                            x = width - x_intersect
                            if positions[key](x, y_intersect):
                                start_pos = key
                                print(x, y_intersect)
                                print(key, "blue")
                                break
                        if direction and positions[key](x_intersect, y_intersect):
                            start_pos = key
                            print(x_intersect, y_intersect)
                            print(key, "orange")
                            break
                    
                    with open("values.csv", "+a") as f:
                        #print(x_intersect, y_intersect)
                        f.write(f"{x_intersect}, {y_intersect}\n")
                        f.close()

                    cv2.circle(frame, (x_intersect, y_intersect), 5, (0, 255, 0), -1)
            stream.show("blue", blue)
            stream.show("orange", orange)

        if mode == "blocks":
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
            opened = cv2.erode(orange_mask, kernel, iterations=2)

            cnts, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)

            red_blocks = []

            new_blocks = ["n", "n", "n"]
            for cnt in cnts:
                red_blocks.append(cnt_middle(cnt))
                cv2.circle(frame, red_blocks[-1], 5, (0, 0, 255), -1)
                x, y = red_blocks[-1]
                
                for i in range(3):
                    if block_pos[i](y):
                        new_blocks[i] = "r"
                

            cnts, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            stream.show("green", green_mask)
            green_blocks = []
            for cnt in cnts:
                green_blocks.append(cnt_middle(cnt))
                cv2.circle(frame, green_blocks[-1], 5, (0, 255, 0), -1)
                x, y = green_blocks[-1]

                for i in range(3):
                    if block_pos[i](y):
                        new_blocks[i] = "g"
                

            global blocks
            blocks = new_blocks                
            stream.show("open", opened)
                    

        stream.show("frame", frame)
        #stream.show("red", red_block)

        #time_prev = time.time()