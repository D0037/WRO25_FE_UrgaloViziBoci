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

def tan(x: float) -> float:
    return math.tan(math.pi * x / 180)

positions = {
    "front_inner":  lambda x, y: y > tan(10) * x + 375 and y > tan(10) * -x + 330,

    "front_middle": lambda x, y: y > tan(10) * x + 375 and tan(10) * -x + 250 < y < tan(10) * -x + 330,

    "front_outer":  lambda x, y: y > tan(10) * x + 375 and y < tan(10) * -x + 250,

    "back_inner":   lambda x, y: y < tan(10) * x + 375 and y > tan(8) * -x + 292.69,

    "back_middle":  lambda x, y: y < tan(10) * x + 375 and tan(6) * -x + 270 < y < tan(8) * -x + 292.69,

    "back_outer":   lambda x, y: y < tan(10) * x + 375 and y < tan(6) * -x + 270,
}

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
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        orange_line = [None, None, None, None]
        blue_line = [None, None, None, None]

        lower_green = np.array([50, 50, 50])
        upper_green = np.array([80, 255, 255])

        lower_blue = np.array([90, 35, 20])
        upper_blue = np.array([170, 255, 255])

        lower_orange = np.array([0, 50, 100])
        upper_orange = np.array([10, 255, 255])

        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        frame = cv2.GaussianBlur(frame, (5, 5), 1.4)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        black_mask = gray < 65
        
        bottom_black_row = black_mask.shape[0] - np.argmax(np.flipud(black_mask), axis=0) - 1
        for x in range(frame.shape[1]):
            frame[:bottom_black_row[x] + 10, x] = 0
            hsv[:bottom_black_row[x] + 10, x] = 0

        edges = cv2.Canny(frame, threshold1=100, threshold2=200)
        stream.show("edges", edges)

        
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)
        blue = np.zeros(blue_mask.shape, dtype=np.uint8)
        orange = np.zeros(orange_mask.shape, dtype=np.uint8)

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

        if obstacle:
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            opened = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel, iterations=5)

            stream.show("open", opened)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            dilated = cv2.dilate(opened, kernel, iterations=1)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            mask = cv2.bitwise_not(dilated)
            orange_mask = cv2.bitwise_and(orange_mask, mask)
            cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
            #cnts, _ = cv2.findContours(orange_mask)
            #orange_mask = cv2.erode(orange_mask, kernel, iterations=1)
        
        orange_cnts, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        red_block = np.zeros(orange_mask.shape, dtype=np.uint8)
        if len(orange_cnts) > 0:
            filtered_cnt = sorted(orange_cnts, key=cv2.contourArea, reverse=True)
            filtered_cnt = filtered_cnt[:round(90 / (cv2.contourArea(filtered_cnt[0]) + 1)) + 1]
            cv2.drawContours(orange, filtered_cnt, -1, 255, -1)

            """for cnt in filtered_cnt:
                approx = cv2.approxPolyDP(cnt, epsilon=5, closed=True)
                filtered_cnt = [approx]
                cv2.drawContours(smooth, [approx], -1, 255, -1)

                stream.show("smooth", smooth)"""

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

        stream.show("frame", frame)
        stream.show("orange", orange)
        #stream.show("red", red_block)
        stream.show("blue", blue)

        #time_prev = time.time()