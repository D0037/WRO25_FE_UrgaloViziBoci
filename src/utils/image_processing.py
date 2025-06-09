import cv2
import numpy as np
import utils.stream as stream
import time
import threading
import copy

# Flags
lines = False
kill = False
new_frame = None

def image_thread():
    print("Image thread started")
    cap = cv2.VideoCapture(0)
    stream.init()
 
    x_min_0 = -500
    y_min_0 = 400
    time_prev = time.time()
    _, frame = cap.read()

    count = 0
    x_avg = 0
    x_min = 2000
    x_max = 0

    y_avg = 0
    y_min = 2000
    y_max = 0

    x_values = []
    y_values = []

    while not kill:
        _, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        orange_line = [None, None, None, None]
        blue_line = [None, None, None, None]

        lower_red = np.array([164, 100, 100])
        upper_red = np.array([179, 255, 255])

        lower_green = np.array([104, 100, 100])
        upper_green = np.array([124, 255, 255])

        lower_blue = np.array([90, 55, 35])
        upper_blue = np.array([170, 255, 255])

        lower_orange = np.array([0, 40, 110])
        upper_orange = np.array([8, 255, 255])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        black_mask = gray < 50

        bottom_black_row = black_mask.shape[0] - np.argmax(np.flipud(black_mask), axis=0) - 1
        for x in range(frame.shape[1]):
            frame[:bottom_black_row[x] + 10, x] = 0
            hsv[:bottom_black_row[x] + 10, x] = 0
        
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)
        blue = np.zeros(blue_mask.shape, dtype=np.uint8)
        orange = np.zeros(orange_mask.shape, dtype=np.uint8)

        blue_cnts: list | None = None
        blue_cnts, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(blue_cnts) > 0:
            filtered_cnt = sorted(blue_cnts, key=cv2.contourArea, reverse=True)
            filtered_cnt = filtered_cnt[:round(50 / (cv2.contourArea(filtered_cnt[0]) + 1)) + 2]
            cv2.drawContours(blue, filtered_cnt, -1, 255, -1)
            print("blue area:", cv2.contourArea(filtered_cnt[0]), "length:", len(filtered_cnt))
        
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
        if len(orange_cnts) > 0:
            filtered_cnt = sorted(orange_cnts, key=cv2.contourArea, reverse=True)
            filtered_cnt = filtered_cnt[:round(100 / (cv2.contourArea(filtered_cnt[0]) + 1)) + 2]
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
                if x_intersect < x_min_0 and y_intersect < y_min_0:
                    pass

                global lines
                lines = True
                print("line!", x_intersect, y_intersect)

                x_min = min(x_min, x_intersect)
                y_min = min(y_min, y_intersect)

                x_max = max(x_max, x_intersect)
                y_max = max(y_max, y_intersect)

                if count != 0:
                    x_avg = (x_avg * count + x_intersect) / (count + 1)
                    y_avg = (y_avg * count + y_intersect) / (count + 1)
                else:
                    x_avg = x_intersect
                    y_avg = y_intersect
                count += 1

                print(f"x max: {x_max}, min: {x_min}, avg: {x_avg}")
                print(f"y max: {y_max}, min: {y_min}, avg: {y_avg}")

                x_values.append(x_intersect) 
                y_values.append(y_intersect) 
                with open("values.txt", '+a') as f:
                    f.write(f"{x_intersect},{y_intersect}\n")

                cv2.circle(frame, (x_intersect, y_intersect), 5, (0, 255, 0), -1)

        stream.show("frame", frame)
        stream.show("orange", orange)
        stream.show("blue", blue)


        print(time.time() - time_prev)
        time_prev = time.time()