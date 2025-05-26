import cv2
import numpy as np
import stream

cap = cv2.VideoCapture(0)
stream.init()

x_min_0 = -500
y_min_0 = 400

while True:
    ret, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    orange_line = [0, 0, 0, 0]
    blue_line = [0, 0, 0, 0]


    lower_blue = np.array([90, 50, 30])
    upper_blue = np.array([170, 255, 255])

    lower_orange = np.array([0, 90, 110])
    upper_orange = np.array([15, 255, 255])

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

    blue_cnts, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(blue_cnts) > 0:

        points = np.vstack(blue_cnts).squeeze()
        if len(points) > 2:
            [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

            left_y = int((-x0 * vy / vx) + y0)
            right_y = int(((blue_mask.shape[1] - x0) * vy / vx) + y0)
            start = (0, left_y)
            end = (blue_mask.shape[1]-1, right_y)

            blue_line[0], blue_line[1] = start
            blue_line[2], blue_line[3] = end

            cv2.circle(frame, start, 5, (255, 0, 0), -1)
            cv2.circle(frame, end, 5, (255, 0, 0), -1)
            cv2.line(frame, start, end, (255, 0, 0), 2)

    orange_cnts, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    new_cnts = []
    for cnt in orange_cnts:
        area = cv2.contourArea(cnt)
        if area > 100:
            new_cnts.append(cnt)
    orange_cnts = new_cnts
    if len(orange_cnts) > 0:
        cv2.drawContours(frame, orange_cnts, -1, (0, 255, 0), 2)
        points = np.vstack(orange_cnts).squeeze()
        if len(points) > 2:
            [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

            left_y = int((-x0 * vy / vx) + y0)
            right_y = int(((orange_mask.shape[1] - x0) * vy / vx) + y0)
            start = (0, left_y)
            end = (orange_mask.shape[1]-1, right_y)

            orange_line[0], orange_line[1] = start
            orange_line[2], orange_line[3] = end

            cv2.circle(frame, start, 5, (0, 0, 255), -1)
            cv2.circle(frame, end, 5, (0, 0, 255), -1)
            cv2.line(frame, start, end, (0, 0, 255), 2)

    orange_slope = 0
    blue_slope = 0

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
            print(0)

        print(x_intersect, y_intersect)
        cv2.circle(frame, (x_intersect, y_intersect), 5, (0, 255, 0), -1)

    """cv2.imshow("blue mask", blue_mask)
    cv2.imshow("orange mask", orange_mask)
    cv2.imshow("idk", frame)"""

    stream.show("frame", frame)
    stream.show("orange", orange_mask)

    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break