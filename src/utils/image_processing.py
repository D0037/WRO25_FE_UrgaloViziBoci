# Import necessary libraries
import cv2
import numpy as np
import utils.stream as stream
import time
import threading
import copy
import math

# Flags and global variables
lines = False       # Indicates if lines (intersecting lines) have been detected
kill = False       # Flag to stop the image processing thread
blocks = ["n", "n", "n"]  # Array to store block positions (e.g., 'r' for red)
mode = "start_pos"  # Mode of processing: 
                        # 'start_pos' for initial position
                        # 'blocks' for block detection
                        # '1st_obstacle' to detect the obstacle from the parking spot
                        # 'parking' to detec heading from parking spot
first_block = "n"   # Stores the first obstacle
in_parking_heading = 1 # Used to store determined heading when in parking spot
in_parking_heading_sum = 0

# Utility functions

def tan(x: float) -> float:
    """Calculate tangent of an angle in degrees."""
    return math.tan(math.pi * x / 180)

def get_blocks():
    """Return current block positions."""
    return blocks

def set_mode(new_mode: str):
    """Set the current image processing mode."""
    global mode
    mode = new_mode

def cnt_middle(cnt):
    """Calculate the centroid of a contour."""
    M = cv2.moments(cnt)
    if M["m00"] != 0:  # To avoid division by zero
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0
    return cx, cy

# Define position boundaries based on x, y coordinates.
# These are used to classify the starting position of the robot.
positions = {
    "front_inner":  lambda x, y: y > tan(10) * x + 375 and y > tan(10) * -x + 360,
    "front_middle": lambda x, y: y > tan(10) * x + 375 and tan(10) * -x + 310 < y < tan(10) * -x + 360,
    "front_outer":  lambda x, y: y > tan(10) * x + 375 and y < tan(10) * -x + 310,
    "back_inner":   lambda x, y: y < tan(10) * x + 375 and y > tan(8) * -x + 285.69,
    "back_middle":  lambda x, y: y < tan(10) * x + 375 and tan(6) * -x + 270 < y < tan(8) * -x + 285.69,
    "back_outer":   lambda x, y: y < tan(10) * x + 375 and y < tan(6) * -x + 270,
}

# Block position thresholds (based on y coordinate)
block_pos = [
    lambda y: 175 < y,          # front
    lambda y: 150 <= y < 175,   # middle
    lambda y: y < 150,           # back
]

# Dictionary to store measured block positions
measured_block_pos = {
    "['n', 'n', 'n']": 0
}

# Global variables
start_pos = None          # Starting position of the robot
direction = True        # Direction flag: True if orange is further, False if blue is further
height, width = 0, 0    # Frame dimensions
obstacle = True         # Obstacle flag (not used in current code)

# === MAIN IMAGE PROCESSING THREAD ===
def image_thread():
    """Main thread function for capturing and processing images from the camera."""
    print("Image thread started")
    cap = cv2.VideoCapture(0)  # Initialize camera
    stream.init()              # Initialize stream (assumed to handle display windows)
 
    # Variables for black line detection
    x_min_0 = -500
    y_min_0 = 400
    time_prev = time.time()
    _, frame = cap.read()
    global height, width
    height, width, _ = frame.shape  # Get frame dimensions

    # Initialize CSV file to log detected points
    with open("values.csv", "w") as f:
        f.write("x,y\n")

    # Main loop
    while not kill:
        _, frame = cap.read()
        # Apply Gaussian blur to reduce noise
        frame = cv2.GaussianBlur(frame, (11, 11), 1.4)
        # Convert frame to HSV color space for color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Initialize variables for detected lines
        orange_line = [None, None, None, None]
        blue_line = [None, None, None, None]

        # Define color ranges in HSV
        lower_green = np.array([50, 100, 50])
        upper_green = np.array([80, 255, 255])

        lower_blue = np.array([90, 20, 20])
        upper_blue = np.array([170, 255, 255])

        lower_orange = np.array([0, 50, 100])
        upper_orange = np.array([10, 255, 255])

        # Convert frame to grayscale for black line detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lower_black = np.array([0])
        upper_black = np.array([50])
        # Create mask for black areas
        black_mask = cv2.inRange(gray, lower_black, upper_black)

        # Define morphological kernel
        if mode == "start_pos":
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
        else:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 2))
        # Dilate black mask to strengthen close gaps
        black_mask = cv2.dilate(black_mask, kernel, iterations=2)
        #stream.show("black", black_mask)  # Display black mask

        # Remove black lines from frame and hsv for better detection
        bottom_black_row = black_mask.shape[0] - np.argmax(np.flipud(black_mask), axis=0) - 1
        #for x in range(frame.shape[1]):
        #    frame[:bottom_black_row[x], x] = 0
        #    hsv[:bottom_black_row[x], x] = 0

        # Edge detection using Canny
        edges = cv2.Canny(frame, threshold1=100, threshold2=200)
        stream.show("edges", edges)

        # Create masks for green, blue, and orange colors
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Initialize empty images for contours
        blue = np.zeros(blue_mask.shape, dtype=np.uint8)
        orange = np.zeros(orange_mask.shape, dtype=np.uint8)

        # === START POSITION DETECTION MODE ===
        if mode == "start_pos":
            # Find contours for blue (assumed to be the starting line)
            blue_cnts: list | None = None
            blue_cnts, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if len(blue_cnts) > 0:
                # Filter contours based on area
                filtered_cnt = sorted(blue_cnts, key=cv2.contourArea, reverse=True)
                filtered_cnt = filtered_cnt[:round(90 / (cv2.contourArea(filtered_cnt[0]) + 1)) + 2]
                # Draw the filtered contours
                cv2.drawContours(blue, filtered_cnt, -1, 255, -1)
            
                # Fit a line to the contour points
                points = np.vstack(filtered_cnt).squeeze()
                if len(points) > 2:
                    [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
                    # Calculate start and end points of the line
                    left_y = int((-x0 * vy / vx) + y0)
                    right_y = int(((blue_mask.shape[1] - x0) * vy / vx) + y0)
                    start = (0, left_y)
                    end = (blue_mask.shape[1] - 1, right_y)

                    # Store line points
                    blue_line[0], blue_line[1] = start
                    blue_line[2], blue_line[3] = end

                    # Draw detected line on frame
                    cv2.line(frame, start, end, (255, 0, 0), 2)

            # Find contours for orange (possible blocks)
            orange_cnts, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            red_block = np.zeros(orange_mask.shape, dtype=np.uint8)
            if len(orange_cnts) > 0:
                # Select the largest orange contours (to avoid small noise)
                filtered_cnt = sorted(orange_cnts, key=cv2.contourArea, reverse=True)
                filtered_cnt = filtered_cnt[:round(90 / (cv2.contourArea(filtered_cnt[0]) + 1)) + 1]
                cv2.drawContours(orange, filtered_cnt, -1, 255, -1)
                
                # Stack all contour points and fit a line
                points = np.vstack(orange_cnts).squeeze()
                if len(points) > 2:
                    [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

                    # Generate two endpoints for the fitted line (to draw it)
                    left_y = int((-x0 * vy / vx) + y0)
                    right_y = int(((orange_mask.shape[1] - x0) * vy / vx) + y0)
                    start = (0, left_y)
                    end = (orange_mask.shape[1]-1, right_y)

                    # Store orange line for intersection calculations later
                    orange_line[0], orange_line[1] = start
                    orange_line[2], orange_line[3] = end

                    cv2.line(frame, start, end, (0, 0, 255), 2)
            
            # --- Compute slopes and intersection of orange & blue lines ---
            orange_slope = 0
            blue_slope = 0

            # Avoid division by zero
            if orange_line[0] is not None and blue_line[0] is not None:
                if orange_line[0] - orange_line[2] != 0:
                    orange_slope = (orange_line[1] - orange_line[3]) / (orange_line[0] - orange_line[2])
                if blue_line[0] - blue_line[2] != 0:
                    blue_slope = (blue_line[1] - blue_line[3]) / (blue_line[0] - blue_line[2])
                
                x_intersect = None
                y_intersect = None
                # Calculate intersection point (x, y) of two lines
                if (blue_slope - orange_slope) != 0:
                    x_intersect = round((orange_line[1] - blue_line[1]) / (blue_slope - orange_slope))
                    y_intersect = round(blue_slope * x_intersect + blue_line[1])
                
                if x_intersect is not None and y_intersect is not None:
                    global lines
                    lines = True

                    # Determine which region the intersection falls into
                    for key in positions.keys():
                        global start_pos, direction
                        direction = abs(orange_slope) > abs(blue_slope) # Determine which line is "further"

                        if not direction: # Blue line is further (reverse x-axis)
                            x = width - x_intersect
                            if positions[key](x, y_intersect):
                                start_pos = key
                                print(x, y_intersect)
                                print(key, "blue")
                                break
                        if direction and positions[key](x_intersect, y_intersect): # Orange is further
                            start_pos = key
                            print(x_intersect, y_intersect)
                            print(key, "orange")
                            break
                    
                    # Save data to CSV
                    with open("values.csv", "+a") as f:
                        #print(x_intersect, y_intersect)
                        f.write(f"{x_intersect}, {y_intersect}\n")
                        f.close()

                    # Visual feedback: mark intersection with green dot
                    cv2.circle(frame, (x_intersect, y_intersect), 5, (0, 255, 0), -1)
            
            # Show debug masks
            stream.show("blue", blue)
            stream.show("orange", orange)

        # === BLOCK DETECTION (Second Round) ===
        elif mode == "blocks":
            # Preprocessing: erode the orange mask to remove noise
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 10))
            opened = cv2.erode(orange_mask, kernel, iterations=2)

            # Find contours of orange blocks (red)
            cnts, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)

            red_blocks = []
            new_blocks = ["n", "n", "n"] # Reset block stat

            global blocks
            
            # Classify red blocks into front, middle, back
            for cnt in cnts:
                red_blocks.append(cnt_middle(cnt))
                cv2.circle(frame, red_blocks[-1], 5, (0, 0, 255), -1)
                x, y = red_blocks[-1]
                
                for i in range(3):
                    if block_pos[i](y) and y != 0:
                        new_blocks[i] = "r"
                
            # Detect green blocks
            cnts, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            stream.show("green", green_mask)
            green_blocks = []
            for cnt in cnts:
                green_blocks.append(cnt_middle(cnt))
                cv2.circle(frame, green_blocks[-1], 5, (0, 255, 0), -1)
                x, y = green_blocks[-1]

                for i in range(3):
                    if block_pos[i](y) and y != 0:
                        new_blocks[i] = "g"
            
            # Inference fix: if exactly one block is unknown, infer from neighbors
            if new_blocks.count("n") == 1:
                if blocks[0] == "n":
                    new_blocks[0] = new_blocks[1]
                    new_blocks[1] = "n"
                elif new_blocks[2] == "n":
                    new_blocks[2] = new_blocks[1]
                    new_blocks[1] = "n"
            
            blocks = new_blocks

            # Display final cleaned mask
            stream.show("open", opened)
        
        # Determine the first obstacles from the parking spot
        elif mode == "1st_obstacle":

            # Find blue line
            blue_line = [0, 0, 0, 0]
            blue_cnts: list | None = None
            blue_cnts, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if len(blue_cnts) > 0:

                # Filter noise
                filtered_cnt = sorted(blue_cnts, key=cv2.contourArea, reverse=True)
                filtered_cnt = filtered_cnt[:round(90 / (cv2.contourArea(filtered_cnt[0]) + 1)) + 2]
                cv2.drawContours(blue, filtered_cnt, -1, 255, -1)

                # Fit line
                points = np.vstack(filtered_cnt).squeeze()
                if len(points) > 2:
                    [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

                    # Extend lines to the edges of frame
                    left_y = int((-x0 * vy / vx) + y0)
                    right_y = int(((blue_mask.shape[1] - x0) * vy / vx) + y0)
                    start = (0, left_y)
                    end = (blue_mask.shape[1]-1, right_y) 

                    blue_line[0], blue_line[1] = start
                    blue_line[2], blue_line[3] = end    

                    cv2.line(frame, start, end, (255, 0, 0), 2)

            # Determine y coordinate of line from x
            def line(x):
                start_x, start_y, end_x, end_y = blue_line[0], blue_line[1], blue_line[2], blue_line[3]
                slope = -(start_y - end_y) / 640
                return slope * x + start_y
            
            # Scan through coloumns and set pixels above the blue line to black
            for x in range(640):
                y = min(max(round(line(x)), 0), 480)
                print(x, y)
                frame[:y, x] = 0
                hsv[:y, x] = 0
            
            # Remove noise and interfering lines
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 15))
            orange_mask = cv2.erode(orange_mask, kernel, iterations=2)
            
            # Find greatest area
            cnts, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
            red_area = 0
            if len(cnts) > 0:
                red_area = cv2.contourArea(cnts[0])
                cv2.drawContours(frame, cnts, 0, (0, 0, 255), -1)

            # Remove noise
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 6))
            green_mask = cv2.erode(green_mask, kernel, iterations=2)

            # Find greatest area
            cnts, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
            green_area = 0
            if len(cnts) > 0:
                green_area = cv2.contourArea(cnts[0])
            
            # Compare areas and determine first obstacle
            global first_block
            if green_area > red_area:
                first_block = "g"
            elif green_area < red_area:
                first_block = "r"
            else:
                first_block = "n"


        # Determine orientation from the parking spot
        elif mode == "parking":
            # Define magenta
            lower_magenta = np.array([0, 10, 10])
            upper_magenta = np.array([15, 255, 255])
            
            # Create mask
            magenta_mask = cv2.inRange(hsv, lower_magenta, upper_magenta)

            # Perform morphological close
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            magenta_mask = cv2.morphologyEx(magenta_mask, cv2.MORPH_CLOSE, kernel, iterations=3)


            # Find contours
            cnt, _ = cv2.findContours(magenta_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnt = sorted(cnt, key=cv2.contourArea, reverse=True)

            # Skip if no magenta can be seen
            if len(cnt) == 0:
                continue

            # Approximate contours to reduce noise
            approx = cv2.approxPolyDP(cnt[0], 10, True)

            # Fit rectangle
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), -1)
            
            black_mask[:y + 50, :] = 0

            cnts, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)

            black_mask[:, :] = 0
            cv2.drawContours(black_mask, cnts, 0, (255), -1)
            
            if len(cnts) > 0:
                x_middle, y_middle = cnt_middle(cnts[0])
                cv2.circle(frame, (x_middle, y_middle), 5, (0, 0, 0), -1)
                global in_parking_heading, in_parking_heading_sum
                if x_middle < width / 2:
                    in_parking_heading_sum += 1
                else:
                    in_parking_heading_sum -= 1
            
                in_parking_heading = 1 if in_parking_heading_sum > 0 else -1

            stream.show("black", black_mask)


            # Determine heading

        stream.show("frame", frame)