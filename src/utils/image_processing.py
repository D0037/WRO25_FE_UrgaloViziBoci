# Import necessary libraries
import cv2
import numpy as np
import utils.stream as stream
from picamera2 import Picamera2
import time
import math

# Flags and global variables
lines = False           # Indicates if lines (intersecting lines) have been detected
kill = False            # Flag to stop the image processing thread
blocks = ["n", "n", "n"]# Array to store block positions (e.g., 'r' for red)
mode = "none"           # Mode of processing: 
                        # 'heading' true if orange line is further
                        # 'start_pos' for initial position
                        # 'blocks' for block detection
                        # '1st_obstacle' to detect the obstacle from the parking spot
                        # 'parking' to detec heading from parking spot
first_block = "n"       # Stores the first obstacle

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
direction = True        # Direction flag: True if orange is further, False if blue is further
black_count = 0        # Count of black pixels in the middle row
height, width = 0, 0    # Frame dimensions
blue_visible = False
orange_visible = False
x_intersect = 0
y_intersect = 0

# Define color ranges in HSV
lower_green = np.array([40, 0, 20])
upper_green = np.array([90, 255, 255])

lower_blue = np.array([100, 70, 70])
upper_blue = np.array([140, 255, 255])

lower_orange = np.array([0, 20, 50])
upper_orange = np.array([10, 255, 255])

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

def area_filter(mask, size_coefficient, min_num):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(cnts) < 1:
        return None, None
    
    # Filter contours based on size keep the biggest n
    # where n is dependent of the largest countour
    # This is useful to remove noise, but if barely anything can be seen, that won't be filtered
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
    cnts = cnts[:round(size_coefficient / max(cv2.contourArea(cnts[0]), 1)) + min_num]
    
    # set every pixel to black
    mask[:, :] = 0

    # Draw the filtered contours on the mask
    cv2.drawContours(mask, cnts, -1, 255, -1)
    return mask, cnts


# === MAIN IMAGE PROCESSING THREAD (it must be started as a new thread)===
def image_thread():
    """Main thread function for capturing and processing images from the camera."""
    print("Image thread started")


    # Initialize camera
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        main={"format": "RGB888", "size": (1536, 864)} # Lowest resolution, highest FPS
    ))
    picam2.set_controls({"AfMode": 2}) # Set autofocus mode to continuous
    picam2.start()
    
    stream.init()              # Initialize stream (assumed to handle display windows)
 
    frame = picam2.capture_array()

    global height, width
    height, width, _ = frame.shape  # Get frame dimensions

    # Main loop
    while not kill:
        frame = picam2.capture_array()

        # Used to disable processing
        if mode == "none":
            time.sleep(0.02)  # Sleep to avoid busy-waiting
            continue

        # Apply Gaussian blur to reduce noise
        #frame = cv2.GaussianBlur(frame, (11, 11), 1.4) # idk if this help

        # Convert frame to HSV color space for color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Convert frame to grayscale for black line detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lower_black = np.array([0])
        upper_black = np.array([90])

        # Mask area abovo black walls
        black_mask = cv2.inRange(gray, lower_black, upper_black)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 15)) # Vertical kernel
        black_mask = cv2.dilate(black_mask, kernel, iterations=3) # Dilate to fill gaps and remove noise
        bottom_black_row = black_mask.shape[0] - np.argmax(np.flipud(black_mask), axis=0) - 1 # Find the bottom row of black pixels in each column

        # Set pixels above the bottom black row to black
        for x in range(black_mask.shape[1]):
            frame[:bottom_black_row[x], x] = 0
            hsv[:bottom_black_row[x], x] = 0
            black_mask[:bottom_black_row[x], x] = 255
        
        # Count black pixels in the middle row
        # This is used to correctly determine between some tricky spots, that the lidar is not enough for
        global black_count
        black_count = np.count_nonzero(black_mask[int(black_mask.shape[0] / 2), :])

        stream.show("black", black_mask)  # Display black mask over network (for debugging)

        # Create masks for green, blue, and orange colors
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Initialize empty images for contours
        blue = np.zeros(blue_mask.shape, dtype=np.uint8)
        orange = np.zeros(orange_mask.shape, dtype=np.uint8)

        # === START POSITION DETECTION MODE ===
        if mode == "heading":
            # Find contours for blue (assumed to be the starting line)
            blue, blue_cnts = area_filter(blue_mask, 70, 1)

            # for debugging only
            stream.show("orange_mask", orange_mask)
            stream.show("blue_mask", blue_mask)
            stream.show("blue", blue)

            orange_slope = None
            blue_slope = None

            # format: start_x, start_y, end_x, end_y
            blue_line = [None, None, None, None]
            orange_line = [None, None, None, None]

            if blue_cnts is not None:
                # Fit a line to the contour points
                points = np.vstack(blue_cnts).squeeze()
                if len(points) > 2:
                    [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

                    blue_slope = vy / vx # Calculate slope from vector

                    # Extend the line to the edges of the frame
                    left_y = int((-x0 * vy / vx) + y0)
                    right_y = int(((blue_mask.shape[1] - x0) * vy / vx) + y0)
                    start = (0, left_y)
                    end = (int(blue_mask.shape[1] - 1), right_y)

                    # Store the line endpoints
                    blue_line[0], blue_line[1] = start
                    blue_line[2], blue_line[3] = end

                    # Draw detected line on frame
                    cv2.line(frame, start, end, (255, 0, 0), 2)
                    global blue_visible
                    blue_visible = True

            # Find contours for orange (possible blocks)
            orange_mask, orange_cnts = area_filter(orange_mask, 90, 4)
            if orange_cnts is not None:
                # Stack all contour points and fit a line
                points = np.vstack(orange_cnts).squeeze()
                if len(points) > 2:
                    [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

                    orange_slope = vy / vx # Calculate slope from vector

                    # Extend the line to the edges of the frame
                    left_y = int((-x0 * vy / vx) + y0)
                    right_y = int(((orange_mask.shape[1] - x0) * vy / vx) + y0)
                    start = (0, left_y)
                    end = (int(orange_mask.shape[1]-1), right_y)

                    # Store the line endpoints
                    orange_line[0], orange_line[1] = start
                    orange_line[2], orange_line[3] = end

                    # Draw detected line on frame for debugging
                    cv2.line(frame, start, end, (0, 0, 255), 2)
                    global orange_visible
                    orange_visible = True
            
            # Find heading by the difference of slopes (also calculate intersection point  ((who knows, maybe we will need it lol)))
            if orange_slope != None and blue_slope != None:
                global direction, x_intersect, y_intersect
                direction = orange_slope < 0 # True if orange is further
                #print(orange_slope, blue_slope)
                
                # Intersection point calculation
                x = round((orange_line[1] - blue_line[1]) / (blue_slope[0] - orange_slope[0]))
                y = round(blue_slope[0] * x + blue_line[1])

                x_intersect = x
                y_intersect = y

                # Draw intersection point on the frame (for debugging)
                cv2.circle(frame, (x, y), 6, (0, 255, 0), -1)
                #print(f"intersect: {x}, {y}")

            # Show detected colors on the stream
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

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
            green_mask

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
                #print(x, y)
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
            stream.show("green", green_mask)

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
            lower_magenta = np.array([150, 10, 10])
            upper_magenta = np.array([179, 255, 255])
            
            # Create mask
            magenta_mask = cv2.inRange(hsv, lower_magenta, upper_magenta)
            stream.show("magenta", magenta_mask)

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

        stream.show("frame", frame)