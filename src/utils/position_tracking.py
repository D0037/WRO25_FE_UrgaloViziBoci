import math
import struct
import threading

conversion_constant = 60

class PositionTracker:
    def __init__(self):
        # Initialize variables
        self.x = 0.0
        self.y = 0.0

        self.kill_switch = False

        # Start tracking thread
        self.thread = threading.Thread(target=self.position_tracker)
        self.thread.start()

    def position_tracker(self):
        with open("/dev/input/mice", "rb") as mouse:
            while not self.kill_switch:
                data = mouse.read(3)
                _, dx, dy = struct.unpack("bbb", data)
                self.x += dx / conversion_constant
                self.y += dy / conversion_constant

    # Updates the current position based on change in rotation and movement detected by the mouse sensor
    def update(self, angle_deg, dx, dy):
        # Convert degreees the radian
        angle_rad = math.radians(angle_deg)
        
        # Calculate change in position
        x_d = dx * math.cos(angle_rad) - dy * math.sin(angle_rad)
        y_d = dx * math.sin(angle_rad) + dy * math.cos(angle_rad)

        # Update current postion
        self.x += x_d
        self.y += y_d
    
    def get(self):
        return self.x, self.y

    def reset(self):
        self.x = 0
        self.y = 0
        #TODO: reset gyro?
    
    def kill(self):
        self.kill_switch = True