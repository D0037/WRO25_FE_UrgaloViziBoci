import time
import smbus2
import threading

# registers
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
GYRO_XOUT_H = 0x43
ACCEL_XOUT_H = 0x3B

def read_i16(bus, addr, reg_high):
    # read 2 bytes
    high = bus.read_byte_data(addr, reg_high)
    low = bus.read_byte_data(addr, reg_high + 1)

    # combine
    value = (high << 8) | low

    # unsigned to signed
    if value & 0x8000:
        value -= 0x10000

    return value

class Gyro:
    # fs_sel: 0: 250, 131; 1: 500, 65.5; 2: 1000, 32.8; 3: 2000, 16.4
    def __init__(self, addr = 0x68, fs_sel = 0, afs_sel = 0):
        self.addr = addr

        # calculate scal factors
        self.gyro_scale_factor = 131.0 / (fs_sel + 1)
        self.accel_scal_factor = 16384 / (2 ** afs_sel) 

        # used to kill the gyro process
        self.kill_switch = False

        # used to store current rotation
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # to prevent drift
        self.offset = {"x": 0.0, "y": 0.0, "z": 0.0}

        # i2c bus
        self.bus = smbus2.SMBus(1)

        # set scale factor
        current = self.bus.read_byte_data(self.addr, GYRO_CONFIG)
        new_cfg = (current & ~0x18) | (fs_sel << 3)
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, new_cfg)

        current = self.bus.read_byte_data(self.addr, ACCEL_CONFIG)
        new_cfg = (current & ~0x18) | (afs_sel << 3)
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, new_cfg)
        
        # calibrate to avoid drift
        print("Calibrating...")
        self.offset = self.calibrate_gyro()
        print("Done!")

        # spawn gyro process
        self.prev_time = time.time()
        self.gyro_thread = threading.Thread(target=self.gyro_process)
        self.gyro_thread.start()
    
    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def get(self):
        return {"x": self.x, "y": self.y, "z": self.z}
    
    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_z(self):
        return self.z

    def gyro_process(self):
        while not self.kill_switch:
            self.time_elapsed = time.time() - self.prev_time
            self.prev_time = time.time()

            data = self.read_gyro()

            # correct values
            x_speed = data["x"] - self.offset["x"]
            y_speed = data["y"] - self.offset["y"]
            z_speed = data["z"] - self.offset["z"]

            # update current rotation
            self.x += x_speed * self.time_elapsed
            self.y += y_speed * self.time_elapsed
            self.z += z_speed * self.time_elapsed

    def calibrate_gyro(self, samples=1000):
        total = {'x': 0, 'y': 0, 'z': 0}

        for _ in range(samples):
            data = self.read_gyro()
            for axis in ['x', 'y', 'z']:
                total[axis] += data[axis]
            time.sleep(0.005)  # ~200 Hz

        offset = {axis: total[axis] / samples for axis in total}
        return offset
    
    def kill(self):
        self.kill_switch = True
    
    
    def read_gyro(self):
        return {
            "x": read_i16(self.bus, self.addr, GYRO_XOUT_H) / self.gyro_scale_factor,
            "y": read_i16(self.bus, self.addr, GYRO_XOUT_H + 2) / self.gyro_scale_factor,
            "z": read_i16(self.bus, self.addr, GYRO_XOUT_H + 4) / self.gyro_scale_factor
        }
    
    def read_accel(self):
        return {
            "x": read_i16(self.bus, self.addr, ACCEL_XOUT_H) / self.accel_scale_factor,
            "y": read_i16(self.bus, self.addr, ACCEL_XOUT_H + 2) / self.accel_scale_factor,
            "z": read_i16(self.bus, self.addr, ACCEL_XOUT_H + 4) / self.accel_scale_factor
        }
    
    def get_ax(self):
        return read_i16(self.bus, self.addr, ACCEL_XOUT_H) / self.accel_scale_factor

    def get_ay(self):
        return read_i16(self.bus, self.addr, ACCEL_XOUT_H + 2) / self.accel_scale_factor

    def get_az(self):
        return read_i16(self.bus, self.addr, ACCEL_XOUT_H + 4) / self.accel_scale_factor