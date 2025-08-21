import time

# PID algorithm for the turning 
class PID:
    def __init__(self, p: float, i: float, d: float):
        self.p = p
        self.i = i
        self.d = d
        self.sum = 0
        self.prev = 0
        self.prev_time = time.time()
    
    def update(self, error):
        p = error * self.p

        self.sum += error
        i = self.sum * self.i

        d = (self.prev - error) / (self.prev_time - time.time()) * self.d
        self.prev_time = time.time()
        self.prev = error

        correction = p + i + d
        return correction, p, i, d