class PID:
    def __init__(self, p: float, i: float, d: float):
        self.p = p
        self.i = i
        self.d = d
        self.sum = 0
        self.prev = 0
    
    def update(self, error):
        self.sum += error
        d = (error - self.prev) * self.d
        self.prev = error
        correction = (error * self.p) + (self.sum * self.i) + d
        self.prev = error
        return correction, error * self.p, self.sum * self.i, d