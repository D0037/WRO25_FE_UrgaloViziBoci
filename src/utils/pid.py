class PID:
    def __init__(self, optimal: float, p: float, i: float, d: float):
        self.optimal = optimal
        self.p = p
        self.i = i
        self.d = d
        self.sum = 0
        self.prev = 0
    
    def update(self, value):
        error = self.optimal - value
        self.sum += error
        correction = error * self.p + self.sum * self.i + (error - self.prev) * self.d
        #print(self.sum * self.i, (error - self.prev) * self.d, error * self.p)
        d = (error - self.prev) * self.d
        self.prev = error
        return correction, error * self.p, self.sum * self.i, d
