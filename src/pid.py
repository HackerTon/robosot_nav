import numpy as numpy

class PID:
    def __init__(self,kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
