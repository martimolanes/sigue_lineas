import numpy as np

class Car:
    def __init__(self, position=(0.0, 0.0), direction=0.0, velocity=1.0):
        self.position = position
        self.direction = direction
        self.velocity = velocity

    def update(self):
        x = self.position[0] + np.cos(self.direction) * self.velocity
        y = self.position[1] + np.sin(self.direction) * self.velocity

        self.position = x, y