import numpy as np

class Vector():
    def __init__(self, name, x0, y0, z0, units):
        self.units = units
        self.name = name
        self.x = x0
        self.y = y0
        self.z = z0

    def __str__(self) -> str:
        return f'{self.name}: [{self.x},{self.y},{self.z}] {self.units}'
    
    def __add__(self, b):
        if isinstance(b, Vector):
            return Vector(
                self.name + b.name,
                self.x + b.x,
                self.y + b.y,
                self.z + b.z,
                self.units,
            )
        


class Dron():
    def __init__(self):
        # self.pos = Vector('pos', 0, 0, 0, units='m')
        # self.vel = Vector('vel', 0, 0, 0, units='m/s')
        # self.acc = Vector('acc', 0, 0, 0, units='m/s²')

        self.pos = np.ndarray([0,0,0])
        self.vel = np.ndarray([0,0,0])
        self.acc = np.ndarray([0,0,0])


def main():
    pass