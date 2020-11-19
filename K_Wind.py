import math
from K_Geometry import K_2DPointVelocity

class Wind() :
    def __init__(self, wind_velocity, angle = 180 ):
        a = angle * math.pi / 180.
        vx = wind_velocity * math.cos(a)
        vy = wind_velocity * math.sin(a)
        self.wind = K_2DPointVelocity(0, [vx], [vy], [-1])
