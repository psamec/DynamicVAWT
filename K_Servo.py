import math

class Servo() :
    def __init__(self,
                 speed,      # Given in Degrees per second
                 torque=0):  # Given in Nm, defauls to zero as mos of the wing loads have yet to be defined
        self.speed = speed * math.pi / 180
        self.torque = torque
        self.current_position = 0

    def SetNewPosition(self, target, dt):
        max_step = dt*self.speed
        ask_step = target - self.current_position
        fabs_step = math.fabs(ask_step)
        signage = ask_step/fabs_step

        # Compute the maximum step increment
        if fabs_step > max_step :
            step =  max_step * signage
        else :
            step = ask_step

        self.current_position += step
        return step


