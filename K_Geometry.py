#----------------------------------------------------------------------------------------------------------------------
# Kalya Energy Basic geometry and kinematics classes
#----------------------------------------------------------------------------------------------------------------------

import math
import numpy as np
from scipy.optimize import curve_fit
from scipy.interpolate import InterpolatedUnivariateSpline

def exponential_fit(x, a, b, c):
    return a * np.exp(-b * x) + c


def K_angle(x, y):  # compute polar angle

    if x == 0:
        if y > 0: return math.pi / 2
        return -math.pi / 2

    xx = math.atan(y / x)

    if x < 0:
        if y > 0: return -xx + math.pi / 2
        return -xx - math.pi / 2
    return xx


def K_rotate_xy (xy, a) : # returns the new x y cordinates once rotated....
    return [math.cos(a)*xy[0] - math.sin(a) * xy[1], math.sin(a) * xy[0] + math.cos(a) * xy[1]]

def K_xy2ra (x,y) :
    return K_angle(x,y), math.sqrt(x*x+y*y)

def K_ra2xy (r,a) :
    return math.cos(a) * r, math.sin(a)  * r


class K_2DPointVelocity:
    def __init__(self, point, vx, vy, t):
        # assume that input is an array of coordinates
        self.Vxn = [vx[i] for i in range(0, vx.__len__())]
        self.Vyn = [vy[i] for i in range(0, vy.__len__())]
        self.Tn  = [t[i] for i in range(0, t.__len__())]
        self.Point = point

class K_2DPointAcceleration:
    def __init__(self, point, ax, ay, t):
        # assume that input is an array of coordinates
        self.Axn = [ax[i] for i in range(0, ax.__len__())]
        self.Ayn = [ay[i] for i in range(0, ay.__len__())]
        self.Tn  = [t[i] for i in range(0, t.__len__())]
        self.Point = point

    def Ax(self, t=-1):
        return self.Axn[t]

    def Ay(self, t=-1):
        return self.Ayn[t]

    def At(self, t=-1):
        return self.Tn[t]


class K_2DPoint:
    def __init__(self, frame, x, y, t) :
        # assume that input is an array of coordinates
        self.Xn = [x[i] for i in range(0, x.__len__())]
        self.Yn = [y[i] for i in range(0, y.__len__())]
        self.Tn = [t[i] for i in range(0, t.__len__())]

        self.Frame = frame

    def duplicate(self):
        return K_2DPoint(self.Frame, self.Xn, self.Yn, self.Tn)

    def X(self, t=-1):  # returns the current X location
        n = len(self.Tn)
        if n  == 1 : x = self.Xn[0]    # the object has no history
        else : x = self.Xn[t]          # the object has a history
        return x

    def Y(self, t=-1): # returns the current Y location
        n = len(self.Tn)
        if n  == 1 : y = self.Yn[0]    # the object has no history
        else : y = self.Yn[t]          # the object has a history
        return y

    def T(self, t=-1): # returns the current Y location
        n = len(self.Tn)
        if n  == 1 : t = self.Tn[0]    # the object has no history
        else : t = self.Tn[t]          # the object has a history
        return t

    def SetX(self, x, t=-1):  # returns the current X location
        n = len(self.Xn)
        if n  == 1 : self.Xn[0]  = x   # the object has no history
        else : self.Xn[t] = x          # the object has a history

    def SetY(self, y, t=-1):  # returns the current X location
        n = len(self.Yn)
        if n == 1: self.Yn[0] = y  # the object has no history
        else: self.Yn[t] = y       # the object has a history

    def SetT(self, t=-1):  # returns the current X location
        n = len(self.Yn)
        if n == 1: self.Tn[0] = t  # the object has no history
        else: self.Tn[t] = t      # the object has a history

    def SetXY(self, xy, frame, t=-1):
        self.SetX(xy[0], t)
        self.SetY(xy[1], t)
        self.Frame = frame

    def AddXYTimeStep(self, t):
        self.Xn.append(self.Xn[-1])
        self.Yn.append(self.Yn[-1])
        dt = self.Tn[-1] - self.Tn[-2]
        self.Tn.append(self.Tn[-1] + dt)


    def XY(self, t=-1):
        return [self.X(t), self.Y(t)]

    def Vx(self, t=-1): # returns the horizontal velocity
        return (self.Xn[t]-self.Xn[t-1])/(self.Tn[t] - self.Tn[t-1])

    def Vy(self, t=-1): # returns the horizontal velocity
        return (self.Yn[t]-self.Yn[t-1])/(self.Tn[t] - self.Tn[t-1])

    def V(self, t=-1):
        return K_2DPointVelocity(self, [self.Vx(t)], [self.Vy(t)], [self.Tn[t]])

    def X_Interpolate(self, t=-1):

        time_sample = np.array([self.Tn[t - 2], self.Tn[t - 1], self.Tn[t]])
        x    = np.array([self.Xn[t - 2], self.Xn[t - 1], self.Xn[t]])
        dt = (self.Tn[t] - self.Tn[t - 1])
        s = InterpolatedUnivariateSpline(time_sample, x, k=2)
        next_t = self.Tn[t] + dt
        next_y = s(next_t)
        return next_t, next_y

        #time = np.array([self.Tn[t - 2], self.Tn[t - 1], self.Tn[t]])
        #x      = np.array([self.Xn[t - 2], self.Xn[t - 1], self.Xn[t]])
        #dt = (self.Tn[t] - self.Tn[t - 1])
        #fitting_parameters, covariance = curve_fit(exponential_fit, time, x)
        #a, b, c = fitting_parameters
        #next_t = self.Tn[t] + dt
        #next_x = exponential_fit(next_t, a, b, c)
        #return next_t, next_x

    def Y_Interpolate(self, t=-1):

        time_sample = np.array([self.Tn[t - 2], self.Tn[t - 1], self.Tn[t]])
        y    = np.array([self.Yn[t - 2], self.Yn[t - 1], self.Yn[t]])
        dt = (self.Tn[t] - self.Tn[t - 1])
        s = InterpolatedUnivariateSpline(time_sample, y, k=2)
        next_t = self.Tn[t] + dt
        next_y = s(next_t)
        return next_t, next_y

    def A(self, t=-1):
        next_t, next_x = self.X_Interpolate(t)
        next_t, next_y = self.Y_Interpolate(t)

        dt = (self.Tn[t] - self.Tn[t - 1])
        ax = (next_x - 2 * self.Xn[t] + self.Xn[t - 1])/ (dt * dt)
        ay = (next_y - 2 * self.Yn[t] + self.Yn[t - 1]) / (dt * dt)
        return K_2DPointAcceleration(self, [ax], [ay], [self.Tn[t]])


    def TimeSample(self, n_samples, end_t=-1): # number of samples, and end time of the sequence
        # Returns a new point with just a sample of the time history
        x=[]
        y=[]
        t=[]
        o_index = end_t-n_samples

        if end_t <0 : o_index = self.Xn.__len__()+end_t-1

        for i in range(0, n_samples):
            x.append(self.Xn[o_index+i])
            y.append(self.Yn[o_index+i])
            t.append(self.Tn[o_index+i])

        return K_2DPoint(self.Frame, x, y, t)

    def XYInParentCoordinates(self, t=-1):
        nxy = self.XY(t)             # This assumes that we are plotting a rigid solid where the internal coordinates have no history
        a = self.Frame.Oa(t)
        nxy = K_rotate_xy (nxy, a)
        nxy[0] += self.Frame.Ox(t)
        nxy[1] += self.Frame.Oy(t)
        return nxy

    def PointInAncestorCoordinates(self, end_frame, t=-1):

        new_point = self.duplicate()

        for i in range(0, len(new_point.Tn)): # ITERATE OVER THE ENTIRE TIME SERIES
            current_frame = self.Frame
            new_point.Frame = self.Frame
            while current_frame.GetDescription() != end_frame.GetDescription():
                xy = new_point.XYInParentCoordinates(t-i)
                new_point.SetXY(xy, current_frame.Frame, t-i)
                current_frame = current_frame.Frame
            new_point.Frame = end_frame

        #current_frame = self.Frame
        #new_point.Frame = self.Frame
        #while current_frame.GetDescription() != end_frame.GetDescription():
        #    xy = new_point.XYInParentCoordinates()
        #    new_point.SetXY(xy, current_frame.Frame)
        #    current_frame = current_frame.Frame
        #new_point.Frame = end_frame
        return new_point

class K_2DAngle:
    def __init__(self, frame, a, t) :
        self.Frame = frame

        self.An = [a[i] for i in range(0, a.__len__())]
        self.Tn = [t[i] for i in range(0, t.__len__())]

    def duplicate(self):
        return K_2DAngle(self.Frame, self.An, self.Tn)

    def A(self, t=-1): # returns the current Y location
        n = len(self.An)
        if n  == 1 : a = self.An[0]    # the object has no history
        else : a = self.An[t]          # the object has a history

        return a

    def dt(self, t=-1):
        n = len(self.Tn)
        if n  < 2 :
            print("Exception : Trying to get dt on a frame that doesn't have time history")   # the object has not enough history
            return -1
        else :
            dt = self.Tn[t] - self.Tn[t-1]          # the object has a history
            return dt


    def SetA(self, a, frame, t=-1):  # returns the current X location
        n = len(self.An)
        if n  == 1 : self.An[0]  = a   # the object has no history
        else : self.An[t] = a         # the object has a history

        self.Frame = frame

    def AddATimeStep(self, t) :
        self.An.append(self.An[-1])
        dt = self.Tn[-1] - self.Tn[-2]
        self.Tn.append(self.Tn[-1] + dt)

    def Va(self, t=-1):
        return (self.An[t] - self.An[t-1]) / (self.Tn[t]-self.Tn[t-1])

    def Aa(self, t=-1):
        dt = (self.Tn[t]-self.Tn[t-1])
        return (self.An[t] - 2 * self.An[t-1] + self.An[t-2]) / (dt * dt)

    def AInParentCoordinates(self, t=-1):
        a = self.A(t)             # This assumes that we are plotting a rigid solid where the internal coordinates have no history
        a += self.Frame.Oa(t)
        return a

    def AngleInAncestorCoordinates(self, end_frame, t=-1):
        new_angle = self.duplicate()

        for i in range(0, len(new_angle.Tn)):
            current_frame = self.Frame
            new_angle.Frame = self.Frame
            while current_frame.GetDescription() != end_frame.GetDescription():
                a = new_angle.AInParentCoordinates(t - i)
                new_angle.SetA(a, current_frame.Frame, t - i)
                current_frame = current_frame.Frame
        new_angle.Frame = end_frame

        return new_angle

    def TimeSample(self, n_samples, end_t=-1):  # number of samples, and end time of the sequence
        # Returns a new point with just a sample of the time history
        a = []
        t = []
        o_index = end_t - n_samples

        if end_t < 0: o_index = self.An.__len__() + end_t - 1

        for i in range(0, n_samples):
            a.append(self.An[o_index + i])
            t.append(self.Tn[o_index + i])

        return K_2DAngle(self.Frame, a, t)


class K_2DFrame:
    def __init__(self, name, parent_frame, x, y, a, t) :
        self.Name = name
        self.OriginInParent = K_2DPoint(parent_frame,x,y,t)
        self.AngleInParent = K_2DAngle(parent_frame, a, t)
        self.Frame= parent_frame
        if parent_frame != 0 :
            self.Frame.AddChild(self)
        self.Children = []

    def AddChild(self, child):
        self.Children.append(child)

    def GetDescription(self):
        return self.Name

    def Oxy(self, t=-1):
        return self.Ox(t), self.Oy(t)

    def Ox(self, t=-1):
        return self.OriginInParent.X(t)

    def Oy(self, t=-1):
        return self.OriginInParent.Y(t)

    def Oa(self, t=-1):
        return self.AngleInParent.A(t)

    # Will need to get cleaned up later.....
    def GetXYInParent(self, xy, t=-1):
        nxy = []
        a = self.Oa(t)
        nxy = K_rotate_xy (xy, a)
        nxy[0] += self.Ox(t)
        nxy[1] += self.Oy(t)
        return nxy

    def XYRel2Ground (self, xy = [0,0], t=-1):
        current_frame = self
        while current_frame.Frame.GetDescription() != "Ground Frame" :
            xy = current_frame.GetXYInParent(xy, t)
            current_frame = current_frame.Frame

        return xy

    def GetFrame(self, desc):
        current_frame = self
        while current_frame.Frame.GetDescription() != desc:
            current_frame = current_frame.Frame

        return current_frame.Frame

    def ExtendXYTStep(self, t) :
        if t != len(self.OriginInParent.Tn) + 1 :
            print("Error, a step too far")
        else :
            self.OriginInParent.AddXYTimeStep(t)
            self.AngleInParent.AddATimeStep(t)


    #def VoxyRel2Ground(self, t=-1):
    #
    # Create a new point with just the last n=2 time history
    #   point = self.OriginInParent.TimeSample(2, -1) # Creates a point with the last two time samples
    #   root_frame = self.GetFrame("Ground Frame")
    #   point_in_parent = point.PointInAncestorCoordinates (root_frame)
    #   return point_in_parent.V()

