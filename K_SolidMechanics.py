# ----------------------------------------------------------------------------------------------------------------------
# Kalya Energy Two Dimensional Solids
# ----------------------------------------------------------------------------------------------------------------------
# Define Solid as Tree Nodes with a referential : Solids are Generic, Wing, Core, Plateau
import pandas as pd
from scipy import interpolate
import math
from K_Graph import K_Node, K_Tree, K_Link
from K_Geometry import K_2DFrame, K_2DPoint, K_ra2xy, K_2DPointVelocity, K_2DPointAcceleration, K_rotate_xy, K_angle

class SolidOutline :
    def __init__(self):
        self.outline = [] #Outline is a list of Points.

class DOF :
    def __init__(self, dof_type, mechanical_linkage):
        self.Type = dof_type                # Fixed, Time Function, Compute, Free
        self.Linkage = mechanical_linkage   # MechanicalLinkage for which this DOF is in action.....

class XY_DOF(DOF) :
    def __init__(self, dof_type, mechanical_linkage):
        DOF.__init__(self, dof_type, mechanical_linkage)

        if self.Type == "Fixed" :
            self.n_dofs = 0
        elif self.Type == "Computed" :
            self.T_dofs = 0
        elif self.Type == "Free":
            self.n_dofs = 2
        else :
            self.n_dofs = 0
            print("Exception: unknown DOF")

class A_DOF(DOF) :
    def __init__(self, dof_type, mechanical_linkage):
        DOF.__init__(self, dof_type, mechanical_linkage)

        if self.Type == "Fixed" :
            self.n_dofs = 0
        elif self.Type == "Free":
            self.n_dofs = 1
        elif self.Type == "Servo" :
            self.n_dofs = 0
        elif self.Type == "Computed" :
            self.n_dofs = 0
        else :
            print("Exception, unknow Angular DOF")


class Force :
    def __init__(self, force_type, application_point, end_point) :
        self.Type = force_type
        self.PointOfApplication = application_point
        self.Force = end_point

    def Reversed(self) :
        new_point = K_2DPoint(self.PointOfApplication.Frame,
                              [-self.Force.X(-1)],
                              [-self.Force.Y(-1)],
                              [-1])
        return Force(self.Type, self.PointOfApplication, new_point)

    def Moment(self, at_point) :
        if self.PointOfApplication.Frame != at_point.Frame :
            print("Exeption : Moment Forces application points, and axle cordinates must be in the same frame")
        ox, oy = at_point.XY()
        x, y   = self.PointOfApplication.XY()
        vx = x-ox ; vy = y - oy
        fx, fy = self.Force.XY()
        return Moment("Force Moment", at_point, fx*vy-fy*vx)

    def ForceInAncestorCoordinates(self, to_frame):
        application_point_transformed = self.PointOfApplication.PointInAncestorCoordinates(to_frame, -1)
        force_transformed = self.Force.PointInAncestorCoordinates(to_frame, -1)
        new_force = Force(self.Type,
                          application_point_transformed,
                          force_transformed)
        return new_force


class InertialForce(Force) :
    def __init__(self, solid, eval_frame) :
        self.Type = "Inertial Force"
        self.PointOfApplication = solid.GetCG()
        fx = - solid.OAxy.Ax(-1) * solid.Mass
        fy = - solid.OAxy.Ay(-1) * solid.Mass
        t = solid.OAxy.At(-1)
        self.Force = K_2DPoint(eval_frame, [fx], [fy], [t])


class Moment :
    def __init__(self, moment_type, evaluation_point, amplitude) :
        self.Type = moment_type
        self.PointOfEvaluation = evaluation_point
        self.Amplitude = amplitude

    def Reversed(self) :
        return Moment(self.Type, self.PointOfEvaluation, -self.Amplitude)


class InertialMoments(Moment) :
    def __init__(self, solid) :
        self.type = "Inertial Moment"
        self.PointOfEvaluation = solid.Frame.OriginInParent
        self.Amplitude = solid.OAa * solid.MomentOfInertia

class Solid(K_Node) :
    def __init__(self, desc, frame, mass, momentum):
        K_Node.__init__(self)
        self.Description = desc
        self.Frame = frame
        self.CG = K_2DPoint(frame, [0], [0], [0])
        self.Mass = mass
        self.Outline = []
        self.MomentOfInertia = momentum #This is the momentum around the origin of the frame.....
        self.DOFs = []
        self.Forces = []
        self.Moments = []
        self.Oxy_Sample = None
        self.Oa_Sample = None
        self.OVxy  = None
        self.OVa   = None
        self.OAxy  = None
        self.OAa   = None
        self.Color = None

    def GetCG(self):
        return self.CG

    def ForcesAndTorquesEvaluation(self, t, args):

# Initialize everything
        if self.Description == 'Ground': return
        ground_frame = self.Frame.GetFrame("Ground Frame")
        self.Forces = []
        self.Moments = []

# Compute all the Acceleration Forces at time t and at the Origin of the frame anchoring the solid.
        self.Oxy_Sample = self.Frame.OriginInParent.TimeSample(3, t)
        self.Oa_Sample  = self.Frame.AngleInParent.TimeSample(3,t)

        #Translate coordinates to the ground Frame.
        self.Oxy_Sample = self.Oxy_Sample.PointInAncestorCoordinates(ground_frame, -1)


        self.OVxy  = self.Oxy_Sample.V(-1)
        self.OVa   = self.Oa_Sample.Va(-1)

        self.OAxy  = self.Oxy_Sample.A(-1)
        self.OAa   = self.Oa_Sample.Aa(-1)

        # Append Forces and Moments
        #self.Forces.append(InertialForce(self, ground_frame))
        #self.Moments.append(InertialMoments(self))

    def ComputeDOFs(self): # Compute the degrees of freedom of a node.
        if self.Link != None :
            if self.Link.LinkageType == "Free" :  self.DOFs = [('x', 0), ('y', 0), ('a', 0)]
            if self.Link.LinkageType == "Fixed" : self.DOFs = []
            if self.Link.LinkageType == "Servo" : self.DOFs = []
            if self.Link.LinkageType == "Axial Magnetic": self.DOFs = [('a', 0)]
            if self.Link.LinkageType == "Free Axial": self.DOFs = [('a', 0)]

    def AddTimeStep(self, t) :
        # First extend x,y,t
        self.Frame.ExtendXYTStep(t)
        if self.Frame.Name == 'Ground Frame' : return
        ground_frame = self.Frame.GetFrame("Ground Frame")

        for dof in self.DOFs :
            # then override x, y, , for specific DOFS

            if dof[0][0] == 'a' :
                # Get the moments from the parent
                resulting_moment = 0
                for moment in self.Link.MomentsFromParentToChild :
                    frame_origin = self.Frame.OriginInParent.PointInAncestorCoordinates(ground_frame, -1)
                    moment_evaluation = moment.PointOfEvaluation.PointInAncestorCoordinates(ground_frame, -1)

                    delta_x = math.fabs(frame_origin.X() - moment_evaluation.X())
                    delta_y = math.fabs(frame_origin.Y() - moment_evaluation.Y())
                    if frame_origin.X()  == moment_evaluation.X() and frame_origin.Y()  == moment_evaluation.Y() :
                        resulting_moment += moment.Amplitude
                    else :
                        print("Exception : Evaluation point of Origin are different")

                # Get the moments fom the children
                for child in self.ChildrenNodes :
                    for moment in child.Link.MomentsFromParentToChild :
                        # Add the moments
                        frame_origin = self.Frame.OriginInParent.PointInAncestorCoordinates(ground_frame, -1)
                        moment_evaluation = moment.PointOfEvaluation.PointInAncestorCoordinates(ground_frame, -1)

                        delta_x = math.fabs(frame_origin.X() - moment_evaluation.X())
                        delta_y = math.fabs(frame_origin.Y() - moment_evaluation.Y())
                        error = 0.001
                        if delta_x < error and delta_y < error :
                            resulting_moment += moment.Amplitude
                        else:
                            print("Exception : Evaluation point of Origin are different")

                # Add the self moment. This is especially important for the wings stabilization moments....
                if self.Description == "Simple Wing" :
                    resulting_moment += self.AerodynamicMoment

                # Update a
                at = self.Frame.AngleInParent.A(t-2)
                atm = self.Frame.AngleInParent.A(t-3)
                dt = self.Frame.AngleInParent.dt(t-2)
                atp = - resulting_moment * dt * dt / self.MomentOfInertia + 2 * at - atm
                self.Frame.AngleInParent.SetA(atp, self.Frame, t-1)

            else :
                # extend x, y, a histories
                print("Updating all")

        if self.Link.LinkageType == "Servo" :
            # Get Angle
            if self.Description == "SimpleWing" :
                ThisWing = self
            else :
                ThisWing = self.ParentNode

            x = ThisWing.Oxy_Sample.Xn[-1]
            y = ThisWing.Oxy_Sample.Yn[-1]
            turbine_rotation_angle = K_angle(x, y)
            if turbine_rotation_angle < 0 : turbine_rotation_angle += 2 * math.pi

            # Compute the new wing incidence angle
            turbine_rotation_speed = ThisWing.Link.To.OVa
            wing_angle = self.Link.ServoInputFunction(turbine_rotation_speed, turbine_rotation_angle)
            link_step = self.Link.Servo.SetNewPosition(wing_angle[0], 0.001)
            rpm = turbine_rotation_speed*60 / 2 / math.pi
            angle1 = turbine_rotation_angle * 180 / math.pi
            wing_computed_angle = wing_angle[-1] * 180 / math.pi
            print ("Turbine Rotation (in d) ", angle1,
                   "Wing set angle (in d)", wing_computed_angle,
                   "Turbine RPM (in rpm)", rpm)

            #Used to be : self.Frame.AngleInParent.An[-1] = wing_angle[-1]
            # Now only accepting the steps the servos can deliver.....
            self.Frame.AngleInParent.An[-1] += link_step

            print(" ")

    def SpecializedForces(self) :
        print("          Standard Solid have not specialized Forces and Moment")


    def Print(self, t, args):
        print_frame = args
        if self.ParentNode == None :
            print (self.Description, 'Root Node at time index n', t,' Relative to ', print_frame.Name)
        else :
            print (self.Description, '(',self.ParentNode.Description, ') Mass of ', self.Mass, ' Moment of Inertia of ', self.MomentOfInertia)
            if self.Frame != None :
                print ('       In Frame', self.Frame.Name, ' at origin in parent x(m)=', self.Frame.Ox(), ' y(m)=', self.Frame.Oy(), ' a(degrees)=', self.Frame.Oa()*180/math.pi)
                #Oxy = self.Frame.XYRel2Ground()
                o = self.Frame.OriginInParent.PointInAncestorCoordinates(print_frame, -1)
                print ('       with Absolute coordinates x(m)=', o.X(-1), ' y(m)= ', o.Y(-1))
                Vo = o.V(-1)
                vx = Vo.Vxn[-1] ; vy = Vo.Vyn[-1]
                print ('       and  Absolute Velocity v(m/s)=', math.sqrt(vx*vx + vy*vy), 'with components Vx= ', vx, ' Vy= ', vy, '\n')

    def GetDescription(self):
        return self.Description

    def Plot(self, t, args):
        print("Plotting", self.Description)
        if self.Frame.Name != "Ground Frame" : ground_frame = self.Frame.GetFrame("Ground Frame")
        else : ground_frame = self.Frame
        plot_area = args[0].TurbineSubPlot
        xx = []
        yy = []
        if len(self.Outline) != 0 :
            for current_point in self.Outline :
                new_point = current_point.PointInAncestorCoordinates(ground_frame, t)
                xy = new_point.XY()
                xx.append(xy[0])
                yy.append(xy[1])

            plot_area.plot(xx, yy, 'k-', lw=2)  # plot wing foil
            if self.Description == 'Simple Wing' and self.Color != None :
                plot_area.fill(xx, yy, self.Color)

        else :
            print ('This solid outline is not set')

    def Plot_KinematicAndDynamicProperties(self, t, plot_area, dashboard, wind):
        #print("Printing Apparent Velocity, and Forces")
        # self.Plot(-1, [args]) although tempting, I can not use the general plot method as it would involve a cordinate transfer...

        # Plot the orginal solid Outline.....
        if self.Description == "Simple Wing" : plot_area.clear() # First time around, and before drawing the children of the wing,
        if dashboard != None :
            plot_area.set(xlim=(-dashboard.a*3, dashboard.a*3), ylim=(-dashboard.a, dashboard.a))


        xx = []
        yy = []
        if len(self.Outline) != 0 :

            for current_point in self.Outline: # We are actually plotting in the local coordinates
                if self.Description == "Aileron":
                    x_plot, y_plot = current_point.XYInParentCoordinates(t)
                else :
                    x_plot, y_plot = current_point.XY(t)
                xx.append(x_plot)
                yy.append(y_plot)

            plot_area.plot(xx, yy, 'k-', lw=2)  # plot wing foil
            if self.Description == 'Simple Wing' and self.Color != None :
                plot_area.fill(xx, yy, self.Color, alpha=0.2)

        else :
            print ('This solid outline is not set')



class Ground(Solid) :
    def __init__(self, frame) :
        super().__init__("Ground", frame, 0, 0)


class Turbine_Plateau(Solid) :
    def __init__(self, frame, mass_per_branch, core_radius, arm_length, number_of_branches):

        super().__init__("Turbine Plateau", frame, 0, 0)
        angle_between_branches = 2 * math.pi / number_of_branches

        for i in range(number_of_branches) :
            x, y = K_ra2xy(arm_length, i*angle_between_branches);         self.Outline.append(K_2DPoint (frame, [x], [y], [0]))
            x, y = K_ra2xy(core_radius, (i+0.5)*angle_between_branches) ; self.Outline.append(K_2DPoint (frame, [x], [y], [0]))
            x, y = K_ra2xy(arm_length, (i+1)*angle_between_branches) ;    self.Outline.append(K_2DPoint (frame, [x], [y], [0]))

        self.Mass = mass_per_branch*number_of_branches
        self.MomentOfInertia = number_of_branches * mass_per_branch * arm_length * arm_length / 3


class Cylindrical_Solid(Solid):
    def __init__(self, frame, radius, mass, n_outline_samples):
        super().__init__("Cylinder", frame, mass, mass*radius*radius/2)
        angle = 0.0

        for i  in range(n_outline_samples):
            x, y = K_ra2xy(radius, angle)
            point = K_2DPoint(frame, [x], [y], [0])
            self.Outline.append(point)
            angle = angle + 2 * math.pi / n_outline_samples


class SimpleWing(Solid):
    def __init__(self, frame, span, length, chord, mass):
        super().__init__("Simple Wing", frame, mass, 0)

    # Initialize Wing Geometry
        self.span = span
        self.length = length
        self.mass = mass

        self.Outline.append(K_2DPoint (frame, [-length * 2 / 3],    [0],      [0]))
        self.Outline.append(K_2DPoint (frame, [0],                  [-chord], [0]))
        self.Outline.append(K_2DPoint (frame, [length / 3],         [0],      [0]))
        self.Outline.append(K_2DPoint (frame, [0],                  [chord],  [0]))
        self.Outline.append(K_2DPoint (frame, [-length * 2 / 3],    [0],      [0]))

        self.Tail = K_2DPoint (frame, [-length * 2 / 3], [0], [0])
        self.Nose = K_2DPoint (frame, [length / 3],      [0], [0])
        self.MomentOfInertia = mass/3 * (chord/3 * chord/3) /3 + mass * (chord*chord) * 8 / 81 # back of the wing assuming homogeneous plankself

    # Initialize Lift and drag coefficients
        self.Cl = []
        self.Cd = []
        self.Cm = []
        self.Cx = []
        ExcelRead = pd.read_excel(r"/Users/pierresamec/PycharmProjects/Data/BasicNaca12ClCdCm.xlsx")
        self.Cl = ExcelRead.Cl.to_numpy().real
        self.Cd = ExcelRead.Cd.to_numpy().real
        self.Cm = ExcelRead.Cm.to_numpy().real
        self.Cx = ExcelRead.I.to_numpy().real
        self.theta = self.Cx * math.pi / 180.
        self.CriticalAngle = 11.6 * math.pi / 180.

    # Initialize the instantaneous values
        self.Cl_Instantaneous = 0
        self.Cd_Instantaneous = 0
        self.Cm_Instantaneous = 0
        self.Fxy = None
        self.AerodynamicMoment = 0
        self.Incidence_Instantaneous = 0.
        self.AparentVelocity = None
        self.CriticalAngle = 15 * math.pi / 180.


    # Compute the interpolation parameters
        self.f_Cl = interpolate.interp1d(self.theta, self.Cl, kind='cubic')
        self.f_Cd = interpolate.interp1d(self.theta, self.Cd, kind='cubic')
        self.f_Cm = interpolate.interp1d(self.theta, self.Cm, kind='cubic')


    def ClCd(self, a):

        Cl_interpolated = self.f_Cl(a)
        Cd_interpolated = self.f_Cd(a)
        Cm_interpolated = self.f_Cm(a)

        return Cl_interpolated, Cd_interpolated, Cm_interpolated

    def ForcesAndTorquesEvaluation(self, t, args):
        dashboard = args[0]
        wind = args[1]
        ground_frame = self.Frame.GetFrame("Ground Frame")
        Solid.ForcesAndTorquesEvaluation(self, t, args) # Evluate the standard Forces fr a Solid (Acceleration forces, etc....
        vxy,fx,fy, i_angle = self.AerodynamicForcesEvaluation(t, wind)
        self.Forces.append(Force("Aerodynamic Forces", self.Frame.OriginInParent, K_2DPoint(ground_frame, [fx], [fy], [t])))
        self.Moments.append(Moment("Aerodynamic Forces",self.Frame.OriginInParent,self.AerodynamicMoment))

    def AerodynamicForcesEvaluation(self, t, w):

        # Compute all the Acceleration Forces at time t
        eval_frame = self.Frame.GetFrame("Ground Frame")
        o = self.Frame.OriginInParent.PointInAncestorCoordinates(eval_frame, -1)
        Vo = o.V(-1)

        # Substract the wind velocity from the actual wing velocity
        Vo.Vxn[-1] -= w.wind.Vxn[-1]
        Vo.Vyn[-1] -= w.wind.Vyn[-1]
        a = self.Frame.AngleInParent.AngleInAncestorCoordinates(eval_frame, -1)
        vxy = K_rotate_xy([Vo.Vxn[-1], Vo.Vyn[-1]], -a.An[-1])

        # Compute Lift and Drag Forces with a different method for computing forces on Ailerons, where the wind effect is now // to the wind, and therefore the servo angle is the incidence angle.
        if self.Description == "Aileron" :
            incidence_angle = - self.Link.Servo.current_position # this is now the servo angle of the aileron since the air is laminar at the tail of the wing.
            # Apparent velocity is set to parallel to the X axis.
            velocity_value = math.sqrt(vxy[0]*vxy[0]+vxy[1]*vxy[1])
            vxy = []
            vxy.append(velocity_value)
            vxy.append(0)
            efficiency = 1.0
            if self.ParentNode.Incidence_Instantaneous > self.ParentNode.CriticalAngle :
                efficiency = .3 # If the parent wing of the aileron is past the critical angle, Aileron efficiency is reduced.
        else :
            incidence_angle = K_angle(vxy[0], vxy[1])
            efficiency = 1

        Cl, Cd, Cm = self.ClCd(incidence_angle)
        rho = 1.2
        force = .5 * rho * (vxy[0]*vxy[0] + vxy[1]*vxy[1]) * self.length * self.span
        lift = efficiency * force * Cl
        drag = efficiency * force * Cd
        moment = force * Cm
        fxy = K_rotate_xy([-drag, -lift], incidence_angle)
        print("Wind Apparent Velocity ", vxy, "in m/s and Incidence Angle ", incidence_angle *180 / math.pi, "i n degrees")
        print("Cl, Cd ", Cl, Cd, "Lift and drag  =n ", -lift,-drag, "in N")

        self.Cl_Instantaneous = Cl
        self.Cd_Instantaneous = Cd
        self.Fxy = fxy
        self.Incidence_Instantaneous = incidence_angle
        self.AparentVelocity = vxy
        self.Cm_Instantaneous = Cm
        self.AerodynamicMoment = moment

        return vxy, fxy[0], fxy[1], incidence_angle

    def Plot_KinematicAndDynamicProperties(self, t, plot_area, dashboard, w):
        Solid.Plot_KinematicAndDynamicProperties(self, t, plot_area, dashboard, w)

        for nodes in self.ChildrenNodes : # Plot all the things attached to a wing....
            nodes.Plot_KinematicAndDynamicProperties(t, plot_area, dashboard, w)

        # Initialize the plotting parameters
        vxy = self.AparentVelocity
        fx = self.Fxy[0]
        fy = self.Fxy[0]
        i_angle =  self.Incidence_Instantaneous

        # Setup basic display computations
        v = math.sqrt(vxy[0]*vxy[0]+vxy[1]*vxy[1])
        f = math.sqrt(fx*fx+fy*fy)
        v_scale = dashboard.a / v # 1/10 for startup of the wind turbine
        f_scale = dashboard.a / f
        servo_angle = self.Frame.AngleInParent.An[-1]
        label  = str('Apparent Velocity in m/s ') + str(round(v,2))+str('\n')
        label += str('Aerodynamic Force in N   ') + str(round(f,2))+str('\n')
        label += str('Angle of Incidence in d  ') + str(round(i_angle*180/math.pi, 2)) + str('\n')
        if self.Description == "Aileron" :
            label += str('Servo Angle              ') + str(round(servo_angle*180/math.pi, 2)) + str('\n')
        else :
            label += str('Wing Angle              ') + str(round(servo_angle*180/math.pi, 2)) + str('\n')



        # appaent velocity
        if self.Description != 'Simple Wing' :
            side = -1.2
            x_plot, y_plot = self.Frame.OriginInParent.XY(t)
        else :
            side = +.5
            x_plot = 0.0; y_plot = 0.0

        plot_area.text(dashboard.a/2 + x_plot, side * dashboard.a*.9 + y_plot, label, fontsize=5)

        # speed = str(int(math.sqrt(vxy[0]*vxy[0] + vxy[1]*vxy[1]))) +'m/s'
        plot_area.annotate(None, xy=(vxy[0] * v_scale + x_plot, vxy[1] * v_scale + y_plot), xycoords='data',
                  xytext=(x_plot, y_plot), textcoords='data',
                  arrowprops=dict(arrowstyle='<|-', color='blue', lw=1, ls='-'))

        # Plot the acceleration forces f = ma

        # Plot the wind lift
        #plot_area.arrow(0, 0, drag * f_scale, lift * f_scale, head_width=0.05, head_length=0.1, fc='k', ec='k', color='red')

        plot_area.annotate(None, xy=(fx * f_scale + x_plot, fy * f_scale+ y_plot), xycoords='data',
                  xytext=(x_plot, y_plot), textcoords='data',
                  arrowprops=dict(arrowstyle='-|>', color='red', lw=1, ls='-'))

class Aileron(SimpleWing):
    def __init__(self, frame, span, length, chord, mass):
        SimpleWing.__init__(self, frame, span, length, chord, mass)

        self.Description = "Aileron"



class MechanicalLinkage(K_Link) :
    def __init__(self, child, parent, description, map=0):
        K_Link.__init__(self, child, parent)
        self.LinkagePointInParent = child.Frame.OriginInParent
        self.LinkageType = description
        self.ForcesFromParentToChild = []
        self.MomentsFromParentToChild = []
        self.XYdof = None
        self.Adof = None
        self.ServoInputFunction = map


    def GetDescription(self):
        return self.LinkageType

    def ComputeLinkageMoments(self):
        return 0, 0, 0

class FreeAxialLinkage(MechanicalLinkage) :
    def __init__(self, child, parent):
        MechanicalLinkage.__init__(self, child, parent, "Free Axial", map=0)
        self.XYdof = XY_DOF("Fixed", self)
        self.Adof = A_DOF("Free", self)

    def UpdateLinkageForcesAndTorques(self) :
    # Initialize all the local variables for this function
            if self.To.Description == 'Ground' : return
            self.ForcesFromParentToChild = []
            self.MomentsFromParentToChild = []
            ground_frame = self.To.Frame.GetFrame("Ground Frame")


    # Move the forces application Points
            for f in self.From.Forces :
                #Changing Force Coordinates
                nxy = f.PointOfApplication.XYInParentCoordinates()
                point = K_2DPoint(self.To.Frame, [nxy[0]], [nxy[1]], [-1])

                # Append
                if self.XYdof.Type == "Fixed" or self.XYdof.Type == "Servo":                 # On a fixed linkage Forces Balance
                    self.ForcesFromParentToChild.append(Force("Link Force", point, f.Force)) # This force evaluation might

    # Evaluate Moments

            # Compute Moments of the children's forces..... BTW I am not sure this moment evaluatin should be here
            # It is necessary to compute it here to evaluate the generative moments, BUT it belongs in a recursion above.....
            for f in self.From.Forces :
                #Changing Force Coordinates : This will have to be fixed When I add the Ailerons.....
                f_in_parent_coordinates = f.ForceInAncestorCoordinates(ground_frame)
                at_point = K_2DPoint(self.To.Frame, [0], [0], [-1]) # Evaluate the moments at the Origin of the to frame.
                at_point = at_point.PointInAncestorCoordinates(ground_frame, -1)
                moment = f_in_parent_coordinates.Moment(at_point)

                # Append
                if self.XYdof.Type == "Fixed" or self.XYdof.Type == "Servo":    # On a fixed linkage Forces Balance
                    self.MomentsFromParentToChild.append(moment)

            # Apply the reversed Torque from the linkage to the parent.
            #for m in resulting_moments :
            #   resulting_moments_reversed.append(m.Reversed()) # Note the reversed Force here

            # There are no Moments generated by a free linkage (Unlike the generator from which this code was taken...
            #type, torque_point, generative_moment = self.ComputeLinkageMoments()
            #if torque_point != 0 or generative_moment != 0:
            #    m = Moment(type, torque_point, generative_moment)
            #    self.MomentsFromParentToChild.append(m)



class FixedLinkage(MechanicalLinkage) :
    def __init__(self, child, parent, description, map, servo):
        MechanicalLinkage.__init__(self, child, parent, description, map=0)
        self.XYdof = XY_DOF("Fixed", self)
        self.Adof  = A_DOF("Fixed", self)
        if description == "Servo" and map !=0 :
            self.ServoInputFunction = map
            self.Servo = servo

    def UpdateLinkageForcesAndTorques(self) :
    # Initialize all the local variables for this function :
            if self.To.Description == 'Ground' : return
            self.ForcesFromParentToChild = []
            self.MomentsFromParentToChild = []
            ground_frame = self.To.Frame.GetFrame("Ground Frame")


    # Move the forces application Points
            for f in self.From.Forces :
                #Changing Force Coordinates
                nxy = f.PointOfApplication.XYInParentCoordinates()
                point = K_2DPoint(self.To.Frame, [nxy[0]], [nxy[1]], [-1])

                # Append
                if self.XYdof.Type == "Fixed" or self.XYdof.Type == "Servo":                 # On a fixed linkage Forces Balance
                    self.ForcesFromParentToChild.append(Force("Link Force", point, f.Force)) # This force evaluation might

    # Evaluate Moments

            # Compute Moments of the children's forces..... BTW I am not sure this moment evaluatin should be here
            # It is necessary to compute it here to evaluate the generative moments, BUT it belongs in a recursion above.....
            for f in self.From.Forces :
                #Changing Force Coordinates : This will have to be fixed When I add the Ailerons.....
                f_in_parent_coordinates = f.ForceInAncestorCoordinates(ground_frame)
                at_point = K_2DPoint(self.To.Frame, [0], [0], [-1]) # Evaluate the moments at the Origin of the to frame.
                at_point = at_point.PointInAncestorCoordinates(ground_frame, -1)
                moment = f_in_parent_coordinates.Moment(at_point)

                # Append
                if self.XYdof.Type == "Fixed" or self.XYdof.Type == "Servo":    # On a fixed linkage Forces Balance
                    self.MomentsFromParentToChild.append(moment)

            # Apply the reversed Torque from the linkage to the parent.
            #for m in resulting_moments :
            #   resulting_moments_reversed.append(m.Reversed()) # Note the reversed Force here

            # Compute linkage Torques
            type, torque_point, generative_moment = self.ComputeLinkageMoments()
            if torque_point != 0 or generative_moment != 0:
                m = Moment(type, torque_point, generative_moment)
                self.MomentsFromParentToChild.append(m)


class GeneratorAxleLinkage(FixedLinkage) : # Exception : A generator Axle Linkage is actually not a fixed linkage
    # This is a fudge because we wan the GeneratorAxleLinkage to use the UpdateLinkageForcesAndTorques
    # Which itself is wrong as it only works for axial forces and moments.......
    def __init__(self, child, parent, description) :
        FixedLinkage.__init__(self, child, parent, description, 0, 0)

        # Initialize the generator based on the generator power curve
        ExcelRead = pd.read_excel(r"/Users/pierresamec/PycharmProjects/Data/Generator/ChineseGenerator.xlsx")
        RPM = ExcelRead.RPM.to_numpy().real
        Watts = ExcelRead.Watts.to_numpy().real
        Torques = ExcelRead.Watts.to_numpy().real
        Angular_Velocity = RPM * 2 * math.pi / 60
        self.f_Torque = interpolate.interp1d(Angular_Velocity, Torques, kind='quadratic')
        self.f_Watts  = interpolate.interp1d(Angular_Velocity, Watts, kind='quadratic')

        self.XYdof = XY_DOF("Fixed", self)
        self.Adof  = A_DOF("Computed", self)

    def ComputeLinkageMoments (self) :
        type = "Generative Linkage"
        rotor_velocity = self.From.OVa
        return type, self.From.Frame.OriginInParent, self.f_Torque(rotor_velocity)

    def ComputeRotorAngularAcceleration(self, rotor_velociy, rotor_torque):

        stator_torque = self.f_Torque(rotor_velociy)
        # M * Angular Acceleration = Sum ( Moments )
        angular_acceleration = (stator_torque - rotor_torque) / self.To.MomentOfInertia
        return angular_acceleration

class MechanicalSystem(K_Tree) :
    def __init__(self, anchor_solid):
        K_Tree.__init__(self, anchor_solid)
        self.DOFs = []
        self.Anchor = anchor_solid

    def __str__(self):
        return_string = self.Anchor.GetDescription()
        return_string += str(" Solid In Frame  ") + self.Anchor.Frame.GetDescription() +"\n  "
        self.TraverseFromRoottoLeaves('Get Description', return_string)
        return return_string


    def AddSolidtoSystem(self, new_solid, root_solid, linkage = 0) :
        if linkage == 0 : linkage = MechanicalLinkage(new_solid, root_solid, "Default", 0)
        super().InsertLeaf(new_solid, root_solid, linkage)

    def ComputeForcesandTorques(self, t, args=-1) :
        self.TraverseFromLeavestoRoot("Forces and Moments Evaluation", t, args)

    def ComputeDOFs(self):
        self.DOFs = [] # Clear the list of DOFs since during execution a link can break delivering different dynamics
        self.TraverseFromLeavestoRoot("Compute Degrees Of Freedom", 0, 0)

    def LinkageUpdateMomentAndTorques(self):
        self.TraverseFromLeavestoRoot("Update Linkage Moment And Torques", 0, 0)

    def Step(self, t, dashboard, wind) :
        self.ComputeDOFs()  # To get the list of Solids which need resolution : The DOF list requiring computations
        arguments = [dashboard, wind]
        self.ComputeForcesandTorques(t, arguments)  # Compute all the external forces and torques on the solids
        self.LinkageUpdateMomentAndTorques() #Balance the forces and torques at the linkages
        self.TraverseFromRoottoLeaves("Add Time Step", t+1, arguments)


    def Debug(self, t):
        rpm = self.LevelInformation[2].NodesList[0].OVa * 60 / 2 / math.pi
        print('Debuging Entire System at time ', t, ' RPM ', rpm, '---------------------------------------------------------------')
        generative_torque = 0
        wind_torque = 0
        for node in self.LevelInformation[2].NodesList :
            for moment in node.Link.MomentsFromParentToChild:
                print('    Moment From ', node.Link.LinkageType, moment.Amplitude)
                generative_torque += moment.Amplitude
            for child in node.ChildrenNodes :
                for moment in child.Link.MomentsFromParentToChild :
                    print('    Moment From ', child.Description, moment.Amplitude)
                    wind_torque += moment.Amplitude

        return generative_torque, wind_torque

    def Plot(self, t, dashboard, wind) :
        new_args = [dashboard, wind]
        self.TraverseFromRoottoLeaves('Plot', t, new_args)
        return