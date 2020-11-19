"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Second iteration of the Wind Turbine Project. Wind 1 implemented the basic classes, but didn't have the right frame 
hierarchies, and had messy math and displays. This is a restart directly including animations !!!

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
import math
import numpy as np
import xml.etree.ElementTree as ET

Pi = math.pi

# Global variables to setup the expriment. Note I have to bring back all the file names in here
class K_Experiment_Variables :
    def __init__(self, turbine_model, experiment_name = None):
        if experiment_name == "Hard_Wired" :
            self.experiment_name = "Hard Wired"
            self.dt = 0.001
            self.n_steps = 100
            self.omega = 0   # Works well with 2*pi
            self.n_outline_samples = 50
            self.wind_velocity = 6 # Velocity in m/s, this is the wind absolute velocity
            self.wind_angle = 270  # Wind angle in degrees
            self.turbine_type = K_Turbine_Configuration_Parameters(turbine_model)
        else :
            print("Exception : Turbine Experiment"+str(experiment_name)+str(" is not implemented yet"))

class K_Turbine_Configuration_Parameters :
    def __init__(self, turbine_name = None):
        if turbine_name == "V3-Hardwired" :
            self.turbine_name = turbine_name
            self.number_of_branches = 3
            self.core_radius = .15
            self.arm_length = 1.2
            self.wing_length = 2.0
            self.wing_width = .6
            self.aileron_width = .1
            self.wing_chord = wing_width *.15
            self.wing_mass = .3
        else :
            print("Exception : Turbine Configuration"+str(turbine_name)+str(" is not implemented yet"))


# Import what matters
import matplotlib
#import matplotlib.pyplot as plt
#from matplotlib.gridspec import GridSpec
#import matplotlib.animation as animation

from K_Turbine_Initializations import K_Initialize_Darius, K_Initialize_Floating_Wing_V1, K_Initialize_Floating_Wing_V3
from K_Graphics import Dashboard
from K_Wind import Wind

# Set TkArg to allow display outside of the PyCharm environment. Shame......
matplotlib.use("TkAgg")


if __name__ == "__main__":

# Initialize Graphics
#   dashboard = Dashboard()

# Initialize the experiment
    experiment_configuration_path = "/Users/pierresamec/PycharmProjects/Tests/Experiment-S3-000.xml"
    tree = ET.parse(experiment_configuration_path)
    experiment_xml_root = tree.getroot()

    experiment = K_Experiment_Variables(experiment_xml_root)
    title = "V3"
    #title = "Daerius"
    number_of_branches = 3        # Number of wings in the turbine
    n_outline_samples = 50          # Numbe of samples in outline of gemetrical forms
    core_radius = .1               # Generator radius
    arm_length = 1.2               # length of the turbine arm
    dt = 0.001
    omega = 0.
    wing_length = 2.0                # Length of the wing
    wing_width = .65                # width of the wing profile
    wing_chord = .1               # Thickness of the wing, and finally mass
    wing_mass = 1.5
    aileron_width = .1
    n_steps = 3000
    wind_velocity = 6.0
    wind_angle = 180
    current_time = [0, dt, 2 * dt]
    force_drawing_scale = 1/10
    max_power = 70
    display_increment =100

#experiment = K_Experiment_Variables(experiment_xml_root)

# Initialize a Darius Wind Turbine
    if title == "Daerius" :
                turbine_system, ground_frame  = K_Initialize_Darius(
                        current_time,               # Initial time steps
                        number_of_branches,         # Number of wings in the turbine
                        n_outline_samples,          # Numbe of samples in outline of gemetrical forms
                        core_radius,                # Generator radius
                        arm_length,                 # length of the turbine arm
                        dt,                         # time step
                        omega,                      # Turbine angular rotation speed in rad/sec
                        wing_length,                # Length of the wing
                        wing_width,                 # width of the wing profile
                        wing_chord,                 # Thickness of the wing, and finally mass
                        wing_mass)

    if title == "V1" :
        turbine_system, ground_frame = K_Initialize_Floating_Wing_V1(
                        current_time,               # Initial time steps
                        number_of_branches,         # Number of wings in the turbine
                        n_outline_samples,          # Numbe of samples in outline of gemetrical forms
                        core_radius,                # Generator radius
                        arm_length,                 # length of the turbine arm
                        dt,
                        omega,
                        wing_length,                # Length of the wing
                        wing_width,                 # width of the wing profile
                        wing_chord,                 # Thickness of the wing, and finally mass
                        wing_mass)
    if title == "V3" :
        turbine_system, ground_frame = K_Initialize_Floating_Wing_V3(
                        current_time,               # Initial time steps
                        number_of_branches,         # Number of wings in the turbine
                        n_outline_samples,          # Numbe of samples in outline of gemetrical forms
                        core_radius,                # Generator radius
                        arm_length,                 # length of the turbine arm
                        dt,
                        omega,
                        wing_length,                # Length of the wing
                        wing_width,                 # width of the wing profile
                        wing_chord,                 # Thickness of the wing, and finally mass
                        wing_mass,
                        aileron_width)

    # Initialize the rest of the important stuff.
    dashboard = Dashboard(wing_width, number_of_branches, arm_length, n_steps, max_power)
    wind = Wind(wind_velocity, wind_angle)

#Display the turbine first three steps
    turbine_system.Print("Top to Bottom", -1, ground_frame)   # -1 indicaes that we are printing at the latest time

    # Plotting the initial time steps for the turbine is no longer required.......
    #for i in range (0,3) :
    #   turbine_system.Plot(i, ground_frame, dashboard)      # -1 here indicates that we are plotting the latest frame
    #  dashboard.ShowFrame()

    for i in range (0, n_steps) :
        dashboard.TurbineSubPlot.clear()
        dashboard.TurbineSubPlot.set(xlim=(-arm_length*1.1,arm_length*1.1), ylim=(-arm_length*1.1,arm_length*1.1))

        turbine_system.Step(i+3, dashboard, wind)
        turbine_system.Plot(i+3, dashboard, wind)
        turbine_system.Debug(i-3)

        # Plot the wing properties
        j = 0
        for level in turbine_system.LevelInformation :
            for solid in level.NodesList :
                if solid.Description == 'Simple Wing' :
                    solid.Plot_KinematicAndDynamicProperties(i+3, dashboard.WingSubPlots[j], dashboard, wind)
                    j += 1

        # Print the basic information in the turbine main plot
        generative_torque, wind_torque = turbine_system.Debug(i+4)
        dashboard.TorqueSubPlot.plot(i, generative_torque, 'bs', i, -wind_torque, 'g^')

        label  = str(" RPM  ")+str(round(turbine_system.LevelInformation[2].NodesList[0].OVa * 60 / 2 / math.pi, 2)) + str('\n')
        label += str(" Angle")+str(round(turbine_system.LevelInformation[2].NodesList[0].Oa_Sample.An[-1] * 180 / math.pi, 2)) + str('\n')
        label += str(" Time ")+str((i+3)*dt)+ str('\n')
        label += str(" Wind Torque in N/m      ")+ str(round(wind_torque, 2))+ str('\n')
        label += str(" Generative Torque in N/m ")+ str(round(generative_torque, 2))
        dashboard.TurbineSubPlot.text(-1.1, -.5, label, fontsize=6)

        # Plot each of the wings contributing torques in the turbine subplot as arrows.
        j = 0
        for level in turbine_system.LevelInformation :
            for solid in level.NodesList :
                if solid.Description == 'Simple Wing' :
                    moment_amplitude = 0
                    for moment in solid.Link.MomentsFromParentToChild :
                        moment_amplitude -= moment.Amplitude
                    # x_plot, y_plot = solid.Frame.OriginInParent.XY() # Get the wing frame orgin coordinate
                    x_plot, y_plot = solid.Frame.XYRel2Ground()  # Get the wing frame orgin coordinate
                    ox_plot = x_plot / 2 ; oy_plot = y_plot / 2  # Plot the arrow orgin in the mid point between the wing CG and the turbin axis assumed to be at 0, 0
                    norm = math.sqrt(ox_plot * ox_plot + oy_plot * oy_plot)
                    ex_plot = ox_plot - oy_plot * moment_amplitude * force_drawing_scale / norm
                    ey_plot = oy_plot + ox_plot * moment_amplitude * force_drawing_scale / norm
                    if moment_amplitude < 0 : arrow_color = 'blue'
                    else : arrow_color = 'red'
                    dashboard.TurbineSubPlot.annotate(str(round(moment_amplitude, 2)),
                                       xy    = (ox_plot, oy_plot), xycoords='data',
                                       xytext= (ex_plot, ey_plot), textcoords='data',
                                       arrowprops=dict(arrowstyle='<|-', color=arrow_color, lw=4, ls='-'))
                    j += 1



        ideal_angle = []
        plotx = []
        ploty = []
        signage = []
        scale = 10
        for i in range(0, 360, 1) :
            turbine_angle = i * 2 * math.pi / 360
            rpm = turbine_system.LevelInformation[2].NodesList[0].OVa
            if title == "V3":
                aa = solid.Link.ServoInputFunction(rpm, turbine_angle)
                ideal_angle.append(aa[-1])
                xx = math.fabs(ideal_angle[-1]) * math.cos(turbine_angle)*scale
                yy = math.fabs(ideal_angle[-1]) * math.sin(turbine_angle)*scale

                plotx.append(xx)
                ploty.append(yy)
                signage.append(ideal_angle[-1])

        x = np.array(plotx)
        y = np.array(ploty)
        #t = np.array(signage)

        dashboard.TurbineSubPlot.plot(x, y)



        dashboard.ShowFrame()

    #ani = animation.FuncAnimation(dashboard.fig,
    #                                  turbine_system.Plot(ground_frame, dashboard),
    #                                  np.arange(1, 3),
    #                                  init_func=turbine_system.Plot(0, ground_frame, dashboard),
    #                                  interval=25,
    #                                  blit=True)

    dashboard.ShowEnd()