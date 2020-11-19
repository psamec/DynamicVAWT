
import math
import xlrd
import numpy as np

from scipy import interpolate
from K_Geometry import K_2DFrame
from K_SolidMechanics import MechanicalSystem, Cylindrical_Solid, Ground, Turbine_Plateau, SimpleWing, Aileron, \
                             FixedLinkage,FreeAxialLinkage, GeneratorAxleLinkage
from K_Servo import Servo

wing_color = ["r", "g", "b", "r", "g", "b"]

def K_Initialize_Darius(
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
                        wing_mass):

    wing_initial_angle = math.pi / 2
    initial_plateau_angle = math.pi/3

    ground_frame = K_2DFrame("Ground Frame", 0, [0, 0, 0], [0, 0, 0], [0,0,0], current_time)
    stator_frame = K_2DFrame("Stator Frame", ground_frame, [0, 0, 0], [0, 0, 0], [0,0,0], current_time)
    angle = [initial_plateau_angle, initial_plateau_angle+dt*omega, initial_plateau_angle+2*dt*omega]
    plateau_frame = K_2DFrame("Turbine Plateau Frame", stator_frame, [0, 0, 0], [0, 0, 0],angle, current_time)

    wing_frames = []
    for i in range(number_of_branches) :
        a = 2 * math.pi * i/number_of_branches  # Angle between branches
        x = arm_length * math.cos(a)
        Ox = [x, x, x]
        y = arm_length * math.sin(a)
        Oy = [y,y,y]
        angle = [wing_initial_angle + a, wing_initial_angle + a, wing_initial_angle + a] # the wing is not rotating on it's axes
        wing_frames.append(K_2DFrame("Wing "+str(i+1), plateau_frame, Ox, Oy, angle, current_time))

# Initialize Solids
    ground = Ground(ground_frame)
    turbine_system = MechanicalSystem(ground)

    # Turbine Stator Genertor is fixed to the ground
    turbine_stator = Cylindrical_Solid(stator_frame, core_radius, 1, n_outline_samples)
    anchor = FixedLinkage(turbine_stator, ground, "Fixed", 0, 0)
    turbine_system.AddSolidtoSystem(turbine_stator, ground, anchor)

    # Turbine plateau contains the rotor of the generator.
    turbine_plateau = Turbine_Plateau(plateau_frame,
                                        .1,                   # Mass of a branch
                                        core_radius*1.05,     # Diameter of the internal cage around the generator
                                        arm_length,           # Length of the turbine's arms
                                        number_of_branches)   # Number of Branches
    magnetic_linkage = GeneratorAxleLinkage(turbine_plateau, turbine_stator, "Axial Magnetic")
    turbine_system.AddSolidtoSystem(turbine_plateau, turbine_stator, magnetic_linkage)

    # Add the  wing
    wing = []
    for i in range(number_of_branches) :
        wing.append(SimpleWing(wing_frames[i],
                            wing_length,
                            wing_width,     # wing emplanture length
                            wing_chord,    # wing chord
                            wing_mass))     # wing mass
        if i < 6 : wing[i].Color = wing_color[i]
        wing_linkage = FixedLinkage(wing[i], turbine_plateau, "Fixed", 0, 0)
        turbine_system.AddSolidtoSystem(wing[i], turbine_plateau, wing_linkage)  # This should turn into a time history over time
        # wing[i].Plot_KinematicAndDynamicProperties(2, dashboard.WingSubPlots[i])

    return turbine_system, ground_frame

def K_Initialize_Floating_Wing_V1(
                        current_time,               # Initial time steps
                        number_of_branches,         # Number of wings in the turbine
                        n_outline_samples,          # Numbe of samples in outline of gemetrical forms
                        core_radius,                # Generator radius
                        arm_length,                 # length of the turbine arm
                        dt,                         # time step
                        omega,
                        wing_length,                # Length of the wing
                        wing_width,                 # width of the wing profile
                        wing_chord,                 # Thickness of the wing, and finally mass
                        wing_mass):


    loc = ("/Users/pierre.samec/PycharmProjects/Data/WingIncidence(RotationAngle RPM)-V1-Test.xlsx")

    wb = xlrd.open_workbook(loc)
    sheet = wb.sheet_by_index(0)

    # For row 0 and column 0
    x = []
    y = []

    for i in range(1, sheet.ncols):
        rpm = sheet.cell_value(0, i)
        x.append(rpm * math.pi / 30)

    for j in range(1, sheet.nrows):
        rotation = sheet.cell_value(j, 0)
        y.append(rotation * math.pi / 180)

    xx, yy = np.meshgrid(x, y)
    zz = np.sin(xx + yy)

    for i in range(1, sheet.ncols):
        for j in range(1, sheet.nrows):
            incidence = sheet.cell_value(j, i)
            zz[j-1, i-1]= incidence * math.pi / 180

    print(x)
    print(y)
    print(zz)

    f = interpolate.interp2d(x, y, zz, kind='cubic')

    initial_plateau_angle = 0

    ground_frame = K_2DFrame("Ground Frame", 0, [0, 0, 0], [0, 0, 0], [0,0,0], current_time)
    stator_frame = K_2DFrame("Stator Frame", ground_frame, [0, 0, 0], [0, 0, 0], [0,0,0], current_time)
    angle = [initial_plateau_angle, initial_plateau_angle+dt*omega, initial_plateau_angle+2*dt*omega]
    plateau_frame = K_2DFrame("Turbine Plateau Frame", stator_frame, [0, 0, 0], [0, 0, 0],angle, current_time)

    wing_frames = []
    for i in range(number_of_branches) :
        a = 2 * math.pi * i / number_of_branches  # Angle between branches
        x = arm_length * math.cos(a)
        Ox = [x, x, x]
        y = arm_length * math.sin(a)
        Oy = [y,y,y]
        wing_angle = 0 # f(a, 0) would set the wings to their default position for a given rpm, but instead should be aligned to the wind...
        angle = [wing_angle, wing_angle, wing_angle] # the wing is not rotating on it's axes
        wing_frames.append(K_2DFrame("Wing "+str(i+1), plateau_frame, Ox, Oy, angle, current_time))

# Initialize Solids
    ground = Ground(ground_frame)
    turbine_system = MechanicalSystem(ground)

    # Turbine Stator Genertor is fixed to the ground
    turbine_stator = Cylindrical_Solid(stator_frame, core_radius, 1, n_outline_samples)
    anchor = FixedLinkage(turbine_stator, ground, "Fixed", 0, 0)
    turbine_system.AddSolidtoSystem(turbine_stator, ground, anchor)

    # Turbine plateau contains the rotor of the generator.
    turbine_plateau = Turbine_Plateau(plateau_frame,
                                        .1,                   # Mass of a branch
                                        core_radius*1.05,     # Diameter of the internal cage around the generator
                                        arm_length,           # Length of the turbine's arms
                                        number_of_branches)   # Number of Branches
    magnetic_linkage = GeneratorAxleLinkage(turbine_plateau, turbine_stator, "Axial Magnetic")
    turbine_system.AddSolidtoSystem(turbine_plateau, turbine_stator, magnetic_linkage)

    # Add the  wing
    wing = []
    for i in range(number_of_branches) :
        wing.append(SimpleWing(wing_frames[i],
                            wing_length,
                            wing_width,     # wing emplanture length
                            wing_chord,    # wing chord
                            wing_mass))     # wing mass
        if i < 6 : wing[i].Color = wing_color[i]
        wing_servo = Servo(1000, 100)
        wing_linkage = FixedLinkage(wing[i], turbine_plateau, "Servo", f, wing_servo)
        turbine_system.AddSolidtoSystem(wing[i], turbine_plateau, wing_linkage)  # This should turn into a time history over time

        # wing[i].Plot_KinematicAndDynamicProperties(dashboard.WingSubPlots[i])

    return turbine_system, ground_frame

def K_Initialize_Floating_Wing_V3(   # This version has ailerons !!!!
                        current_time,               # Initial time steps
                        number_of_branches,         # Number of wings in the turbine
                        n_outline_samples,          # Numbe of samples in outline of gemetrical forms
                        core_radius,                # Generator radius
                        arm_length,                 # length of the turbine arm
                        dt,                         # time step
                        omega,
                        wing_length,                # Length of the wing
                        wing_width,                 # width of the wing profile
                        wing_chord,                 # Thickness of the wing, and finally mass
                        wing_mass,
                        aileron_width,
                        max_servo_speed = 700,       # max speed in degree per seconds
                        max_servo_torque = 100):    # max torque


    loc = ("/Users/pierresamec/PycharmProjects/Data/AileronIncidence(RotationAngle RPM)-V1.xlsx")

    wb = xlrd.open_workbook(loc)
    sheet = wb.sheet_by_index(0)

    # For row 0 and column 0
    x = []
    y = []

    for i in range(1, sheet.ncols):
        rpm = sheet.cell_value(0, i)
        x.append(rpm * math.pi / 30)

    for j in range(1, sheet.nrows):
        rotation = sheet.cell_value(j, 0)
        y.append(rotation * math.pi / 180)

    xx, yy = np.meshgrid(x, y)
    zz = np.sin(xx + yy)

    for i in range(1, sheet.ncols):
        for j in range(1, sheet.nrows):
            incidence = sheet.cell_value(j, i)
            zz[j-1, i-1]= incidence * math.pi / 180

    print(x)
    print(y)
    print(zz)

    f = interpolate.interp2d(x, y, zz, kind='cubic')

    initial_plateau_angle = 0

    ground_frame = K_2DFrame("Ground Frame", 0, [0, 0, 0], [0, 0, 0], [0,0,0], current_time)
    stator_frame = K_2DFrame("Stator Frame", ground_frame, [0, 0, 0], [0, 0, 0], [0,0,0], current_time)
    angle = [initial_plateau_angle, initial_plateau_angle+dt*omega, initial_plateau_angle+2*dt*omega]
    plateau_frame = K_2DFrame("Turbine Plateau Frame", stator_frame, [0, 0, 0], [0, 0, 0],angle, current_time)

    wing_frames = []
    aileron_frame = []
    for i in range(number_of_branches) :
        a = 2 * math.pi * i / number_of_branches  # Angle between branches
        x = arm_length * math.cos(a)
        Ox = [x, x, x]
        y = arm_length * math.sin(a)
        Oy = [y,y,y]
        wing_angle = 0 # f(a, 0) would set the wings to their default position for a given rpm, but instead should be aligned to the wind...
        angle = [wing_angle, wing_angle, wing_angle] # the wing is not rotating on it's axes
        wing_frames.append(K_2DFrame("Wing "+str(i+1), plateau_frame, Ox, Oy, angle, current_time))
        x = -(wing_width * 2 / 3) - (aileron_width / 3)
        Ax = [x, x, x]
        Ay = [0, 0, 0]
        aileron_frame.append(K_2DFrame("Aileron "+str(i+1), wing_frames[-1], Ax, Ay, angle, current_time))

# Initialize Solids
    ground = Ground(ground_frame)
    turbine_system = MechanicalSystem(ground)

    # Turbine Stator Genertor is fixed to the ground
    turbine_stator = Cylindrical_Solid(stator_frame, core_radius, 1, n_outline_samples)
    anchor = FixedLinkage(turbine_stator, ground, "Fixed", 0, 0)
    turbine_system.AddSolidtoSystem(turbine_stator, ground, anchor)

    # Turbine plateau contains the rotor of the generator.
    turbine_plateau = Turbine_Plateau(plateau_frame,
                                        .1,                   # Mass of a branch
                                        core_radius*1.05,     # Diameter of the internal cage around the generator
                                        arm_length,           # Length of the turbine's arms
                                        number_of_branches)   # Number of Branches
    magnetic_linkage = GeneratorAxleLinkage(turbine_plateau, turbine_stator, "Axial Magnetic")
    turbine_system.AddSolidtoSystem(turbine_plateau, turbine_stator, magnetic_linkage)

    # Add the  wing
    wing = []
    ailerons = []
    for i in range(number_of_branches) :
        wing.append(SimpleWing(wing_frames[i],
                            wing_length,
                            wing_width,     # wing emplanture length
                            wing_chord,    # wing chord
                            wing_mass))     # wing mass
        if i < 6 : wing[i].Color = wing_color[i]
        wing_linkage = FreeAxialLinkage(wing[i], turbine_plateau)
        turbine_system.AddSolidtoSystem(wing[i], turbine_plateau, wing_linkage)  # This should turn into a time history over time

        ailerons.append(Aileron(aileron_frame[i], #Aileron is in the frame
                               wing_length,    # Ailerons have the same length as the wing for now.
                               aileron_width,  # wing emplanture length
                               wing_chord/5,   # wing chord
                               wing_mass/10))  # wing mass
        aileron_servo = Servo(max_servo_speed, max_servo_torque)
        aileron_linkage = FixedLinkage(ailerons[i], wing[i], "Servo", f, aileron_servo)
        turbine_system.AddSolidtoSystem(ailerons[i], wing[i], aileron_linkage)  # This should turn into a time history over time

    return turbine_system, ground_frame