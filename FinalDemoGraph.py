import sys
import WayPoint
import Orbiting
import PathFollowing
import WaypointManager

sys.path.append("./")  # python is horrible, no?
sys.path.append("..")  # python is horrible, no?

import os
import math
import copy
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations

import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Modeling.WindModel as WM
import ece163.Modeling.VehicleAerodynamicsModel as VAM

import ece163.Controls.VehicleTrim as VehicleTrim
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Controls.VehicleControlGains as VCG
import ece163.Controls.VehicleClosedLoopControl as VCLC

import ece163.Sensors.SensorsModel as SM

import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Containers.Linearized as Linearized
import ece163.Containers.Controls as Controls
import ece163.Containers.Sensors as Sensors

import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Constants.VehicleSensorConstants as VSC

import ece163.FinalModules.WayPoint as WayPoint
import ece163.FinalModules.WaypointManager as WaypointManager

##### GAINS #####
gains = Controls.controlGains() # UAV gains object
gains.kp_roll = 3.247869110823198
gains.kd_roll = 0.6979579782269094
gains.ki_roll = 0.001
gains.kp_sideslip = 0.6453323815752218
gains.ki_sideslip = 0.010000000000000472
gains.kp_course = 1.8867394278316714
gains.ki_course = 0.04394797121647837
gains.kp_pitch = -9.285321126158204
gains.kd_pitch = -0.3270270972286049
gains.kp_altitude = 0.03963394749037365
gains.ki_altitude = 0.01639071806938184
gains.kp_SpeedfromThrottle = 0.020000011427928222
gains.ki_SpeedfromThrottle = 0.017583562590727155
gains.kp_SpeedfromElevator = -0.1986227880728017
gains.ki_SpeedfromElevator = -0.0019862278473563194

##### WAYPOINT MANAGER #####
origin = [[10], [25], [-100]] # world origin

waypoint1 = WayPoint.WayPoint(
    n=0,
    e=0,
    d=-100,
    radius=50,
    direction=1,
    time=100
)
waypoint2 = WayPoint.WayPoint(
    n=300,
    e=0,
    d=-300,
    radius=100,
    direction=-1,
    time=50
)
waypoint3=WayPoint.WayPoint(
    n=0,
    e=300,
    d=-200,
    radius=150,
    direction=1,
    time=150
)

# orbit and path following gains
k_orbit = 1
k_path = 0.01 # how fast we transition into the path

WpList = [waypoint1, waypoint2, waypoint3]
WM = WaypointManager.WaypointManager(origin=origin, WaypointList=WpList, k_orbit=k_orbit, k_path=k_path)

##### VEHICLE SETUP #####
Va = 20 # airspeed

vclc = VCLC.VehicleClosedLoopControl() # vehicle PID controller
vclc.setControlGains(gains) # set the gains in the UAV
vclc.setVehicleState(States.vehicleState( # initial vehicle state
    pn=origin[0][0],
    pe=origin[1][0],
    pd=[2][0],
    u=Va
))

##### SIMULATION SETUP ###
# time step dT
dT = vclc.getVehicleAerodynamicsModel().getVehicleDynamicsModel().dT
totalTime = 240 # total simulation time [s]
n_steps = int(totalTime / dT) # simulation steps
t_data = [i*dT for i in range(n_steps)] # time array

chi_commanded = [0 for i in range(n_steps)] # commanded course
chi_true = [0 for i in range(n_steps)] # true course
chi_error = [0 for i in range(n_steps)] # course error

height_commanded = [0 for i in range(n_steps)] # commanded height
height_true = [0 for i in range(n_steps)] # UAV height
height_error = [0 for i in range(n_steps)] # height error

n = [0 for i in range(n_steps)] # north coordinate
e = [0 for i in range(n_steps)] # east coordinate
d = [0 for i in range(n_steps)] # down coordinate

##### SIMULATE #####
for i in range(n_steps):
    # Update the reference commands
    height_commanded[i], chi_commanded[i] = WM.Update(state=vclc.getVehicleState())
    controls = Controls.referenceCommands(
        courseCommand = chi_commanded[i],
        altitudeCommand=height_commanded[i],
        airspeedCommand=Va
    )

    # udpate the UAV state








