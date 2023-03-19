import sys
import ece163.FinalModules.WayPoint as WayPoint
import ece163.FinalModules.Orbiting as Orbiting
import ece163.FinalModules.WaypointManager as WaypointManager
# import Orbiting
# import PathFollowing
# import WaypointManager

sys.path.append("./")  # python is horrible, no?
sys.path.append("..")  # python is horrible, no?

import os
import math
import copy
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
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
from ece163.FinalModules.BezierCurve import normalizeBezierDirection, controlPtsFromWayPts, unpackBezierPosition

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
    n=10,
    e=200,
    d=-150,
    radius=75,
    direction=1,
    time=25
)
waypoint2 = WayPoint.WayPoint(
    n=200,
    e=200,
    d=-200,
    radius=150,
    direction=-1,
    time=75
)
waypoint3=WayPoint.WayPoint(
    n=200,
    e=10,
    d=-125,
    radius=50,
    direction=1,
    time=50
)
waypoint4=WayPoint.WayPoint(
    n=10,
    e=25,
    d=-100,
    radius=100,
    direction=-1,
    time=75
)

# orbit and path following gains
k_orbit = 1
k_path = 0.05 # how fast we transition into the path
k_s = 0.5 # bezier curve gain
d_min = 50

WpList = [waypoint1, waypoint2, waypoint3, waypoint4]
WM = WaypointManager.WaypointManager(origin=origin, WaypointList=WpList, k_orbit=k_orbit, k_path=k_path, k_s=k_s, d_min=d_min)

##### VEHICLE SETUP #####
Va = 25.0 # airspeed

vclc = VCLC.VehicleClosedLoopControl() # vehicle PID controller

# TRIM
trim = VehicleTrim.VehicleTrim()
Vastar = 25.0
Gammastar = 0.0
Kappastar = 0.0
trim.computeTrim(Vastar=Vastar, Gammastar=Gammastar, Kappastar=Kappastar)

vclc.setControlGains(gains) # set the gains in the UAV
vclc.setTrimInputs(trim.getTrimControls())
vState = trim.getTrimState()
vState.pn = origin[0][0]
vState.pe = origin[1][0]
vState.pd = origin[2][0]
vclc.setVehicleState(vState)

##### WAYPOINT PLOTTING #####

##### SIMULATION SETUP ###
# time step dT
dT = vclc.getVehicleAerodynamicsModel().getVehicleDynamicsModel().dT
totalTime = 50 # total simulation time [s]
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
u = [0 for i in range(n_steps)] # down coordinate

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
    vclc.Update(controls)
    chi_true[i] = vclc.getVehicleState().chi # measure course
    height_true[i] = -vclc.getVehicleState().pd # measure the height

    # measure the error
    chi_error[i] = chi_true[i] - chi_commanded[i]
    while(chi_error[i] < math.pi):
        chi_error[i] += 2*math.pi
    while(chi_error[i] > math.pi):
        chi_error[i] -= 2*math.pi

    chi_error[i] = math.degrees(chi_error[i])
    height_error[i] = height_true[i] - height_commanded[i]

    # convert to enu coordinates
    enu_pos =Rotations.ned2enu([[
        vclc.getVehicleState().pn,
        vclc.getVehicleState().pe,
        vclc.getVehicleState().pd
    ]])

    n[i] = enu_pos[0][0]
    e[i] = enu_pos[0][1]
    u[i] = enu_pos[0][2]

    # ax.plot3D(n[i], e[i], u[i]) # plot the current point


##### PLOTTING #####
fig = plt.figure(tight_layout =True)
ax = fig.add_subplot(1,2,1, projection='3d')
wp1 = Rotations.ned2enu([[waypoint1.location[0][0], waypoint1.location[1][0], waypoint1.location[2][0]]])
wp2 = Rotations.ned2enu([[waypoint2.location[0][0], waypoint2.location[1][0], waypoint2.location[2][0]]])
wp3 = Rotations.ned2enu([[waypoint3.location[0][0], waypoint3.location[1][0], waypoint3.location[2][0]]])
wp4 = Rotations.ned2enu([[waypoint4.location[0][0], waypoint4.location[1][0], waypoint4.location[2][0]]])
ogn = Rotations.ned2enu([[origin[0][0], origin[1][0], origin[2][0]]])

ax.plot3D(wp1[0][0], wp1[0][1], wp1[0][2], marker="o", markersize=5, color='r')
ax.plot3D(wp2[0][0], wp2[0][1], wp2[0][2],  marker="o", markersize=5, color='y')
ax.plot3D(wp3[0][0], wp3[0][1], wp3[0][2],  marker="o", markersize=5, color='k')
ax.plot3D(wp4[0][0], wp4[0][1], wp4[0][2],  marker="o", markersize=5, color='c')
ax.plot3D(ogn[0][0], ogn[0][1], ogn[0][2], marker="x", markersize=5, color='g')
ax.plot3D(n, e, u);
# fig1 = plt.figure()
# ax1 = plt.axes(projection='3d')
# # fig1, ax1 = plt.subplots(projec)
# def animate(i):
#     """
#     Animation Function
#     """
#     # grab the points
#     # ax1.clear()
#     this_n = n[i]
#     this_e = e[i]
#     this_u = u[i]
#     ax1.plot3D(this_n, this_e, this_u, 'r.')
#     # plt.show()

# animation = FuncAnimation(fig=fig1, func=animate, frames=n_steps, repeat=False, interval=0.00001)

ax.set_title("UAV Position [ENU]")
ax.set_xlabel("N [m]")
ax.set_ylabel("E [m]")
ax.set_zlabel("U [m]")

ax = fig.add_subplot(2,2,2)
ax.plot(t_data, chi_error)
ax.set_title(" ")
ax.set_xlabel("t [s]")
ax.set_ylabel("Course Error [deg]")

ax = fig.add_subplot(2,2,4)
ax.plot(t_data, height_error)
ax.set_title(" ")
ax.set_xlabel("t [s]")
ax.set_ylabel("Altitude Error [m]")

plt.show()










