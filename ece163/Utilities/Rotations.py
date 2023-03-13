import sys
sys.path.append("..") #python is horrible, no?
sys.path.append("./")

import math
import ece163.Utilities.MatrixMath as mm


'''
Author: Connor Guzikowski 
Purpose of this file is to create the functions required for Lab0 and provide the basis of rotating sets of points.
These functions are used by the simulator to transform the vertices of the aircraft to model specified rotations of yaw, pitch, and roll.
'''

# Creates the rotation matrix for yaw
def R_Yaw(yaw):
    mat = [[math.cos(yaw), math.sin(yaw), 0],
            [-math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]]
    return mat

# Creates the rotation matrix for pitch
def R_Pitch(pitch):
    mat = [[math.cos(pitch), 0, -math.sin(pitch)],
            [0, 1, 0],
            [math.sin(pitch), 0, math.cos(pitch)]]
    return mat

# Creates the rotation matrix for roll
def R_Roll(roll):
    mat = [[1, 0, 0],
            [0, math.cos(roll), math.sin(roll)],
            [0, -math.sin(roll), math.cos(roll)]]
    return mat

# Generates the DCM from Euler angles
def euler2DCM(yaw=0, pitch=0, roll=0):
    pitch_R = R_Pitch(pitch)
    yaw_R = R_Yaw(yaw)
    roll_R = R_Roll(roll)
    return mm.multiply(mm.multiply(roll_R, pitch_R), yaw_R)

# Generates the Euler angles from DCM
def dcm2Euler(DCM):
    if (DCM[0][2] > 1):
        DCM[0][2] = 1
    elif (DCM[0][2] < -1):
        DCM[0][2] = -1
    pitch = -math.asin(DCM[0][2])
    roll = math.atan2(DCM[1][2], DCM[2][2])
    yaw = math.atan2(DCM[0][1], DCM[0][0])

    return yaw, pitch, roll

# Switches points in NED coordinates to ENU
def ned2enu(points):
    R = [[0, 1,0], [1, 0 ,0], [0, 0, -1]]
    newpoints = []
    for vec in points:
        transposed = [[p] for p in vec]
        newpoints.append(mm.transpose(mm.multiply(R, transposed))[0])
    return newpoints
