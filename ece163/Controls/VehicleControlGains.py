"""
This file implements functions to compute gains from tuning parameters or tuning parameters from gains.

Author: Connor Guzikowski
Email: cguzikow@ucsc.edu

"""

import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations

'''
Function calculates the proper gains based on given tuning parameters.
All equations are taken from lecture notes.
'''
def computeGains(tuningparameters: Controls.controlTuning, linearizedModel:Linearized.transferFunctions):
    kp_roll = tuningparameters.Wn_roll**2 / linearizedModel.a_phi2
    kd_roll = (2 * tuningparameters.Zeta_roll * tuningparameters.Wn_roll - linearizedModel.a_phi1) / linearizedModel.a_phi2
    ki_roll = 1e-3 

    kp_sideslip = (2 * tuningparameters.Zeta_sideslip * tuningparameters.Wn_sideslip - linearizedModel.a_beta1) / linearizedModel.a_beta2
    ki_sideslip = tuningparameters.Wn_sideslip ** 2 / linearizedModel.a_beta2

    kp_course = 2 * tuningparameters.Zeta_course * tuningparameters.Wn_course * linearizedModel.Va_trim / VPC.g0
    ki_course = tuningparameters.Wn_course**2 * linearizedModel.Va_trim / VPC.g0

    kp_pitch = (tuningparameters.Wn_pitch**2 - linearizedModel.a_theta2) / linearizedModel.a_theta3
    kd_pitch = (2 * tuningparameters.Zeta_pitch * tuningparameters.Wn_pitch - linearizedModel.a_theta1) / linearizedModel.a_theta3
    k_pitch_dc = kp_pitch * linearizedModel.a_theta3 / (linearizedModel.a_theta2 + kp_pitch * linearizedModel.a_theta3)

    kp_altitude = 2 * tuningparameters.Zeta_altitude * tuningparameters.Wn_altitude / (k_pitch_dc * linearizedModel.Va_trim)
    ki_altitude = tuningparameters.Wn_altitude ** 2 / (k_pitch_dc * linearizedModel.Va_trim)

    kp_SpeedfromThrottle = (2 * tuningparameters.Zeta_SpeedfromThrottle * tuningparameters.Wn_SpeedfromThrottle - linearizedModel.a_V1) / (linearizedModel.a_V2)
    ki_SpeedfromThrottle = tuningparameters.Wn_SpeedfromThrottle**2 / linearizedModel.a_V2

    kp_SpeedfromElevator = (linearizedModel.a_V1 - (2 * tuningparameters.Zeta_SpeedfromElevator * tuningparameters.Wn_SpeedfromElevator)) / (k_pitch_dc * VPC.g0)
    ki_SpeedfromElevator = -tuningparameters.Wn_SpeedfromElevator ** 2 / (k_pitch_dc * VPC.g0)

    controlGains = Controls.controlGains(kp_roll=kp_roll, kd_roll=kd_roll, ki_roll=ki_roll, kp_sideslip=kp_sideslip, ki_sideslip=ki_sideslip, kp_course=kp_course,
                                        ki_course=ki_course, kp_pitch=kp_pitch, kd_pitch=kd_pitch, kp_altitude=kp_altitude, ki_altitude=ki_altitude, kp_SpeedfromThrottle=kp_SpeedfromThrottle,
                                        ki_SpeedfromThrottle=ki_SpeedfromThrottle, kp_SpeedfromElevator=kp_SpeedfromElevator, ki_SpeedfromElevator=ki_SpeedfromElevator)
    return controlGains

'''
Does opposite of previous function and computes the tuning paramers based on given gains.
Functions taken from lecture notes
'''
def computeTuningParameters(controlGains:Controls.controlGains, linearizedModel: Linearized.transferFunctions):
    try:
        Wn_roll = math.sqrt(controlGains.kp_roll * linearizedModel.a_phi2)
        Zeta_roll = (linearizedModel.a_phi1 + linearizedModel.a_phi2 * controlGains.kd_roll) / (2*Wn_roll)

        Wn_course = math.sqrt(controlGains.ki_course * VPC.g0 / linearizedModel.Va_trim)
        Zeta_course = (controlGains.kp_course * VPC.g0)/ (2 * Wn_course * linearizedModel.Va_trim)

        Wn_sideslip = math.sqrt(linearizedModel.a_beta2 * controlGains.ki_sideslip)
        Zeta_sideslip = (linearizedModel.a_beta1 + linearizedModel.a_beta2 * controlGains.kp_sideslip) / (2 * Wn_sideslip)

        Wn_pitch = math.sqrt(controlGains.kp_pitch * linearizedModel.a_theta3 + linearizedModel.a_theta2)
        Zeta_pitch = (controlGains.kd_pitch * linearizedModel.a_theta3 + linearizedModel.a_theta1) / (2 * Wn_pitch)
        k_pitch_dc = controlGains.kp_pitch * linearizedModel.a_theta3 / (linearizedModel.a_theta2 + controlGains.kp_pitch * linearizedModel.a_theta3)

        Wn_altitude = math.sqrt(controlGains.ki_altitude * k_pitch_dc * linearizedModel.Va_trim)
        Zeta_altitude = (controlGains.kp_altitude * k_pitch_dc * linearizedModel.Va_trim) / (2 * Wn_altitude)

        Wn_SpeedfromThrottle = math.sqrt(linearizedModel.a_V2 * controlGains.ki_SpeedfromThrottle)
        Zeta_SpeedfromThrottle = (controlGains.kp_SpeedfromThrottle * linearizedModel.a_V2 + linearizedModel.a_V1) / (2 * Wn_SpeedfromThrottle)

        Wn_SpeedfromElevator = math.sqrt(-controlGains.ki_SpeedfromElevator * k_pitch_dc * VPC.g0)
        Zeta_SpeedfromElevator = (controlGains.kp_SpeedfromElevator * k_pitch_dc * VPC.g0 - linearizedModel.a_V1) / (-2 * Wn_SpeedfromElevator)

        tuningparameters = Controls.controlTuning(Wn_roll=Wn_roll, Zeta_roll=Zeta_roll, Wn_course=Wn_course, Zeta_course=Zeta_course, Wn_sideslip=Wn_sideslip, 
                                                Zeta_sideslip=Zeta_sideslip, Wn_pitch=Wn_pitch, Zeta_pitch=Zeta_pitch, Wn_altitude=Wn_altitude, Zeta_altitude=Zeta_altitude,
                                                Wn_SpeedfromThrottle=Wn_SpeedfromThrottle, Zeta_SpeedfromThrottle=Zeta_SpeedfromThrottle, Wn_SpeedfromElevator=Wn_SpeedfromElevator,
                                                Zeta_SpeedfromElevator=Zeta_SpeedfromElevator)
        return tuningparameters
    except:
        tuningparameters = Controls.controlTuning()
        return tuningparameters