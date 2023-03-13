'''
Author: Bailen Lawson (bjlawson@ucsc.edu), #1682078

The Orbiting module will implement the orbit following method described
in Beard Section 10.2[1]. Specifically, This module will calculate the
command height and command course based on the original position
of the UAV, the center location of the waypoint it is currently
orbiting, and the distance from the center of the way point. 
'''

import math
from . import MatrixMath

def CalcDistFromCenter(state, center):
    """
    Calculates the UAV's distance from the center of the orbit which is
    the way point location.

    Parameters:
    {state}     Vehicle state
    {center}    3x1 matrix for orbit center coordinates

    Returns:
    {d} Vehicle distance from orbit center
    """
    a = (state.pn - center[0][0])**2
    b = (state.pe - center[1][0])**2
    d = math.sqrt(a + b)
    return d

def CalcAngleAlongCircle(state, center):
    """
    Calculates the angle along the orbit circle the UAV's position in
    the inertial frame, and the orbit circles center point in the
    inertial frame.

    Parameters:
    {state}     Vehicle state
    {center}    3x1 matrix for orbit center coordinates

    Returns:
    {phi_orbit} Angle along the orbit circle
    """
    phi_orbit = math.atan2(
        state.pe - center[1][0],
        state.pn - center[0][0]
    )

    # Keep -pi <= phi_chi <= pi
    while (phi_orbit - state.chi) < -math.pi:
        phi_orbit += 2*math.pi
    while (phi_orbit - state.chi) > math.pi:
        phi_orbit -= 2*math.pi

    return phi_orbit

def CalcCommandedHeight(center):
    """
    Calculates the commanded height based on the orbit center's height.

    Parameters:
    {center}    3x1 matrix for orbit center coordinates

    Returns:
    {h_c}   Commanded height
    """
    return -center[2][0]

def CalcCommandedCourse(state, center, dir, rho, k_orbit):
    """
    Calculates the commanded course based on the orbit center's
    height, vehicle state, orbit direction, orbit circle radius, and
    transition gain.

    Parameters:
    {state}     Vehicle State
    {center}    3x1 matrix for orbit center coordinates
    {dir}       Orbit direction
    {rho}       Desired distance from orbit center
    {k_orbit}   Transition gain

    Returns:
    {chi_c} Commanded course
    """
    chi_orbit = CalcAngleAlongCircle(state, center)
    d = CalcDistFromCenter(state, center)
    chi_c = chi_orbit + dir * ((math.pi/2) + math.atan(k_orbit * (d - rho) / rho))
    return chi_c

def getCommandedInputs(state, center, dir, rho, k_orbit):
    """
    Calculates the commanded course and commanded height based on the
    orbit center's height, vehicle state, orbit direction, orbit
    circle radius, and transition gain.

    Parameters:
    {state}     Vehicle State
    {center}    3x1 matrix for orbit center coordinates
    {dir}       Orbit direction
    {rho}       Desired distance from orbit center
    {k_orbit}   Transition gain

    Returns:
    {h_c}   Commanded height
    {chi_c} Commanded course
    """
    h_c = CalcCommandedHeight(center)
    chi_c = CalcCommandedCourse(state, center, dir, rho, k_orbit)
    return h_c, chi_c