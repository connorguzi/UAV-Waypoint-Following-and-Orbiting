'''
Author: Bailen Lawson (bjlawson@ucsc.edu), #1682078

The Orbiting module will implement the orbit following method described
in Beard Section 10.2[1]. Specifically, This module will calculate the
command height and command course based on the original position
of the UAV, the center location of the waypoint it is currently
orbiting, and the distance from the center of the way point. 
'''

from copy import deepcopy
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