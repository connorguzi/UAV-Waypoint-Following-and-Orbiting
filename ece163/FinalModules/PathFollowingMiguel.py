"""
Path Following functions (Miguel's part)
Date: 03.13.2022
"""

from ece163.Utilities import MatrixMath as mm
from FinalModules.PathFollowingConnor import CalcRelativePathError
import math

def CalcUnitNormalVector(q:list[list[float]]):
    """
    Calculate the unit vector normal to the q-k^i plane.
    @param: q -> path direction of the unit vector k^i
    """

    ki = [[0], [0], [1]] # vertical unit vector
    cross = mm.crossProduct(q, ki)
    mag = math.hypot(ki[0][0], ki[1][0], ki[2][0])

    return mm.scalarMultiply(1.0/mag, cross)

def CalcProjectedRelativeErrorVector(state, origin, R, n:list[list[float]], r:list[list[float]]):
    """
    @param: state -> vehicle state
    @param: origin -> vector representing path origin
    @param: R ->
    """

    ep = CalcRelativePathError()

    pass

def CalcCommandedHeight():
    """
    Calculate
    """
    pass
