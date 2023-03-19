"""
Bezier Curve helper functions
"""
import math
import sys

sys.path.append('./')
sys.path.append('../')

from ece163.FinalModules.WayPoint import WayPoint
from ece163.Utilities import MatrixMath as mm
from ece163.Utilities import Rotations
from matplotlib import bezier

def normalizeBezierDirection(coords: 'tuple()', curHeight: float, endHeight: float):
    unnormalizedDirectionVector = [[coords[0]],
                                   [coords[1]], [endHeight - curHeight]]
    normalizedDirectionVector = mm.vectorNorm(unnormalizedDirectionVector)
    return normalizedDirectionVector


def unpackBezierPosition(coords: 'tuple()', curHeight: float):
    ned = [
        [coords[0]],
        [coords[1]],
        [curHeight]
    ]
    return ned


def controlPtsFromWayPts(wp1: WayPoint, wp2: WayPoint, phi1: float, dmin: float):
    """
    Calculates the position of the 4 control points used to define a
    3rd degree Bezier curev to take the UAV from the orbit of
    waypoint 1 to the orbit of waypoint 2
    @param: wp1 -> WayPoint.WayPoint object for start waypoint
    @param: wp2 -> WayPoint.WayPoint object for end waypoint.
    @param: phi1 -> Current orbit angle around waypoint 1
    @param: dmin -> Minimum length between control points
    """
    # phi2 = phi1 if dir1 != dir2, = -phi1 if dir1 == dir2
    phi2 = -1*wp1.direction*wp2.direction*phi1  # Orbit angle around waypoint 2

    # Distance between control points 1 and 2
    d1 = wp1.direction * (dmin + wp1.radius)
    # Distance between control points 3 and 4
    d2 = wp1.direction * (dmin + wp2.radius)
    # Tangent angle to wp1 at current orbit angle
    chi0 = phi1 + wp1.direction * math.pi/2
    # Rotation matrix from inertial to tangent direction
    R = Rotations.euler2DCM(chi0, 0, 0)

    # Calculate control points
    cp1 = [
        wp1.location[0][0] + wp1.radius * math.sin(phi1),
        wp1.location[1][0] + wp1.radius * math.cos(phi1)
    ]

    cp4 = [
        wp2.location[0][0] + wp2.radius * math.sin(phi2),
        wp2.location[1][0] + wp2.radius * math.cos(phi2)
    ]

    cp2_ext = mm.multiply(R, [[wp1.direction * d1], [0], [0]])
    cp2 = [
        cp1[0] + cp2_ext[0][0],
        cp1[1] + cp2_ext[1][0]
    ]

    cp3_ext = mm.multiply(R, [[wp1.direction * d2], [0], [0]])
    cp3 = [
        cp4[0] + cp3_ext[0][0],
        cp4[1] + cp3_ext[1][0]
    ]

    return [cp1, cp2, cp3, cp4]

def getBezierDerivative(curve: bezier.BezierSegment):
    """
    Calculates the derivative of the bezier curve as a new bezier
    curve
    @param: curve -> Bezier cirve to find the derivative of
    """
    controlPoints = curve.control_points
    n = curve.degree
    controlPoints_derivative = [
        [n * (controlPoints[i+1][0] - controlPoints[i][0]),
         n * (controlPoints[i+1][1] - controlPoints[i][1])]
        for i in range(n)
    ]
    curveDerivative = bezier.BezierSegment(controlPoints_derivative)
    return curveDerivative