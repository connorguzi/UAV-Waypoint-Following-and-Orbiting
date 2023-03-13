# CalcPathCourseAngle
# CalcRotationMatrix_Inertial2Path
# CalcRelativePathError
# CalcCommandedCourse
# getCommandedInputs

import ece163.Utilities.MatrixMath as mm
import math
import ece163.Containers.States as States


def CalcPathCourseAngle(q: 'list[list[float]]'):
    """
    Calculate the unit vector normal to the q-k^i plane.
    @param: q -> path direction of the unit vector k^i
    """
    return math.atan2(q[0][0], q[1][0])

def CalcR_Inertial2Path(chi: float):
    """
    Calculate the rotation matrix from inertial to path
    @param: chi -> path course angle
    """
    return [[math.cos(chi), math.sin(chi), 0], [-math.sin(chi), math.cos(chi), 0], [0,0,1]]

def CalcRelativePathError(state: States.vehicleState, origin:'list[list[float]]' , R: 'list[list[float]]'):
    """
    Calculate the Relative Path Error
    @param: state -> current state of UAV
    @param: origin -> position of origin point that the line to waypoint is.
    @param: R -> Rotation matrix inertial to path

    """
    position = [[state.pn], [state.pe], [state.pd]]
    return mm.multiply(R, mm.subtract(position, origin))

def CalcCommandedCourse(q: 'list[list[float]]', origin: 'list[list[float]]', chi_inf: float, k_path: float, state:States.vehicleState):
    """
    Calculate the Commanded Course, uses previous functions
    @param: q -> path direction of the unit vector k^i
    @param: origin -> position of origin point that the line to waypoint is.
    @param: chi_inf -> gain that indicates what angle the UAV will approach waypoint from afar
    @param: k_path -> gain that indicates how soon the UAV starts turning
    @param: state -> current state of the UAV

    """
    chi_q = CalcPathCourseAngle(q)
    R = CalcR_Inertial2Path(chi_q)
    e = CalcRelativePathError(state=state, origin=origin, R=R)
    return chi_q - chi_inf * (2 / math.pi) * math.atan(k_path * e)