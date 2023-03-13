# CalcPathCourseAngle
# CalcRotationMatrix_Inertial2Path
# CalcRelativePathError
# CalcCommandedCourse
# getCommandedInputs

import ece163.Utilities.MatrixMath as mm
import math
import ece163.Containers.States as States


def CalcPathCourseAngle(q):
    return math.atan2(q[0][0], q[1][0])

def CalcR_Inertial2Path(chi):
    return [[math.cos(chi), math.sin(chi), 0], [-math.sin(chi), math.cos(chi), 0], [0,0,1]]

def CalcRelativePathError(state: States.vehicleState, origin, R):
    position = [[state.pn], [state.pe], [state.pd]]
    return mm.multiply(R, mm.subtract(position, origin))

def CalcCommandedCourse(q,origin,chi_inf,k_path,state:States.vehicleState):
    chi_q = CalcPathCourseAngle(q)
    R = CalcR_Inertial2Path(chi_q)
    e = CalcRelativePathError(state=state, origin=origin, R=R)
    return chi_q - chi_inf * (2 / math.pi) * math.atan(k_path * e)