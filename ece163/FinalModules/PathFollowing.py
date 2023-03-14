import sys
sys.path.append("./")  # python is horrible, no?
sys.path.append("..")  # python is horrible, no?
sys.path.append("../..")

import ece163.Utilities.MatrixMath as mm
import math
import ece163.Containers.States as States


def CalcPathCourseAngle(q: 'list[list[float]]'):
    """
    Author: Connor Guzikowski (cguzikow)
    Date: 03.13.2023
    Calculate the unit vector normal to the q-k^i plane.
    @param: q -> path direction of the unit vector k^i
    """
    return math.atan2(q[0][0], q[1][0])

def CalcR_Inertial2Path(chi: float):
    """
    Author: Connor Guzikowski (cguzikow)
    Date: 03.13.2023    
    Calculate the rotation matrix from inertial to path
    @param: chi -> path course angle
    """
    return [[math.cos(chi), math.sin(chi), 0.0], [-math.sin(chi), math.cos(chi), 0.0], [0.0,0.0,1.0]]

def CalcRelativePathError(state: States.vehicleState, origin:'list[list[float]]' , R: 'list[list[float]]'):
    """
    Author: Connor Guzikowski (cguzikow)
    Date: 03.13.2023    
    Calculate the Relative Path Error
    @param: state -> current state of UAV
    @param: origin -> position of origin point that the line to waypoint is.
    @param: R -> Rotation matrix inertial to path

    """
    position = [[state.pn], [state.pe], [state.pd]]
    return mm.multiply(R, mm.subtract(position, origin))

def CalcCommandedCourse(q: 'list[list[float]]', origin: 'list[list[float]]', chi_inf: float, k_path: float, state:States.vehicleState):
    """
    Author: Connor Guzikowski (cguzikow)
    Date: 03.13.2023
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
    return chi_q - chi_inf * (2.0 / math.pi) * math.atan(k_path * e[1][0])

def CalcUnitNormalVector(q:'list[list[float]]'):
    """
    Author: Miguel Tamayo (miatamay)
    Date: 03.13.2023
    Calculate the unit vector normal to the q-k^i plane.
    @param: q -> path direction of the unit vector k^i
    """

    ki = [[0], [0], [1]] # vertical unit vector
    cross = mm.crossProduct(q, ki)
    mag = math.hypot(cross[0][0], cross[1][0], cross[2][0])

    return mm.scalarMultiply(1.0/mag, cross)

def CalcProjectedRelativeErrorVector(state:States.vehicleState, n:'list[list[float]]', origin:'list[list[float]]'):
    """
    Author: Miguel Tamayo (miatamay)
    Date: 03.13.2023
    @param: state -> vehicle state
    @param: origin -> vector representing path origin
    @param: R -> rotation matrix from inertial to path frame
    @param: n -> unit normal vector
    @param: origin -> vector distance from origin of path
    """

    # relative error vector in the inertial frame
    epi = [[state.pn - origin[0][0]], [state.pe - origin[1][0]], [state.pd - origin[2][0]]]
    print("EPI: ", epi)
    epi_dot_n = mm.dotProduct(epi, n)[0][0] # epi.n
    print("EPI DOT: ", epi_dot_n)
    epi_n = mm.scalarMultiply(epi_dot_n, n) # (epi.n)n
    
    si = mm.subtract(epi, epi_n) # epi - (epi.n)n
    return si

def CalcCommandedHeight(s, origin, q):
    """
    Author: Miguel Tamayo (miatamay)
    Date: 03.13.2023
    Calculate commanded height

    @param: s -> projection of error vector
    @param: r -> vector form origin to path
    @param: q -> unit vector with path direction
    """
    
    hd = -origin[2][0] + math.hypot(s[0][0], s[1][0]) * (q[2][0] / math.hypot(q[0][0], q[1][0]))

    return hd

def getCommandedInputs(origin:'list[list[float]]', q: 'list[list[float]]',
                       chi_inf:float, k_path:float, state:States.vehicleState):
    """
    wrapper function to return the commanded course and commanded height
    for path following and orbiting
    @param: s -> projection of error vector ep
    @param: r -> vector from origin to path
    @param: q -> path direction of the unit vector k^i
    @param: origin -> position of origin point that the line to waypoint is.
    @param: chi_inf -> gain that indicates what angle the UAV will approach waypoint from afar
    @param: k_path -> gain that indicates how soon the UAV starts turning
    @param: state -> current state of the UAV
    """
    n = CalcUnitNormalVector(q=q)
    s = CalcProjectedRelativeErrorVector(state=state,n=n,origin=origin)
    commandedHeight = CalcCommandedHeight(s=s, origin=origin, q=q)
    commandedCourse = CalcCommandedCourse(q=q, origin=origin, chi_inf=chi_inf, k_kapth=k_path, state=state)

    return commandedHeight, commandedCourse