import ece163.Utilities.MatrixMath as mm
import math
import ece163.Containers.States as States

def CalcDirectionVector(state: States.vehicleState, waypoint):
    """
    Calculates the unit direction vector from the UAV to the waypoint
    @param: state -> current state of UAV
    @param: waypoint -> position of desired waypoint

    """
    position = [[state.pn], [state.pe], [state.pd]]
    difference = mm.subtract(position, waypoint)
    mag = mm.mag(difference)
    return mm.scalarDivide(mag, difference)

def InWaypointRadius(state: States.vehicleState, waypoint, radius):
    """
    Function to determine whether UAV is in radius of waypoint or not
    @param: state -> current state of UAV
    @param: waypoint -> position of desired waypoint
    @param: radius -> defined radius around the waypoint

    """
    position = [[state.pn], [state.pe], [state.pd]]
    return mm.mag(mm.subtract(position, waypoint)) <= radius

