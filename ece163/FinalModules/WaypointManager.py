import math
import enum

import sys

sys.path.append("./")  # python is horrible, no?
sys.path.append("..")  # python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Containers.States as States

class WaypointStates(enum.Enum):
	"""
	class WaypointStates(enum.Enum):
	Enumeration class for the waypoint following/orbiting state machine. Defines three states that we will be using to reset the PI integrators
	when switching between different states.
	"""
	TRANSITIONING = enum.auto()
	PATH_FOLLOWING = enum.auto()
	ORBITING = enum.auto()

def CalcDirectionVector(state: States.vehicleState, waypoint):
    """
    Calculates the unit direction vector from the UAV to the waypoint
    @param: state -> current state of UAV
    @param: waypoint -> position of desired waypoint

    """
    position = [[state.pn], [state.pe], [state.pd]]
    difference = mm.subtract(waypoint, position)
    mag = mm.mag(difference)
    return mm.scalarDivide(mag, difference)

def CalcDirectionVectorTest(position: 'list[list[float]]', waypoint: 'list[list[float]]'):
    """
    Calculates the unit direction vector from the UAV to the waypoint
    @param: state -> current state of UAV
    @param: waypoint -> position of desired waypoint

    """
    difference = mm.subtract(waypoint, position)
    mag = math.hypot(difference[0][0], difference[1][0])
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

