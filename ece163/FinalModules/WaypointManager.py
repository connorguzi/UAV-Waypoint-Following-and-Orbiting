import ece163.Utilities.MatrixMath as mm
import math
import ece163.Containers.States as States

def CalcDirectionVector(state: States.vehicleState, waypoint):
    position = [[state.pn], [state.pe], [state.pd]]
    difference = mm.subtract(position, waypoint)
    mag = mm.mag(difference)
    return mm.scalarDivide(mag, difference)

def InWaypointRadius(state: States.vehicleState, waypoint, radius):
    position = [[state.pn], [state.pe], [state.pd]]
    return mm.mag(mm.subtract(position, waypoint)) <= radius

