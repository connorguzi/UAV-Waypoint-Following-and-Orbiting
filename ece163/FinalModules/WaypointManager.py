import math
import enum

import sys

sys.path.append("./")  # python is horrible, no?
sys.path.append("..")  # python is horrible, no?
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Utilities.MatrixMath as mm
import ece163.Containers.States as States
from WayPoint import WayPoint
class WaypointStates(enum.Enum):
	"""
	class WaypointStates(enum.Enum):
	Enumeration class for the waypoint following/orbiting state machine. Defines three states that we will be using to reset the PI integrators
	when switching between different states.
	"""
	TRANSITIONING = enum.auto()
	PATH_FOLLOWING = enum.auto()
	ORBITING = enum.auto()


class WaypointManager():
    def __init__(self, WaypointList) -> None:
        self.WaypointState = WaypointStates.PATH_FOLLOWING
        self.WaypointList = WaypointList
        if self.WaypointList:
            self.CurrentWaypoint = self.WaypointList[0]
        pass
        self.elapsedOrbit = 0
        self.dT = VPC.dT

    def CalcDirectionVector(self, state: States.vehicleState, waypoint: WayPoint):
        """
        Author: Connor Guzikowski (cguzikow)
        Date: 03.13.2023        
        Calculates the unit direction vector from the UAV to the waypoint
        @param: state -> current state of UAV
        @param: waypoint -> position of desired waypoint

        """
        p_waypoint = waypoint.location
        position = [[state.pn], [state.pe], [state.pd]]
        difference = mm.subtract(p_waypoint, position)
        mag = mm.mag(difference)
        return mm.scalarDivide(mag, difference)

    def CalcDirectionVectorTest(self, position: 'list[list[float]]', waypoint: 'list[list[float]]'):
        """
        Author: Connor Guzikowski (cguzikow)
        Date: 03.13.2023
        Calculates the unit direction vector from the UAV to the waypoint
        @param: state -> current state of UAV
        @param: waypoint -> position of desired waypoint

        """
        difference = mm.subtract(waypoint, position)
        mag = math.hypot(difference[0][0], difference[1][0])
        return mm.scalarDivide(mag, difference)

    def InWaypointRadius(self, state: States.vehicleState, waypoint: WayPoint, radius: float):
        """
        Author: Connor Guzikowski (cguzikow)
        Date: 03.13.2023
        Function to determine whether UAV is in radius of waypoint or not
        @param: state -> current state of UAV
        @param: waypoint -> position of desired waypoint
        @param: radius -> defined radius around the waypoint

        """
        p_waypoint = waypoint.location
        position = [[state.pn], [state.pe], [state.pd]]
        return mm.mag(mm.subtract(position, p_waypoint)) <= radius

    def SetWaypointList(self, waypoints:'list[WayPoint]'):
        """
        Author: Connor Guzikowski (cguzikow)
        Date: 03.14.2023
        Function to set the waypoint list attribute
        @param: waypoints -> list of the waypoints to orbit to 

        """
        self.WaypointList = waypoints
        self.CurrentWaypoint = self.WaypointList[0]
        return

    def Update(self, state: States.vehicleState):
        """
        Author: Connor Guzikowski (cguzikow)
        Date: 03.14.2023
        Function to run the overall state machine for the path planning and orbiting.
        @param: state -> current state of UAV
        @returns course, height
        """
        position = [[state.pn], [state.pe], [state.pd]]
        p_waypoint = self.CurrentWaypoint.location
        if self.WaypointState == WaypointStates.PATH_FOLLOWING:
            if(self.InWaypointRadius(state=state, waypoint=self.CurrentWaypoint)):
                self.WaypointState = WaypointStates.ORBITING
            pass
        elif self.WaypointState == WaypointStates.ORBITING:
            if(self.elapsedOrbit >= self.CurrentWaypoint.time):
                self.WaypointState = WaypointStates.TRANSITIONING
            pass
        else:
            self.CurrentWaypoint = self.WaypointList[self.WaypointList.index(self.CurrentWaypoint) + 1]
            self.WaypointState = WaypointStates.ORBITING
            pass
        return 