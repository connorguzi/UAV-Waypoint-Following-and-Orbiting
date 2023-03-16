import math
import enum

import sys

sys.path.append("./")  # python is horrible, no?
sys.path.append("..")  # python is horrible, no?
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Utilities.MatrixMath as mm
import ece163.Containers.States as States
from WayPoint import WayPoint
import ece163.FinalModules.Orbiting as Orbiting
import ece163.FinalModules.PathFollowing as PathFollowing

class WaypointStates(enum.Enum):
	"""
	class WaypointStates(enum.Enum):
	Enumeration class for the waypoint following/orbiting state machine. Defines three states that we will be using to reset the PI integrators
	when switching between different states.
	"""
	PATH_FOLLOWING = enum.auto()
	ORBITING = enum.auto()


class WaypointManager():
    def __init__(self, WaypointList=None, k_path = 0.05, k_orbit = 0.05, origin = [[0], [0], [0]]) -> None:
        self.WaypointState = WaypointStates.PATH_FOLLOWING
        self.WaypointList = WaypointList
        if self.WaypointList:
            self.CurrentWaypoint = self.WaypointList[0]
        else:
            self.CurrentWaypoint = WayPoint(0,0,0)
            self.WaypointList = [self.CurrentWaypoint]
        self.elapsedOrbit = 0
        self.dT = VPC.dT
        self.chi_inf = math.pi / 2
        self.k_path = k_path
        self.k_orbit = k_orbit
        self.origin = origin

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

    def InWaypointRadius(self, state: States.vehicleState, waypoint: WayPoint):
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
        return mm.mag(mm.subtract(position, p_waypoint)) <= waypoint.radius

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
        @returns height, course
        """
        # Direction Vector
        q = self.CalcDirectionVector(state=state, waypoint=self.CurrentWaypoint)
        
        if self.WaypointState == WaypointStates.PATH_FOLLOWING:
            # Check to see if the UAV is in the orbit radius
            if(self.InWaypointRadius(state=state, waypoint=self.CurrentWaypoint)):
                self.WaypointState = WaypointStates.ORBITING
                height_command, course_command = Orbiting.getCommandedInputs(state=state, waypoint=self.CurrentWaypoint, k_orbit=self.k_orbit)

            else:
                height_command, course_command = PathFollowing.getCommandedInputs(state=state, origin=self.origin, q=q, chi_inf=self.chi_inf, k_path=self.k_path)
       
        elif self.WaypointState == WaypointStates.ORBITING:
            # See if the orbit has orbited for the set time
            if(self.elapsedOrbit >= self.CurrentWaypoint.time):
                # Set the next waypoint, change state, and reset orbit time
                self.origin = self.CurrentWaypoint.location
                # Make sure that the index does not exceed list length
                i = self.WaypointList.index(self.CurrentWaypoint) + 1
                if i >= len(self.WaypointList):
                    i = 0
                self.CurrentWaypoint = self.WaypointList[i]
                self.WaypointState = WaypointStates.PATH_FOLLOWING
                self.elapsedOrbit = 0
                height_command, course_command = Orbiting.getCommandedInputs(state=state, waypoint=self.CurrentWaypoint, k_orbit=self.k_orbit)
            
            # Increase elapsed time and adjust the commands
            else:
                self.elapsedOrbit += self.dT
                height_command, course_command = Orbiting.getCommandedInputs(state=state, waypoint=self.CurrentWaypoint, k_orbit=self.k_orbit)
            
        return height_command, course_command