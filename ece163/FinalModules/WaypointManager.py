import math
import enum

import sys

sys.path.append("./")  # python is horrible, no?
sys.path.append("../")  # python is horrible, no?
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Utilities.MatrixMath as mm
import ece163.Containers.States as States
from ece163.FinalModules.WayPoint import WayPoint
# from ece163.WayPoint import WayPoint
import ece163.FinalModules.Orbiting as Orbiting
import ece163.FinalModules.PathFollowing as PathFollowing

from matplotlib import bezier
from ece163.FinalModules.BezierCurve import normalizeBezierDirection, controlPtsFromWayPts, unpackBezierPosition, getBezierDerivative

class WaypointStates(enum.Enum):
	"""
	class WaypointStates(enum.Enum):
	Enumeration class for the waypoint following/orbiting state machine. Defines three states that we will be using to reset the PI integrators
	when switching between different states.
	"""
	PATH_FOLLOWING = enum.auto()
	ORBITING = enum.auto()


class WaypointManager():
    def __init__(self, WaypointList=None, k_path = 0.05, k_orbit = 0.05, k_s=0.5, d_min=10, origin = [[0], [0], [0]], dt=None) -> None:
        self.WaypointState = WaypointStates.ORBITING
        self.WaypointList = WaypointList
        if self.WaypointList:
            self.CurrentWaypoint = self.WaypointList[0]
        else:
            self.CurrentWaypoint = WayPoint(0,0,0)
            self.WaypointList = [self.CurrentWaypoint]
        self.elapsedOrbit = 0
        if(dt is None):
            self.dT = VPC.dT
        else:
            self.dT = dt
        self.chi_inf = math.pi / 2
        self.k_path = k_path
        self.k_orbit = k_orbit
        self.k_s = k_s
        self.origin = origin
        self.original_origin = origin
        self.d_min = d_min
        self.s = 0
        
        
        # starting_waypoint = WayPoint(origin[0][0], origin[1][0], origin[2][0])
        # control_points = controlPtsFromWayPts(starting_waypoint, self.CurrentWaypoint, 0, self.d_min)
        # # self.bezier_curve = bezier.BezierSegment(starting_waypoint.location)
        # # self.bezier_derivative = getBezierDerivative(self.bezier_curve)

        # self.bezier_curve = bezier.BezierSegment(control_points=control_points)
        # self.bezier_derivative = getBezierDerivative(self.bezier_curve)

    # def CalcDirectionVector(self, state: States.vehicleState, waypoint: WayPoint):
    #     """
    #     Author: Connor Guzikowski (cguzikow)
    #     Date: 03.13.2023        
    #     Calculates the unit direction vector from the UAV to the waypoint
    #     @param: state -> current state of UAV
    #     @param: waypoint -> position of desired waypoint

    #     """
    #     p_waypoint = waypoint.location
    #     position = [[state.pn], [state.pe], [state.pd]]
    #     difference = mm.subtract(p_waypoint, position)
    #     mag = mm.mag(difference)
    #     return mm.scalarDivide(mag, difference)

    def CalcDirectionVector(self, loc1: 'list[list[float]]', loc2: 'list[list[float]]'):
        """
        Author: Connor Guzikowski (cguzikow)
        Date: 03.13.2023        
        Modified: Bailen Lawson 03.16.2023
        Calculates the unit direction vector from loc1 to loc2
        @param: state -> current state of UAV
        @param: waypoint -> position of desired waypoint

        """
        difference = mm.subtract(loc2, loc1)
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

    # def InWaypointRadius(self, state: States.vehicleState, waypoint: WayPoint):
    #     """
    #     Author: Connor Guzikowski (cguzikow)
    #     Date: 03.13.2023
    #     Function to determine whether UAV is in radius of waypoint or not
    #     @param: state -> current state of UAV
    #     @param: waypoint -> position of desired waypoint
    #     @param: radius -> defined radius around the waypoint

    #     """
    #     p_waypoint = waypoint.location
    #     position = [[state.pn], [state.pe], [state.pd]]
    #     return mm.mag(mm.subtract(position, p_waypoint)) <= waypoint.radius

    def InWaypointRadius(self, state: States.vehicleState, waypoint: WayPoint):
        """
        Author: Connor Guzikowski (cguzikow)
        Date: 03.13.2023
        Function to determine whether UAV is in radius of waypoint or not
        @param: state -> current state of UAV
        @param: waypoint -> position of desired waypoint
        @param: radius -> defined radius around the waypoint

        """
        p_waypoint = [[waypoint.location[0][0]], [waypoint.location[1][0]], [0]]
        position = [[state.pn], [state.pe], [0]]
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

    def reset(self):
        self.CurrentWaypoint = self.WaypointList[0]
        self.elapsedOrbit = 0
        self.WaypointState = WaypointStates.PATH_FOLLOWING
        self.origin = self.original_origin
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
        # q = self.CalcDirectionVector(state=state, waypoint=self.CurrentWaypoint)
        q = self.CalcDirectionVector(self.origin, self.CurrentWaypoint.location)
        
        if self.WaypointState == WaypointStates.PATH_FOLLOWING:
            # Check to see if the UAV is in the orbit radius
            if(self.InWaypointRadius(state=state, waypoint=self.CurrentWaypoint)):
                self.WaypointState = WaypointStates.ORBITING
                height_command, course_command = Orbiting.getCommandedInputs(state=state, waypoint=self.CurrentWaypoint, k_orbit=self.k_orbit)

            else:
                new_s = PathFollowing.getPosAlongPath(
                    self.s, self.dT, self.origin, q, self.k_s, state)
                if new_s > self.s:
                    self.s = new_s
                self.origin = unpackBezierPosition(self.bezier_curve(self.s), state.pd)
                q = normalizeBezierDirection(self.bezier_derivative(self.s), state.pd, self.CurrentWaypoint.location[2][0])
                
                height_command, course_command = PathFollowing.getCommandedInputs(state=state, origin=self.origin, q=q, chi_inf=self.chi_inf, k_path=self.k_path)
                height_command = -self.CurrentWaypoint.location[2][0]
       
        elif self.WaypointState == WaypointStates.ORBITING:
            # See if the orbit has orbited for the set time
            if(self.elapsedOrbit >= self.CurrentWaypoint.time):
                # Set the next waypoint, change state, and reset orbit time
                if len(self.WaypointList) != 1:
                    self.origin = self.CurrentWaypoint.location

                # Make sure that the index does not exceed list length
                waypoint1 = self.CurrentWaypoint
                i = self.WaypointList.index(self.CurrentWaypoint) + 1
                if i >= len(self.WaypointList):
                    i = 0
                self.CurrentWaypoint = self.WaypointList[i]
                waypoint2 = self.CurrentWaypoint
                self.WaypointState = WaypointStates.PATH_FOLLOWING
                self.elapsedOrbit = 0
                height_command, course_command = Orbiting.getCommandedInputs(state=state, waypoint=self.CurrentWaypoint, k_orbit=self.k_orbit)

                # calculate control points
                control_points = controlPtsFromWayPts(waypoint1, waypoint2, Orbiting.CalcAngleAlongCircle(state, waypoint1.location), self.d_min)
                self.bezier_curve = bezier.BezierSegment(control_points=control_points)
                self.bezier_derivative = getBezierDerivative(self.bezier_curve)


            
            # Increase elapsed time and adjust the commands
            else:
                self.elapsedOrbit += self.dT
                height_command, course_command = Orbiting.getCommandedInputs(state=state, waypoint=self.CurrentWaypoint, k_orbit=self.k_orbit)
            
        return height_command, course_command