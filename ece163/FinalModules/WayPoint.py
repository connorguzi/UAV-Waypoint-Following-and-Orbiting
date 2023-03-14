"""
Author: Miguel Taamayo (miatamay)
Date: 03.13.2023

Waypoint module used to keep attributes of waypoints that
will be visited by the UAV
"""
import sys
import math

sys.path.append("./")  # python is horrible, no?
sys.path.append("..")  # python is horrible, no?
import ece163.Containers.States as States

class WayPoint():
    def __init__(self, n:float=0.0, e:float=0.0, d:float=0.0, radius:float=10.0, direction:float=1, time:float=1.0) -> None:
        """
        WayPoint module with NED location
        @param: n -> North point
        @param: e -> East point
        @param: d -> Down point (altitude)
        @param: direction -> direction of orbit (1 CW, -1 CCW)
        @param: time -> time to circle around the point for
        """

        self.location = [[n], [e], [d]] # lcoation of the waypoint as NED coordinates
        self.time = time # time to orbit around the waypoint
        self.radius = radius # desired distance from the waypoint center
        self.distance = 0.0 # holder for the distance from UAV to waypoint
        self.direction = direction # direction of orbit (1 CW, -1 CCW)
        pass

    def getPointTime(self):
        """
        Return the time to orbit around the point
        @return: time
        """

        return self.time
    
    def getPointLocation(self):
        """
        Return the location of the waypoint
        @return: location -> 3x1 array
        """
        return self.location
    
    def CalcPointDistance(self, state:States.vehicleState):
        """
        Updates and returns self.distance which is the distance from the waypoint to the UAV.
        This is the horizontal distance (doesn't include altitude difference).
        @param state -> UAV state
        """

        self.distance = math.hypot((self.n - state.pn), (self.e - state.pe))

        return self.distance
