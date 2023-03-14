"""
Author: Miguel Taamayo (miatamay)
Date: 03.13.2023

Waypoint module used to keep attributes of waypoints that
will be visited by the UAV
"""
import ece163.Containers.States as States
import math

class WayPoint():
    def __init__(self, n:float=0.0, e:float=0.0, d:float=0.0, time:float=1.0) -> None:
        """
        WayPoint module with NED location
        @param: n -> North point
        @param: e -> East point
        @param: d -> Down point (altitude)
        @param: time -> time to circle around the point for
        """

        self.location = [[n], [e], [d]] # lcoation of the waypoint as NED coordinates
        self.time = time # time to orbit around the waypoint
        self.distance = 0.0 # holder for the distance from UAV to waypoint
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
