"""
Author: Miguel Taamayo (miatamay)
Date: 03.13.2023

Waypoint module used to keep attributes of waypoints that
will be visited by the UAV
"""

class WayPoint():
    def __init__(self, n:float=0.0, e:float=0.0, d:float=0.0, time:float=1.0) -> None:
        """
        WayPoint module with NED location
        @param: n -> North point
        @param: e -> East point
        @param: d -> Down point (altitude)
        @param: time -> time to circle around the point for
        """

        self.pos = [[n], [e], [d]] # lcoation of the waypoint as NED coordinates
        self.time = time # time to orbit around the waypoint
        pass