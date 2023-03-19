"""
widget which handles the creation of linear models and determining the gains
"""

import PyQt5.QtWidgets as QtWidgets
from .SliderWithValue import SliderWithValue
from ..Constants import VehiclePhysicalConstants as VPC
from ..Containers.Controls import referenceCommands
from ..Containers.States import vehicleState
import math
from ..FinalModules.WaypointManager import WaypointManager
import ece163.FinalModules.WayPoint as WayPoint
# Airspeed, Altitude, Course
origin = [[VPC.InitialNorthPosition], [VPC.InitialEastPosition], [VPC.InitialDownPosition]] # world origin

waypoint1 = WayPoint.WayPoint(
    n=100,
    e=0,
    d=-100,
    radius=25,
    direction=1,
    time=400
)
waypoint2 = WayPoint.WayPoint(
    n=200,
    e=200,
    d=-150,
    radius=25,
    direction=-1,
    time=400
)
waypoint3=WayPoint.WayPoint(
    n=30,
    e=300,
    d=-200,
    radius=25,
    direction=1,
    time=400
)

# orbit and path following gains
k_orbit = 1
k_path = 0.01 # how fast we transition into the path

WpList = [waypoint1, waypoint2, waypoint3]

class WaypointControl(QtWidgets.QWidget):
    def __init__(self, callBackOnChange=None, parent=None, state = vehicleState()):
        super().__init__()
        # Change airspeed to accept pickle file for trim conditions
        self.airspeed = 25
        # self.callBack = callBackOnChange
        # self.usedLayout = QtWidgets.QVBoxLayout()
        # self.setLayout(self.usedLayout)
        self.currentReference = referenceCommands()

        # self.airSpeedInput = SliderWithValue(
        #     "Airspeed", 20, 50, VehiclePhysicalConstants.InitialSpeed, self.referenceChanged)
        # self.usedLayout.addWidget(self.airSpeedInput)

        # self.altitudeInput = SliderWithValue(
        #     'Altitude', 50, 250, -VehiclePhysicalConstants.InitialDownPosition, self.referenceChanged)
        # self.usedLayout.addWidget(self.altitudeInput)

        # self.courseInput = SliderWithValue(
        #     'Course', -180, 180, VehiclePhysicalConstants.InitialYawAngle, self.referenceChanged)
        # self.usedLayout.addWidget(self.courseInput)

        self.WM = WaypointManager(k_orbit=k_orbit, k_path=k_path, origin=origin, WaypointList=WpList, dt=0.1)

        self.buildCurrentReferences(state=state)
        # self.usedLayout.addStretch()
        return

    def reset(self):
        self.WM.reset()
        return

    def referenceChanged(self, unusedValue, unusedName):
        self.buildCurrentReferences()
        return

    def buildCurrentReferences(self, state:vehicleState):
        try:
            altitudeCommand, courseCommand = self.WM.Update(state)
            airspeedCommand = self.airspeed
            self.currentReference = referenceCommands(
                courseCommand, altitudeCommand, airspeedCommand)
        except AttributeError:  # this stops weird loading bug due to referencing it before it exists
            pass
