"""
widget which handles the creation of linear models and determining the gains
"""

import PyQt5.QtWidgets as QtWidgets
from .SliderWithValue import SliderWithValue
from ..Constants import VehiclePhysicalConstants
from ..Containers.Controls import referenceCommands
import math
from ..FinalModules.WaypointManager import WaypointManager
# Airspeed, Altitude, Course


class WaypointControl(QtWidgets.QWidget):
    def __init__(self, callBackOnChange=None, parent=None):
        super().__init__()
        # Change airspeed to accept pickle file for trim conditions
        self.airspeed = 25
        self.callBack = callBackOnChange
        self.usedLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.usedLayout)
        self.currentReference = referenceCommands()

        self.airSpeedInput = SliderWithValue(
            "Airspeed", 20, 50, VehiclePhysicalConstants.InitialSpeed, self.referenceChanged)
        self.usedLayout.addWidget(self.airSpeedInput)

        self.altitudeInput = SliderWithValue(
            'Altitude', 50, 250, -VehiclePhysicalConstants.InitialDownPosition, self.referenceChanged)
        self.usedLayout.addWidget(self.altitudeInput)

        self.courseInput = SliderWithValue(
            'Course', -180, 180, VehiclePhysicalConstants.InitialYawAngle, self.referenceChanged)
        self.usedLayout.addWidget(self.courseInput)
        self.WM = WaypointManager()

        self.buildCurrentReferences()
        self.usedLayout.addStretch()
        return

    def referenceChanged(self, unusedValue, unusedName):
        self.buildCurrentReferences()
        return

    def buildCurrentReferences(self):
        try:
            altitudeCommand, courseCommand = self.WM.Update()
            courseCommand = math.radians(self.courseInput.curValue)
            altitudeCommand = self.altitudeInput.curValue
            airspeedCommand = self.airSpeedInput
            self.currentReference = referenceCommands(
                courseCommand, altitudeCommand, airspeedCommand)
        except AttributeError:  # this stops weird loading bug due to referencing it before it exists
            pass
