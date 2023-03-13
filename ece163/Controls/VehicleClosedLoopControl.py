"""
This file implements all of the basic closed loop control for the autopilot. There are classes for PI, PD, and PID controllers,
as well as the overall class for the closed loop control

Author: Connor Guzikowski
Email: cguzikow@ucsc.edu

"""


import math
import sys
import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VAM
import ece163.Containers.States as States

class PDControl():
    # Initialize controller
    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0) -> None:
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return
    
    # Set the corresponding gains
    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return
    
    def Update(self, command = 0.0, current = 0.0, derivative=0.0):
        #   Calculate error and u
        error = command - current
        u = self.kp * error - self.kd * derivative + self.trim

        # If at saturation, set to limit
        if u > self.highLimit:
            u = self.highLimit
        elif u < self.lowLimit:
            u = self.lowLimit

        return u
    
class PIControl():
    # Initializing controller
    def __init__(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0) -> None:
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.dT = dT
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0
        self.prevError = 0.0
        return
    
    # Set corresponding gains
    def setPIGains(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.dT = dT
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0
        self.prevError = 0.0
        return
    
    def resetIntegrator(self):
        self.accumulator = 0.0
        return
    
    def Update(self, command=0.0, current=0.0):
        # Calculate error, accumulator, and u
        error = command - current
        self.accumulator += 1/2 * (error + self.prevError) * self.dT
        u = self.kp * error + self.ki * self.accumulator + self.trim
        
        # If at saturation, set to limit and then undo the increment of the accumulator
        if u > self.highLimit:
            u = self.highLimit
            self.accumulator -= 1/2 * (error + self.prevError) * self.dT
        elif u < self.lowLimit:
            u = self.lowLimit
            self.accumulator -= 1/2 * (error + self.prevError) * self.dT

        # Update prev error        
        self.prevError = error
        return u
    
class PIDControl():
    # Initializing controller
    def __init__(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.trim = trim
        self.dT = dT
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0
        self.prevError = 0.0
        return
    
    # Set corresponding gains  
    def setPIDGains(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.trim = trim
        self.dT = dT
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0
        self.prevError = 0.0
        return

    def resetIntegrator(self):
        self.accumulator = 0.0
        return

    def Update(self, command = 0.0, current = 0.0, derivative=0.0):
        # Calculate error, accumulator, and u
        error = command - current
        self.accumulator += 1/2 * (error + self.prevError) * self.dT
        u = self.kp * error + self.ki * self.accumulator - self.kd * derivative + self.trim
        
        # If at saturation, set to limit and then undo the increment of the accumulator
        if u > self.highLimit:
            u = self.highLimit
            self.accumulator -= 1/2 * (error + self.prevError) * self.dT
        elif u < self.lowLimit:
            u = self.lowLimit
            self.accumulator -= 1/2 * (error + self.prevError) * self.dT

        # Update prev error
        self.prevError = error
        return u
    
class VehicleClosedLoopControl():

    # Initializing the various controllers
    def __init__(self, dT=0.01, rudderControlSource='SIDESLIP') -> None:
        self.VAM = VAM.VehicleAerodynamicsModel()
        self.controlGains = Controls.controlGains()
        self.trims = Inputs.controlInputs()
        self.controls = Inputs.controlInputs()
        self.mode = Controls.AltitudeStates.HOLDING
        self.rollFromCourse = PIControl()
        self.rudderFromSideslip = PIControl()
        self.throttleFromAirspeed = PIControl()
        self.pitchFromAltitude = PIControl()
        self.pitchFromAirspeed = PIControl()
        self.elevatorFromPitch = PDControl()
        self.aileronFromRoll = PIDControl()
        return
    
    # Function goes through and sets gain for each controller, quite tedious
    def setControlGains(self, controlGains=Controls.controlGains()):
        self.controlGains = controlGains
        self.rollFromCourse.setPIGains(kp=controlGains.kp_course, ki=controlGains.ki_course, 
                                       lowLimit=-math.radians(VPC.bankAngleLimit), highLimit=math.radians(VPC.bankAngleLimit))
        self.rudderFromSideslip.setPIGains(kp=controlGains.kp_sideslip, ki=controlGains.ki_sideslip,
                                           lowLimit=VPC.minControls.Rudder, highLimit=VPC.maxControls.Rudder, trim=self.trims.Rudder)
        
        self.throttleFromAirspeed.setPIGains(kp=controlGains.kp_SpeedfromThrottle, ki=controlGains.ki_SpeedfromThrottle, 
                                             lowLimit=VPC.minControls.Throttle, highLimit=VPC.maxControls.Throttle, trim=self.trims.Throttle)
        
        self.pitchFromAltitude.setPIGains(kp=controlGains.kp_altitude, ki=controlGains.ki_altitude, 
                                          lowLimit=-math.radians(VPC.pitchAngleLimit), highLimit=math.radians(VPC.pitchAngleLimit))
        
        self.pitchFromAirspeed.setPIGains(kp=controlGains.kp_SpeedfromElevator, ki=controlGains.ki_SpeedfromElevator, 
                                          lowLimit=-math.radians(VPC.pitchAngleLimit), highLimit=math.radians(VPC.pitchAngleLimit))
        
        self.elevatorFromPitch.setPDGains(kp=controlGains.kp_pitch, kd=controlGains.kd_pitch, 
                                          lowLimit=VPC.minControls.Elevator, highLimit=VPC.maxControls.Elevator, trim=self.trims.Elevator)
        
        self.aileronFromRoll.setPIDGains(kp=controlGains.kp_roll, kd=controlGains.kd_roll, ki=controlGains.ki_roll, 
                                         lowLimit=VPC.minControls.Aileron, highLimit=VPC.maxControls.Aileron, trim=self.trims.Aileron)
               
        return
    
    def setTrimInputs(self, trimInputs=Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)):
        self.trims = trimInputs
        return
    
    def setVehicleState(self, state):
        self.VAM.setVehicleState(state=state)
        return
    
    def getControlGains(self):
        return self.controlGains
    
    def getVehicleState(self):
        return self.VAM.getVehicleState()
    
    def getVehicleAerodynamicsModel(self):
        return self.VAM
    
    def getVehicleControlSurfaces(self):
        return self.controls
    
    def getTrimInputs(self):
        return self.trims

    # Reset all integrators
    def reset(self):
        self.rollFromCourse.resetIntegrator()
        self.rudderFromSideslip.resetIntegrator()
        self.throttleFromAirspeed.resetIntegrator()
        self.pitchFromAltitude.resetIntegrator()
        self.pitchFromAirspeed.resetIntegrator()
        self.aileronFromRoll.resetIntegrator()
        self.VAM.reset()
        return
    
    def UpdateControlCommands(self, referenceCommands: Controls.referenceCommands, state: States.vehicleState):
        ## Setting upper thresholds and the current mode 
        upper_thresh = referenceCommands.commandedAltitude + VPC.altitudeHoldZone
        lower_thresh = referenceCommands.commandedAltitude - VPC.altitudeHoldZone
        if -state.pd > upper_thresh:
            if self.mode != Controls.AltitudeStates.DESCENDING:
                self.pitchFromAirspeed.resetIntegrator()
                self.mode = Controls.AltitudeStates.DESCENDING
        elif -state.pd < lower_thresh:
            if self.mode != Controls.AltitudeStates.CLIMBING:
                self.pitchFromAirspeed.resetIntegrator()
                self.mode = Controls.AltitudeStates.CLIMBING
        else:
            if self.mode != Controls.AltitudeStates.HOLDING:
                self.pitchFromAltitude.resetIntegrator()
                self.mode = Controls.AltitudeStates.HOLDING

        # Adjusting Chi
        if (referenceCommands.commandedCourse - state.chi) <= -math.pi:
            state.chi -= 2 * math.pi
        elif (referenceCommands.commandedCourse - state.chi) >= math.pi:
            state.chi += 2 * math.pi
        
        # Calculating rudder, roll, and aileron
        rudder = self.rudderFromSideslip.Update(command=0.0, current=state.beta)
        roll = self.rollFromCourse.Update(command=referenceCommands.commandedCourse, current=state.chi)
        aileron = self.aileronFromRoll.Update(command=roll, current=state.roll, derivative=state.p)

        # Pitch and throttle state machine
        if self.mode == Controls.AltitudeStates.HOLDING:
            pitch = self.pitchFromAltitude.Update(command=referenceCommands.commandedAltitude, current=-state.pd)
            throttle = self.throttleFromAirspeed.Update(command=referenceCommands.commandedAirspeed, current=state.Va)
        elif self.mode == Controls.AltitudeStates.CLIMBING:
            pitch = self.pitchFromAirspeed.Update(command=referenceCommands.commandedAirspeed, current=state.Va)
            throttle = VPC.maxControls.Throttle
        else:
            pitch = self.pitchFromAirspeed.Update(command=referenceCommands.commandedAirspeed, current=state.Va)
            throttle = VPC.minControls.Throttle
        
        # Now doing elevator
        elevator = self.elevatorFromPitch.Update(command=pitch, current=state.pitch, derivative=state.q)

        # Assigning proper values to return variables
        referenceCommands.commandedRoll = roll
        referenceCommands.commandedPitch = pitch
        commands = Inputs.controlInputs(Throttle=throttle, Aileron=aileron, Elevator=elevator, Rudder=rudder)
        return commands

    # This funciton just calls the updateControlCommands function and sends that to the vehicle aerodynamics model
    def Update(self, referenceCommands=Controls.referenceCommands):
        state = self.getVehicleState()
        self.controls = self.UpdateControlCommands(referenceCommands=referenceCommands, state=state)
        self.VAM.Update(self.controls)
