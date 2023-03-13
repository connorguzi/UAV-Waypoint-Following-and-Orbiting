import math
from ..Containers import States
from ..Containers import Inputs
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

class VehicleDynamicsModel():
    def __init__(self, dT=0.01):
        self.dT = dT
        self.state = States.vehicleState()
        self.dot = States.vehicleState()
        return

    def reset(self):
        self.state = States.vehicleState()
        self.dot = States.vehicleState()

    def getVehicleState(self):
        return self.state

    def getVehicleDerivative(self):
        return self.dot
    
    def setVehicleState(self, state):
        self.state = state
        return
    
    def setVehicleDerivative(self, dot):
        self.dot = dot
        return

    def derivative(self, state: States.vehicleState(), forcesMoments: Inputs.forcesMoments()):
        ## Setting up variables used in the calculations
        dot = States.vehicleState()
        yaw, pitch, roll = state.yaw, state.pitch, state.roll
        skew = mm.skew(state.p, state.q, state.r)
        vel_vec = [[state.u], [state.v], [state.w]]
        forces = [[forcesMoments.Fx], [forcesMoments.Fy], [forcesMoments.Fz]]
        moments = [[forcesMoments.Mx], [forcesMoments.My], [forcesMoments.Mz]]
        euler_dot_mat = [[1, math.sin(roll)* math.tan(pitch), math.cos(roll) *  math.tan(pitch)],
                        [0, math.cos(roll), -math.sin(roll)],
                        [0, math.sin(roll) / math.cos(pitch), math.cos(roll) / math.cos(pitch)]]
        rot_vec = [[state.p], [state.q], [state.r]]

        ## Calculating the derivative values
        positions = mm.multiply(mm.transpose(state.R), vel_vec)
        vels = mm.multiply(mm.scalarMultiply(-1, skew), vel_vec)
        vels = mm.add(vels, mm.scalarDivide(VPC.mass, forces))
        R_dot = mm.multiply(mm.scalarMultiply(-1, skew), state.R)
        euler_rates = mm.multiply(euler_dot_mat, rot_vec)

        rot_rates = mm.multiply(mm.multiply(skew, VPC.Jbody), rot_vec)
        rot_rates = mm.subtract(moments, rot_rates)
        rot_rates = mm.multiply(VPC.JinvBody, rot_rates)

        # Storing in the dot value
        dot.pn, dot.pe, dot.pd = positions[0][0], positions[1][0], positions[2][0]
        dot.u, dot.v, dot.w = vels[0][0], vels[1][0], vels[2][0]
        dot.yaw, dot.pitch, dot.roll = euler_rates[2][0], euler_rates[1][0], euler_rates[0][0]
        dot.R = R_dot
        dot.p, dot.q, dot.r = rot_rates[0][0], rot_rates[1][0], rot_rates[2][0] 

        return dot

    def Rexp(self, dT, state: States.vehicleState(), dot: States.vehicleState()):
        # Getting the values for omega and omega dot
        omega = [[state.p], [state.q], [state.r]]
        omega_dot = [[dot.p], [dot.q], [dot.r]]
        omega = mm.add(omega, mm.scalarMultiply(dT/2, omega_dot))
        # Generate magnitude and skew and skew^2
        mag = math.hypot(omega[0][0], omega[1][0], omega[2][0])
        skew = mm.skew(omega[0][0], omega[1][0], omega[2][0])
        skew_squared = mm.multiply(skew, skew)
        I = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

        # If the magnitude is too low, approximate or else the terms blow up
        if(mag > 0.2):
            first_term = mm.scalarMultiply(math.sin(mag * dT)/mag, skew)
            second_term = mm.scalarMultiply((1 - math.cos(mag * dT)) / (mag**2), skew_squared)
            Rexp = mm.subtract(I, first_term)
            return mm.add(Rexp, second_term)
        
        else:
            first_term = dT - (((dT ** 3) * (mag ** 2))/6 )+ (((dT ** 5) * (mag ** 4))/120)
            first_term = mm.scalarMultiply(first_term, skew)
            second_term = (dT**2)/2 - (((dT ** 4) * (mag ** 2))/24 )+ (((dT ** 6) * (mag ** 4))/720)
            second_term = mm.scalarMultiply(second_term, skew_squared)
            Rexp = mm.subtract(I, first_term)
            return mm.add(Rexp, second_term)

    # Simple forward euler integration x_new = x_old + x_dot * dt
    def ForwardEuler(self, dT, state, dot):
        newstate = States.vehicleState()
        newstate.pn = state.pn + dot.pn * dT
        newstate.pe = state.pe + dot.pe * dT
        newstate.pd = state.pd + dot.pd * dT

        newstate.u = state.u + dot.u * dT
        newstate.v = state.v + dot.v * dT
        newstate.w = state.w + dot.w * dT

        newstate.roll = state.roll + dot.roll * dT
        newstate.pitch = state.pitch + dot.pitch * dT
        newstate.yaw = state.yaw + dot.yaw * dT

        newstate.R = mm.add(state.R, mm.scalarMultiply(dT, dot.R))
        
        newstate.p = state.p + dot.p * dT
        newstate.q = state.q + dot.q * dT
        newstate.r = state.r + dot.r * dT

        return newstate

    # Combination of Forward Euler and Rexp
    def IntegrateState(self, dT, state, dot):
        newstate = States.vehicleState()
        newstate.pn = state.pn + dot.pn * dT
        newstate.pe = state.pe + dot.pe * dT
        newstate.pd = state.pd + dot.pd * dT

        newstate.u = state.u + dot.u * dT
        newstate.v = state.v + dot.v * dT
        newstate.w = state.w + dot.w * dT

        newstate.R = mm.multiply(self.Rexp(dT, state, dot), state.R)
        
        newstate.yaw, newstate.pitch, newstate.roll = Rotations.dcm2Euler(newstate.R)

        newstate.p = state.p + dot.p * dT
        newstate.q = state.q + dot.q * dT
        newstate.r = state.r + dot.r * dT

        newstate.Va = state.Va
        newstate.alpha = state.alpha
        newstate.beta = state.beta
        newstate.chi = math.atan2(dot.pe, dot.pn)

        return newstate

    # Update to the correct values
    def Update(self, forcesMoments):
        state = self.getVehicleState()
        dot = self.derivative(state, forcesMoments)
        newstate = self.IntegrateState(self.dT, state, dot)
        self.setVehicleState(newstate)
