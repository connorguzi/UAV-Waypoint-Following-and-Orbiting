import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

def sigma(a, a0, M):
    num = (1 + math.exp(-M * (a - a0)) + math.exp(M * (a + a0)))
    den = (1 + math.exp(-M * (a - a0))) * (1 + math.exp(M*(a+a0)))
    return num / den

class VehicleAerodynamicsModel():
    def __init__(self, initialSpeed=25.0, initialHeight=-100) -> None:
        self.initialSpeed = initialSpeed
        self.initialHeight = initialHeight
        self.vehicleDynamics = VehicleDynamicsModel.VehicleDynamicsModel()
        self.vehicleDynamics.state = States.vehicleState(pd=self.initialHeight, u=self.initialSpeed)
        self.windModel = WindModel.WindModel()
        return

    def reset(self):
        self.vehicleDynamics = VehicleDynamicsModel.VehicleDynamicsModel()
        self.vehicleDynamics.state = States.vehicleState(pd=self.initialHeight, u=self.initialSpeed)
        self.windModel = WindModel.WindModel()
        return

    def getVehicleDynamicsModel(self):
        return self.vehicleDynamics
    
    def getVehicleState(self):
        return self.vehicleDynamics.state

    def getWindModel(self):
        return self.windModel    

    def setVehicleState(self, state):
        self.vehicleDynamics.state = state
        return

    def setWindModel(self, windModel):
        self.windModel = windModel
        return 

    def gravityForces(self, state: States.vehicleState()):
        gravity = [[0], [0], [VPC.g0]]
        force = mm.scalarMultiply(VPC.mass, mm.multiply(state.R, gravity))
        return Inputs.forcesMoments(Fx=force[0][0], Fy=force[1][0], Fz=force[2][0])
    
    def CalculateCoeff_alpha(self, alpha):
        CLPre = VPC.CL0 + VPC.CLalpha * alpha
        CDPre = VPC.CDp + (CLPre)**2/(math.pi * VPC.AR *VPC.e)
        CLPost = 2 * math.sin(alpha) * math.cos(alpha)
        CDPost = 2 *(math.sin(alpha)**2 )
        CL = (1 - sigma(alpha, VPC.alpha0, VPC.M)) * CLPre + sigma(alpha, VPC.alpha0, VPC.M) * CLPost
        CD = (1 - sigma(alpha, VPC.alpha0, VPC.M)) * CDPre + sigma(alpha, VPC.alpha0, VPC.M) * CDPost
        CM = VPC.CM0 + VPC.CMalpha * alpha
        return (CL, CD, CM)
    
    def aeroForces(self, state: States.vehicleState()):
        CLa = self.CalculateCoeff_alpha(state.alpha)[0]
        CDa = self.CalculateCoeff_alpha(state.alpha)[1]
        CMa = self.CalculateCoeff_alpha(state.alpha)[2]
        if(state.Va):
            Fl = -1/2 * VPC.rho * (state.Va**2) * VPC.S * (CLa + (VPC.CLq * state.q * (VPC.c / (state.Va * 2))))
            Fd = -1/2 * VPC.rho * (state.Va**2) * VPC.S * (CDa + (VPC.CDq * state.q * (VPC.c / (state.Va * 2))))
            Fy = 1/2 * VPC.rho * (state.Va**2) * VPC.S * (VPC.CY0 + VPC.CYbeta * state.beta + VPC.CYp * (VPC.b * state.p / (state.Va * 2)) + VPC.CYr * (VPC.b * state.r / (state.Va * 2)))
            # Put the lift and drag into the body frame
            forces_stability = [[Fd], [Fl]]
            R = [[math.cos(state.alpha), -math.sin(state.alpha)], [math.sin(state.alpha), math.cos(state.alpha)]]
            forces = mm.multiply(R, forces_stability)
            Fd, Fl = forces[0][0], forces[1][0]

            l = 1/2 * VPC.rho * (state.Va**2) * VPC.S * VPC.b * (VPC.Cl0 + VPC.Clbeta * state.beta + VPC.Clp * (VPC.b * state.p / (state.Va * 2)) + VPC.Clr * (VPC.b * state.r / (state.Va * 2)))
            My = 1/2 * VPC.rho * (state.Va**2) * VPC.S * VPC.c * (CMa + VPC.CMq * state.q * (VPC.c / (state.Va * 2)))
            n = 1/2 * VPC.rho * (state.Va**2) * VPC.S * VPC.b * (VPC.Cn0 + VPC.Cnbeta * state.beta + VPC.Cnp * (VPC.b * state.p / (state.Va * 2)) + VPC.Cnr * (VPC.b * state.r / (state.Va * 2)))
        else:
            Fl = 0
            Fd = 0
            Fy = 0

            l = 0
            My = 0
            n = 0

        aeroForces = Inputs.forcesMoments(Fx=Fd, Fy=Fy, Fz=Fl, Mx=l, My=My, Mz=n)
        return aeroForces
    
    def CalculatePropForces(self, Va, Throttle):
        # Linear relationship with Vin and Vmax
        Vin = VPC.V_max * Throttle
        # Setup for quadratic equation
        a = (VPC.rho * (VPC.D_prop ** 5) * VPC.C_Q0) / (4 * (math.pi ** 2))
        b = ((VPC.rho * (VPC.D_prop ** 4) * Va * VPC.C_Q1) / (2 * (math.pi)) )+ ((VPC.KQ ** 2) / (VPC.R_motor))
        c = (VPC.rho * (VPC.D_prop ** 3) * (Va**2) * VPC.C_Q2) - ((VPC.KQ * Vin) / (VPC.R_motor)) + (VPC.KQ * VPC.i0)
        # Calculate Omega and J
        try:
            omega = (-b + math.sqrt((b**2) - 4 * a * c)) / (2 * a)
        except:
            omega = 100.0
        J = (2 * math.pi * Va) / (omega * VPC.D_prop)
        # Calculate Coefficients for moments and forces
        CT = VPC.C_T0 + VPC.C_T1 * J + VPC.C_T2 * J**2
        CQ = VPC.C_Q0 + VPC.C_Q1 * J + VPC.C_Q2 * J**2
        # Calculate moment and force
        F = (VPC.rho * (VPC.D_prop ** 4) * (omega ** 2) * CT) / (4 * (math.pi ** 2))
        M = -(VPC.rho * (VPC.D_prop ** 5) * (omega ** 2) * CQ) / (4 * (math.pi ** 2))
        return (F, M)
    
    def controlForces(self, state: States.vehicleState(), controls: Inputs.controlInputs()):
        #Forces
        Fl = -1/2 * VPC.rho * (state.Va**2) * VPC.S * VPC.CLdeltaE * controls.Elevator
        Fd = -1/2 * VPC.rho * (state.Va**2) * VPC.S * VPC.CDdeltaE * controls.Elevator
        Fy = 1/2 * VPC.rho * (state.Va**2) * VPC.S * (VPC.CYdeltaA * controls.Aileron + VPC.CYdeltaR * controls.Rudder)

        # Stability to body
        forces_stability = [[Fd], [Fl]]
        R = [[math.cos(state.alpha), -math.sin(state.alpha)], [math.sin(state.alpha), math.cos(state.alpha)]]
        forces = mm.multiply(R, forces_stability)
        Fd, Fl = forces[0][0], forces[1][0]

        # Moments
        l = 1/2 * VPC.rho * (state.Va**2) * VPC.S * VPC.b * (VPC.CldeltaA * controls.Aileron + VPC.CldeltaR * controls.Rudder)
        My = 1/2 * VPC.rho * (state.Va**2) * VPC.S * VPC.c * VPC.CMdeltaE * controls.Elevator
        n = 1/2 * VPC.rho * (state.Va**2) * VPC.S * VPC.b * (VPC.CndeltaA * controls.Aileron + VPC.CndeltaR * controls.Rudder)

        # Prop forces
        Fprop, Mprop = self.CalculatePropForces(state.Va, controls.Throttle)
        Fd += Fprop
        l += Mprop
        controlForces = Inputs.forcesMoments(Fx=Fd, Fy=Fy, Fz=Fl, Mx=l, My=My, Mz=n)
        return controlForces

    def CalculateAirspeed(self, state, wind):
        # Putting the steady/gust wind and velocities into vectors
        windConst = [[wind.Wn], [wind.We], [wind.Wd]]
        windGust = [[wind.Wu], [wind.Wv], [wind.Ww]]
        velocities = [[state.u], [state.v], [state.w]]

        # Calculating angles for the stability frame
        wind_chi = math.atan2(wind.We, wind.Wn)
        if wind.Wn or wind.We or wind.Wd:
            wind_gamma = -math.asin(wind.Wd / math.hypot(wind.Wn, wind.We, wind.Wd))
        else:
            wind_gamma = 0
        R_wind = mm.transpose(Rotations.euler2DCM(yaw=wind_chi, pitch=wind_gamma))
        winVec = mm.add(windConst, mm.multiply(R_wind, windGust))
        winVec = mm.multiply(state.R, winVec)
        total = mm.subtract(velocities, winVec)
        Va = math.hypot(total[0][0], total[1][0], total[2][0])
        if total[0][0]:
            alpha = math.atan2(total[2][0], total[0][0])
        else:
            if total[2][0]:
                alpha = math.pi * math.copysign(1, total[2][0]) / 2
            else:
                alpha = 0
        if Va != 0:
            beta = math.asin(total[1][0] / Va)
        else:
            beta = 0
        return (Va, alpha, beta)

    def updateForces(self, state, controls, wind=None):
        # Calculate Va, Alpha, And Beta first using wind : winstate()
        if wind:
            # Update state Va, Alpha, and Beta
            state.Va, state.alpha, state.beta = self.CalculateAirspeed(state, wind)[0], self.CalculateAirspeed(state, wind)[1], self.CalculateAirspeed(state, wind)[2]
        else:
            if(state.u):
                state.Va = math.hypot(state.u, state.v, state.w)
                state.alpha = math.atan2(state.w, state.u)
                state.beta = math.asin(state.v / state.Va)
        # Generate all of the moments and forces with previous functions
        FxAero, FyAero, FzAero = self.aeroForces(state).Fx, self.aeroForces(state).Fy, self.aeroForces(state).Fz
        MxAero, MyAero, MzAero = self.aeroForces(state).Mx, self.aeroForces(state).My, self.aeroForces(state).Mz
        FxControl, FyControl, FzControl = self.controlForces(state, controls).Fx, self.controlForces(state, controls).Fy, self.controlForces(state, controls).Fz
        MxControl, MyControl, MzControl = self.controlForces(state, controls).Mx, self.controlForces(state, controls).My, self.controlForces(state, controls).Mz
        FxGrav, FyGrav, FzGrav = self.gravityForces(state).Fx, self.gravityForces(state).Fy, self.gravityForces(state).Fz
        # Add the corresponding forces together
        Fx = FxAero + FxControl + FxGrav
        Fy = FyAero + FyControl + FyGrav
        Fz = FzAero + FzControl + FzGrav
        Mx = MxAero + MxControl
        My = MyAero + MyControl
        Mz = MzAero + MzControl
        ForcesMoments = Inputs.forcesMoments(Fx=Fx, Fy=Fy, Fz=Fz, Mx=Mx, My=My, Mz=Mz)
        return ForcesMoments
    
    def Update(self, controls):
        wind = self.getWindModel().getWind()
        state = self.vehicleDynamics.state
        ForcesMoments = self.updateForces(state, controls, wind=wind)
        self.vehicleDynamics.Update(ForcesMoments)
        return