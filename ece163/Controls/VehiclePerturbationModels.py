import math
from ece163.Modeling import VehicleAerodynamicsModel as VAM
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath

VAM_Instance = VAM.VehicleAerodynamicsModel()

def CreateTransferFunction(trimState, trimInputs):
    # Calculating the trim conditions
    Va = math.hypot(trimState.u, trimState.v, trimState.w)
    gamma = trimState.pitch - trimState.alpha
    # Calculating coefficients of transfer functions 
    a_phi1 = -(1/4) * VPC.rho * trimState.Va * VPC.S * VPC.b**2 * VPC.Cpp 
    a_phi2 = (1/2) * VPC.rho * trimState.Va**2 * VPC.S * VPC.b * VPC.CpdeltaA
    a_beta1 = -VPC.rho * trimState.Va * VPC.S * VPC.CYbeta / (2*VPC.mass)
    a_beta2 = VPC.rho * trimState.Va * VPC.S * VPC.CYdeltaR / (2*VPC.mass)
    a_theta1 = -VPC.rho * trimState.Va**2 * VPC.c * VPC.S * VPC.CMq * VPC.c / (4 * VPC.Jyy * trimState.Va)
    a_theta2 = -VPC.rho * trimState.Va**2 * VPC.c * VPC.S * VPC.CMalpha / (2 * VPC.Jyy)
    a_theta3 = VPC.rho * trimState.Va**2 * VPC.c * VPC.S * VPC.CMdeltaE / (2 * VPC.Jyy)
    a_V1 = -(VPC.rho * Va * VPC.S * (-VPC.CD0 - VPC.CDalpha * trimState.alpha - VPC.CDdeltaE * trimInputs.Elevator) + dThrust_dVa(Va=Va, Throttle=trimInputs.Throttle)) / VPC.mass
    a_V2 = dThrust_dThrottle(Va=Va, Throttle=trimInputs.Throttle,epsilon=0.010000003) / VPC.mass
    a_V3 = VPC.g0 * math.cos(trimState.pitch - trimState.alpha)

    Tf = Linearized.transferFunctions(Va_trim=Va, alpha_trim=trimState.alpha, beta_trim=trimState.beta, gamma_trim=gamma, theta_trim=trimState.pitch, 
                                    phi_trim=trimState.roll, a_phi1=a_phi1, a_phi2=a_phi2, a_beta1=a_beta1, a_beta2=a_beta2, a_theta1=a_theta1, 
                                    a_theta2=a_theta2, a_theta3=a_theta3, a_V1=a_V1, a_V2=a_V2, a_V3=a_V3)
    return Tf

def dThrust_dThrottle(Va, Throttle, epsilon=0.01):
    # Calculating the difference of prop forces
    Fx_plus = VAM_Instance.CalculatePropForces(Va=Va, Throttle=Throttle+epsilon)[0]
    Fx = VAM_Instance.CalculatePropForces(Va=Va, Throttle=Throttle)[0]
    return (Fx_plus - Fx) / epsilon

def dThrust_dVa(Va, Throttle, epsilon=0.5):
    # Calculating the difference of prop forces
    Fx_plus = VAM_Instance.CalculatePropForces(Va=Va+epsilon, Throttle=Throttle)[0]
    Fx = VAM_Instance.CalculatePropForces(Va=Va, Throttle=Throttle)[0]
    return (Fx_plus - Fx) / epsilon