import math
import random
from ..Containers import States
from ..Utilities import MatrixMath as mm
from ..Constants import VehiclePhysicalConstants as VPC

class WindModel():
    def __init__(self, dT=VPC.dT, Va=VPC.InitialSpeed, drydenParamters=VPC.DrydenNoWind) -> None:
        drydenParameters = drydenParamters
        self.dT = dT
        self.Va = Va
        self.windState = States.windState()
        self.drydenParameters = drydenParamters
        self.Xu = [[0]]
        self.Xv = [[0],[0]]
        self.Xw = [[0],[0]]
        # Generate Phi, Gamma, and H for u
        if(drydenParameters.Lu):
            self.Phi_u = [[math.exp(-Va * dT / drydenParameters.Lu)]]
            self.Gamma_u = [[drydenParameters.Lu / Va * (1 - math.exp(-Va * dT / drydenParameters.Lu))]]
            self.H_u = [[drydenParameters.sigmau * math.sqrt(2 * Va / (math.pi * drydenParameters.Lu))]]
        else:
            self.Phi_u = [[1]]
            self.Gamma_u = [[0]]
            self.H_u = [[1]]
        
        # Generate Phi, Gamma, and H for v
        if(drydenParameters.Lv):
            Phi_mat_v = [[1-Va*dT/drydenParameters.Lv, -(Va/drydenParameters.Lv)**2 * dT], [dT, 1 + Va*dT/drydenParameters.Lv]]
            self.Phi_v = mm.scalarMultiply(math.exp(-Va * dT/drydenParameters.Lv ), Phi_mat_v)
            Gamma_mat_v = [[dT], [(drydenParameters.Lv / Va)**2 * (math.exp(Va * dT/drydenParameters.Lv ) - 1) - (drydenParameters.Lv * dT / Va)]]
            self.Gamma_v = mm.scalarMultiply(math.exp(-Va * dT/drydenParameters.Lv ), Gamma_mat_v)
            self.H_v = mm.scalarMultiply(drydenParameters.sigmav * math.sqrt(3 * Va / (math.pi * drydenParameters.Lv)), [[1, Va / (math.sqrt(3) * drydenParameters.Lv)]])
        else:
            self.Phi_v = [[1, 0], [0, 1]]
            self.Gamma_v = [[0], [0]]
            self.H_v = [[1,1]]
        
        # Generate Phi, Gamma, and H for w
        if(drydenParameters.Lw):
            Phi_mat_w = [[1-Va*dT/drydenParameters.Lw, -(Va/drydenParameters.Lw)**2 * dT], [dT, 1 + Va*dT/drydenParameters.Lw]]
            self.Phi_w = mm.scalarMultiply(math.exp(-Va * dT/drydenParameters.Lw ), Phi_mat_w)
            Gamma_mat_w = [[dT], [((drydenParameters.Lw / Va)**2) * (math.exp(Va * dT/drydenParameters.Lw ) - 1.) - (drydenParameters.Lw * dT / Va)]]
            self.Gamma_w = mm.scalarMultiply(math.exp(-Va * dT/drydenParameters.Lw), Gamma_mat_w)
            self.H_w = mm.scalarMultiply(drydenParameters.sigmaw * math.sqrt(3 * Va / (math.pi * drydenParameters.Lw)), [[1, Va / (math.sqrt(3) * drydenParameters.Lw)]])
        else:
            self.Phi_w = [[1, 0], [0, 1]]
            self.Gamma_w = [[0], [0]]
            self.H_w = [[1,1]]

        return

    def reset(self):
        self.windState = States.windState()
        self.Xu = 0
        self.Xv = [[0],[0]]
        self.Xw = [[0],[0]]
        return
    
    def getWind(self):
        return self.windState
    
    def setWind(self, windState):
        self.windState = windState
        return
    
    def getDrydenTransferFns(self):
        return self.Phi_u, self.Gamma_u, self.H_u, self.Phi_v, self.Gamma_v, self.H_v, self.Phi_w, self.Gamma_w, self.H_w

    def CreateDrydenTransferFns(self, dT, Va, drydenParameters):
        # Generate Phi, Gamma, and H for u
        if(Va == 0):
            raise ArithmeticError("Va must be nonzero")
        if(drydenParameters.Lu):
            self.Phi_u = [[math.exp(-Va * dT / drydenParameters.Lu)]]
            self.Gamma_u = [[drydenParameters.Lu / Va * (1 - math.exp(-Va * dT / drydenParameters.Lu))]]
            self.H_u = [[drydenParameters.sigmau * math.sqrt(2 * Va / (math.pi * drydenParameters.Lu))]]
        else:
            self.Phi_u = [[1]]
            self.Gamma_u = [[0]]
            self.H_u = [[1]]
        
        # Generate Phi, Gamma, and H for v
        if(drydenParameters.Lv):
            Phi_mat_v = [[1-Va*dT/drydenParameters.Lv, -(Va/drydenParameters.Lv)**2 * dT], [dT, 1 + Va*dT/drydenParameters.Lv]]
            self.Phi_v = mm.scalarMultiply(math.exp(-Va * dT/drydenParameters.Lv ), Phi_mat_v)
            Gamma_mat_v = [[dT], [(drydenParameters.Lv / Va)**2 * (math.exp(Va * dT/drydenParameters.Lv ) - 1) - (drydenParameters.Lv * dT / Va)]]
            self.Gamma_v = mm.scalarMultiply(math.exp(-Va * dT/drydenParameters.Lv ), Gamma_mat_v)
            self.H_v = mm.scalarMultiply(drydenParameters.sigmav * math.sqrt(3 * Va / (math.pi * drydenParameters.Lv)), [[1, Va / (math.sqrt(3) * drydenParameters.Lv)]])
        else:
            self.Phi_v = [[1, 0], [0, 1]]
            self.Gamma_v = [[0], [0]]
            self.H_v = [[1,1]]
        
        # Generate Phi, Gamma, and H for w
        if(drydenParameters.Lw):
            Phi_mat_w = [[1-Va*dT/drydenParameters.Lw, -(Va/drydenParameters.Lw)**2 * dT], [dT, 1 + Va*dT/drydenParameters.Lw]]
            self.Phi_w = mm.scalarMultiply(math.exp(-Va * dT/drydenParameters.Lw ), Phi_mat_w)
            Gamma_mat_w = [[dT], [((drydenParameters.Lw / Va)**2) * (math.exp(Va * dT/drydenParameters.Lw ) - 1.) - (drydenParameters.Lw * dT / Va)]]
            self.Gamma_w = mm.scalarMultiply(math.exp(-Va * dT/drydenParameters.Lw), Gamma_mat_w)
            self.H_w = mm.scalarMultiply(drydenParameters.sigmaw * math.sqrt(3 * Va / (math.pi * drydenParameters.Lw)), [[1, Va / (math.sqrt(3) * drydenParameters.Lw)]])
        else:
            self.Phi_w = [[1, 0], [0, 1]]
            self.Gamma_w = [[0], [0]]
            self.H_w = [[1,1]]
        return
    
    def setWindModelParameters(self, Wn=0.0, We=0.0, Wd=0.0, drydenParamters=VPC.DrydenNoWind):
        # Setting parameters
        self.drydenParameters = drydenParamters
        self.windState.Wn = Wn
        self.windState.We = We
        self.windState.Wd = Wd
        self.CreateDrydenTransferFns(dT=self.dT, Va=self.Va, drydenParameters=self.drydenParameters)
        return
    
    def Update(self, uu=None, uv=None, uw=None):
        # Generate white noise
        if uu is None:
            uu = random.gauss(0,1)
        if uv is None:
            uv = random.gauss(0,1)
        if uw is None:
            uw = random.gauss(0,1)
        
        # Calculate transfer functions then use the update the wind gusts
        self.CreateDrydenTransferFns(dT=self.dT, Va=self.Va, drydenParameters=self.drydenParameters)
        self.Xu = mm.add(mm.multiply(self.Phi_u, self.Xu), mm.scalarMultiply(uu, self.Gamma_u))
        self.windState.Wu = mm.multiply(self.H_u, self.Xu)[0][0]

        self.Xv = mm.add(mm.multiply(self.Phi_v, self.Xv), mm.scalarMultiply(uv, self.Gamma_v))
        self.windState.Wv = mm.multiply(self.H_v, self.Xv)[0][0]

        self.Xw = mm.add(mm.multiply(self.Phi_w, self.Xw), mm.scalarMultiply(uw, self.Gamma_w))
        self.windState.Ww = mm.multiply(self.H_w, self.Xw)[0][0]

        return