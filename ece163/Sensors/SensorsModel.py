import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath as mm
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel
from ..Containers import States

class GaussMarkov():
    def __init__(self, dT=0.01, tau=1000000.0, eta=0.0) -> None:
        self.dT = dT
        self.tau = tau
        self.eta = eta
        self.v = 0
        pass

    def reset(self):
        self.v = 0

    def update(self,vnoise=None):
        if vnoise != None:
            self.v = math.exp(-self.dT / self.tau) * self.v + vnoise

        else:
            self.v = math.exp(-self.dT / self.tau) * self.v + random.gauss(0, self.eta)

        return self.v

class GaussMarkovXYZ():
        def __init__(self, dT=0.01, tauX=1000000.0, etaX=0.0, tauY=None, etaY=None, tauZ=None, etaZ=None) -> None:
            self.gaussX = GaussMarkov(dT=dT, tau=tauX, eta=etaX)
            self.gaussY = GaussMarkov(dT=dT, tau=tauY if tauY else tauX, eta=etaY if etaY else etaX)
            self.gaussZ = GaussMarkov(dT=dT, tau=tauZ if tauZ else self.gaussY.tau, eta=etaZ if etaZ else self.gaussY.eta)
            pass
        
        def reset(self):
            self.gaussX.reset()
            self.gaussY.reset()
            self.gaussZ.reset()
        
        def update(self, vXnoise=None, vYnoise=None, vZnoise=None):
            self.gaussX.update(vXnoise)
            self.gaussY.update(vYnoise)
            self.gaussZ.update(vZnoise)
            return self.gaussX.v, self.gaussY.v, self.gaussZ.v
        
class SensorsModel():
    def __init__(self,aeroModel=VehicleAerodynamicsModel.VehicleAerodynamicsModel(),taugyro=400.0, etagyro=0.0012740903539558606, tauGPS=1100.0, etaGPSHorizontal=0.21,etaGPSVertical=0.4, gpsUpdateHz=1.0 ):
        self.VAM = aeroModel
        self.gyroGauss = GaussMarkovXYZ(tauX=taugyro, etaX=etagyro)
        self.gpsGauss = GaussMarkovXYZ(dT = gpsUpdateHz,tauX=tauGPS, etaX=etaGPSHorizontal, etaY=etaGPSHorizontal, etaZ=etaGPSVertical)
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsBiases = Sensors.vehicleSensors()
        self.sensorsSigmas = Sensors.vehicleSensors()
        self.sensorsBiases = self.initializeBiases()
        self.sensorsSigmas = self.initializeSigmas()
        self.sensorsNoisy = Sensors.vehicleSensors()
        self.updateTicks = 0
        self.gpsTickUpdate = gpsUpdateHz / VPC.dT
        self.dT = aeroModel.getVehicleDynamicsModel().dT
        return
    
    def getSensorsNoisy(self):
        return self.sensorsNoisy
    
    def getSensorsTrue(self):
        return self.sensorsTrue
    
    def initializeBiases(self, gyroBias=0.08726646259971647, accelBias=0.9810000000000001, magBias=500.0, baroBias=100.0, pitotBias=20.0):
        self.sensorsBiases.accel_x , self.sensorsBiases.accel_y, self.sensorsBiases.accel_z = random.uniform(-accelBias, accelBias), random.uniform(-accelBias, accelBias), random.uniform(-accelBias, accelBias)
        self.sensorsBiases.gyro_x, self.sensorsBiases.gyro_y, self.sensorsBiases.gyro_z = random.uniform(-gyroBias, gyroBias), random.uniform(-gyroBias, gyroBias), random.uniform(-gyroBias, gyroBias)
        self.sensorsBiases.mag_x, self.sensorsBiases.mag_y, self.sensorsBiases.mag_z = random.uniform(-magBias,magBias),random.uniform(-magBias,magBias),random.uniform(-magBias,magBias)
        self.sensorsBiases.baro = random.uniform(-baroBias, baroBias)
        self.sensorsBiases.pitot = random.uniform(-pitotBias, pitotBias)
        
        return self.sensorsBiases

    def initializeSigmas(self, gyroSigma=0.002617993877991494, accelSigma=0.24525000000000002, magSigma=25.0, baroSigma=10.0, pitotSigma=2.0, gpsSigmaHorizontal=0.4, gpsSigmaVertical=0.7, gpsSigmaSOG=0.05, gpsSigmaCOG=0.002):
        self.sensorsSigmas.accel_x, self.sensorsSigmas.accel_y, self.sensorsSigmas.accel_z = [accelSigma] * 3
        self.sensorsSigmas.gyro_x, self.sensorsSigmas.gyro_y, self.sensorsSigmas.gyro_z = [gyroSigma] * 3
        self.sensorsSigmas.mag_x, self.sensorsSigmas.mag_y, self.sensorsSigmas.mag_z = [magSigma] * 3
        self.sensorsSigmas.baro = baroSigma
        self.sensorsSigmas.pitot = pitotSigma
        self.sensorsSigmas.gps_e = gpsSigmaHorizontal
        self.sensorsSigmas.gps_n = gpsSigmaVertical
        self.sensorsSigmas.gps_cog = gpsSigmaCOG
        self.sensorsSigmas.gps_sog = gpsSigmaSOG

        return self.sensorsSigmas

    def reset(self):
        self.gpsGauss.reset()
        self.gyroGauss.reset()
        self.gpsTickUpdate = 0
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsBiases = Sensors.vehicleSensors()
        self.sensorsSigmas = Sensors.vehicleSensors()
        self.sensorsNoisy = Sensors.vehicleSensors()
        self.initializeBiases()
        self.initializeSigmas()

    def updateAccelsTrue(self, state: States.vehicleState, dot: States.vehicleState):
        self.sensorsTrue.accel_x = dot.u + state.q * state.w - state.r * state.v + VPC.g0 * math.sin(state.pitch)
        self.sensorsTrue.accel_y = dot.v + state.r * state.u - state.p * state.w - VPC.g0 * math.cos(state.pitch) * math.sin(state.roll)
        self.sensorsTrue.accel_z = dot.w + state.p * state.v - state.q * state.u - VPC.g0 * math.cos(state.pitch) * math.cos(state.roll)
        return self.sensorsTrue.accel_x, self.sensorsTrue.accel_y, self.sensorsTrue.accel_z

    def updateGyrosTrue(self, state: States.vehicleState):
        self.sensorsTrue.gyro_x = state.p
        self.sensorsTrue.gyro_y = state.q
        self.sensorsTrue.gyro_z = state.r
        return self.sensorsTrue.gyro_x, self.sensorsTrue.gyro_y, self.sensorsTrue.gyro_z
    
    def updateMagsTrue(self,state: States.vehicleState):
        mag_vec = mm.multiply(state.R, VSC.magfield)
        self.sensorsTrue.mag_x, self.sensorsTrue.mag_y, self.sensorsTrue.mag_z = mag_vec[0][0],mag_vec[1][0], mag_vec[2][0]
        return self.sensorsTrue.mag_x, self.sensorsTrue.mag_y, self.sensorsTrue.mag_z 
    
    def updateGPSTrue(self, state: States.vehicleState, dot: States.vehicleState):
        self.sensorsTrue.gps_sog = math.hypot(dot.pn, dot.pe)
        self.sensorsTrue.gps_cog = math.atan2(dot.pe, dot.pn)
        self.sensorsTrue.gps_n = state.pn
        self.sensorsTrue.gps_e = state.pe
        self.sensorsTrue.gps_alt = -state.pd
        return self.sensorsTrue.gps_n, self.sensorsTrue.gps_e, self.sensorsTrue.gps_alt, self.sensorsTrue.gps_sog, self.sensorsTrue.gps_cog
    
    def updatePressureSensorsTrue(self, state: States.vehicleState):
        self.sensorsTrue.baro = VSC.Pground + VPC.rho * VPC.g0 * state.pd
        self.sensorsTrue.pitot = 1/2 * VPC.rho * state.Va ** 2
        return self.sensorsTrue.baro, self.sensorsTrue.pitot
    
    def updateSensorsTrue(self, prevTrueSensors: Sensors.vehicleSensors, state, dot):
        self.updateAccelsTrue(state=state, dot=dot)
        self.updateGyrosTrue(state=state)
        self.updateMagsTrue(state=state)
        self.updatePressureSensorsTrue(state=state)
        if not self.updateTicks % self.gpsTickUpdate:
            self.updateGPSTrue(state=state, dot=dot)
        else:
            self.sensorsTrue.gps_n = prevTrueSensors.gps_n
            self.sensorsTrue.gps_e = prevTrueSensors.gps_e
            self.sensorsTrue.gps_alt = prevTrueSensors.gps_alt
            self.sensorsTrue.gps_cog = prevTrueSensors.gps_cog
            self.sensorsTrue.gps_sog = prevTrueSensors.gps_sog
        return self.sensorsTrue
    
    def updateSensorsNoisy(self, trueSensors=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                            accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,
                            gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0),
                            noisySensors=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                            accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,
                            gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0),
                            sensorBiases=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                            accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,
                            gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0),
                            sensorSigmas=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                            accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,
                            gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0)):
        self.sensorsNoisy.accel_x = trueSensors.accel_x + sensorBiases.accel_x + random.gauss(0, sensorSigmas.accel_x) 
        self.sensorsNoisy.accel_y = trueSensors.accel_y + sensorBiases.accel_y + random.gauss(0, sensorSigmas.accel_y) 
        self.sensorsNoisy.accel_z = trueSensors.accel_z + sensorBiases.accel_z + random.gauss(0, sensorSigmas.accel_z) 
        self.sensorsNoisy.gyro_x = trueSensors.gyro_x + sensorBiases.gyro_x + random.gauss(0, sensorSigmas.gyro_x) + self.gyroGauss.gaussX.update()
        self.sensorsNoisy.gyro_y = trueSensors.gyro_y + sensorBiases.gyro_y + random.gauss(0, sensorSigmas.gyro_y)+ self.gyroGauss.gaussY.update()
        self.sensorsNoisy.gyro_z = trueSensors.gyro_z + sensorBiases.gyro_z + random.gauss(0, sensorSigmas.gyro_z)+ self.gyroGauss.gaussY.update()
        self.sensorsNoisy.baro = trueSensors.baro + sensorBiases.baro + random.gauss(0, sensorSigmas.baro)
        self.sensorsNoisy.pitot = trueSensors.pitot + sensorBiases.pitot + random.gauss(0, sigma=sensorSigmas.pitot)
        self.sensorsNoisy.mag_x = trueSensors.mag_x + sensorBiases.mag_x + random.gauss(0, sensorSigmas.mag_x)
        self.sensorsNoisy.mag_y = trueSensors.mag_y + sensorBiases.mag_y + random.gauss(0, sensorSigmas.mag_y)
        self.sensorsNoisy.mag_z = trueSensors.mag_z + sensorBiases.mag_z + random.gauss(0, sensorSigmas.mag_z)
        if not self.updateTicks % self.gpsTickUpdate:
            self.sensorsNoisy.gps_n = trueSensors.gps_n + sensorBiases.gps_n + random.gauss(0, sensorSigmas.gps_n) + self.gpsGauss.gaussX.update()
            self.sensorsNoisy.gps_e = trueSensors.gps_e + sensorBiases.gps_e + random.gauss(0, sensorSigmas.gps_e) + self.gpsGauss.gaussY.update()
            self.sensorsNoisy.gps_alt = trueSensors.gps_alt + sensorBiases.gps_alt + random.gauss(0, sensorSigmas.gps_alt) + self.gpsGauss.gaussZ.update()
            self.sensorsNoisy.gps_sog = trueSensors.gps_sog + sensorBiases.gps_sog + random.gauss(0, sensorSigmas.gps_sog)
            self.sensorsNoisy.gps_cog = trueSensors.gps_cog + sensorBiases.gps_cog + random.gauss(0, sensorSigmas.gps_cog)
            if self.sensorsNoisy.gps_cog < -2 * math.pi:
                self.sensorsNoisy.gps_cog += 2 * math.pi
            if self.sensorsNoisy.gps_cog > 2 * math.pi:
                self.sensorsNoisy.gps_cog -= 2 * math.pi

        return self.sensorsNoisy
    
    def update(self):
        self.updateTicks += 1
        self.updateSensorsTrue(state=self.VAM.getVehicleState(), dot=self.VAM.getVehicleDynamicsModel().getVehicleDerivative(), prevTrueSensors=self.sensorsTrue)
        self.updateSensorsNoisy(trueSensors=self.sensorsTrue, noisySensors=self.sensorsNoisy, sensorBiases=self.sensorsBiases, sensorSigmas=self.sensorsSigmas)
        return