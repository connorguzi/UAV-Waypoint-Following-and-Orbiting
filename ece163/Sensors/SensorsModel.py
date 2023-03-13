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
    def __init__(self, dT:float=0.01, tau:float=1000000.0, eta:float=0.0) -> None:
        self.dT = dT
        self.tau = tau
        self.eta = eta
        self.v = 0.0 # Gauss Markov state
        return

    def reset(self):
        self.v = 0.0
        return

    def update(self, vnoise:float=None) -> float:
        """
        update the noise by 1 time step
        update the internal self.v and return the new value as well
        """
        
        if vnoise == None:
            self.v = math.e**(-self.dT/self.tau) * self.v + random.gauss(0, self.eta)
        else:
            self.v = math.e**(-self.dT/self.tau) * self.v + vnoise
        
        return self.v

class GaussMarkovXYZ():
    def __init__(self, dT:float=0.01, tauX:float=1000000.0, etaX:float=0.0, tauY:float=None,
                 etaY:float=None, tauZ:float=None, etaZ:float=None) -> None:
        
        self.dT = dT

        # X process
        self.vX = GaussMarkov(dT, tauX, etaX)

        # Y process
        if tauY == None and etaY == None:
            self.vY = GaussMarkov(dT, tauX, etaX) # use parameteres from X process
        else:
            self.vY = GaussMarkov(dT, tauY, etaY) # use its own parameteres

        # Z process
        if tauZ == None and etaZ == None:
            if tauY == None and etaY == None:
                self.vZ = GaussMarkov(dT, tauX, etaX) # use parameteres from X process
            else:
                self.vZ = GaussMarkov(dT, tauY, etaY) # use parameteres from Y process
        else:
            self.vZ = GaussMarkov(dT, tauZ, etaZ) # use its own parameteres

        return

    def reset(self):
        self.vZ.reset()
        self.vX.reset()
        self.vY.reset()
        return

    def update(self, vXnoise:float=None, vYnoise:float=None, vZnoise:float=None):
        return self.vX.update(vXnoise), self.vY.update(vYnoise), self.vZ.update(vZnoise)
    
class SensorsModel():
    def __init__(self, aeroModel=VehicleAerodynamicsModel.VehicleAerodynamicsModel(), taugyro:float=VSC.gyro_tau,
                                                            etagyro:float=VSC.gyro_eta,
                                                            tauGPS:float=VSC.GPS_tau,
                                                            etaGPSHorizontal:float=VSC.GPS_etaHorizontal,
                                                            etaGPSVertical:float=VSC.GPS_etaVertical,
                                                            gpsUpdateHz:float=VSC.GPS_rate) -> None:
        self.aeroModel = aeroModel
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsBias = Sensors.vehicleSensors()
        self.sensorsSigmas = Sensors.vehicleSensors()
        self.sensorsNoisy = Sensors.vehicleSensors()

        # intiialize sigmas and biases
        self.sensorsSigmas = self.initializeSigmas()
        self.sensorsBias = self.initializeBiases()

        # markov processes
        self.gyroMarkov = GaussMarkovXYZ(tauX=taugyro, etaX=etagyro)
        self.gpsMarkov = GaussMarkovXYZ(tauX=tauGPS, etaX=etaGPSHorizontal, tauZ=tauGPS, etaZ=etaGPSVertical)

        self.dT = aeroModel.getVehicleDynamicsModel().dT
        self.updateTicks = 0
        self.gpsTickUpdate = (1.0/gpsUpdateHz) / self.dT;

        return
    
    def getSensorsNoisy(self) -> Sensors.vehicleSensors:
        return self.sensorsNoisy
    
    def getSensorsTrue(self) -> Sensors.vehicleSensors:
        return self.sensorsTrue
    
    def initializeSigmas(self, gyroSigma:float=VSC.gyro_sigma, accelSigma:float=VSC.accel_sigma,
                                magSigma:float=VSC.mag_sigma, baroSigma:float=VSC.baro_sigma,
                                pitotSigma:float=VSC.pitot_sigma, gpsSigmaHorizontal:float=VSC.GPS_sigmaHorizontal,
                                gpsSigmaVertical:float=VSC.GPS_sigmaVertical, gpsSigmaSOG:float=VSC.GPS_sigmaSOG,
                                gpsSigmaCOG:float=VSC.GPS_sigmaCOG) -> Sensors.vehicleSensors:
        
        sigma = Sensors.vehicleSensors()
        sigma.gyro_x = gyroSigma
        sigma.gyro_y = gyroSigma
        sigma.gyro_z = gyroSigma

        sigma.accel_x = accelSigma
        sigma.accel_y = accelSigma
        sigma.accel_z = accelSigma

        sigma.mag_x = magSigma
        sigma.mag_y = magSigma
        sigma.mag_z = magSigma

        sigma.baro = baroSigma

        sigma.pitot = pitotSigma

        sigma.gps_e = gpsSigmaHorizontal
        sigma.gps_n = gpsSigmaHorizontal
        sigma.gps_alt = gpsSigmaVertical
        sigma.gps_sog = gpsSigmaSOG
        sigma.gps_cog = gpsSigmaCOG

        return sigma
    
    def initializeBiases(self, gyroBias:float=0.08726646259971647,
                                accelBias:float=0.9810000000000001,
                                magBias:float=500.0,
                                baroBias:float=100.0,
                                pitotBias:float=20.0) -> Sensors.vehicleSensors:
        
        bias = Sensors.vehicleSensors()
        bias.gyro_x = random.uniform(-gyroBias, gyroBias)
        bias.gyro_y = random.uniform(-gyroBias, gyroBias)
        bias.gyro_z = random.uniform(-gyroBias, gyroBias)

        bias.accel_x = random.uniform(-accelBias, accelBias)
        bias.accel_y = random.uniform(-accelBias, accelBias)
        bias.accel_z = random.uniform(-accelBias, accelBias)

        bias.mag_x = random.uniform(-magBias, magBias)
        bias.mag_y = random.uniform(-magBias, magBias)
        bias.mag_z = random.uniform(-magBias, magBias)

        bias.baro = random.uniform(-baroBias, baroBias)

        bias.pitot = random.uniform(-pitotBias, pitotBias)

        return bias

    def reset(self):
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsBias = Sensors.vehicleSensors()
        self.sensorsSigmas = Sensors.vehicleSensors()
        self.sensorsNoisy = Sensors.vehicleSensors()

        # intiialize sigmas and biases
        self.sensorsSigmas = self.initializeSigmas()
        self.sensorsBias = self.initializeBiases()

        self.gyroMarkov.reset()
        self.gpsMarkov.reset()
    
    def updateAccelsTrue(self, state:States.vehicleState, dot:States.vehicleState) -> tuple:
        ax = dot.u + state.q*state.w - state.r*state.v + VPC.g0*math.sin(state.pitch)
        ay = dot.v + state.r*state.u - state.p*state.w  - VPC.g0*math.cos(state.pitch)*math.sin(state.roll)
        az = dot.w + state.p*state.v - state.q*state.u - VPC.g0*math.cos(state.pitch)*math.cos(state.roll)

        return ax, ay, az

    def updateGyrosTrue(self, state:States.vehicleState):
        return state.p, state.q, state.r
    
    def updateGPSTrue(self, state:States.vehicleState, dot:States.vehicleState) -> tuple:
        gpsN = state.pn
        gpsE = state.pe
        gpsD = -state.pd

        gpsSOG = math.hypot(dot.pn, dot.pe)
        gpsCOG = math.atan2(dot.pe, dot.pn)

        return gpsN, gpsE, gpsD, gpsSOG, gpsCOG

    def updateMagsTrue(self, state:States.vehicleState) -> tuple:
        Mtrue = mm.multiply(state.R, VSC.magfield)

        return Mtrue[0][0], Mtrue[1][0], Mtrue[2][0]
    
    def updatePressureSensorsTrue(self, state:States.vehicleState) -> tuple:
        baro = VSC.Pground + VPC.rho*VPC.g0*state.pd
        pitot = VPC.rho * (state.Va**2) / 2

        return baro, pitot
    
    def updateSensorsTrue(self, prevTrueSensors:Sensors.vehicleSensors, state:States.vehicleState, dot:States.vehicleState) -> Sensors.vehicleSensors:
        self.sensorsTrue = Sensors.vehicleSensors()

        self.sensorsTrue.accel_x, self.sensorsTrue.accel_y, self.sensorsTrue.accel_z = self.updateAccelsTrue(state, dot)
        self.sensorsTrue.gyro_x, self.sensorsTrue.gyro_y, self.sensorsTrue.gyro_z = self.updateGyrosTrue(state)
        self.sensorsTrue.mag_x, self.sensorsTrue.mag_y, self.sensorsTrue.mag_z = self.updateMagsTrue(state)
        self.sensorsTrue.baro, self.sensorsTrue.pitot = self.updatePressureSensorsTrue(state)

        if self.updateTicks % self.gpsTickUpdate== 0:
            self.sensorsTrue.gps_n, self.sensorsTrue.gps_e, self.sensorsTrue.gps_alt, self.sensorsTrue.gps_sog, self.sensorsTrue.gps_cog = self.updateGPSTrue(state, dot)
        else:
            self.sensorsTrue.gps_n = prevTrueSensors.gps_n
            self.sensorsTrue.gps_e = prevTrueSensors.gps_e
            self.sensorsTrue.gps_alt = prevTrueSensors.gps_alt
            self.sensorsTrue.gps_sog = prevTrueSensors.gps_sog
            self.sensorsTrue.gps_cog = prevTrueSensors.gps_cog


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

        true = trueSensors
        bias = sensorBiases

        # update acceleration
        self.sensorsNoisy.accel_x = true.accel_x + bias.accel_x + random.gauss(0 , sensorSigmas.accel_x)
        self.sensorsNoisy.accel_y = true.accel_y + bias.accel_y + random.gauss(0, sensorSigmas.accel_y)
        self.sensorsNoisy.accel_z = true.accel_z + bias.accel_y + random.gauss(0, sensorSigmas.accel_z)

        # update gyros
        self.sensorsNoisy.gyro_x = true.gyro_x + bias.gyro_x + self.gyroMarkov.vX.update()
        self.sensorsNoisy.gyro_y = true.gyro_y + bias.gyro_y + self.gyroMarkov.vY.update()
        self.sensorsNoisy.gyro_z = true.gyro_z + bias.gyro_z + self.gyroMarkov.vZ.update()

        # update magnetometer
        self.sensorsNoisy.mag_x = true.mag_x + bias.mag_x + random.gauss(0, sensorSigmas.mag_x)
        self.sensorsNoisy.mag_y = true.mag_y + bias.mag_y + random.gauss(0, sensorSigmas.mag_y)
        self.sensorsNoisy.mag_z = true.mag_z + bias.mag_z + random.gauss(0, sensorSigmas.mag_z)

        # update GPS
        if self.updateTicks % self.gpsTickUpdate == 0:
            # sigma_w = sensorSigmas.gps_cog * VPC.InitialSpeed / se
            self.sensorsNoisy.gps_n = true.gps_n + self.gpsMarkov.vX.update() + random.gauss(0, sensorSigmas.gps_n)
            self.sensorsNoisy.gps_e = true.gps_e + self.gpsMarkov.vY.update() + random.gauss(0, sensorSigmas.gps_e)
            self.sensorsNoisy.gps_alt = true.gps_alt + self.gpsMarkov.vZ.update() + random.gauss(0, sensorSigmas.gps_alt)
            self.sensorsNoisy.gps_sog = true.gps_sog + random.gauss(0, sensorSigmas.gps_sog)
            self.sensorsNoisy.gps_cog = true.gps_cog + random.gauss(0, sensorSigmas.gps_cog)

        else:
            self.sensorsNoisy.gps_n = noisySensors.gps_n
            self.sensorsNoisy.gps_e = noisySensors.gps_e
            self.sensorsNoisy.gps_alt = noisySensors.gps_alt
            self.sensorsNoisy.gps_cog = noisySensors.gps_cog
            self.sensorsNoisy.gps_sog = noisySensors.gps_sog
            self.sensorsNoisy.gps_cog = noisySensors.gps_cog

        self.sensorsNoisy.baro = true.baro + bias.baro + random.gauss(0, sensorSigmas.baro)
        self.sensorsNoisy.pitot = true.pitot + bias.pitot + random.gauss(0, sensorSigmas.pitot)

        return self.sensorsNoisy
    
    def update(self):
        # update sensors true
        self.updateSensorsTrue(self.sensorsTrue, self.aeroModel.getVehicleState(), self.aeroModel.getVehicleDynamicsModel().getVehicleDerivative())
        self.updateSensorsNoisy(self.sensorsTrue, self.sensorsNoisy, self.sensorsBias, self.sensorsSigmas)
        self.updateTicks += 1
