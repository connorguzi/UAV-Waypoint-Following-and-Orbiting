"""This file is a test harness for the module VehicleAerodynamicsModel. 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter4.py (from the root directory) -or-
python testChapter4.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehicleAerodynamicsModel module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?
sys.path.append("./")
import numpy as np
import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Modeling.VehicleAerodynamicsModel as VAM
import ece163.Modeling.WindModel as WM
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

def compare2x1Vectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(2)]
	return all(el_close)

def compareRows(a, b):
	el_close = [isclose(a[i], b[i]) for i in range(2)]
	return all(el_close)

def compareMatrices(A,B):
	"""A quick tool to compare two matrices"""
	if(mm.size(A) != mm.size(B)):
		return False
	el_close = [compareRows(A[i], B[i]) for i in range(len(A))]
	return all(el_close)
#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-11], [0], [-1]]))
assert(not compareVectors([[1e8], [0], [-1]],[[1e8+1], [0], [-1]]))

failed = []
passed = []
def evaluateTest(test_name, boolean):
	"""evaluateTest prints the output of a test and adds it to one of two 
	global lists, passed and failed, which can be printed later"""
	if boolean:
		print(f"   passed {test_name}")
		passed.append(test_name)
	else:
		print(f"   failed {test_name}")
		failed.append(test_name)
	return boolean

#%% VAM.init():
print("Testing Init() function")
cur_test = "Init Test"
testVAM = VAM.VehicleAerodynamicsModel()
testState =  States.vehicleState(pd=-100, u=25)
if(testVAM.vehicleDynamics.state == testState ):
	passed.append(cur_test)
else:
	failed.append(cur_test)

#%% VAM.gravityForces():
print("Testing gravityForces() function")
cur_test = "Gravity Test 1"
forces = testVAM.gravityForces(testVAM.vehicleDynamics.state)
if(forces.Fz == VPC.mass*VPC.g0 ):
	passed.append(cur_test)
else:
	failed.append(cur_test)

cur_test = "Gravity Test 2"
testVAM.vehicleDynamics.state = States.vehicleState(pitch=90 * math.pi / 180, pd=-100, u=25)
forces = testVAM.gravityForces(testVAM.vehicleDynamics.state)

if(forces.Fx == -VPC.mass*VPC.g0 ):
	passed.append(cur_test)
else:
	failed.append(cur_test)
#%% VAM.CalculateCoeff_alpha():
print("Testing CalculateCoeff_alpha() function")
cur_test = "Alpha Test 1"
CLA_Pos = testVAM.CalculateCoeff_alpha(20 * math.pi / 180)[0]
CLA_Neg = testVAM.CalculateCoeff_alpha(-20 * math.pi / 180)[0]
if(CLA_Neg < 0) and (CLA_Pos > 0):
	passed.append(cur_test)
else:
	failed.append(cur_test)

# Making sure that CLalpha term drops off after a0
cur_test = "Alpha Test 2"
CLA_Max = testVAM.CalculateCoeff_alpha(27 * math.pi / 180)[0]
CLA_Lower = testVAM.CalculateCoeff_alpha(27.1 * math.pi / 180)[0]
if(CLA_Max > CLA_Lower):
	passed.append(cur_test)
else:
	failed.append(cur_test)

# Making sure that CDAlpha increases with alpha - even post stall
cur_test = "Alpha Test 3"
CDA_Higher = testVAM.CalculateCoeff_alpha(70 * math.pi / 180)[1]
CDA_Lower = testVAM.CalculateCoeff_alpha(27 * math.pi / 180)[1]
if(CDA_Higher > CDA_Lower):
	passed.append(cur_test)
else:
	failed.append(cur_test)
    
# Quick CMAlpha test
cur_test = "Alpha Test 4"
CMA = testVAM.CalculateCoeff_alpha(90 * math.pi / 180)[2]

if(CMA == (VPC.CM0 + VPC.CMalpha * (90 * math.pi / 180))):
	passed.append(cur_test)
else:
	failed.append(cur_test)

#%% VAM.aeroForces():
print("Testing aeroForces() function")
testVAM.reset()
cur_test = "Aero Forces Test 1"
AeroForces1 = testVAM.aeroForces(testVAM.vehicleDynamics.state)
# Negative Force = Lift
if(AeroForces1.Fz < 0):
	passed.append(cur_test)
else:
	failed.append(cur_test)

# More lift and drag with more speed
cur_test = "Aero Forces Test 2"
testState = States.vehicleState(pd=-100, u=50)
testVAM.setVehicleState(testState)
AeroForces2 = testVAM.aeroForces(testVAM.vehicleDynamics.state)
if((AeroForces1.Fz > AeroForces2.Fz) and (AeroForces1.Fx > AeroForces2.Fx)):
	passed.append(cur_test)
else:
	failed.append(cur_test)

#%% VAM.CalculatePropForces():
print("Testing CalculatePropForces() function")
testVAM.reset()
testInput = Inputs.controlInputs(Throttle=0)
cur_test = "Prop Test 1"
PropForces1 = testVAM.CalculatePropForces(testVAM.vehicleDynamics.state.Va, testInput.Throttle)
# Negative Force from prop
if(PropForces1[0] < 0):
	passed.append(cur_test)
else:
	failed.append(cur_test)

# More Force with more throttle
cur_test = "Prop Test 2"
testInput = Inputs.controlInputs(Throttle=0.26)
PropForces2 = testVAM.CalculatePropForces(testVAM.vehicleDynamics.state.Va, testInput.Throttle)
if((PropForces1[0] < PropForces2[0]) and (PropForces1[1] > PropForces2[1])):
	passed.append(cur_test)
else:
	failed.append(cur_test)

#%% VAM.controlForces():
print("Testing controlForces() function")
testVAM.reset()
testInput = Inputs.controlInputs(Throttle=0, Elevator=5*math.pi/180)
cur_test = "Controls Test 1"
ControlForces1 = testVAM.controlForces(testVAM.vehicleDynamics.state, testInput)
# Making sure lift and drag are in right direction
if(ControlForces1.Fx < 0 and ControlForces1.Fz < 0 ):
	passed.append(cur_test)
else:
	failed.append(cur_test)

# More Force with more elevator
cur_test = "Controls Test 2"
testInput = Inputs.controlInputs(Throttle=0, Elevator=10*math.pi/180)
ControlForces2 = testVAM.controlForces(testVAM.vehicleDynamics.state, testInput)
if((ControlForces1.Fx > ControlForces2.Fx) and (ControlForces1.Fz > ControlForces2.Fz)):
	passed.append(cur_test)
else:
	failed.append(cur_test)

#%% VAM.updateForces():
print("Testing updateForces() function")
testVAM.reset()
testInput = Inputs.controlInputs(Throttle=0, Elevator=5*math.pi/180)
cur_test = "Update Test 1"
AeroForces = testVAM.aeroForces(testVAM.getVehicleState())
ControlForces = testVAM.controlForces(testVAM.getVehicleState(), testInput)
GravityForces = testVAM.gravityForces(testVAM.getVehicleState())
TotalForces = testVAM.updateForces(testVAM.vehicleDynamics.state, testInput)
# All of the previous functions work, making sure the correct forces are being summed now
if(TotalForces.Fx == AeroForces.Fx + ControlForces.Fx + GravityForces.Fx):
	passed.append(cur_test)
else:
	failed.append(cur_test)

#%% VAM.CalculateAirSpeed():
cur_test = "Airspeed Test 1"
testVAM.reset()
testWind = States.windState(Wn=25)

if testVAM.CalculateAirspeed(testVAM.getVehicleState(), testWind)[0] == 0:
	passed.append(cur_test)
else:
	failed.append(cur_test)

cur_test = "Airspeed Test 2"
testVAM.reset()
testWind = States.windState(Wn=5, Wu=-5, Ww=3, We= 3)
if testVAM.CalculateAirspeed(testVAM.getVehicleState(), testWind) == (24.475778077104778, -0.12289800094596329, -0.017468003017585935):
	passed.append(cur_test)
else:
	failed.append(cur_test)

#%% WM.CreateDrydenTransferFns():
cur_test = "Dryden Transfer Functions Test 1"
testWM = WM.WindModel()
testWM.CreateDrydenTransferFns(VPC.dT, 25, testWM.drydenParameters)

if compare2x1Vectors(mm.transpose(testWM.H_v), [[1], [1]]):
	passed.append(cur_test)
else:
	failed.append(cur_test)

cur_test = "Dryden Transfer Functions Test 2"
testWM = WM.WindModel(drydenParamters=VPC.DrydenLowAltitudeLight)
testWM.CreateDrydenTransferFns(VPC.dT, 25, testWM.drydenParameters)

expected_Hu = 0.29902047928031084
expected_phi = [[0.9900374167967189, -0.002487531197981706], [0.009950124791926824, 0.9999875415886457]]
if compareMatrices((testWM.Phi_w), expected_phi) and testWM.H_u[0][0] == expected_Hu:
	passed.append(cur_test)
else:
	failed.append(cur_test)

#%% WM.Update():
cur_test = "Wind Model Update Test 1"
testWM = WM.WindModel()
hv = []
for i in range(1000):
	testWM.Update()
	hv.append(testWM.H_v)


if np.std(hv) == 0:
	passed.append(cur_test)
else:
	failed.append(cur_test)

cur_test = "Wind Model Update Test 2"
testWM = WM.WindModel(drydenParamters=VPC.DrydenLowAltitudeLight)
hv = []
for i in range(1000):
	testWM.Update()
	hv.append(testWM.H_u)
print(np.std(hv))

if isclose(np.std(hv), 0):
	passed.append(cur_test)
else:
	failed.append(cur_test)
#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]