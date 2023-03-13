"""This file is a test harness for the modules VehicleControlGains and VehicleClosedLoopControl. 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter5.py (from the root directory) -or-
python testChapter6.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the modules

Author: Connor Guzikowski
Email: cguzikow@ucsc.edu

"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?
sys.path.append("./")
import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Modeling.WindModel as WM
import ece163.Controls.VehicleTrim as VehicleTrim
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Containers.Controls as Controls
import ece163.Containers.Linearized as Linearized
import ece163.Controls.VehicleControlGains as VCG
import ece163.Controls.VehicleClosedLoopControl as CLC

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
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

vTrim = VehicleTrim.VehicleTrim()
Vastar = 25.0
Gammastar = 0
Kappastar = 0

#%% VCG.computeGains()
curtest = 'Compute Gains Test 1'
tuningparameters = Controls.controlTuning(Wn_roll = 1.0, Zeta_roll = 1.0, Wn_course = 1.0, Zeta_course =1.0, Wn_sideslip = 1.0, Zeta_sideslip = 1.0, Wn_pitch =1.0, Zeta_pitch = 1.0, Wn_altitude =1.0	, Zeta_altitude = 1.0, Wn_SpeedfromThrottle = 1.0, Zeta_SpeedfromThrottle =1.0, Wn_SpeedfromElevator = 1.0, Zeta_SpeedfromElevator = 1.0)
linearizedModel = VPM.CreateTransferFunction(vTrim.getTrimState(), vTrim.getTrimControls())
gains = VCG.computeGains(tuningparameters=tuningparameters, linearizedModel=linearizedModel)

if  gains.ki_roll == 0.001 and gains.ki_sideslip == 6.640161355920949:
	passed.append(curtest)
else:
	failed.append(curtest)

#%% VCG.computeTuningParameters()
curtest = 'Compute Parameters Test 1'
parameters = VCG.computeTuningParameters(gains, linearizedModel)
print(parameters)

if parameters.Wn_altitude == 1 and parameters.Zeta_roll == 1:
	passed.append(curtest)
else:
	failed.append(curtest)

#%% CLC PI, PD, and PID loops
curtest = 'PD Test 1'
testPd = CLC.PDControl(kp=1, kd=-1, lowLimit=-5, highLimit=5)
u = testPd.Update(1, 0, 1)


if u==2:
	passed.append(curtest)
else:
	failed.append(curtest)

# Testing for saturation
curtest = 'PD Test 2'
u = testPd.Update(20, 0, 1)
if u==5:
	passed.append(curtest)
else:
	failed.append(curtest)

curtest = 'PI Test 1'
testPi = CLC.PIControl(kp=1, ki=-1, lowLimit=-5, highLimit=5)
u = testPi.Update(1, 0)
if u==0.995:
	passed.append(curtest)
else:
	failed.append(curtest)

curtest = 'PI Test 2'
u = testPi.Update(3, -1)
if u==3.97:
	passed.append(curtest)
else:
	failed.append(curtest)

curtest = 'PID Test 1'
testPi = CLC.PIDControl(kp=1, ki=-1, kd=3, lowLimit=-5, highLimit=5)
u = testPi.Update(1, 0, 2)
if u==-5:
	passed.append(curtest)
else:
	failed.append(curtest)

curtest = 'PID Test 2'
testPi = CLC.PIDControl(kp=1, ki=-1, kd=3, lowLimit=-5, highLimit=5)
u = testPi.Update(4, 1, 0.2)
if u==2.385:
	passed.append(curtest)
else:
	failed.append(curtest)


#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]