"""This file is a test harness for the module VehiclePerturbationModels. 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter5.py (from the root directory) -or-
python testChapter5.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehiclePerturbationModels module"""

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
import ece163.Containers.Linearized as Linearized

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


#%% PUT A TEST HERE?
vTrim = VehicleTrim.VehicleTrim()
Vastar = 25.0
Gammastar = math.radians(6.0)
Kappastar = -1.0 / 150.0

check = vTrim.computeTrim(Vastar, Kappastar, Gammastar)
if check:
 print("Optimization successful")
else:
 print("Model converged outside of valid inputs, change parameters and try again")
	   
 


#%% VPM.CreateTransferFunction()
tF = VPM.CreateTransferFunction(vTrim.getTrimState(), vTrim.getTrimControls())
curtest = 'Create Transfer Function Test 1'
expectedTf = Linearized.transferFunctions(Va_trim=25.000000000000004, alpha_trim=0.05643956420670598, 
			beta_trim=0.0, gamma_trim=0.10008261850847358, theta_trim=0.15652218271517956, 
			phi_trim=-0.39967477522059336, a_phi1=22.62885069325777, a_phi2=130.88367819945046, 
			a_beta1=0.7767725000000002,a_beta2=0.15059875000000003, a_theta1=5.29473829798902,
			 a_theta2=99.94742162885468, a_theta3=-36.11238956662998, a_V1=0.30042541657357785, 
			 a_V2=11.345223241419559, a_V3=9.760909914321076) 
if expectedTf == tF:
	passed.append(curtest)
else:
	failed.append(curtest)

#%% VPM.dThrust_dThrottle()
curtest = 'dThrust_dThrottle test 1'
Fx = VPM.dThrust_dThrottle(Va=25, Throttle=0.2)
expectedFx = 13.187036606095859
if Fx == expectedFx:
	passed.append(curtest)
else:
	failed.append(curtest)

curtest = 'dThrust_dThrottle test 2'
Fx = VPM.dThrust_dThrottle(Va=25, Throttle=0.9)
expectedFx = 123.95157539505632
if Fx == expectedFx:
	passed.append(curtest)
else:
	failed.append(curtest)

#%% VPM.dThrust_dVa()
curtest = 'dThrust_dVa test 1'
Fx = VPM.dThrust_dVa(Va=25, Throttle=0.2)
expectedFx = -1.996312526005262
if Fx == expectedFx:
	passed.append(curtest)
else:
	failed.append(curtest)

curtest = 'dThrust_dVa test 2'
Fx = VPM.dThrust_dVa(Va=15, Throttle=0.2)
expectedFx = -1.2754521695461722
if Fx == expectedFx:
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