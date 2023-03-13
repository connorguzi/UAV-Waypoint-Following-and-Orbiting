"""This file is a test harness for the module VehicleDynamicsModel. 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter3.py (from the root directory) -or-
python testChapter3.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehicleDynamicsModel module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?
sys.path.append("./")
import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
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


#%% VDM.init():
cur_test = "Init Test"
testVDM = VDM.VehicleDynamicsModel()
testState =  States.vehicleState()
if(testVDM.state == testState ):
	passed.append(cur_test)
else:
	failed.append(cur_test)

#%% VDM.derivative():
print("Beginning testing of VDM.derivative()")

cur_test = "Derivative test p_dot x dir"

testVDM = VDM.VehicleDynamicsModel()
testState =  States.vehicleState()
testFm = Inputs.forcesMoments()
testState.pitch = 30*math.pi/180
testState.R = Rotations.euler2DCM(pitch=30*math.pi/180)
testState.u = 10
testDot = testVDM.derivative(testState, testFm)

# print(testState)
if testDot.pd < 0:
	passed.append(cur_test)
else:
	failed.append(cur_test)

cur_test = "Derivative test HW 1 Question 1"

testVDM = VDM.VehicleDynamicsModel()
testState =  States.vehicleState()


Fd = 2.5
Fl = 4.8
Fg = VPC.mass * 9.81
Fx = -(Fd + Fg * math.sin(40*math.pi/180))
Fz = Fl - Fg * math.cos(40*math.pi/180)
testFm = Inputs.forcesMoments(Fx=Fx, Fy=0, Fz=Fz)
testState.pitch = 40*math.pi/180
testState.R = Rotations.euler2DCM(pitch=40*math.pi/180)
testDot = testVDM.derivative(testState, testFm)
actual_acc = [[testDot.u], [testDot.v], [testDot.w]]
expected_acc = [[-6.533019178297678], [0], [-7.078532350633538]]
if not evaluateTest(cur_test, compareVectors(expected_acc, actual_acc) ):
	print(f"{expected_acc} != {actual_acc}")



#%% VDM.reset()

cur_test = "Reset Test"
empty_state = States.vehicleState()

testVDM.reset()

if(testVDM.state == empty_state ):
	passed.append(cur_test)
else:
	failed.append(cur_test)

#%% VDM.ForwardEuler()

cur_test = "Forward Euler Test 1"

testVDM = VDM.VehicleDynamicsModel()
testState =  States.vehicleState()
testDot = States.vehicleState()

testDot.pn = 10

testVDM.setVehicleState(testVDM.ForwardEuler(1, testState, testDot))

if testVDM.state.pn == 10:
	passed.append(cur_test)
else:
	failed.append(cur_test)

cur_test = "Forward Euler Test 2"	
testVDM.reset()
testState =  States.vehicleState()
testDot = States.vehicleState()

testDot.pn, testDot.u, testDot.pd = -1, 20, 2.5
testVDM.setVehicleState(testVDM.ForwardEuler(1, testState, testDot))

if testVDM.state.pn == -1 and testVDM.state.u == 20 and testVDM.state.pd == 2.5:
	passed.append(cur_test)
else:
	failed.append(cur_test)

#%% VDM.Rexp()
cur_test = "Rexp Test 1"	
R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
testState =  States.vehicleState()
testDot = States.vehicleState()
testState.p, testState.q, testState.r = 1,1,1
testDot.p, testDot.q, testDot.r = 0,0,0

rexp = testVDM.Rexp(0.1, testState, testDot)
if Rotations.dcm2Euler(mm.multiply(rexp, R)) == (0.10515176406702156, 0.09465451616718257, 0.10515176406702156):
	passed.append(cur_test)
else:
	failed.append(cur_test)

cur_test = "Rexp Test 2"	
R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
testState =  States.vehicleState()
testDot = States.vehicleState()
testState.p, testState.q, testState.r = 1,19,8
testDot.p, testDot.q, testDot.r = 3,-12,20

rexp = testVDM.Rexp(0.1, testState, testDot)

if Rotations.dcm2Euler(mm.multiply(rexp, R)) == (2.3511443181439278, 0.862177348944424, 1.852621888449196):
	passed.append(cur_test)
else:
	failed.append(cur_test)

## Note: Update() and IntegrateState() don't have tests because everything in them is already tested
#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]