"""This file is a test harness for the module ece163.Utilities.Rotations,
and for the method ece163.Modeling.VehicleGeometry.getNewPoints(). 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter2.py (from the root directory) -or-
python testChapter2.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the Rotations module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?
sys.path.append("./")
import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareRows(a, b):
	el_close = [isclose(a[i], b[i]) for i in range(3)]
	return all(el_close)

def compareMatrices(A,B):
	"""A quick tool to compare two vectors"""
	if(mm.size(A) != mm.size(B)):
		return False
	el_close = [compareRows(A[i], B[i]) for i in range(len(A))]
	return all(el_close)

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


#%% Euler2dcm():
print("Beginning testing of Rotations.Euler2dcm()")

cur_test = "Euler2dcm yaw test 1"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(90*math.pi/180, 0, 0)
orig_vec = [[1],[0],[0]]
expected_vec = [[0],[-1],[0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


#%%  

cur_test = "Euler2dcm with no rotation"
R = Rotations.euler2DCM(0, 0, 0)
orig_vec = [[1],[0],[0]]
expected_vec = [[1],[0],[0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


cur_test = "Euler2dcm yaw test 2"
R = Rotations.euler2DCM(math.pi, 0, 0)
orig_vec = [[1],[0],[0]]
expected_vec = [[-1],[0],[0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")

# Rolling 90 degrees causes right wing to point down
cur_test = "Euler2dcm roll test 1"
R = Rotations.euler2DCM(0, 0, 90*math.pi/180)
orig_vec = [[0],[1],[0]]
expected_vec = [[0],[0],[-1]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")

cur_test = "Euler2dcm test using 8b from Homework0"
R = Rotations.euler2DCM(135*math.pi/180, -35.26*math.pi/180, 90*math.pi/180)

expected_mat = [[-0.5773815451999802, 0.5773815451999803, 0.5772877120855479],[0.40820405591135805, -0.4082040559113582, 0.8165408118857462],[0.7071067811865476, 0.7071067811865475, 0]]
actual_mat = R
if not evaluateTest(cur_test, compareVectors(expected_mat, actual_mat) ):
	print(f"{expected_mat} != {actual_mat}")

#%% dcm2Euler():
print("Beginning testing of Rotations.dcm2Euler()")

cur_test = "dcm2Euler test using identity Matrix"

dcm = [[1, 0, 0], [0, 1, 0], [0,0,1]]
expected_vals = [0, 0, 0]
yaw, pitch, roll = Rotations.dcm2Euler(dcm)
actual_vals = [yaw, pitch, roll]
if not evaluateTest(cur_test, compareRows(expected_vals, actual_vals) ):
	print(f"{expected_vals} != {actual_vals}")



cur_test = "dcm2Euler test using 8b from Homework0"

dcm = [[-0.5773815451999802, 0.5773815451999803, 0.5772877120855479],[0.40820405591135805, -0.4082040559113582, 0.8165408118857462],[0.7071067811865476, 0.7071067811865475, 0]]
expected_vals = [135*math.pi/180, -35.26*math.pi/180, 90*math.pi/180]
yaw, pitch, roll = Rotations.dcm2Euler(dcm)
actual_vals = [yaw, pitch, roll]
if not evaluateTest(cur_test, compareRows(expected_vals, actual_vals) ):
	print(f"{expected_vals} != {actual_vals}")

#%% ned2enu():
print("Beginning testing of Rotations.ned2enu()")

cur_test = "ned2enu() test 1"

ned = [[0, 1, 1]]
enu = Rotations.ned2enu(ned)
expected = [[1	, 0, -1]]
if not evaluateTest(cur_test, compareMatrices(expected, enu) ):
	print(f"{expected} != {enu}")

cur_test = "ned2enu() test 2"

ned = [[18, 0, 80], [45, -12, -76.1]]
enu = Rotations.ned2enu(ned)
expected = [[0, 18, -80], [-12, 45, 76.1]]
if not evaluateTest(cur_test, compareMatrices(expected, enu) ):
	print(f"{expected} != {enu}")


#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]