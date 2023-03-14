"""
Testing for path following algorythm
"""
import sys
sys.path.append("./")  # python is horrible, no?
sys.path.append("..")  # python is horrible, no?

import math
import PathFollowing
import ece163.Containers.States as States

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
def isclose(a, b): return math.isclose(a, b, abs_tol=1e-12)


def compareVectors(a, b):
    """A quick tool to compare two vectors"""
    el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
    return all(el_close)


def compareMatrix(a, b):
    '''A quick tool to compare two matrices'''
    similar = True

    for i in range(len(a)):
        for j in range(len(a[i])):
            similar = similar and (isclose(a[i][j], b[i][j]))

    return similar


def compareList_String(a, b):
    '''A quick tool to compare two lists of strings'''
    similar = True

    for i in range(len(a)):
        similar = similar and (a[i] == b[i])

    return similar

assert (compareVectors([[0], [0], [-1]], [[1e-13], [0], [-1+1e-9]]))
assert (not compareVectors([[0], [0], [-1]], [[1e-11], [0], [-1]]))
assert (not compareVectors([[1e8], [0], [-1]], [[1e8+1], [0], [-1]]))

assert (compareMatrix([[1, 2, 3], [-4, -5, -6], [0.7, 0.8, 0.9]],
                      [[1, 2, 3], [-4, -5, -6], [0.7, 0.8, 0.9]]))
assert (not compareMatrix([[-1, 2, 3], [-4, -5, -6], [0.7, 0.8, 0.9]],
                          [[1, 2, 3], [-4, -5, -6], [0.7, 0.8, 0.9]]))
assert (not compareMatrix([[1e8, 2, 3], [-4, -5, -6], [0, 0.8, 0.9]],
                          [[1e8+1, 2, 3], [-4, -5, -6], [7e-11, 0.8, 0.9]]))

passed = []
failed = []
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

### PATH FOLLOWING EQUATIONS TEST ###
# this is for numerical testing of each individual function

######################################
print("Testing CalcPathCourseAngle()")
q = [[2.0], [3.0], [4.0]]
cur_test = "CalcPathCourseAngle Normal"
PathFollowing.CalcPathCourseAngle(q)

if PathFollowing.CalcPathCourseAngle(q) == 0.5880026035475675:
    print("Passed test 1")
    passed.append(cur_test)
else:
    print("Failed test 1")
    failed.append(cur_test)

q = [[0.0], [1.0], [4.0]]
cur_test = "CalcPathCourseAngle qn=0"
PathFollowing.CalcPathCourseAngle(q)

if PathFollowing.CalcPathCourseAngle(q) == 0.0:
    print("Passed test 2")
    passed.append(cur_test)
else:
    print("Failed test 2")
    failed.append(cur_test)

q = [[1.0], [0.0], [4.0]]
cur_test = "CalcPathCourseAngle qe=0"
PathFollowing.CalcPathCourseAngle(q)

if PathFollowing.CalcPathCourseAngle(q) == 1.5707963267948966:
    print("Passed test 3")
    passed.append(cur_test)
else:
    print("Failed test 3")
    failed.append(cur_test)

######################################
print("Testing CalcR_Inertial2Path()")
cur_test = "CalcR_Inertial2Path chi=0"
chi = 0
R = PathFollowing.CalcR_Inertial2Path(chi)
expected = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

if compareMatrix(expected, R):
    print("Passed test 1")
    passed.append(cur_test)
else:
    print("Failed test 1")
    failed.append(cur_test)

cur_test = "CalcR_Inertial2Path chi=pi/2"
chi = math.pi / 2
R = PathFollowing.CalcR_Inertial2Path(chi)
expected = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]

if compareMatrix(expected, R):
    print("passed test 2")
    passed.append(cur_test)
else:
    print("Failed test 2")
    failed.append(cur_test)

#####################################
print("Test CalcRelativePathError()")
cur_test = "CalcRelativePathError at origin"
state = States.vehicleState(pn=0.0, pe=0.0, pd=0.0)
o = [[1.0], [1.0], [1.0]]
R = PathFollowing.CalcR_Inertial2Path(0)
expected = [[-1.0], [-1.0], [-1.0]]
error = PathFollowing.CalcRelativePathError(state, o, R)

if compareVectors(expected, error):
    print("Passed test 1")
    passed.append(cur_test)
else:
    print("Failed test 1")
    failed.append(cur_test)

cur_test = "CalcRelativePathError moved"
state.pn = 2.0
state.pe = 3.0
state.pd = 4.0
expected = [[1.0], [2.0], [3.0]]
error = PathFollowing.CalcRelativePathError(state, o, R)

if compareVectors(expected, error):
    print("Passed test 2")
    passed.append(cur_test)
else:
    print("Failed test 2")
    failed.append(cur_test)

######################################
print("Testing CalcCommandedCourse()")
cur_test = "CalcCommandedCourse"
q = [[0.0], [1.0], [2.0]]
chi_inf = math.pi / 2.0
k_path = 10
course = PathFollowing.CalcCommandedCourse(q=q, origin=o, chi_inf=chi_inf, k_path=10, state=state)
expected = -1.520837931

if isclose(expected, course):
    print("Passed test")
    passed.append(cur_test)
else:
    print("Failed test")
    failed.append(cur_test)

#################################################
print("Testing CalcUnitNormalVector")
q = [[1.0], [0.0], [0.0]]
unit_vector = PathFollowing.CalcUnitNormalVector(q)
expected = [[0.0], [-1.0], [0.0]]

if compareVectors(expected, unit_vector):
    print("Passed Test")
else:
    print("Failed Test")

# print(unit_vector)

#################################################
print("Testing CalcProjectedRelativeErrorVector")
state = States.vehicleState(pn = 2.0, pe=3.0, pd=4.0)
o = [[1.0], [1.0], [1.0]]

projected_error = PathFollowing.CalcProjectedRelativeErrorVector(state=state, n=unit_vector, origin=o)
expected = [[1.0], [0.0], [3.0]]
if compareVectors(expected, projected_error):
    print("Passed Test")
else:
    print("Failed Test")

####################################
print("Testing CalcCOmmandedHeight")
commanded_height = PathFollowing.CalcCommandedHeight(projected_error, o, q)

if commanded_height == -1.0:
    print("Passed Test")
else:
    print("Failed Test")











### PATH FOLLOWING CONCEPTUAL TEST ###
# this is for graphing some path following interactions