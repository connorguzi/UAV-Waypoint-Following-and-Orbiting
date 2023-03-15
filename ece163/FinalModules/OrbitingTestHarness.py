"""
Author: Bailen Lawson (bjlawson@ucsc.edu), #1682078
This file is a test harness for the module Orbiting.py.

It is meant to be run from the Testharnesses directory of the repo
with:

python ./FinalModules/OrbitingTestHarness.py (from the root directory) -or-
python OrbitingTestHarness.py (from inside the FinalModules directory)

at which point it will execute various tests on the SensorsModel modules
"""

# %% Initialization of test harness and helpers:

# autopep8: off
import sys
import Orbiting
import WayPoint

sys.path.append("./")  # python is horrible, no?
sys.path.append("..")  # python is horrible, no?

import os
import math
import copy
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations

import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Modeling.WindModel as WM
import ece163.Modeling.VehicleAerodynamicsModel as VAM

import ece163.Controls.VehicleTrim as VehicleTrim
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Controls.VehicleControlGains as VCG
import ece163.Controls.VehicleClosedLoopControl as VCLC

import ece163.Sensors.SensorsModel as SM

import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Containers.Linearized as Linearized
import ece163.Containers.Controls as Controls
import ece163.Containers.Sensors as Sensors

import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Constants.VehicleSensorConstants as VSC
# autopep8: on


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


# of course, you should test your testing tools too:
assert (compareVectors([[0], [0], [-1]], [[1e-13], [0], [-1+1e-9]]))
assert (not compareVectors([[0], [0], [-1]], [[1e-11], [0], [-1]]))
assert (not compareVectors([[1e8], [0], [-1]], [[1e8+1], [0], [-1]]))

assert (compareMatrix([[1, 2, 3], [-4, -5, -6], [0.7, 0.8, 0.9]],
                      [[1, 2, 3], [-4, -5, -6], [0.7, 0.8, 0.9]]))
assert (not compareMatrix([[-1, 2, 3], [-4, -5, -6], [0.7, 0.8, 0.9]],
                          [[1, 2, 3], [-4, -5, -6], [0.7, 0.8, 0.9]]))
assert (not compareMatrix([[1e8, 2, 3], [-4, -5, -6], [0, 0.8, 0.9]],
                          [[1e8+1, 2, 3], [-4, -5, -6], [7e-11, 0.8, 0.9]]))


def compareObjects(a, b):
    '''
    A quick tool to compare two objects. The objects must be the same, and the
    objects cannot have lists of dictionaries as variables. The objects may have
    lists, numbers (ints, floats), and other objects as parameters.

    Parameters:
    {a} An object to compare
    {b} An object to compare

    Returns:
    {same}          True if the states are the same, false otherwise
    {failedValues}  list, [item, val_a, val_b]. Empty if the states are the same
                    If an object is a variable, returns [item, [item, val_a, val_b]]
    '''
    # Initialize Variables
    vars_a = vars(a)
    vars_b = vars(b)

    valsMatched = True
    failedValues = []

    for key in vars_a.keys():
        # Handle object values
        if isinstance(vars_a[key], VDM.VehicleDynamicsModel) \
                or isinstance(vars_a[key], VAM.VehicleAerodynamicsModel) \
                or isinstance(vars_a[key], WM.WindModel) \
                or isinstance(vars_a[key], Inputs.controlInputs) \
                or isinstance(vars_a[key], Inputs.forcesMoments) \
                or isinstance(vars_a[key], Inputs.drydenParameters) \
                or isinstance(vars_a[key], States.vehicleState) \
                or isinstance(vars_a[key], States.windState) \
                or isinstance(vars_a[key], Controls.controlGains) \
                or isinstance(vars_a[key], Controls.controlTuning) \
                or isinstance(vars_a[key], VCLC.PIControl) \
                or isinstance(vars_a[key], VCLC.PDControl) \
                or isinstance(vars_a[key], VCLC.PIDControl) \
                or isinstance(vars_a[key], Sensors.vehicleSensors) \
                or isinstance(vars_a[key], SM.GaussMarkov) \
                or isinstance(vars_a[key], SM.GaussMarkovXYZ) \
                or isinstance(vars_a[key], SM.SensorsModel):
            matched, tempFailedValues = compareObjects(
                vars_a[key], vars_b[key])
            if not matched:
                failedValues.append([key, tempFailedValues])

        # Handle list values
        elif isinstance(vars_a[key], list):
            matched = compareMatrix(vars_a[key], vars_b[key])
            if not matched:
                failedValues.append([key, vars_a[key], vars_b[key]])

        # Handle strings
        elif isinstance(vars_a[key], str):
            matched = vars_a[key] == vars_b[key]
            if not matched:
                failedValues.append([key, vars_a[key], vars_b[key]])

        # Handle other values
        else:
            matched = isclose(vars_a[key], vars_b[key])
            if not matched:
                failedValues.append([key, vars_a[key], vars_b[key]])

    # Return if the objects are the same, and return all failed values (if any)
    objectsSame = len(failedValues) == 0
    return objectsSame, failedValues


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


def printErrorMessage_compareObject(failedVals=[], keyName="key", obj1="obj1", obj2="obj2", frontSpacing=8):
    '''A quick tool to print all failed values obtained when comparing two Objects'''
    print(f"{'': <{frontSpacing}}{'|' : <2}{keyName : <30}| {obj1 : <18}| {obj2 : <18}")
    print(f"{'': <{frontSpacing}}{'|' : <1}{'' :-<31}|{'' :-<19}|{'' :-<19}")
    for i in failedVals:
        # Check for nested lists
        if (i[0] == "R") or (i[0] == "Phi_u") or (i[0] == "Phi_v") or (i[0] == "Phi_w")\
                or (i[0] == "Gamma_u") or (i[0] == "Gamma_v") or (i[0] == "Gamma_w") \
                or (i[0] == "H_u") or (i[0] == "H_v") or (i[0] == "H_w") \
                or (i[0] == "xu") or (i[0] == "xv") or (i[0] == "xw"):
            print(f"{'': <{frontSpacing}}{'|' : <2}{i[0] : <10}")
            mm.matrixPrint(i[1])
            print("")
            mm.matrixPrint(i[2])
        elif isinstance(i[1], list):
            print(f"{'': <{frontSpacing}}{'|' : <2}{i[0] : <10}")
            printErrorMessage_compareObject(
                i[1], keyName, obj1, obj2, frontSpacing+5)
        elif isinstance(i[1], str):
            print(
                f"{'': <{frontSpacing}}{'|' : <2}{i[0] : <30}| {i[1]: <18}| {i[2]: <18}")
        else:
            print(
                f"{'': <{frontSpacing}}{'|' : <2}{i[0] : <30}| {i[1]: <18}| {i[2]: <18}")

# Test Tools


# def testing_tools():
    print("\nBeginning testing of 'tool' functions:")

    # %% Testing compareObjects()
    cur_test = "compareObjects Same 1: forcesMoments"
    state1 = Inputs.forcesMoments()
    state2 = Inputs.forcesMoments()

    statesSame, failedVals = compareObjects(state1, state2)
    if not evaluateTest(cur_test, statesSame):
        printErrorMessage_compareObject(
            failedVals, keyName="Value", obj1="State 1", obj2="State 2")

    # %%
    cur_test = "compareObjects Different 1: forcesMoments"
    state1 = Inputs.forcesMoments()
    state2 = Inputs.forcesMoments()

    state1.Fx = 1
    state1.Fz = 3
    state1.My = 5

    state2.Fy = 2
    state2.Mx = 4
    state2.Mz = 6

    statesSame, failedVals = compareObjects(state1, state2)
    if not evaluateTest(cur_test, not statesSame):
        printErrorMessage_compareObject(
            failedVals, keyName="Value", obj1="State 1", obj2="State 2")

    # %%
    cur_test = "compareObjects Same 2: VDM"
    state1 = VAM.VehicleAerodynamicsModel()
    state2 = VAM.VehicleAerodynamicsModel()

    statesSame, failedVals = compareObjects(state1, state2)
    if not evaluateTest(cur_test, statesSame):
        printErrorMessage_compareObject(
            failedVals, keyName="Value", obj1="State 1", obj2="State 2")

    # %%
    cur_test = "compareObjects Different 2: VAM"
    state1 = VAM.VehicleAerodynamicsModel()
    state2 = VAM.VehicleAerodynamicsModel()

    # Positions
    state1.VDM.state.pn = 1
    state1.VDM.state.pe = 1
    state1.VDM.state.pd = 1
    state1.VDM.dot.pn = 1
    state1.VDM.dot.pe = 1
    state1.VDM.dot.pd = 1

    # Velocities
    state1.VDM.state.u = 1
    state1.VDM.state.v = 1
    state1.VDM.state.w = 1
    state1.VDM.dot.u = 1
    state1.VDM.dot.v = 1
    state1.VDM.dot.w = 1

    # Euler Angles
    state1.VDM.state.yaw = 1
    state1.VDM.state.pitch = 1
    state1.VDM.state.roll = 1
    state1.VDM.dot.yaw = 1
    state1.VDM.dot.pitch = 1
    state1.VDM.dot.roll = 1

    # DCM
    state1.VDM.dot.R = \
        Rotations.euler2DCM(state1.VDM.state.yaw, state1.VDM.state.pitch,
                            state1.VDM.state.roll)
    state1.VDM.state.R = \
        Rotations.euler2DCM(state1.VDM.state.yaw, state1.VDM.state.pitch,
                            state1.VDM.state.roll)

    # Rotation Rates
    state1.VDM.state.p = 1
    state1.VDM.state.q = 1
    state1.VDM.state.r = 1
    state1.VDM.dot.p = 1
    state1.VDM.dot.q = 1
    state1.VDM.dot.r = 1

    # Airspeed & Flight Angles
    state1.VDM.state.Va = 1
    state1.VDM.state.alpha = 1
    state1.VDM.state.beta = 1
    state1.VDM.state.chi = 1
    state1.VDM.dot.Va = 1
    state1.VDM.dot.alpha = 1
    state1.VDM.dot.beta = 1
    state1.VDM.dot.chi = 1

    statesSame, failedVals = compareObjects(state1, state2)
    if not evaluateTest(cur_test, not statesSame):
        printErrorMessage_compareObject(
            failedVals, keyName="Value", obj1="State 1", obj2="State 2")

# Test Orbiting


def testing_Orbiting_CalcDistFromCenter():
    print("\nBeginning testing of Orbiting.CalcDistFromCenter:")

    # %%
    cur_test = "Orbiting.CalcDistFromCenter Test 1: All 0"

    testState = States.vehicleState(
        pn=0,
        pe=0,
        pd=0
    )
    testCenter = [
        [0],
        [0],
        [0]
    ]

    ref_d = 0
    act_d = Orbiting.CalcDistFromCenter(testState, testCenter)

    if not evaluateTest(cur_test, isclose(ref_d, act_d)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <30}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<31}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'d' : <30}| {ref_d : <18}| {act_d : <18}")

    # %%
    cur_test = "Orbiting.CalcDistFromCenter Test 2: 0 state, moved center"

    testState = States.vehicleState(
        pn=0,
        pe=0,
        pd=0
    )
    testCenter = [
        [3],
        [4],
        [5]
    ]

    ref_d = 5
    act_d = Orbiting.CalcDistFromCenter(testState, testCenter)

    if not evaluateTest(cur_test, isclose(ref_d, act_d)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'d' : <6}| {ref_d : <18}| {act_d : <18}")

    # %%
    cur_test = "Orbiting.CalcDistFromCenter Test 3: moved state, 0 center"

    testState = States.vehicleState(
        pn=3,
        pe=4,
        pd=5
    )
    testCenter = [
        [0],
        [0],
        [0]
    ]

    ref_d = 5
    act_d = Orbiting.CalcDistFromCenter(testState, testCenter)

    if not evaluateTest(cur_test, isclose(ref_d, act_d)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'d' : <6}| {ref_d : <18}| {act_d : <18}")

    # %%
    cur_test = "Orbiting.CalcDistFromCenter Test 4: moved state, moved center"

    testState = States.vehicleState(
        pn=-5,
        pe=201,
        pd=5
    )
    testCenter = [
        [-8],
        [197],
        [-9.0]
    ]

    ref_d = 5
    act_d = Orbiting.CalcDistFromCenter(testState, testCenter)

    if not evaluateTest(cur_test, isclose(ref_d, act_d)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'d' : <6}| {ref_d : <18}| {act_d : <18}")


def testing_Orbiting_CalcAngleAlongCircle():
    print("\nBeginning testing of Orbiting.CalcAngleAlongCircle:")

    # %%
    cur_test = "Orbiting.CalcAngleAlongCircle Test 1: All 0"

    testState = States.vehicleState(
        pn=0,
        pe=0,
        pd=0
    )
    testCenter = [
        [0],
        [0],
        [0]
    ]

    ref_phi = 0
    act_phi = Orbiting.CalcAngleAlongCircle(testState, testCenter)

    if not evaluateTest(cur_test, isclose(ref_phi, act_phi)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'phi' : <6}| {ref_phi : <18}| {act_phi : <18}")

    # %%
    cur_test = "Orbiting.CalcAngleAlongCircle Test 2: 0 state, moved center"

    testState = States.vehicleState(
        pn=0,
        pe=0,
        pd=0
    )
    testCenter = [
        [-1],
        [2],
        [3]
    ]

    ref_phi = math.radians(-63.434948822922)
    act_phi = Orbiting.CalcAngleAlongCircle(testState, testCenter)

    if not evaluateTest(cur_test, isclose(ref_phi, act_phi)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'phi' : <6}| {ref_phi : <18}| {act_phi : <18}")

    # %%
    cur_test = "Orbiting.CalcAngleAlongCircle Test 3: moved state, 0 center"

    testState = States.vehicleState(
        pn=-1,
        pe=-2,
        pd=3
    )
    testCenter = [
        [0],
        [0],
        [0]
    ]

    ref_phi = math.radians(63.434948822922-180)
    act_phi = Orbiting.CalcAngleAlongCircle(testState, testCenter)

    if not evaluateTest(cur_test, isclose(ref_phi, act_phi)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'phi' : <6}| {ref_phi : <18}| {act_phi : <18}")

    # %%
    cur_test = "Orbiting.CalcAngleAlongCircle Test 4: moved state, moved center"

    testState = States.vehicleState(
        pn=-1,
        pe=-2,
        pd=3
    )
    testCenter = [
        [4],
        [5],
        [6]
    ]

    ref_phi = math.radians(54.462322208026-180)
    act_phi = Orbiting.CalcAngleAlongCircle(testState, testCenter)

    if not evaluateTest(cur_test, isclose(ref_phi, act_phi)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'phi' : <6}| {ref_phi : <18}| {act_phi : <18}")


def testing_Orbiting_CalcCommandedHeight():
    print("\nBeginning testing of Orbiting.CalcCommandedHeight:")

    # %%
    cur_test = "Orbiting.CalcCommandedHeight Test 1: cd = 0"

    testCenter = [
        [0],
        [0],
        [0]
    ]

    ref_h = 0
    act_h = Orbiting.CalcCommandedHeight(testCenter)

    if not evaluateTest(cur_test, isclose(ref_h, act_h)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'h' : <6}| {ref_h : <18}| {act_h : <18}")

    # %%
    cur_test = "Orbiting.CalcCommandedHeight Test 2: All cd = -100"

    testCenter = [
        [0],
        [0],
        [-100]
    ]

    ref_h = 100
    act_h = Orbiting.CalcCommandedHeight(testCenter)

    if not evaluateTest(cur_test, isclose(ref_h, act_h)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'h' : <6}| {ref_h : <18}| {act_h : <18}")


def testing_Orbiting_CalcCommandedCourse():
    print("\nBeginning testing of Orbiting.CalcCommandedCourse:")

    # %%
    cur_test = "Orbiting.CalcCommandedCourse Test 1: All 0"

    testState = States.vehicleState(
        pn=0,
        pe=0,
        pd=0
    )
    testCenter = [
        [0],
        [0],
        [0]
    ]
    dir = 1
    rho = 1
    k_orbit = 0

    ref_chi = math.pi/2
    act_chi = Orbiting.CalcCommandedCourse(
        testState, testCenter, dir, rho, k_orbit)

    if not evaluateTest(cur_test, isclose(ref_chi, act_chi)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'chi' : <6}| {ref_chi : <18}| {act_chi : <18}")

    # %%
    cur_test = "Orbiting.CalcCommandedCourse Test 1: Basic Values"

    testState = States.vehicleState(
        pn=1,
        pe=2,
        pd=0
    )
    testCenter = [
        [-3],
        [-1],
        [0]
    ]
    dir = 1
    rho = 10
    k_orbit = 2

    ref_chi = 1.4288992721907
    act_chi = Orbiting.CalcCommandedCourse(
        testState, testCenter, dir, rho, k_orbit)

    if not evaluateTest(cur_test, isclose(ref_chi, act_chi)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'chi' : <6}| {ref_chi : <18}| {act_chi : <18}")


def testing_Orbiting_getCommandedInputs():
    print("\nBeginning testing of Orbiting.getCommandedInputs:")

    # %%
    cur_test = "Orbiting.getCommandedInputs Test 1: All 0"

    testState = States.vehicleState(
        pn=0,
        pe=0,
        pd=0
    )
    testWaypoint = WayPoint.WayPoint(
        n = 0,
        e = 0,
        d = 0,
        radius = 1,
        direction = 1,
    )
    k_orbit = 0

    ref_chi = math.pi/2
    ref_h = 0
    act_h, act_chi = Orbiting.getCommandedInputs(
        testState, testWaypoint, k_orbit)

    if not evaluateTest(cur_test, isclose(ref_chi, act_chi) and isclose(ref_h, act_h)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'h' : <6}| {ref_h : <18}| {act_h : <18}")
        print(f"{'': <{8}}{'|' : <2}{'chi' : <6}| {ref_chi : <18}| {act_chi : <18}")

    # %%
    cur_test = "Orbiting.getCommandedInputs Test 2: Basic Values"

    testState = States.vehicleState(
        pn=1,
        pe=2,
        pd=-100
    )
    testWaypoint = WayPoint.WayPoint(
        n = -3,
        e = -1,
        d = -200,
        radius = 10,
        direction = 1,
    )
    k_orbit = 2

    ref_chi = 1.4288992721907
    ref_h = 200
    act_h, act_chi = Orbiting.getCommandedInputs(
        testState, testWaypoint, k_orbit)

    if not evaluateTest(cur_test, isclose(ref_chi, act_chi) and isclose(ref_h, act_h)):
        print(f"{'': <{8}}{'|' : <2}{'attr' : <6}| {'ref' : <18}| {'act' : <18}")
        print(f"{'': <{8}}{'|' : <1}{'' :-<7}|{'' :-<19}|{'' :-<19}")
        print(f"{'': <{8}}{'|' : <2}{'h' : <6}| {ref_h : <18}| {act_h : <18}")
        print(f"{'': <{8}}{'|' : <2}{'chi' : <6}| {ref_chi : <18}| {act_chi : <18}")


def testing_Orbiting_Graphical_InitOnOrbit(gains, printPlots=False):
    print("\nBeginning graphical tests of Orbiting:")

    # %%
    cur_test = "Orbiting.getCommandedInputs Test 1: Initial point on orbit path"

    vclc = VCLC.VehicleClosedLoopControl()
    vclc.setControlGains(gains)
    Va = 20
    vclc.setVehicleState(States.vehicleState(
        pn=100,
        pe=0,
        pd=-100,
        u=Va
    ))
    testWaypoint = WayPoint.WayPoint(
        n = 0,
        e = 0,
        d = -100,
        radius = 100,
        direction = 1,
    )
    k_orbit = 1

    dT = vclc.getVehicleAerodynamicsModel().getVehicleDynamicsModel().dT
    totalTime = 180
    breakTime = totalTime / 2
    breakStep = int(breakTime/dT)
    n_steps = int(totalTime/dT)
    t_data = [i*dT for i in range(n_steps)]

    chi_c = [0 for i in range(n_steps)]
    chi_t = [0 for i in range(n_steps)]
    chi_e = [0 for i in range(n_steps)]
    h_c = [0 for i in range(n_steps)]
    h_t = [0 for i in range(n_steps)]
    h_e = [0 for i in range(n_steps)]
    x = [0 for i in range(n_steps)]
    y = [0 for i in range(n_steps)]
    z = [0 for i in range(n_steps)]

    for i in range(n_steps):        
        # Update reference commands
        h_c[i], chi_c[i] = Orbiting.getCommandedInputs(
            vclc.getVehicleState(), testWaypoint, k_orbit)
        controls = Controls.referenceCommands(
            courseCommand=chi_c[i],
            altitudeCommand=h_c[i],
            airspeedCommand=Va
        )

        # Update state
        vclc.Update(controls)
        chi_t[i] = vclc.getVehicleState().chi
        h_t[i] = -vclc.getVehicleState().pd

        chi_e[i] = math.degrees(chi_t[i] - chi_c[i])
        h_e[i] = h_t[i] - h_c[i]

        temp = Rotations.ned2enu([[
            vclc.getVehicleState().pn,
            vclc.getVehicleState().pe,
            vclc.getVehicleState().pd
        ]])
        x[i] = temp[0][0]
        y[i] = temp[0][1]
        z[i] = temp[0][2]

    fig = plt.figure(tight_layout =True)
    ax = fig.add_subplot(2,1,1, projection='3d')
    ax.plot3D(x, y, z)
    ax.set_title("UAV Position [ENU]")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")

    ax = fig.add_subplot(2,2,3)
    ax.plot(t_data, chi_e)
    ax.set_title(" ")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Course Error [deg]")

    ax = fig.add_subplot(2,2,4)
    ax.plot(t_data, h_e)
    ax.set_title(" ")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Altitude Error [m]")

    # Check to show or print plot
    if printPlots:
        plt.savefig(f"Plots/OrbitingTest_InitOnOrbit.png")
    else:
        plt.show()

    evaluateTest(cur_test, True)


def testing_Orbiting_Graphical_InitInOrbit(gains, printPlots=False):
    # %%
    cur_test = "Orbiting.getCommandedInputs Test 2: Initial point in orbit"

    vclc = VCLC.VehicleClosedLoopControl()
    vclc.setControlGains(gains)
    vclc.setVehicleState(States.vehicleState(
        pn=0,
        pe=0,
        pd=-100
    ))
    testWaypoint = WayPoint.WayPoint(
        n = 0,
        e = 0,
        d = -100,
        radius = 100,
        direction = 1,
    )
    k_orbit = 1
    Va = 20

    dT = vclc.getVehicleAerodynamicsModel().getVehicleDynamicsModel().dT
    totalTime = 180
    breakTime = totalTime / 2
    breakStep = int(breakTime/dT)
    n_steps = int(totalTime/dT)
    t_data = [i*dT for i in range(n_steps)]

    chi_c = [0 for i in range(n_steps)]
    chi_t = [0 for i in range(n_steps)]
    chi_e = [0 for i in range(n_steps)]
    h_c = [0 for i in range(n_steps)]
    h_t = [0 for i in range(n_steps)]
    h_e = [0 for i in range(n_steps)]
    x = [0 for i in range(n_steps)]
    y = [0 for i in range(n_steps)]
    z = [0 for i in range(n_steps)]

    for i in range(n_steps):        
        # Update reference commands
        h_c[i], chi_c[i] = Orbiting.getCommandedInputs(
            vclc.getVehicleState(), testWaypoint, k_orbit)
        controls = Controls.referenceCommands(
            courseCommand=chi_c[i],
            altitudeCommand=h_c[i],
            airspeedCommand=Va
        )

        # Update state
        vclc.Update(controls)
        chi_t[i] = vclc.getVehicleState().chi
        h_t[i] = -vclc.getVehicleState().pd

        chi_e[i] = math.degrees(chi_t[i] - chi_c[i])
        h_e[i] = h_t[i] - h_c[i]

        temp = Rotations.ned2enu([[
            vclc.getVehicleState().pn,
            vclc.getVehicleState().pe,
            vclc.getVehicleState().pd
        ]])
        x[i] = temp[0][0]
        y[i] = temp[0][1]
        z[i] = temp[0][2]

    fig = plt.figure(tight_layout =True)
    ax = fig.add_subplot(2,1,1, projection='3d')
    ax.plot3D(x, y, z)
    ax.set_title("UAV Position [ENU]")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")

    ax = fig.add_subplot(2,2,3)
    ax.plot(t_data, chi_e)
    ax.set_title(" ")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Course Error [deg]")

    ax = fig.add_subplot(2,2,4)
    ax.plot(t_data, h_e)
    ax.set_title(" ")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Altitude Error [m]")

    # Print or show plots
    if printPlots:
        plt.savefig(f"Plots/OrbitingTest_InitInOrbit.png")
    else:
        plt.show()

    evaluateTest(cur_test, True)


def testing_Orbiting_Graphical_InitOutOrbit(gains, printPlots=False):
    # %%
    cur_test = "Orbiting.getCommandedInputs Test 3: Initial point outside orbit path"

    vclc = VCLC.VehicleClosedLoopControl()
    vclc.setControlGains(gains)
    vclc.setVehicleState(States.vehicleState(
        pn=200,
        pe=0,
        pd=-100
    ))
    testWaypoint = WayPoint.WayPoint(
        n = 0,
        e = 0,
        d = -100,
        radius = 100,
        direction = 1,
    )
    k_orbit = 1
    Va = 20

    dT = vclc.getVehicleAerodynamicsModel().getVehicleDynamicsModel().dT
    totalTime = 180
    breakTime = totalTime / 2
    breakStep = int(breakTime/dT)
    n_steps = int(totalTime/dT)
    t_data = [i*dT for i in range(n_steps)]

    chi_c = [0 for i in range(n_steps)]
    chi_t = [0 for i in range(n_steps)]
    chi_e = [0 for i in range(n_steps)]
    h_c = [0 for i in range(n_steps)]
    h_t = [0 for i in range(n_steps)]
    h_e = [0 for i in range(n_steps)]
    x = [0 for i in range(n_steps)]
    y = [0 for i in range(n_steps)]
    z = [0 for i in range(n_steps)]

    for i in range(n_steps):        
        # Update reference commands
        h_c[i], chi_c[i] = Orbiting.getCommandedInputs(
            vclc.getVehicleState(), testWaypoint, k_orbit)
        controls = Controls.referenceCommands(
            courseCommand=chi_c[i],
            altitudeCommand=h_c[i],
            airspeedCommand=Va
        )

        # Update state
        vclc.Update(controls)
        chi_t[i] = vclc.getVehicleState().chi
        h_t[i] = -vclc.getVehicleState().pd

        chi_e[i] = math.degrees(chi_t[i] - chi_c[i])
        h_e[i] = h_t[i] - h_c[i]

        temp = Rotations.ned2enu([[
            vclc.getVehicleState().pn,
            vclc.getVehicleState().pe,
            vclc.getVehicleState().pd
        ]])
        x[i] = temp[0][0]
        y[i] = temp[0][1]
        z[i] = temp[0][2]

    fig = plt.figure(tight_layout =True)
    ax = fig.add_subplot(2,1,1, projection='3d')
    ax.plot3D(x, y, z)
    ax.set_title("UAV Position [ENU]")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")

    ax = fig.add_subplot(2,2,3)
    ax.plot(t_data, chi_e)
    ax.set_title(" ")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Course Error [deg]")

    ax = fig.add_subplot(2,2,4)
    ax.plot(t_data, h_e)
    ax.set_title(" ")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Altitude Error [m]")

    # Print or show plots
    if printPlots:
        plt.savefig(f"Plots/OrbitingTest_InitOutOrbit.png")
    else:
        plt.show()

    evaluateTest(cur_test, True)


def testing_Orbiting_Graphical_ChangeOrbit(gains, printPlots=False):
    # %%
    cur_test = "Orbiting.getCommandedInputs Test 4: Change Orbit"

    vclc = VCLC.VehicleClosedLoopControl()
    vclc.setControlGains(gains)
    vclc.setVehicleState(States.vehicleState(
        pn=200,
        pe=0,
        pd=-100
    ))
    waypoint1 = WayPoint.WayPoint(
        n = 0,
        e = 0,
        d = -100,
        radius = 100,
        direction = 1,
    )
    waypoint2 = WayPoint.WayPoint(
        n = 300,
        e = 1000,
        d = -300,
        radius = 200,
        direction = -1,
    )
    testWaypoint = waypoint1
    k_orbit = 1
    Va = 20

    dT = vclc.getVehicleAerodynamicsModel().getVehicleDynamicsModel().dT
    totalTime = 300
    breakTime = totalTime / 2
    breakStep = int(breakTime/dT)
    n_steps = int(totalTime/dT)
    t_data = [i*dT for i in range(n_steps)]

    chi_c = [0 for i in range(n_steps)]
    chi_t = [0 for i in range(n_steps)]
    chi_e = [0 for i in range(n_steps)]
    h_c = [0 for i in range(n_steps)]
    h_t = [0 for i in range(n_steps)]
    h_e = [0 for i in range(n_steps)]
    x = [0 for i in range(n_steps)]
    y = [0 for i in range(n_steps)]
    z = [0 for i in range(n_steps)]

    for i in range(n_steps):
        # Update orbit target  
        if i == breakStep:
            testWaypoint = waypoint2

        # Update reference commands
        h_c[i], chi_c[i] = Orbiting.getCommandedInputs(
            vclc.getVehicleState(), testWaypoint, k_orbit)
        controls = Controls.referenceCommands(
            courseCommand=chi_c[i],
            altitudeCommand=h_c[i],
            airspeedCommand=Va
        )

        # Update state
        vclc.Update(controls)
        chi_t[i] = vclc.getVehicleState().chi
        h_t[i] = -vclc.getVehicleState().pd

        chi_e[i] = math.degrees(chi_t[i] - chi_c[i])
        h_e[i] = h_t[i] - h_c[i]
        
        x[i] = vclc.getVehicleState().pn
        y[i] = vclc.getVehicleState().pe
        z[i] = -vclc.getVehicleState().pd

    # Plot Position and expected orbits
    points = range(0,1001)
    angles = [2*math.pi*i/len(points) for i in points]

    R1 = waypoint1.radius
    center1 = waypoint1.getPointLocation()
    x_data1 = [center1[0][0] + math.cos(angles[i])*R1 for i in points]
    y_data1 = [center1[1][0] + math.sin(angles[i])*R1 for i in points]
    z_data1 = [-center1[2][0] for i in points]

    R2 = waypoint2.radius
    center2 = waypoint2.getPointLocation()
    x_data2 = [center2[0][0] + math.cos(angles[i])*R2 for i in points]
    y_data2 = [center2[1][0] + math.sin(angles[i])*R2 for i in points]
    z_data2 = [-center2[2][0] for i in points]

    fig = plt.figure(tight_layout =True)
    ax = fig.add_subplot(2,1,1, projection='3d')
    ax.plot3D(x, y, z)
    ax.plot3D(x_data1, y_data1, z_data1, color='orange')
    ax.plot3D(x_data2, y_data2, z_data2, color='orange')
    ax.set_title("UAV Position [NEU]")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")

    # Plot Errors
    ax = fig.add_subplot(2,2,3)
    ax.plot(t_data, chi_e)
    ax.set_title(" ")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Course Error [deg]")

    ax = fig.add_subplot(2,2,4)
    ax.plot(t_data, h_e)
    ax.set_title(" ")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Altitude Error [m]")

    # Print or show plots
    if printPlots:
        plt.savefig(f"Plots/OrbitingTest_ChangeOrbit.png")
    else:
        plt.show()

    evaluateTest(cur_test, True)

# %% Start Message
print(f"\n\nRunning {os.path.basename(__file__)}:")

# %% Run Tests

# Test Orbiting
testing_Orbiting_CalcDistFromCenter()
testing_Orbiting_CalcAngleAlongCircle()
testing_Orbiting_CalcCommandedHeight()
testing_Orbiting_CalcCommandedCourse()
testing_Orbiting_getCommandedInputs()

# Get trim state
vTrim = VehicleTrim.VehicleTrim()
Vastar = 25.0
Gammastar = 0
Kappastar = 0

check = vTrim.computeTrim(Vastar, Kappastar, Gammastar)
if check:
    # Calculate transfer function
    tF = VPM.CreateTransferFunction(
        vTrim.getTrimState(),
        vTrim.getTrimControls())
else:
    print("Model converged outside of valid inputs, change parameters and try again")

# Get gains
tuningParameters=Controls.controlTuning(
    Wn_roll=20.61778493279331, Zeta_roll=2.7641222974190165,
    Wn_course=0.1313209195267308, Zeta_course=2.818882757405783,
    Wn_sideslip=0.03880705477100873, Zeta_sideslip=11.260307631650688,
    Wn_pitch=20.86294694878792, Zeta_pitch=0.4099245488649988,
    Wn_altitude=0.5618493957843352, Zeta_altitude=0.6792963357600577,
    Wn_SpeedfromThrottle=0.4198550394801563, Zeta_SpeedfromThrottle=0.58789323485635,
    Wn_SpeedfromElevator=0.1225180174744648, Zeta_SpeedfromElevator=7.322281351008753
)
gains = VCG.computeGains(tuningParameters, tF)

print(f"\tTuningParameters:")
print(f"{'': <{8}}{'|' : <1}{'' :-<31}|{'' :-<31}")
for key, val in vars(tuningParameters).items():
    print(f"{'': <{8}}{'|' : <2}{key : <30}| {val: <18}")

print(f"\tGains:")
print(f"{'': <{8}}{'|' : <1}{'' :-<31}|{'' :-<31}")
for key, val in vars(gains).items():
    print(f"{'': <{8}}{'|' : <2}{key : <30}| {val: <18}")


printPlot = False
# testing_Orbiting_Graphical_InitOnOrbit(gains, printPlot)
# testing_Orbiting_Graphical_InitInOrbit(gains, printPlot)
# testing_Orbiting_Graphical_InitOutOrbit(gains, printPlot)
testing_Orbiting_Graphical_ChangeOrbit(gains, printPlot)

# %% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
    print(f"Failed {len(failed)}/{total} tests:")
    [print("   " + test) for test in failed]
