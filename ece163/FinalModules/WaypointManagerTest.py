#%% Setup
import sys
sys.path.append("..") #python is horrible, no?
# sys.path.append("..")
# sys.path.append("./")
import WaypointManager as WM
from ece163.Utilities import MatrixMath as mm
import math
from ece163.Containers import States as States
import enum
import matplotlib.pyplot as plt
import numpy as np


#%% Direction Vector Test

testState = States.vehicleState(pn=10, pe=5)
waypoint = [[20], [0], [0]]

print(WM.CalcDirectionVector(state=testState, waypoint=waypoint))
# %%
