#%% Setup
import sys
sys.path.append("./")  # python is horrible, no?
sys.path.append("..")  # python is horrible, no?
import WaypointManager as WM
from ece163.Utilities import MatrixMath as mm
import math
from ece163.Containers import States as States
import enum
import matplotlib.pyplot as plt
import numpy as np


#%% Direction Vector Test

testState = States.vehicleState(pn=10, pe=5)
waypoint = [[10], [10], [0]]

# dirvec = WM.CalcDirectionVector(state=testState, waypoint=waypoint)
# dirvec = [dirvec[0][0], dirvec[1][0]]
# print(dirvec)
# plt.grid()
# plt.plot(testState.pn, testState.pe, marker="o", markersize=5, markerfacecolor="blue")
# plt.plot(waypoint[0][0], waypoint[1][0], marker="o", markersize=5, markerfacecolor="red")
# origin = [testState.pn, testState.pe]
# plt.quiver(*origin,*dirvec, color = 'g' )
# plt.show()
# %% Vector field Test
waypoint = [[4], [7]]
x,y, = np.meshgrid(np.linspace(-10, 10, 10), np.linspace(-10, 10, 10))
u = (waypoint[0][0]-x) / np.sqrt((waypoint[0][0] - x)**2 + (waypoint[1][0] -y)**2)
v = (waypoint[1][0]-y) /np.sqrt((waypoint[0][0] - x)**2 + (waypoint[1][0] -y)**2)

# vecs = WM.CalcDirectionVectorTest([x,y], waypoint)
# print(vecs)
# Setting x, y boundary limits
plt.xlim(-10, 10)
plt.ylim(-10, 10)
plt.grid()
print(x)
plt.quiver(x,y,u, v, units='xy', scale=2)
plt.show()