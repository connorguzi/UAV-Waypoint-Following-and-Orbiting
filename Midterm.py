#%% Setup

import sympy as sym
import ece163.Constants.VehiclePhysicalConstants as VPC
import math
import ece163.Utilities.MatrixMath as mm
import matplotlib.pyplot as plt
#%% 2.C

cosW, sinW, cosi, sini, cosw, sinw= sym.symbols('cosW, sinW, cosi, sini, cosw, sinw')
A = sym.Matrix([[cosW, sinW, 0], [-1 * sinW, cosW, 0], [0,0,1]])
B = sym.Matrix([[1, 0, 0], [0,cosi, sini], [0,-sini, cosi]])
C = sym.Matrix([[cosw, sinw, 0], [-sinw, cosw, 0], [0,0,1]])

AB = B * A
ABC = C * AB

# %% 3a
Va = 15
a_theta1 = 2 * VPC.Jyy / (VPC.rho * Va**2 * VPC.S * VPC.c * VPC.CMdeltaE)
a_theta2 = -VPC.CMq * VPC.c / ( 2 * Va * VPC.CMdeltaE)
a_theta3 = -VPC.CMalpha / VPC.CMdeltaE

a1 = a_theta2 / a_theta1
a2 = a_theta3 / a_theta1
b2 = 1 / a_theta1

print(f"a1 = {a1}, a2 = {a2}, b2 = {b2}")

# %%3c
Kp = -8.0
omega = math.sqrt(a2)
zeta = a1 / (2 * omega)
rise_time = 1.8 / omega
settle_time = 4.6 / (zeta * omega)
overshoot = math.exp(-zeta * math.pi / math.sqrt(1 - zeta**2)) * 100
DCgain = (Kp * b2 / a2) / (1 +( Kp * b2 / a2))
print(f"wn = {omega}, zeta = {zeta}, rise = {rise_time}, settle = {settle_time}, overshoot = {overshoot}, gain = {DCgain}")

# %% 3e

#Updating the terms
b2_ideal = b2 * Kp
a2_ideal = a2 + Kp * b2


## State space for ideal
A_Ideal = [[-a1, -a2_ideal], [1,0]]
B_Ideal = [[1], [0]]
C_Ideal = [[0, b2_ideal]]
x_ideal =  [[0], [0]]

## State space for delayed
tau = 0.01
a1_new = a1 + (1 / tau)
a2_new = (a1/tau) + a2 - Kp * b2
a3_new = (a2 + Kp * b2) / tau
b1_new = Kp * b2
b2_new =  Kp * b2 / tau

A_Delayed = [[-a1_new, -a2_new, -a3_new], [1,0,0], [0,1,0]]
B_Delayed = [[1], [0], [0]]
C_Delayed = [[0, b1_new, b2_new]]
x_delayed =  [[0], [0], [0]]


dT = 0.01
T_Tot = 6
n_steps = int(T_Tot / dT)
t_data = [i*dT for i in range(n_steps)]
theta = [(0 if t<0 else 5.73*math.pi / 180) for t in t_data]
x_ideal_data = [0 for i in range(n_steps)]
x_delayed_data = [0 for i in range(n_steps)]

for i in range(n_steps):
    # record data
    x_ideal_data[i] = mm.multiply(C_Ideal, x_ideal)[0][0]
    # find u(t)
    u = theta[i]
    # calculate derivative:
    x_dot_ideal = mm.add(
            mm.multiply(A_Ideal,x_ideal),
            mm.scalarMultiply(u,B_Ideal))
    # forward euler update
    x_ideal = mm.add(
        x_ideal,
        mm.scalarMultiply(dT, x_dot_ideal))
    
    # record data
    x_delayed_data[i] = mm.multiply(C_Delayed, x_delayed)[0][0]
    # find u(t)
    u = theta[i]
    # calculate derivative:
    x_dot_delayed = mm.add(
            mm.multiply(A_Delayed,x_delayed),
            mm.scalarMultiply(u,B_Delayed))
    # forward euler update
    x_delayed = mm.add(
        x_delayed,
        mm.scalarMultiply(dT, x_dot_delayed))

fig, ax = plt.subplots()
ax.plot(t_data, theta, label = 'pitch angle commanded')
ax.plot(t_data, x_ideal_data, label = 'ideal response')
ax.plot(t_data, x_delayed_data, label = 'delayed response')
ax.set_xlabel('time (s)')
ax.set_ylabel('anglge (rad)')
ax.legend()

plt.show()

