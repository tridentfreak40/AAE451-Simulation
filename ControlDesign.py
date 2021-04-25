"""
This is a script to generate control design for cruise for a given aircraft.

Steps:
    1. Ensure this file is located in same directory as jsbsim
    2. run from terminal (python test.py)
"""

#--------------------------------------------------------------------------------------
#----------------------------------------Imports---------------------------------------
#--------------------------------------------------------------------------------------

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import control 

from trim_functions2 import trim_aircraft, simulate
from jsb_plotter import plot_stuff
from get_properties import get_properties

from trim_functions2 import trim_aircraft, simulate
from trim_functions import linearize, rootlocus, clean_tf

#--------------------------------------------------------------------------------------
#--------------------------Control Design: HOVER --------------------------------------
#--------------------------------------------------------------------------------------

# op_hover, props = trim_aircraft(
#     aircraft='XV-3Trim',
#     ic={
#         'ic/h-sl-ft': 650,
#         'ic/vt-fps': 0,
#         'ic/psi-true-deg': 280,
#         'gear/gear-cmd-norm': 1,
#         'fcs/left-brake-cmd-norm': 0,
#         'fcs/right-brake-cmd-norm': 0,
#         'fcs/center-brake-cmd-norm': 0,
#         'fcs/throttle-cmd-norm':0,
#         'fcs/throttle-cmd-norm[1]': 0,
#         'fcs/throttle-cmd-norm[2]': 0,
#         'fcs/throttle-cmd-norm[3]': 0,
#     },
#     eq_constraints = [
#         #lambda fdm: fdm['accelerations/udot-ft_sec2'],
#         #lambda fdm: fdm['accelerations/vdot-ft_sec2'],
#         lambda fdm: fdm['accelerations/wdot-ft_sec2'],
#         #lambda fdm: fdm['accelerations/pdot-rad_sec2'],
#         lambda fdm: fdm['accelerations/qdot-rad_sec2'],
#         #lambda fdm: fdm['accelerations/rdot-rad_sec2'],
#     ],
#     design_vector=[
#         'fcs/throttle-cmd-norm',
#         'fcs/elevator-cmd-norm',
#         #'fcs/aileron-cmd-norm',
#     ],
#     x0=[0.5, 0.5],
#     cost= lambda fdm: fdm['fcs/throttle-cmd-norm'],
#     verbose=True,
#     method='SLSQP',
#     bounds=[[0, 1], [-1, 1]],
# )

#----------------------------------Pitch Control---------------------------------------

# sys = control.ss(*linearize(
#     aircraft='XV-3Trim',
#     states=['ic/q-rad_sec'], # sets state vector , x
#     states_deriv = ['accelerations/qdot-rad_sec2'], # sets x_dot
#     inputs=['fcs/elevator-cmd-norm'], # input for transfer function, elevator command
#     outputs=['ic/q-rad_sec'], # output is updated pitch rate
#     ic=op_hover,
#     dx=1e-3, # finite difference
#     n_round=3 # number of decimals
# ))
# s = control.tf([1, 0], [1]) # this is just an s in laplace
# rad2deg = 180/np.pi
# G_elev_to_pitch = rad2deg*clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s # plant model, G(s)
# print(sys)
# print(G_elev_to_pitch) ###

# # controller, H(s)

# # H_elev_to_pitch = 0.05*(s/0.5+1) # is a proportional-derivative 
# #H_elev_to_pitch = 10*(s/2*(100/(s+100)+1) # low pass filter added: N/(s+N), N is cutoff freq of N rad/sec
# ## 100/(s+100) or 1/((1/100)*s+1) is a low pass filter that is with 100 rad. Derivative filters past anything 100 rad/s. 
# P = 1.668e-6
# D = 0.04722
# N = 3.0247
# H_elev_to_pitch = P + D*(N/(1+(N*(1/s))))
# print(H_elev_to_pitch) ####
# plt.figure(1)
# rootlocus(G_elev_to_pitch*H_elev_to_pitch) # open loop root locus
# plt.plot([0, -1], [0, 1], '--')
# # plt.show()

# Gc_elev_to_pitch = G_elev_to_pitch*H_elev_to_pitch/(1 + G_elev_to_pitch*H_elev_to_pitch) # closed loop transfer function
# print(control.minreal(Gc_elev_to_pitch)) ###

# plt.figure(2)
# step_size = 10 # set step response
# t, y = control.step_response(step_size*Gc_elev_to_pitch, T=np.linspace(0, 10, 1000));
# plt.plot(t, y)
# plt.ylabel('pitch, deg')
# plt.xlabel('t, sec')
# plt.title('output')

# plt.figure()
# # actual error was computed in radians, so, converting back here
# e = np.deg2rad(step_size-y) # error of system 

# t, u = control.forced_response(H_elev_to_pitch, T=t, U=e)
# plt.plot(t, u)
# plt.hlines([-1, 1], t[0], t[-1], linestyles='dashed')
# plt.title('input')
# plt.ylabel('elevator, norm')
# plt.xlabel('t, sec')

# plt.figure(figsize=(15, 7))
# control.gangof4(G_elev_to_pitch, H_elev_to_pitch, Hz=True, dB=True)

# plt.figure()
# control.nyquist(Gc_elev_to_pitch, omega=np.logspace(-3, 3, 1000));

# control.margin(Gc_elev_to_pitch)

# plt.show()

# #----------------------------------Roll Control---------------------------------------

# sys = control.ss(*linearize(
#     aircraft='XV-3Trim',
#     states=['ic/p-rad_sec'],
#     states_deriv = ['accelerations/pdot-rad_sec2'],
#     inputs=['fcs/aileron-cmd-norm'],
#     outputs=['ic/p-rad_sec'],
#     ic=op_hover,
#     dx=1e-3,
#     n_round=3
# ))
# rad2deg = 180/np.pi
# s = control.tf([1, 0], [1])
# G_aileron_to_roll = rad2deg*clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s # plant model, G(s)
# print(sys)
# print(G_aileron_to_roll)

# # feedback model, H(s)

# # H_aileron_to_roll = 0.1*(s/0.5 + 1) # low pass filter: K * 1/(tau*s +1), K is gain of the filter
# P = 0
# D = 0.00363
# N = 2.8645
# H_aileron_to_roll = P + D*(N/(1+(N*(1/s))))
# print(H_aileron_to_roll) ####

# plt.figure()
# rootlocus(G_aileron_to_roll*H_aileron_to_roll)
# plt.plot([0, -1], [0, 1], '--')

# Gc_aileron_to_roll  = G_aileron_to_roll*H_aileron_to_roll/(1 + G_aileron_to_roll*H_aileron_to_roll)
# print(control.minreal(Gc_aileron_to_roll))

# plt.figure()
# rootlocus(Gc_aileron_to_roll)

# plt.figure()
# step_size = 10
# t, y = control.step_response(step_size*Gc_aileron_to_roll, T=np.linspace(0, 10, 1000));
# plt.plot(t, y)
# plt.xlabel('t, sec')
# plt.ylabel('roll, deg')
# plt.title('output')

# plt.figure()
# # actual error was computed in radians, so, converting back here
# e = np.deg2rad(step_size-y)
# t, u = control.forced_response(H_aileron_to_roll, T=t, U=e)
# plt.hlines([-0.1, 0.1], t[0], t[-1], linestyles='dashed')
# plt.plot(t, u)
# plt.xlabel('t, sec')
# plt.ylabel('aileron %')
# plt.title('input')

# plt.figure()
# control.nyquist(Gc_aileron_to_roll, omega=np.logspace(-3, 3, 1000));

# plt.figure(figsize=(15, 7))
# control.gangof4(G_aileron_to_roll, H_aileron_to_roll, Hz=True, dB=True)

# control.margin(Gc_aileron_to_roll)

# plt.show()

# #----------------------------------Yaw Control---------------------------------------

# sys = control.ss(*linearize(
#     aircraft='XV-3',
#     states=['ic/r-rad_sec'],
#     states_deriv = ['accelerations/rdot-rad_sec2'],
#     inputs=['propulsion/engine/yaw-angle-rad'],
#     outputs=['ic/r-rad_sec'],
#     ic=op_hover,
#     dx=1e-3,
#     n_round=3
# ))
# s = control.tf([1, 0], [1])
# G_rudder_to_yaw = -clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s # plant model, G(s)
# print(sys)
# print(G_rudder_to_yaw)

# from Goppert, "Yaw angle seems to have no impact on r moment, need to investigate. Can add another lift-fan to model this if necessary."

#----------------------------------Altitude Control------------------------------------

# sys = control.ss(*linearize(
#     aircraft='XV-3Trim',
#     states=['ic/w-fps'],
#     states_deriv = ['accelerations/wdot-ft_sec2'],
#     inputs=['fcs/throttle-cmd-norm'],
#     outputs=['ic/w-fps'],
#     ic=op_hover,
#     dx=1e-3,
#     n_round=3
# ))

# rad2deg = 180/np.pi
# s = control.tf([1, 0], [1])
# G_throttle_to_alt = -clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s # plant model, G(s)
# print(sys)
# print(G_throttle_to_alt)

# # H_throttle_to_alt = 0.01*(2*s + 1) # feedback model, H(s)
# P = 1.9206e-9
# D = 0.01922
# N = 2.8694
# H_throttle_to_alt = P + D*(N/(1+(N*(1/s))))
# print(H_throttle_to_alt) ####

# plt.figure()
# rootlocus(G_throttle_to_alt*H_throttle_to_alt) # open loop model 
# plt.plot([0, -1], [0, 1], '--')

# Gc_throttle_to_alt = G_throttle_to_alt*H_throttle_to_alt/(1 + G_throttle_to_alt*H_throttle_to_alt) # closed-loop model 
# print(control.minreal(Gc_throttle_to_alt))

# plt.figure()
# step_size = 10
# t, y = control.step_response(step_size*Gc_throttle_to_alt, T=np.linspace(0, 40, 1000));
# plt.plot(t, y)
# plt.xlabel('t, sec')
# plt.ylabel('altitude, ft')
# plt.title('output')

# plt.figure()
# # error computed in ft
# e = step_size-y # error in closed loop model 
# t, u = control.forced_response(H_throttle_to_alt, T=t, U=e)
# plt.hlines([-0.1, 0.1], t[0], t[-1], linestyles='dashed')
# plt.plot(t, u)
# plt.xlabel('t, sec')
# plt.ylabel('throtle %')
# plt.title('input')

# plt.figure()
# control.nyquist(Gc_throttle_to_alt, omega=np.logspace(-3, 3, 1000));

# plt.figure(figsize=(15, 7))
# control.gangof4(G_throttle_to_alt, H_throttle_to_alt, Hz=True, dB=True)

# control.margin(Gc_throttle_to_alt)

# plt.show()

#--------------------------------------------------------------------------------------
#--------------------------Control Design: CRUISE -------------------------------------
#--------------------------------------------------------------------------------------

op_cruise, props = trim_aircraft(
    aircraft='XV-3Trim',
    ic={
        'ic/gamma-deg': 0,
        'ic/vt-fps': 600,
        'ic/h-sl-ft': 10000,
        'gear/gear-cmd-norm': 0,
        'fcs/left-brake-cmd-norm': 0,
        'fcs/right-brake-cmd-norm': 0,
        'fcs/center-brake-cmd-norm': 0,
        'propulsion/engine/pitch-angle-rad': 0,
    },
    design_vector=[
        'fcs/throttle-cmd-norm',
        'fcs/elevator-cmd-norm',
        'fcs/rudder-cmd-norm',
        'fcs/aileron-cmd-norm',
        'ic/alpha-rad',
        'ic/beta-rad',
    ],

    method='SLSQP',
    eq_constraints= [
        lambda fdm: fdm['accelerations/udot-ft_sec2'],
        lambda fdm: fdm['accelerations/vdot-ft_sec2'],
        lambda fdm: fdm['accelerations/wdot-ft_sec2'],
        lambda fdm: fdm['accelerations/pdot-rad_sec2'],
        lambda fdm: fdm['accelerations/qdot-rad_sec2'],
        lambda fdm: fdm['accelerations/rdot-rad_sec2'],
    ],
    cost=lambda fdm: fdm['fcs/throttle-cmd-norm'],
    x0=[0.5, 0, 0, 0, 0, 0],
    verbose=True,
    bounds=[[0, 1], [-1, 1], [-1, 1], [-1, 1], [-1, 1], [-1, 1]],
    tol=1e-3
)

# #----------------------------------Pitch Control---------------------------------------

sys = control.ss(*linearize(
    aircraft='XV-3Trim',
    states=['ic/q-rad_sec'], # sets state vector , x
    states_deriv = ['accelerations/qdot-rad_sec2'], # sets x_dot
    inputs=['fcs/elevator-cmd-norm'], # input for transfer function, elevator command
    outputs=['ic/q-rad_sec'], # output is updated pitch rate
    ic=op_cruise,
    dx=1e-3, # finite difference
    n_round=3 # number of decimals
))
s = control.tf([1, 0], [1]) # this is just an s in laplace
rad2deg = 180/np.pi
G_elev_to_pitch = rad2deg*clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s # plant model, G(s)
print(sys)
print(G_elev_to_pitch) ###

# controller, H(s)

# H_elev_to_pitch = 0.05*(s/0.5+1) # is a proportional-derivative 
#H_elev_to_pitch = 10*(s/2*(100/(s+100)+1) # low pass filter added: N/(s+N), N is cutoff freq of N rad/sec
## 100/(s+100) or 1/((1/100)*s+1) is a low pass filter that is with 100 rad. Derivative filters past anything 100 rad/s. 
P = 0
D = 0.0052
N = 2.8624
H_elev_to_pitch = P + D*(N/(1+(N*(1/s))))
print(H_elev_to_pitch) ####
plt.figure(1)
rootlocus(G_elev_to_pitch*H_elev_to_pitch) # open loop root locus
plt.plot([0, -1], [0, 1], '--')


Gc_elev_to_pitch = G_elev_to_pitch*H_elev_to_pitch/(1 + G_elev_to_pitch*H_elev_to_pitch) # closed loop transfer function
print(Gc_elev_to_pitch) ###

# plt.figure(2)
# step_size = 10 # set step response
# t, y = control.step_response(step_size*Gc_elev_to_pitch, T=np.linspace(0, 10, 1000));
# plt.plot(t, y)
# plt.ylabel('pitch, deg')
# plt.xlabel('t, sec')
# plt.title('output')

# plt.figure()
# # actual error was computed in radians, so, converting back here
# e = np.deg2rad(step_size-y) # error of system 

# t, u = control.forced_response(H_elev_to_pitch, T=t, U=e)
# plt.plot(t, u)
# plt.hlines([-1, 1], t[0], t[-1], linestyles='dashed')
# plt.title('input')
# plt.ylabel('elevator, norm')
# plt.xlabel('t, sec')

# plt.figure(figsize=(15, 7))
# control.gangof4(G_elev_to_pitch, H_elev_to_pitch, Hz=True, dB=True)

# plt.figure()
# control.nyquist(Gc_elev_to_pitch, omega=np.logspace(-3, 3, 1000));

# control.margin(Gc_elev_to_pitch)

# plt.show()

# #----------------------------------Roll Control---------------------------------------

# sys = control.ss(*linearize(
#     aircraft='XV-3Trim',
#     states=['ic/p-rad_sec'],
#     states_deriv = ['accelerations/pdot-rad_sec2'],
#     inputs=['fcs/aileron-cmd-norm'],
#     outputs=['ic/p-rad_sec'],
#     ic=op_cruise,
#     dx=1e-3,
#     n_round=3
# ))
# rad2deg = 180/np.pi
# s = control.tf([1, 0], [1])
# G_aileron_to_roll = rad2deg*clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s # plant model, G(s)
# print(sys)
# print(G_aileron_to_roll)

# # feedback model, H(s)

# # H_aileron_to_roll = 0.1*(s/0.5 + 1) # low pass filter: K * 1/(tau*s +1), K is gain of the filter
# P = 0
# D = 0.0935
# N = 2.86295
# H_aileron_to_roll = P + D*(N/(1+(N*(1/s))))
# print(H_aileron_to_roll) ####

# plt.figure()
# rootlocus(G_aileron_to_roll*H_aileron_to_roll)
# plt.plot([0, -1], [0, 1], '--')

# Gc_aileron_to_roll  = G_aileron_to_roll*H_aileron_to_roll/(1 + G_aileron_to_roll*H_aileron_to_roll)


# plt.figure()
# step_size = 10
# t, y = control.step_response(step_size*Gc_aileron_to_roll, T=np.linspace(0, 10, 1000));
# plt.plot(t, y)
# plt.xlabel('t, sec')
# plt.ylabel('roll, deg')
# plt.title('output')

# plt.figure()
# # actual error was computed in radians, so, converting back here
# e = np.deg2rad(step_size-y)
# t, u = control.forced_response(H_aileron_to_roll, T=t, U=e)
# plt.hlines([-0.1, 0.1], t[0], t[-1], linestyles='dashed')
# plt.plot(t, u)
# plt.xlabel('t, sec')
# plt.ylabel('aileron %')
# plt.title('input')

# plt.figure()
# control.nyquist(Gc_aileron_to_roll, omega=np.logspace(-3, 3, 1000));

# plt.figure(figsize=(15, 7))
# control.gangof4(G_aileron_to_roll, H_aileron_to_roll, Hz=True, dB=True)

# control.margin(Gc_aileron_to_roll)

# plt.show()

# #----------------------------------Yaw Control---------------------------------------

sys = control.ss(*linearize(
    aircraft='XV-3',
    states=['ic/r-rad_sec'],
    states_deriv = ['accelerations/rdot-rad_sec2'],
    inputs=['propulsion/engine/yaw-angle-rad'],
    outputs=['ic/r-rad_sec'],
    ic=op_cruise,
    dx=1e-3,
    n_round=3
))
s = control.tf([1, 0], [1])
G_rudder_to_yaw = -clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s # plant model, G(s)
print(sys)
print(G_rudder_to_yaw)

# # # feedback model, H(s)

# # H_aileron_to_roll = 0.1*(s/0.5 + 1) # low pass filter: K * 1/(tau*s +1), K is gain of the filter
# P = 27
# D = 456
# N = 2.863
# H_rudder_to_yaw = P + D*(N/(1+(N*(1/s))))
# print(H_rudder_to_yaw) ####

# plt.figure()
# rootlocus(G_rudder_to_yaw*H_rudder_to_yaw)
# plt.plot([0, -1], [0, 1], '--')

# Gc_rudder_to_yaw  = G_rudder_to_yaw*H_rudder_to_yaw/(1 + G_rudder_to_yaw*H_rudder_to_yaw)


# plt.figure()
# step_size = 10
# t, y = control.step_response(step_size*Gc_rudder_to_yaw, T=np.linspace(0, 10, 1000));
# plt.plot(t, y)
# plt.xlabel('t, sec')
# plt.ylabel('roll, deg')
# plt.title('output')

# plt.figure()
# # actual error was computed in radians, so, converting back here
# e = np.deg2rad(step_size-y)
# t, u = control.forced_response(H_rudder_to_yaw, T=t, U=e)
# plt.hlines([-0.1, 0.1], t[0], t[-1], linestyles='dashed')
# plt.plot(t, u)
# plt.xlabel('t, sec')
# plt.ylabel('rudder %')
# plt.title('input')

# plt.figure()
# control.nyquist(Gc_rudder_to_yaw, omega=np.logspace(-3, 3, 1000));

# plt.figure(figsize=(15, 7))
# control.gangof4(Gc_rudder_to_yaw, H_rudder_to_yaw, Hz=True, dB=True)

# control.margin(Gc_rudder_to_yaw)

# plt.show()


# from Goppert, "Yaw angle seems to have no impact on r moment, need to investigate. Can add another lift-fan to model this if necessary."

#----------------------------------Altitude Control------------------------------------

# sys = control.ss(*linearize(
#     aircraft='XV-3Trim',
#     states=['ic/w-fps'],
#     states_deriv = ['accelerations/wdot-ft_sec2'],
#     inputs=['fcs/elevator-cmd-norm'], # switch to positive gain
#     outputs=['ic/w-fps'],
#     ic=op_cruise,
#     dx=1e-3,
#     n_round=3
# ))

# rad2deg = 180/np.pi
# s = control.tf([1, 0], [1])
# G_throttle_to_alt = -clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s # plant model, G(s)
# print(sys)
# print(G_throttle_to_alt)

# # H_throttle_to_alt = 0.01*(2*s + 1) # feedback model, H(s)
# P = 8.71e-9
# D = 0.0436
# I = 4.358e-16
# N = 1.384
# H_throttle_to_alt = P + I*(1/s) + D*(N/(1+(N*(1/s))))
# print(H_throttle_to_alt) ####

# plt.figure()
# rootlocus(G_throttle_to_alt*H_throttle_to_alt) # open loop model 
# plt.plot([0, -1], [0, 1], '--')

# Gc_throttle_to_alt = G_throttle_to_alt*H_throttle_to_alt/(1 + G_throttle_to_alt*H_throttle_to_alt) # closed-loop model 

# plt.figure()
# step_size = 10
# t, y = control.step_response(step_size*Gc_throttle_to_alt, T=np.linspace(0, 40, 1000));
# plt.plot(t, y)
# plt.xlabel('t, sec')
# plt.ylabel('altitude, ft')
# plt.title('output')

# plt.figure()
# # error computed in ft
# e = step_size-y # error in closed loop model 
# t, u = control.forced_response(H_throttle_to_alt, T=t, U=e)
# plt.hlines([-0.1, 0.1], t[0], t[-1], linestyles='dashed')
# plt.plot(t, u)
# plt.xlabel('t, sec')
# plt.ylabel('throtle %')
# plt.title('input')

# plt.figure()
# control.nyquist(Gc_throttle_to_alt, omega=np.logspace(-3, 3, 1000));

# plt.figure(figsize=(15, 7))
# control.gangof4(G_throttle_to_alt, H_throttle_to_alt, Hz=True, dB=True)

# control.margin(Gc_throttle_to_alt)
# plt.show()