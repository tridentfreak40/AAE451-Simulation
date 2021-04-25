"""
This is the master script to calculate static thrust properties for a given aircraft.

Steps:
    1. Ensure this file is located in same directory as jsbsim
    2. run from terminal (python test.py)
"""
#--------------------------------------------------------------------------------------
#----------------------------------------Imports---------------------------------------
#--------------------------------------------------------------------------------------
from trim_functions2 import trim_aircraft, simulate
# from trim_functions import trim_aircraft, simulate
from jsb_plotter import plot_stuff
from get_properties import get_properties
import numpy as np
import pandas as pd
#--------------------------------------------------------------------------------------
#------------------------------------Version 3.0---------------------------------------
#--------------------------------------------------------------------------------------
op_hover, props = trim_aircraft(
    aircraft='XV-3Trim',
    ic={
        'ic/h-sl-ft': 650,
        'ic/vt-fps': 0,
        'gear/gear-cmd-norm': 1,
        'fcs/left-brake-cmd-norm': 0,
        'fcs/right-brake-cmd-norm': 0,
        'fcs/center-brake-cmd-norm': 0,
        'fcs/throttle-cmd-norm':0,
        'fcs/throttle-cmd-norm[1]': 0,
        'fcs/throttle-cmd-norm[2]': 0,
        'fcs/throttle-cmd-norm[3]': 0,
        'ic/theta-deg': -0.0,
        'ic/phi-deg': 0,
        'ic/psi-true-deg': 280
    },
    eq_constraints = [
        #lambda fdm: fdm['accelerations/udot-ft_sec2'],
        #lambda fdm: fdm['accelerations/vdot-ft_sec2'],
        lambda fdm: fdm['accelerations/wdot-ft_sec2'],
        #lambda fdm: fdm['accelerations/pdot-rad_sec2'],
        lambda fdm: fdm['accelerations/qdot-rad_sec2'],
        #lambda fdm: fdm['accelerations/rdot-rad_sec2'],
    ],
    design_vector=[
        'fcs/throttle-cmd-norm',
        'fcs/elevator-cmd-norm',
        #'fcs/rudder-cmd-norm',
    ],
    x0=[0.5, 0.5],
    cost= lambda fdm: fdm['fcs/throttle-cmd-norm'],
    verbose=True,
    method='SLSQP',
    bounds=[[0, 1], [-1, 1]],
)
# --------------------------------------------------------------------------------------
# ---------------------------------------OLD VERSION------------------------------------
# --------------------------------------------------------------------------------------
# op_hover, fdm = trim_aircraft(
#     aircraft = 'XV-3Trim',
#     ic = {
#           'ic/h-agl-ft': 100,
#           'ic/vd-fps': 0,
#           'ic/vn-fps': 0*np.cos(np.deg2rad(280)),
#           'ic/ve-fps': 0*np.sin(np.deg2rad(280)),
#           'ic/theta-rad': 0,
#           'gear/gear-cmd-norm': 1,
#           'fcs/left-brake-cmd-norm': 0,
#           'fcs/right-brake-cmd-norm': 0,
#           'fcs/center-brake-cmd-norm': 0,
#         #   'ap/roll-enable': 0,
#         #   'ap/pitch-enable': 0,
#         #   'ap/yaw-enable': 0,
#         #   'ap/h-enable': 0
#     },
#     design_vector = [
#     'fcs/throttle-cmd-norm',
#     'fcs/elevator-cmd-norm'
#     ],

#     x0 = [0.9, 0.5], #0.5, 0.5],
#     verbose = True,
#     bounds = [[0,1], [-1,1]], #, [0,1], [0,1]],
#     tol = 1e-12
# )

# --------------------------------------------------------------------------------------
# ----------------------------TURN ON FOR FLIGHTGEAR------------------------------------
# --------------------------------------------------------------------------------------

log_hover = simulate(
    aircraft='XV-3Trim',
    op_0=op_hover,
    tf=5,
    realtime=False)

# get_properties()
plot_stuff(log_hover)

# 0.88249415, 0.88291196, 0.8923975 , 0.89212139, 0.78291153] original hover
# 0.88247067, 0.8829812 , 0.89240468, 0.89206831, 0.78291631 new hover