"""
This is the master script to calculate static thrust properties for a given aircraft.

Steps:
    1. Ensure this file is located in same directory as jsbsim
    2. run from terminal (python test.py)
"""
#--------------------------------------------------------------------------------------
#----------------------------------------Imports---------------------------------------
#--------------------------------------------------------------------------------------
from trim_functions import trim_aircraft, simulate
from jsb_plotter import plot_stuff
from get_properties import get_properties
import numpy as np
import pandas as pd
#--------------------------------------------------------------------------------------
#------------------------------------Custom Functions----------------------------------
#--------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------
#---------------------------------------Main Loop--------------------------------------
#--------------------------------------------------------------------------------------
op_ground, fdm = trim_aircraft(
    aircraft = 'XV-3',
    ic = {
        'ic/vt-fps': 0,
        'ic/h-agl-ft': 5.28,
        'gear/gear-cmd-norm': 1,
        'fcs/throttle-cmd-norm': 0,
        'fcs/throttle-cmd-norm[1]': 0,
        'fcs/throttle-cmd-norm[2]': 0,
        'fcs/throttle-cmd-norm[3]': 0,
        'fcs/throttle-cmd-norm[4]': 0,
        'fcs/throttle-cmd-norm[5]': 0,
        'fcs/throttle-cmd-norm[6]': 0,
        'fcs/throttle-cmd-norm[7]': 0,
        'fcs/throttle-cmd-norm[8]': 0,
        'fcs/aileron-cmd-norm': 0,
        'fcs/elevator-cmd-norm': 0,
        'fcs/rudder-cmd-norm': 0,
        'fcs/left-brake-cmd-norm': 1,
        'fcs/right-brake-cmd-norm': 1,
        'fcs/center-brake-cmd-norm': 1,
    },
    design_vector = ['ic/theta-deg'],
    x0 = [0.0],
    bounds = [-1,1],
    verbose = True,
    method = 'Nelder-Mead',
    tol = 1e-6
)

ground_log = simulate(
    aircraft = 'XV-3',
    op_0 = op_ground,
    tf = 5,
    realtime = True
)
# get_properties()
plot_stuff(ground_log)
