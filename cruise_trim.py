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
#------------------------------------Custom Functions----------------------------------
#--------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------
#---------------------------------------Main Loop--------------------------------------
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
    tol=1e-12
)

cruise_log = simulate(
    aircraft = 'XV-3Trim',
    op_0 = op_cruise,
    tf = 5,
    realtime = True
)
# get_properties()
plot_stuff(cruise_log)
#  8.35554591e-01,  8.35484354e-01,  8.33759013e-01,  8.33509315e-01,
        # 3.18645317e-02, -1.21775389e-03, -1.69309986e-04,  5.06581212e-02 original cruise
