from pathlib import Path
import time
import pandas as pd
import numpy as np
import scipy.optimize
from logger import Logger
import matplotlib.pyplot as plt 
import control

import jsbsim

def set_operating_point(props,fdm):
    fdm_props = fdm.get_property_catalog('')
    # print(fdm_props)
    # Check if properties being set exist and set them
    for key in props.keys():
        if key not in fdm_props.keys():
            raise KeyError(key)
        else:
            fdm[key] = props[key]


def set_location(fdm):
    # location, Purdue airport
    fdm['ic/terrain-elevation-ft'] = 18
    fdm['ic/lat-geod-deg'] = 21.328634
    fdm['ic/long-gc-deg'] = -157.906316
    fdm['ic/psi-true-deg'] = 233.2
    # fdm['simulation/do_simple_trim'] = 2
    return

def trim_aircraft(aircraft, ic, design_vector, x0, verbose, **kwargs):
    root = Path('.').resolve()
    fdm = jsbsim.FGFDMExec(str(root))
    fdm.set_debug_level(0)
    fdm.load_model(aircraft)
    # Set Aircraft Location
    set_location(fdm)
    # Check if all ic properties and design vector properties are in property catalog
    fdm_props = fdm.get_property_catalog('')
    # print(fdm_props)
    for key in design_vector + list(ic.keys()):
        if key not in fdm_props.keys():
            raise KeyError(key)
    # Trim Cost Function
    def cost(fdm):
        theta = fdm['attitude/theta-deg']
        alpha = fdm['aero/alpha-rad']
        drag = fdm['forces/fwx-aero-lbs']
        lift = fdm['forces/fwz-aero-lbs']
        # Compute Cost
        udot = fdm['accelerations/udot-ft_sec2']
        vdot = fdm['accelerations/vdot-ft_sec2']
        wdot = fdm['accelerations/wdot-ft_sec2']
        pdot = fdm['accelerations/pdot-rad_sec2']
        qdot = fdm['accelerations/qdot-rad_sec2']
        rdot = fdm['accelerations/rdot-rad_sec2']
        return udot**2 + vdot**2 + wdot**2 + pdot**2 + qdot**2 + rdot**2
        # return udot**2 + vdot**2 + wdot**2 + pdot**2 + qdot**2 + rdot**2 + (1e-3*theta)**2
    def trim_cost(fdm,xd):
        # Set ic
        for var in ic.keys():
            fdm[var] = ic[var]
        # Set design vector
        for i, var in enumerate(design_vector):
            fdm[var] = xd[i]
        # Trim Propulsion
        prop = fdm.get_propulsion()
        prop.init_running(-1)
        # Initialize Sim
        fdm.run_ic()
        return cost(fdm)
    res = scipy.optimize.minimize(fun=lambda xd: trim_cost(fdm,xd), x0=x0, **kwargs)
    if verbose:
        print(res)
    for i, key in enumerate(design_vector):
        ic[key] = res['x'][i]
    return ic, fdm

def simulate(aircraft, op_0, op_list=None, tf=50, realtime=False):
    if op_list is None:
        op_list = []

    root = Path('.').resolve()
    fdm = jsbsim.FGFDMExec(str(root))

    # Load model
    fdm.load_model(aircraft)
    fdm.set_output_directive(str(root) + '/data_output/flightgear.xml')
    fdm_props = fdm.get_property_catalog('')
    
    # Set Location
    set_location(fdm)

    # Set operating points
    set_operating_point(op_0,fdm)

    # Start engines
    prop = fdm.get_propulsion()
    prop.init_running(-1)
    fdm.run_ic()
    
    # Start loop
    log = Logger()
    nice = True
    sleep_nseconds = 500
    initial_seconds = time.time()
    frame_duration = fdm.get_delta_t()
    op_count = 0
    result = fdm.run()
    

    while result and fdm.get_sim_time() < tf:
        
        t = fdm.get_sim_time()
        
        if realtime:
            current_seconds = time.time()
            actual_elapsed_time = current_seconds - initial_seconds
            sim_lag_time = actual_elapsed_time - fdm.get_sim_time()

            for _ in range(int(sim_lag_time / frame_duration)):
                result = fdm.run()
                current_seconds = time.time()
        else:
            result = fdm.run()
            if nice:
                time.sleep(sleep_nseconds / 1000000.0)
        log.new_frame(t, fdm.get_property_catalog(''))
        
    
    log = log.to_pandas()
    return log


def linearize(aircraft, states, states_deriv, inputs, outputs, ic, dx, n_round=3):
    assert len(states_deriv) == len(states)

    root = Path('.').resolve()
    fdm = jsbsim.FGFDMExec(str(root)) # The path supplied to FGFDMExec is the location of the folders "aircraft", "engines" and "systems"
    fdm.set_debug_level(0)    
    fdm.load_model(aircraft)
        
    fdm_props = fdm.get_property_catalog('')
    # print(fdm_props)
    for key in states + inputs + outputs + list( ic.keys()):
        if key not in fdm_props.keys():
            raise KeyError(key)
    
    for key in states_deriv:
        if key not in fdm_props.keys() and key != 'approximate':
            raise KeyError(key)
    
    n = len(states)
    p = len(inputs)
    m = len(outputs)
    
    def set_ic():
        set_location(fdm)
        for key in ic.keys():
            fdm[key] = ic[key]
        fdm.get_propulsion().init_running(-1)
        fdm.run_ic()
    
    A = np.zeros((n, n))
    C = np.zeros((m, n))
    for i, state in enumerate(states):
        set_ic()
        start = fdm.get_property_catalog('')
        fdm[state] = start[state] + dx
        fdm.resume_integration()
        fdm.run()
        for j, state_deriv in enumerate(states_deriv):
            if state_deriv == 'approximate':
                A[j, i] = (fdm[states[j]] - start[states[j]])/fdm.get_delta_t()
            else:
                A[j, i] = (fdm[state_deriv] - start[state_deriv]) / dx
        for j, output in enumerate(outputs):
            C[j, i] = (fdm[output] - start[output]) / dx

    set_ic()

    B = np.zeros((n, p))
    D = np.zeros((m, p))
    for i, inp in enumerate(inputs):
        set_ic()
        start = fdm.get_property_catalog('')
        fdm[inp] = start[inp] + dx
        fdm.resume_integration()
        fdm.run()
        for j, state_deriv in enumerate(states_deriv):
            if state_deriv == 'approximate':
                B[j, i] = (fdm[states[j]] - start[states[j]])/fdm.get_delta_t()
            else:
                B[j, i] = (fdm[state_deriv] - start[state_deriv]) / dx
        for j, output in enumerate(outputs):
            D[j, i] = (fdm[output] - start[output]) / dx
    
    del fdm
    A = np.round(A, n_round)
    B = np.round(B, n_round)
    C = np.round(C, n_round)
    D = np.round(D, n_round)
    return (A, B, C, D)


def rootlocus(sys, kvect=None):
    if kvect is None:
        kvect = np.logspace(-3, 0, 1000)
    rlist, klist = control.rlocus(sys, plot=False, kvect=kvect)
    for root in rlist.T:
        plt.plot(np.real(root), np.imag(root))
        plt.plot(np.real(root[-1]), np.imag(root[-1]), 'bs')
    for pole in control.pole(sys):
        plt.plot(np.real(pole), np.imag(pole), 'rx')
    for zero in control.zero(sys):
        plt.plot(np.real(zero), np.imag(zero), 'go')
    plt.grid()
    plt.xlabel('real')
    plt.ylabel('imag')

def clean_tf(G, tol=1e-5):
    num = G.num
    den = G.den
    for poly in num, den:
        for i in range(len(poly)):
            for j in range(len(poly[i])):
                poly[i][j] = np.where(np.abs(poly[i][j]) < tol, 0, poly[i][j])
    return control.tf(num, den)