from pathlib import Path
import time
from copy import deepcopy

import matplotlib.pyplot as plt
import jsbsim
import scipy.optimize
import pandas as pd
import numpy as np
import pandas as pd

class Logger:
    
    def __init__(self, topics=None):
        self.time = []
        self.data = {}
        self.topics = topics

    def new_frame(self, index, data):
        self.time.append(index)
        if self.topics is not None:
            save_keys = self.topics
        else:
            save_keys = data.keys()
        for key in save_keys:
            if key not in self.data.keys():
                self.data[key] = []
            self.data[key].append(data[key])
    
    def to_pandas(self):
        return pd.DataFrame(self.data, pd.Index(self.time, name='t, sec'))

def set_location(fdm):
    # location, Purdue airport
    fdm['ic/terrain-elevation-ft'] = 18
    fdm['ic/lat-geod-deg'] = 21.328634
    fdm['ic/long-gc-deg'] = -157.906316
    fdm['ic/psi-true-deg'] = 233.2
    # fdm['simulation/do_simple_trim'] = 2
    return

def trim_aircraft(aircraft, ic, design_vector, x0, verbose,
         cost=None, eq_constraints=None, tol=1e-6, ftol=None, show=False, **kwargs):
    root = Path('.').resolve()
    fdm = jsbsim.FGFDMExec(str(root)) # The path supplied to FGFDMExec is the location of the folders "aircraft", "engines" and "systems"
    if show:
        fdm.set_output_directive(str(root/ 'data_output' / 'flightgear.xml'))
    fdm.set_debug_level(0)    
    fdm.load_model(aircraft)

    # set location
    set_location(fdm)
    
    fdm_props = fdm.get_property_catalog('')
    for key in design_vector + list(ic.keys()):
        if key not in fdm_props.keys():
            raise KeyError(key)
    
    if cost is None:
        def cost(fdm):
            # compute cost, force moment balance
            theta = fdm['attitude/theta-rad']
            udot = fdm['accelerations/udot-ft_sec2']
            vdot = fdm['accelerations/vdot-ft_sec2']
            wdot = fdm['accelerations/wdot-ft_sec2']
            pdot = fdm['accelerations/pdot-rad_sec2']
            qdot = fdm['accelerations/qdot-rad_sec2']
            rdot = fdm['accelerations/rdot-rad_sec2']
            # (theta < 0) term prevents nose down trim
            return udot**2 + vdot**2 + wdot**2 + pdot**2 + qdot**2 + rdot**2 + (1e-3*(theta < 0))**2

    # cost function for trimming
    def eval_fdm_func(xd, fdm, ic, fdm_func):
        # set initial condition
        for var in ic.keys():
            fdm[var] = ic[var]

        # set design vector
        for i, var in enumerate(design_vector):
            fdm[var] = xd[i]
        
        # trim propulsion
        prop = fdm.get_propulsion()
        prop.init_running(-1)

        # set initial conditions
        fdm.run_ic()
        
        return fdm_func(fdm)

    # setup constraints
    constraints = []
    if eq_constraints is None:
        eq_constraints = []
    for con in eq_constraints:
        constraints.append({
            'type': 'eq',
            'fun': eval_fdm_func,
            'args': (fdm, ic, con)
        })
    
    # solve
    res = scipy.optimize.minimize(
        fun=eval_fdm_func,
        args=(fdm, ic, cost), x0=x0, 
        constraints=constraints, **kwargs)
    
    if verbose:
        print(res)
    
    # update ic
    for i, var in enumerate(design_vector):
        ic[var] = res['x'][i]
    
    if verbose:
        for con in constraints:
            print('constraint', con['type'], con['fun'](res['x'], *con['args']))
    
    if not res['success'] or ftol is not None and abs(res['fun']) > ftol:
        raise RuntimeError('trim failed:\n' + str(res) + '\n' + \
                           str(fdm.get_property_catalog('acceleration')))
    props = fdm.get_property_catalog('')
    del fdm
    return ic, props
    

def simulate(aircraft, op_0, op_list=None, tf=50, realtime=False, verbose=False):
    if op_list is None:
        op_list = []
        
    root = Path('.').resolve()
    fdm = jsbsim.FGFDMExec(str(root)) # The path supplied to FGFDMExec is the location of the folders "aircraft", "engines" and "systems"

    # load model
    fdm.load_model(aircraft)
    fdm.set_output_directive(str(root/ 'data_output' / 'flightgear.xml'))
    fdm_props = fdm.get_property_catalog('')
    
    # add a method to check that properties being set exist
    def set_opereating_point(props):
        for key in props.keys():
            if key not in fdm_props.keys():
                raise KeyError(key)
            else:
                fdm[key] = props[key]
    
    # set location
    set_location(fdm)
    
    # set operating points
    set_opereating_point(op_0)
    
    # start engines
    prop = fdm.get_propulsion()
    prop.init_running(-1)
    fdm.run_ic()

    # start loop
    log = Logger()
    nice = True
    sleep_nseconds = 500
    initial_seconds = time.time()
    frame_duration = fdm.get_delta_t()
    op_count = 0
    result = fdm.run()
    low_fuel_warned = False
    
    while result and fdm.get_sim_time() < tf:
        t = fdm.get_sim_time()
        if op_count < len(op_list) and op_list[op_count][2](fdm):
            if verbose:
                print('operating point reached: ', op_list[op_count][0])
            set_opereating_point(op_list[op_count][1])
            op_count += 1
        
        # rough approximation of grade of runway 28 at purdue airport
        if fdm['position/distance-from-start-mag-mt'] < 2000:
            fdm['position/terrain-elevation-asl-ft'] = 586 + 0.02*fdm['position/distance-from-start-mag-mt']
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
        
        if not low_fuel_warned and fdm['propulsion/total-fuel-lbs'] < 10:
            print('warning: LOW FUEL {:0.2f} lbs, restart simulation'.format(fdm['propulsion/total-fuel-lbs']))
            low_fuel_warned = True

        log.new_frame(t, fdm.get_property_catalog(''))

    log = log.to_pandas()
    del fdm
    return log