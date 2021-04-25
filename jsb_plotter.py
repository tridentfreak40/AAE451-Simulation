import pandas as pd
import matplotlib.pyplot as plt

def plot_stuff(data):
    ft2m = 0.3048
    m2ft = 1/ft2m

    # Fuel Used vs. Time
    # plt.subplot(2,2,1)
    # plt.figure()
    # data['propulsion/engine/fuel-used-lbs'].plot()
    # plt.title('Fuel Used (lbs)')
    # plt.ylabel('lbs')
    # plt.grid()

    # # Fuel Tank Contents vs. Time
    # # plt.subplot(2,2,1)
    # plt.figure()
    # # data['propulsion/tank/contents-lbs'].plot()
    # # data['propulsion/tank[1]/contents-lbs'].plot()
    # # data['propulsion/tank[2]/contents-lbs'].plot()
    # # data['propulsion/tank[3]/contents-lbs'].plot()
    # data['propulsion/tank[4]/contents-lbs'].plot()
    # plt.title('Fuel Tanks Contents (lbs)')
    # plt.ylabel('lbs')
    # plt.grid()

    # # Fuel Burn Rate vs. Time
    # # plt.subplot(2,2,1)
    # # plt.figure()
    # # fuel_used_offset = data['propulsion/engine/fuel-used-lbs']
    # # fuel_used_offset = fuel_used_offset[1:]
    # # fuel_used_delta = fuel_used_offset - data['propulsion/engine/fuel-used-lbs']
    # # burn_rate = fuel_used_delta/data['simulation/dt'][1:]
    # # burn_rate.plot()
    # # plt.title('Fuel Burn Rate (lbs/s)')
    # # plt.ylabel('lbs/s')
    # # plt.grid()

    # # N1, N2 vs. Time
    # # plt.subplot(2,2,2)
    # plt.figure()
    # data['propulsion/engine/n1'].plot()
    # data['propulsion/engine/n2'].plot()
    # plt.title('N1, N2 vs. Time')
    # plt.ylabel('% max')
    # plt.legend(('N1','N2'))
    # plt.grid()

    # Main Engine Thrust vs. Time
    total_thrust = data['propulsion/engine/thrust-lbs'] +\
                   data['propulsion/engine[1]/thrust-lbs'] +\
                   data['propulsion/engine[2]/thrust-lbs'] +\
                   data['propulsion/engine[3]/thrust-lbs']

    # plt.subplot(2,2,3)
    plt.figure()
    data['propulsion/engine/thrust-lbs'].plot()
    data['propulsion/engine[1]/thrust-lbs'].plot()
    data['propulsion/engine[2]/thrust-lbs'].plot()
    data['propulsion/engine[3]/thrust-lbs'].plot()
    total_thrust.plot()
    plt.title('Engine Thrust vs. Time')
    plt.ylabel('lbs')
    plt.legend(('Right Engine 1','Left Engine 1','Right Engine 2','Left Engine 2','Total Thrust'))
    plt.grid()

    # Lift Fan
    total_fan_thrust = data['propulsion/engine[4]/thrust-lbs'] +\
                       data['propulsion/engine[5]/thrust-lbs'] +\
                       data['propulsion/engine[6]/thrust-lbs'] +\
                       data['propulsion/engine[7]/thrust-lbs'] +\
                       data['propulsion/engine[8]/thrust-lbs']
    plt.figure()
    data['propulsion/engine[4]/thrust-lbs'].plot()
    data['propulsion/engine[5]/thrust-lbs'].plot()
    data['propulsion/engine[6]/thrust-lbs'].plot()
    data['propulsion/engine[7]/thrust-lbs'].plot()
    data['propulsion/engine[8]/thrust-lbs'].plot()
    total_fan_thrust.plot()
    plt.title('Fan Thrust and RCS vs. Time')
    plt.ylabel('lbs')
    plt.legend(('Right Liftfan 1','Left Liftfan 1','Right Liftfan 2','Left Liftfan 2','RCS','Total Thrust'))
    plt.grid()

    plt.figure()
    data['propulsion/engine[4]/pitch-angle-rad'].plot()
    data['propulsion/engine[5]/pitch-angle-rad'].plot()
    data['propulsion/engine[6]/pitch-angle-rad'].plot()
    data['propulsion/engine[7]/pitch-angle-rad'].plot()
    plt.title('Fan Pitch Angle vs. Time')
    plt.ylabel('Rad')
    plt.legend(('Right Liftfan 1','Left Liftfan 1','Right Liftfan 2','Left Liftfan 2','Total Thrust'))
    plt.grid()
    # CG vs. Time
    plt.figure()
    data['inertia/cg-x-in'].plot()
    plt.title('CGx vs. Time')
    plt.ylabel('CGx Location (IN)')
    plt.xlabel('Time (s)')
    plt.grid()

    # Moment Balance
    inner_fan = 324
    outer_fan = 328.2
    rcs = 108
    cg_x = data['inertia/cg-x-in']
    right_inner = data['propulsion/engine[4]/thrust-lbs'] * abs(inner_fan-cg_x)
    left_inner = data['propulsion/engine[5]/thrust-lbs'] * abs(inner_fan-cg_x)
    right_outer = data['propulsion/engine[6]/thrust-lbs'] * abs(outer_fan-cg_x)
    left_outer = data['propulsion/engine[7]/thrust-lbs'] * abs(outer_fan-cg_x)
    total_fan = right_inner+left_inner+right_outer+left_outer
    rcs_mom = data['propulsion/engine[8]/thrust-lbs'] * abs(rcs-cg_x)
    plt.figure()
    right_inner.plot()
    left_inner.plot()
    right_outer.plot()
    left_outer.plot()
    total_fan.plot()
    rcs_mom.plot()
    plt.title('Moments vs. Time')
    plt.ylabel('Moments (FT-LBS)')
    plt.xlabel('Time (s)')
    plt.legend(('Right Inner','Left Inner','Right Outer','Left Outer','Total Fan','RCS'))
    plt.grid()
    
    # Angle of Attack vs. Time
    plt.subplot(3,2,4)
    plt.title('Angle of Attack')
    data['aero/alpha-deg'].plot()
    plt.legend()
    plt.grid()
    

    # Attitude vs. Time
    plt.figure()
    data['attitude/phi-deg'].plot()
    data['attitude/theta-deg'].plot()
    data['attitude/psi-deg'].plot()
    plt.ylabel('deg')
    plt.title('Attitude')
    plt.legend()
    plt.grid()

    # Mach vs. Time
    plt.figure()
    data['velocities/mach'].plot()
    plt.ylabel('Mach')
    plt.title('Mach vs. Time')
    plt.legend()
    plt.grid()

    # Position vs. Time
    plt.figure()
    data['position/h-agl-ft'].plot(label='up')
    (data['position/distance-from-start-lat-mt']*m2ft).plot(label='north')
    (data['position/distance-from-start-lon-mt']*m2ft).plot(label='east')
    plt.ylabel('m')
    plt.legend()
    plt.grid()
    plt.title('position')

    # Weight
    plt.figure()
    data['inertia/weight-lbs'].plot()
    plt.ylabel('LBS')
    plt.title('Weight over Time')
    plt.legend()
    plt.grid()
    plt.show()

    # Fan Pitch angle
    plt.figure()
    data['propulsion/engine[4]/pitch-angle-rad'].plot()
    data['propulsion/engine[5]/pitch-angle-rad'].plot()
    data['propulsion/engine[6]/pitch-angle-rad'].plot()
    data['propulsion/engine[7]/pitch-angle-rad'].plot()
    total_thrust.plot()
    plt.title('Fan Pitch vs. Time')
    plt.ylabel('Radians')
    plt.legend(('Right Inner 1','Left Inner 1','Right Outer 2','Left Outer 2'))
    plt.grid()
    return
    