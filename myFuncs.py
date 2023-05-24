import numpy as np
import matplotlib.pyplot as plt

def configure_knee_flex(time_step,settle_duration,flex_duration,max_knee_flex):

    # Simulation consists of two phases:
    #   settle : allow unconstraind knee DOFs to settle into equilbrium 
    #   flex : prescribe the tibiofemoral flexion
    # All time units are in seconds 

    # Simulation time:
    settle_time = np.arange(0,settle_duration,time_step)
    flex_time   = np.arange(settle_duration,flex_duration + settle_duration, time_step)
    time        = np.concatenate((settle_time, flex_time));
    time_points = [0, settle_duration, settle_duration + flex_duration];

    # Simulation timesteps:
    nSettleSteps = settle_time.shape[0];
    nFlexSteps = flex_time.shape[0];
    nTimeSteps = nSettleSteps + nFlexSteps

    # Define knee flexion:
    knee_flex        = [0,0,max_knee_flex]
    smooth_knee_flex = np.reshape(np.interp(time,time_points,knee_flex),(nTimeSteps,1))

    return smooth_knee_flex,time,nTimeSteps