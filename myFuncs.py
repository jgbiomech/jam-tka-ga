import numpy as np
import matplotlib.pyplot as plt
import os

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

def readOpenSimMotFile(filename): # https://gist.github.com/mitkof6/03c887ccc867e1c8976694459a34edc3
    """ Reads OpenSim .sto files.
    Parameters
    ----------
    filename: absolute path to the .sto file
    Returns
    -------
    header: the header of the .sto
    labels: the labels of the columns
    data: an array of the data
    """

    if not os.path.exists(filename):
        print('file do not exists')

    file_id = open(filename, 'r')

    # read header
    next_line = file_id.readline()
    header = [next_line]
    nc = 0
    nr = 0
    while not 'endheader' in next_line:
        if 'datacolumns' in next_line:
            nc = int(next_line[next_line.index(' ') + 1:len(next_line)])
        elif 'datarows' in next_line:
            nr = int(next_line[next_line.index(' ') + 1:len(next_line)])
        elif 'nColumns' in next_line:
            nc = int(next_line[next_line.index('=') + 1:len(next_line)])
        elif 'nRows' in next_line:
            nr = int(next_line[next_line.index('=') + 1:len(next_line)])

        next_line = file_id.readline()
        header.append(next_line)

    # process column labels
    next_line = file_id.readline()
    if next_line.isspace() == True:
        next_line = file_id.readline()

    labels = next_line.split()

    # get data
    data = []
    for i in range(1, nr + 1):
        d = [float(x) for x in file_id.readline().split()]
        data.append(d)

    # Convert data to a numpy array:
    data = np.array(data)

    file_id.close()

    return header, labels, data