import os
import opensim as osim
import numpy as np
import matplotlib.pyplot as plt

# Import the necessary functions from myFuncs.py:
from myFuncs import readOpenSimMotFile 

knee_type = "TKA"
joint     = "knee"
coordID   = "knee_flex"
limb      = "r"

# Pathing:
base_path         = "C:\\opensim-jam\\jam-tka-ga"
results_dir       = base_path+"\\results"
forsim_result_dir = results_dir+"\\forsim\\"+knee_type
results_basename  = "passive_flexion"

filename = forsim_result_dir + '\\' + results_basename + '_states.sto'

header,labels,data = readOpenSimMotFile(filename)

time = data[:,0] # slice data to obtain the time array (the first column in the states file)

idx = labels.index("/jointset/"+joint+"_"+limb+"/"+coordID+"_"+limb+"/value")

knee_flex_data = data[:,idx]

plt.plot(time,knee_flex_data)
plt.show()
