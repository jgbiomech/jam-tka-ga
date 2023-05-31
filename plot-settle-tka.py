import os
import opensim as osim
import numpy as np
import h5py
import matplotlib.pyplot as plt

# Import the necessary functions from myFuncs.py:
from myFuncs import readOpenSimMotFile 
from myFuncs import readH5ContactData 
from myFuncs import readH5LigamentData 

# Knee properties:
knee_type = "TKA"
joint     = "knee"
coordID   = "knee_flex"
limb      = "r"


# Contact properties
contact_location = 'tibia_implant' # location in joint (model-specific definition)
contact_variable = 'total_contact_force'

# Ligament properties:
ligament = 'ITB_r'
ligament_variable = 'total_force'

# Pathing:
base_path        = "C:\\opensim-jam\\jam-tka-ga"
results_dir      = base_path+"\\results"
settling_dir     =  results_dir+"\\settling\\"+knee_type
results_basename = "settling"

# Filenames:
jnt_mech_filename = settling_dir + '\\' + results_basename + '.h5'

# Joint mechanics file:
h5file = h5py.File(jnt_mech_filename,'r')
time   = h5file['time'][()]

# Joint contact forces:
tf_JCF = readH5ContactData(h5file,joint,contact_location,contact_variable)

# Plot JCF data:
plt.subplot(1,3,1)
plt.plot(time,tf_JCF[:,0]) 
plt.title('AP force')
plt.subplot(1,3,2)
plt.plot(time,tf_JCF[:,1]) 
plt.title('SI force')
plt.subplot(1,3,3)
plt.plot(time,tf_JCF[:,2]) 
plt.title('ML force')
plt.show()