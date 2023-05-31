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

MCLs_IDs = np.linspace(1,10,10,dtype=int)

# Contact properties
contact_location = 'tibia_implant' # location in joint (model-specific definition)
contact_variable = 'total_contact_force'

# Ligament properties:
ligament = 'ITB_r'
ligament_variable = 'total_force'

# Pathing:
base_path           = "C:\\opensim-jam\\jam-tka-ga"
results_dir         = base_path+"\\results"
forsim_result_dir   = results_dir+"\\forsim\\"+knee_type
jnt_mech_result_dir = results_dir+"\\joint-mechanics\\"+knee_type
results_basename    = "passive_flexion"

# Filenames:
forsim_filename   = forsim_result_dir + '\\' + results_basename + '_states.sto'
jnt_mech_filename = jnt_mech_result_dir + '\\' + results_basename + '.h5'

# ForSim file:
header,labels,data = readOpenSimMotFile(forsim_filename)

time = data[:,0] # slice data to obtain the time array (the first column in the states file)
idx  = labels.index("/jointset/"+joint+"_"+limb+"/"+coordID+"_"+limb+"/value")
knee_flex_data = data[:,idx]

# Joint mechanics file:
h5file = h5py.File(jnt_mech_filename,'r')

# Joint contact forces:
tf_JCF = readH5ContactData(h5file,joint,contact_location,contact_variable)
data_y = tf_JCF[:,1]

# Ligament force - example: MCLs: 
MCLs_fibre_force = []
for MCLs_ID in MCLs_IDs:
    lig_fibre        = 'MCLs'+str(MCLs_ID)
    lig_force        = readH5LigamentData(h5file,lig_fibre,ligament_variable)
    MCLs_fibre_force = np.append(MCLs_fibre_force,lig_force)
MCLs_fibre_force = MCLs_fibre_force.reshape(knee_flex_data.shape[0],-1,order='F') # Individual force of the ligament
MCLs_bundle_force = np.sum(MCLs_fibre_force,axis=1) # Total bundle force of the ligament

# Plot data:
plt.subplot(1,2,1)
plt.plot(knee_flex_data,MCLs_fibre_force) 
plt.subplot(1,2,2)
plt.plot(knee_flex_data,MCLs_bundle_force) 
plt.show()