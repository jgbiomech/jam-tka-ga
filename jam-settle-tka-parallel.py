# TO DO:
# Pose model with ForSim output states
# Fix XML config of external_loads.xml
# Configure Latin HyperCube of ligament slack lengths/reference strains
# TBD

import os
import time
import opensim as osim
import numpy as np
import multiprocessing as mp
# from bs4 import BeautifulSoup as bs 
import xml.etree.ElementTree as ET
# from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

nPool = mp.cpu_count()-1

# Import the necessary functions from myFuncs.py:
from myFuncs import run_settle_sim, configure_knee_flex

osim.Logger.setLevelString("info")
useVisualizer=False

### Configure inputs:
F_comp          = 50
sim_accuracy    = 1e-3
knee_type       = "TKA"
subjectID       = "DM"
base_path       = "C:\\opensim-jam\\jam-tka-ga"
inputs_dir      = base_path+"\\inputs"
results_dir     = base_path+"\\results"
settling_dir    = results_dir+"\\settling\\"+knee_type
forsim_dir      = results_dir+"\\forsim\\"+knee_type
model_dir       = "..\\jam-resources\\models\knee_"+knee_type+"\\grand_challenge\\"+subjectID

# Configure files:
base_model_file            = model_dir+"\\"+subjectID+".osim"
prescribed_coord_file      = inputs_dir+"\\prescribed_coordinates.sto"
external_loads_config_file = inputs_dir+"\\external_loads.xml"
external_loads_file        = inputs_dir+"\\external_loads.sto"
 
### Configure folders:
os.makedirs(inputs_dir, exist_ok=True)
os.makedirs(results_dir, exist_ok=True)
os.makedirs(settling_dir, exist_ok=True)
os.makedirs(forsim_dir, exist_ok=True)
os.makedirs(model_dir, exist_ok=True)

### Configure ligament input parameters:
ligIDs          = ["Nominal","minOffset_1","minOffset_2","maxOffset_1","maxOffset_2"]
slackOffsets    = [1,0.95,0.975,1.025,1.05]

### Configure flexion profile for ForSim:
time_step       = 0.01
settle_duration = 1
flex_duration   = 0
max_knee_flex   = 0

smooth_knee_flex,sim_time,nTimeSteps = configure_knee_flex(time_step,settle_duration,flex_duration,max_knee_flex)

data_matrix = osim.Matrix.createFromMat(smooth_knee_flex) # convert to OpenSim matrix format

labels = osim.StdVectorString()
labels.append("knee_flex_r")

prescribed_coord_table = osim.TimeSeriesTable(sim_time, data_matrix, labels)

sto = osim.STOFileAdapter()
sto.write(prescribed_coord_table,prescribed_coord_file)

### Configure the applied loads:

# Loads file:
loads_table      = np.zeros([len(sim_time),9])
F_comp_sim       = F_comp*np.ones(len(sim_time))
loads_table[:,1] = F_comp_sim
data_matrix      = osim.Matrix.createFromMat(loads_table) # convert to OpenSim matrix format

labels = osim.StdVectorString()
labels.append("tibia_r_force_vx")
labels.append("tibia_r_force_vy")
labels.append("tibia_r_force_vz")
labels.append("tibia_r_force_px")
labels.append("tibia_r_force_py")
labels.append("tibia_r_force_pz")
labels.append("tibia_r_torque_x")
labels.append("tibia_r_torque_y")
labels.append("tibia_r_torque_z")

external_loads_table = osim.TimeSeriesTable(sim_time, data_matrix, labels)
sto.write(external_loads_table,external_loads_file)

# # Config file:
# tree = ET.parse(external_loads_config_file)
# root = tree.getroot()

### Configure forsim-specific model:
myModel = osim.Model(base_model_file)

myModel.set_gravity(osim.Vec3([0,0,0])) # disable gravity

ForceSet     = myModel.getForceSet()
ForceSetSize = ForceSet.getSize()
for ii in range(0,ForceSetSize-1):
    f = ForceSet.get(ii)
    if f.getConcreteClassName() == 'Blankevoort1991Ligament':
        f.set_appliesForce(False)
    if f.getConcreteClassName() == 'Millard2012EquilibriumMuscle':
        f.set_appliesForce(False)

# Print modified model:
forsim_model_file = model_dir+"\\"+subjectID+"_forsim.osim"
myModel.printToXML(forsim_model_file)

### Configure ForsimTool Settings:
forsim_basename      = "Fcomp"+str(F_comp)
forsim_settings_file = inputs_dir+"\\forsim_settings_"+forsim_basename+".xml"

# ForsimTool:
forsim = osim.ForsimTool()
forsim.set_model_file(forsim_model_file)
forsim.set_results_directory(forsim_dir)
forsim.set_results_file_basename(forsim_basename)
forsim.set_start_time(-1)
forsim.set_stop_time(-1)
forsim.set_integrator_accuracy(sim_accuracy) # Note this should be 1e-6 for research
forsim.set_constant_muscle_control(0.01) # Set all muscles to 2% activation to represent passive state
forsim.set_use_activation_dynamics(False)
forsim.set_use_muscle_physiology(False)
forsim.set_use_tendon_compliance(False)
forsim.set_unconstrained_coordinates(0,'/jointset/knee_r/knee_add_r')
forsim.set_unconstrained_coordinates(1,'/jointset/knee_r/knee_rot_r')
forsim.set_unconstrained_coordinates(2,'/jointset/knee_r/knee_tx_r')
forsim.set_unconstrained_coordinates(3,'/jointset/knee_r/knee_ty_r')
forsim.set_unconstrained_coordinates(4,'/jointset/knee_r/knee_tz_r')
forsim.set_unconstrained_coordinates(5,'/jointset/pf_r/pf_flex_r')
forsim.set_unconstrained_coordinates(6,'/jointset/pf_r/pf_rot_r')
forsim.set_unconstrained_coordinates(7,'/jointset/pf_r/pf_tilt_r')
forsim.set_unconstrained_coordinates(8,'/jointset/pf_r/pf_tx_r')
forsim.set_unconstrained_coordinates(9,'/jointset/pf_r/pf_ty_r')
forsim.set_unconstrained_coordinates(10,'/jointset/pf_r/pf_tz_r')
forsim.set_external_loads_file(external_loads_config_file)
forsim.set_prescribed_coordinates_file(prescribed_coord_file)
forsim.set_use_visualizer(True)
forsim.printToXML(forsim_settings_file)

### Run ForSim to get initial position after tibial compression:
print('Running Forsim Tool...')
forsim.run()

### Run settling simulations in parallel
if __name__ == '__main__':
    pool = mp.Pool(processes=nPool)   
    for ii in range(len(slackOffsets)):
        ligID = ligIDs[ii]
        slackOffset = slackOffsets[ii]
        pool.apply_async(run_settle_sim,args=(ligID,slackOffset,subjectID,base_model_file,knee_type,model_dir,inputs_dir,settling_dir,useVisualizer,sim_accuracy))
    pool.close()
    pool.join()


# for ii in range(len(slackOffsets)):
#     ligID = ligIDs[ii]
#     slackOffset = slackOffsets[ii]
#     t = run_settle_sim(ligID,slackOffset,subjectID,base_model_file,model_dir,inputs_dir,settling_dir)