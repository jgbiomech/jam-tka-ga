import os
import opensim as osim
import numpy as np
# from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# Import the necessary functions from myFuncs.py:
from myFuncs import configure_knee_flex 

osim.Logger.setLevelString("trace")
useVisualizer=True

knee_type = "TKA"
subjectID = "DM"

base_path        = "C:\\opensim-jam\\jam-tka-ga"
# model_file       = "..\\jam-resources\\models\knee_healthy\\lenhart2015\\lenhart2015.osim"
model_file       = "..\\jam-resources\\models\knee_"+knee_type+"\\grand_challenge\\"+subjectID+"\\"+subjectID+".osim"
results_basename = "passive_flexion"
inputs_dir       = base_path+"\\inputs"
results_dir      = base_path+"\\results"

forsim_result_dir   = results_dir+"\\forsim\\"+knee_type
jnt_mech_result_dir = results_dir+"\\joint-mechanics\\"+knee_type

### Create Input Files

os.makedirs(inputs_dir, exist_ok=True)
os.makedirs(results_dir, exist_ok=True)
os.makedirs(forsim_result_dir, exist_ok=True)
os.makedirs(jnt_mech_result_dir, exist_ok=True)

prescribed_coord_file  = inputs_dir+"\\prescribed_coordinates.sto"
forsim_settings_file   = inputs_dir+"\\forsim_settings.xml"
jnt_mech_settings_file = inputs_dir+"\\joint_mechanics_settings.xml"

### Configure knee flexion:

time_step       = 0.01
settle_duration = 0.5
flex_duration   = 2.0
max_knee_flex   = 0

smooth_knee_flex,time,nTimeSteps = configure_knee_flex(time_step,settle_duration,flex_duration,max_knee_flex)

pelvis_tilt = np.ones((nTimeSteps,1))*90

time_std = osim.StdVectorDouble()
for t in np.nditer(time):
    time_std.push_back(float(t))

data_array = np.concatenate((smooth_knee_flex,pelvis_tilt),1)

data_matrix = osim.Matrix.createFromMat(data_array)

labels = osim.StdVectorString()

# labels.append("time")
labels.append("knee_flex_r")
labels.append("pelvis_tilt")

prescribed_coord_table = osim.TimeSeriesTable(time, data_matrix, labels)

sto = osim.STOFileAdapter()
sto.write(prescribed_coord_table,prescribed_coord_file);

### Perform Simulation with ForsimTool
forsim = osim.ForsimTool()
forsim.set_model_file(model_file)
forsim.set_results_directory(forsim_result_dir)
forsim.set_results_file_basename(results_basename)
forsim.set_start_time(-1)
forsim.set_stop_time(-1)
forsim.set_integrator_accuracy(1e-2) # Note this should be 1e-6 for research
forsim.set_constant_muscle_control(0.02) # Set all muscles to 2% activation to represent passive state
# forsim.set_ignore_activation_dynamics(True)
# forsim.set_ignore_tendon_compliance(True)
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
forsim.set_prescribed_coordinates_file(prescribed_coord_file)
forsim.set_use_visualizer(True)
forsim.printToXML(forsim_settings_file)

print('Running Forsim Tool...')
forsim.run()

### Perform Analysis with JointMechanicsTool
jnt_mech = osim.JointMechanicsTool()
jnt_mech.set_model_file(model_file)
jnt_mech.set_input_states_file(forsim_result_dir + '\\' + results_basename + '_states.sto')
jnt_mech.set_results_file_basename(results_basename)
jnt_mech.set_results_directory(jnt_mech_result_dir)
jnt_mech.set_start_time(-1)
jnt_mech.set_stop_time(-1)
jnt_mech.set_normalize_to_cycle(False)
jnt_mech.set_contacts(0,'all')
jnt_mech.set_ligaments(0,'none')
jnt_mech.set_muscles(0,'none')
jnt_mech.set_muscle_outputs(0,'none')
jnt_mech.set_attached_geometry_bodies(0,'/bodyset/femur_r')
jnt_mech.set_attached_geometry_bodies(1,'/bodyset/tibia_r')
jnt_mech.set_attached_geometry_bodies(2,'/bodyset/patella_r')
jnt_mech.set_output_orientation_frame('/bodyset/tibia_r')
jnt_mech.set_output_position_frame('/bodyset/tibia_r')
jnt_mech.set_write_vtp_files(True)
jnt_mech.set_write_h5_file(True)
jnt_mech.set_h5_kinematics_data(True)
jnt_mech.set_h5_states_data(True)
jnt_mech.set_use_visualizer(useVisualizer)
jnt_mech.printToXML(jnt_mech_settings_file)

print('Running JointMechanicsTool...')
jnt_mech.run()
