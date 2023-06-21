import os
import opensim as osim
import numpy as np
# from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from multiprocessing import Pool

# Import the necessary functions from myFuncs.py:
from myFuncs import run_settle_sim

osim.Logger.setLevelString("info")
useVisualizer=False

### Configure inputs:
sim_acc = 1e-3

ligIDs          = ["Nominal","minOffset","maxOffset"]
# ligID           = "Nominal"
slackOffsets    = [1,0.95,1.05]
# slackOffset     = 1
print_slacks    = 0
knee_type       = "TKA"
subjectID       = "DM"
base_path       = "C:\\opensim-jam\\jam-tka-ga"
inputs_dir      = base_path+"\\inputs"
results_dir     = base_path+"\\results"
settling_dir    =  results_dir+"\\settling\\"+knee_type
model_dir       = "..\\jam-resources\\models\knee_"+knee_type+"\\grand_challenge\\"+subjectID
base_model_file = model_dir+"\\"+subjectID+".osim"

### Configure folders:
os.makedirs(inputs_dir, exist_ok=True)
os.makedirs(results_dir, exist_ok=True)
os.makedirs(settling_dir, exist_ok=True)
os.makedirs(model_dir, exist_ok=True)

### Configure the generic COMAKInverseKinematicsTool settings:
IKtool = osim.COMAKInverseKinematicsTool()
IKtool.set_results_directory(settling_dir)
IKtool.set_perform_secondary_constraint_sim(True)
IKtool.set_secondary_constraint_sim_integrator_accuracy(sim_acc)
IKtool.set_secondary_constraint_sim_internal_step_limit(10000)
IKtool.set_print_secondary_constraint_sim_results(True)
IKtool.set_secondary_constraint_sim_settle_threshold(1e-4)
IKtool.set_secondary_coupled_coordinate('/jointset/knee_r/knee_flex_r')
IKtool.set_secondary_coordinates(0,'/jointset/knee_r/knee_add_r')
IKtool.set_secondary_coordinates(1,'/jointset/knee_r/knee_rot_r')
IKtool.set_secondary_coordinates(2,'/jointset/knee_r/knee_tx_r')
IKtool.set_secondary_coordinates(3,'/jointset/knee_r/knee_ty_r')
IKtool.set_secondary_coordinates(4,'/jointset/knee_r/knee_tz_r')
IKtool.set_secondary_coordinates(5,'/jointset/pf_r/pf_flex_r')
IKtool.set_secondary_coordinates(6,'/jointset/pf_r/pf_rot_r')
IKtool.set_secondary_coordinates(7,'/jointset/pf_r/pf_tilt_r')
IKtool.set_secondary_coordinates(8,'/jointset/pf_r/pf_tx_r')
IKtool.set_secondary_coordinates(9,'/jointset/pf_r/pf_ty_r')
IKtool.set_secondary_coordinates(10,'/jointset/pf_r/pf_tz_r')
IKtool.set_secondary_coupled_coordinate_start_value(0)
IKtool.set_secondary_coupled_coordinate_stop_value(0)
IKtool.set_secondary_constraint_sim_sweep_time(0)
IKtool.set_perform_inverse_kinematics(False)
IKtool.set_use_visualizer(useVisualizer)
# Placeholder settings:
IKtool.set_secondary_constraint_function_file('Unassigned')
IKtool.set_constraint_function_num_interpolation_points
IKtool.set_coordinate_file('Unassigned')
IKtool.set_marker_file("..\\jam-resources\\models\knee_"+knee_type+"\\grand_challenge\\"+subjectID+"\\"+subjectID+"_markers.xml")
IKtool.set_output_motion_file('Unassigned')
IKtool.set_ik_accuracy(1e-5)
IKtool.set_ik_constraint_weight(0)
IKtool.set_time_range(0,0)
IKtool.set_time_range(1,0)
IKtool.set_report_errors(False)
IKtool.set_report_marker_locations(False)
IKtool.set_constrained_model_file('Unassigned')

### Configure the generic JointMechanicsTool settings:
JMtool = osim.JointMechanicsTool()
JMtool.set_results_directory(settling_dir)
JMtool.set_start_time(-1)
JMtool.set_stop_time(-1)
JMtool.set_normalize_to_cycle(False)
JMtool.set_contacts(0,'all')
JMtool.set_ligaments(0,'all')
JMtool.set_muscles(0,'none')
JMtool.set_muscle_outputs(0,'none')
JMtool.set_attached_geometry_bodies(0,'/bodyset/femur_r')
JMtool.set_attached_geometry_bodies(1,'/bodyset/tibia_r')
JMtool.set_attached_geometry_bodies(2,'/bodyset/patella_r')
JMtool.set_output_orientation_frame('ground')
JMtool.set_output_position_frame('ground')
JMtool.set_write_vtp_files(False)
JMtool.set_write_h5_file(True)
JMtool.set_h5_kinematics_data(True)
JMtool.set_h5_states_data(True)
JMtool.set_use_visualizer(useVisualizer)

for ii in range(len(slackOffsets)):
    ligID = ligIDs[ii]
    slackOffset = slackOffsets[ii]
    run_settle_sim(ligID,slackOffset,IKtool,JMtool,subjectID,base_model_file,model_dir,inputs_dir,settling_dir)

if __name__ == '__main__':
    pool = Pool(processes=4)
    for ii in range(len(slackOffsets)):
        ligID = ligIDs[ii]
        slackOffset = slackOffsets[ii]
        pool.apply_async(run_settle_sim,args=(ligID,slackOffset,IKtool,JMtool,subjectID,base_model_file,model_dir,inputs_dir,settling_dir))
        print(ii)
    pool.close()
    pool.join()


    
