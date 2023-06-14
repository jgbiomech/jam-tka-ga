import os
import opensim as osim
import numpy as np
# from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# Import the necessary functions from myFuncs.py:
# from myFuncs import ***

osim.Logger.setLevelString("info")
useVisualizer=False

### Configure inputs:
sim_acc = 1e-3

ligIDs          = ["Nominal","minOffset","maxOffset"]
slackOffset     = [1,0.95,1.05]
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
jnt_mech = osim.JointMechanicsTool()
jnt_mech.set_results_directory(settling_dir)
jnt_mech.set_start_time(-1)
jnt_mech.set_stop_time(-1)
jnt_mech.set_normalize_to_cycle(False)
jnt_mech.set_contacts(0,'all')
jnt_mech.set_ligaments(0,'all')
jnt_mech.set_muscles(0,'none')
jnt_mech.set_muscle_outputs(0,'none')
jnt_mech.set_attached_geometry_bodies(0,'/bodyset/femur_r')
jnt_mech.set_attached_geometry_bodies(1,'/bodyset/tibia_r')
jnt_mech.set_attached_geometry_bodies(2,'/bodyset/patella_r')
jnt_mech.set_output_orientation_frame('ground')
jnt_mech.set_output_position_frame('ground')
jnt_mech.set_write_vtp_files(False)
jnt_mech.set_write_h5_file(True)
jnt_mech.set_h5_kinematics_data(True)
jnt_mech.set_h5_states_data(True)
jnt_mech.set_use_visualizer(useVisualizer)

idx = 0
for ligID in ligIDs:

    results_basename = "settling_"+ligID # Simulation-specific filename

    # Settings file names:
    comakIK_settings_file  = inputs_dir+"\\comak_IK_settings_"+results_basename+".xml"
    jnt_mech_settings_file = inputs_dir+"\\joint_mechanics_settings_"+results_basename+".xml"

    # Modify model:
    myModel = osim.Model(base_model_file)

    ForceSet = myModel.getForceSet()
    ForceSetSize = ForceSet.getSize()

    for ii in range(0,ForceSetSize-1):
        f = ForceSet.get(ii)
        if f.getConcreteClassName() == 'Blankevoort1991Ligament':
            slack_length = float(f.getPropertyByName('slack_length').toString())
            osim.PropertyHelper.setValueDouble(slackOffset[idx]*slack_length,f.getPropertyByName('slack_length'))
            if print_slacks == 1:
                print(f.getName()," - default slack = ",np.round(slack_length*1000,1)," mm - updated slack = ",np.round(slackOffset[idx]*slack_length*1000,1))
            

    # Print modified model:
    model_file = model_dir+"\\"+subjectID+"_"+ligID+".osim"
    myModel.printToXML(model_dir+"\\"+subjectID+"_"+ligID+".osim")
    
    #############################################################################
    ### Settling simulation - using the COMAK-IK Tool:
    #############################################################################

    # Simulation-specific inputs: 
    IKtool.set_model_file(model_file)
    IKtool.set_results_prefix(results_basename)
    IKtool.printToXML(comakIK_settings_file) # Print simulation-specific file

    # Run the IK tool to determine the settled knee position
    print('Running COMAK-IK Tool...')
    IKtool.run()

    # Clean up COMAK IK files:
    files = os.listdir(settling_dir)

    for file in files:
        if 'sweep' in file:
            os.remove(settling_dir+'\\'+file)

    for file in files:
        if 'settle' in file:
            os.rename(settling_dir+'\\'+file,settling_dir+'\\'+results_basename+'_states.sto')

    #############################################################################
    ### Perform analysis using the Joint Mechanics Tool:
    #############################################################################       

    jnt_mech.set_model_file(model_file)
    jnt_mech.set_input_states_file(settling_dir + '\\' + results_basename + '_states.sto')
    jnt_mech.set_results_file_basename(results_basename)
    jnt_mech.printToXML(jnt_mech_settings_file)

    print('Running JointMechanicsTool...')
    jnt_mech.run()

    idx += 1