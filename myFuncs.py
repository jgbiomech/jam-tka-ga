import numpy as np
import matplotlib.pyplot as plt
import os
import time
import opensim as osim

def configure_knee_flex(time_step,settle_duration,flex_duration,max_knee_flex):

    # Simulation consists of two phases:
    #   settle : allow unconstraind knee DOFs to settle into equilbrium 
    #   flex : prescribe the tibiofemoral flexion
    # All time units are in seconds 

    # Simulation time:
    settle_time = np.arange(0,settle_duration,time_step)
    flex_time   = np.arange(settle_duration,flex_duration + settle_duration, time_step)
    time        = np.concatenate((settle_time, flex_time))
    time_points = [0, settle_duration, settle_duration + flex_duration]

    # Simulation timesteps:
    nSettleSteps = settle_time.shape[0]
    nFlexSteps = flex_time.shape[0]
    nTimeSteps = nSettleSteps + nFlexSteps

    # Define knee flexion:
    knee_flex        = [0,0,max_knee_flex]
    smooth_knee_flex = np.reshape(np.interp(time,time_points,knee_flex),(nTimeSteps,1))

    return smooth_knee_flex,time,nTimeSteps

def splitnonalpha(s):
   pos = 1
   while pos < len(s) and s[pos].isalpha():
      pos+=1
   return (s[:pos], s[pos:])

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

def readH5ContactData(h5file,contact_joint,contact_location,contact_variable,dims):

    if len(dims) == 0:
        data = h5file['model']['forceset']['Smith2018ArticularContactForce'][contact_joint+'_contact'][contact_location][contact_variable][()]
        return data
    elif len(dims) == 2: # for the tibiofemoral joint: region 4 = +z, region 5 = -z. For right knee: +z = lateral compartment and -z = medial compartment
        data_pos = h5file['model']['forceset']['Smith2018ArticularContactForce'][contact_joint+'_contact'][contact_location][contact_variable][str(dims[0])][()]
        data_neg = h5file['model']['forceset']['Smith2018ArticularContactForce'][contact_joint+'_contact'][contact_location][contact_variable][str(dims[1])][()]
        return data_pos,data_neg
    else:
        raise ValueError('Incorrect number of dimensions')


def readH5LigamentData(h5file,ligament,ligament_variable):
    data = h5file['model']['forceset']['Blankevoort1991Ligament'][ligament][ligament_variable][()]
    return data

def run_settle_sim(ligID,refStrain,Stiffness,refStrainNames,forsim_basename,subjectID,base_model_file,knee_type,model_dir,inputs_dir,results_dir,useVisualizer,sim_accuracy):
    start_time = time.time()

    joint    = 'knee'
    limb     = 'r'
    tf_coordIDs = ['flex','add','rot','tx','ty','tz']
    tf_coordIDs = [joint+'_' + coordID for coordID in tf_coordIDs]

    # Folder config: 
    settling_dir = results_dir+"\\settling\\"+knee_type
    forsim_dir   = results_dir+"\\forsim\\"+knee_type
    os.makedirs(settling_dir, exist_ok=True)

    # Simulation-specific output filename:    
    results_basename = "settling_"+ligID 

    # Import OpenSim model:    
    myModel = osim.Model(base_model_file)
    state = myModel.initSystem()

    # Import ForSim data:
    header,forsim_labels,forsim_data = readOpenSimMotFile(forsim_dir+'\\'+forsim_basename+'_states.sto')

    # Extract knee joint state & pose model::
    for coordID in tf_coordIDs:
        idx  = forsim_labels.index("/jointset/"+joint+"_"+limb+"/"+coordID+"_"+limb+"/value")
        myModel.getCoordinateSet().get(coordID+'_'+limb).setValue(state,forsim_data[-1,idx])

    # Update ligament slack lengths:
    ForceSet     = myModel.getForceSet()
    ForceSetSize = ForceSet.getSize()
    for ii in range(0,ForceSetSize-1):
        f = ForceSet.get(ii)
        if f.getConcreteClassName() == 'Blankevoort1991Ligament':
            ligFiberName = f.getName()
            
            # stiffness_default    = float(f.getPropertyByName('linear_stiffness').toString())
            # slack_length_default = float(f.getPropertyByName('slack_length').toString()) # Default slack length
            # print(ligFiberName+' default slack length: '+str(slack_length_default))

            lig_ref_strain     = refStrain[refStrainNames.index(splitnonalpha(ligFiberName)[0])] # Find the ligament bundle reference strain
            lig_stiff_udpated = Stiffness[refStrainNames.index(splitnonalpha(ligFiberName)[0])]
                     
            lig_length = float(f.getOutput('length').getValueAsString(state)) # Updated slack length
            slack_length_updated = lig_length/(lig_ref_strain + 1)

            osim.PropertyHelper.setValueDouble(lig_stiff_udpated,f.getPropertyByName('linear_stiffness'))
            osim.PropertyHelper.setValueDouble(slack_length_updated,f.getPropertyByName('slack_length'))
    
    # Print modified model:
    model_file = model_dir+"\\"+subjectID+"_"+ligID+".osim"
    myModel.printToXML(model_dir+"\\"+subjectID+"_"+ligID+".osim")

    # Simulation-specific settings filenames:
    comakIK_settings_file  = inputs_dir+"\\comak_IK_settings_"+results_basename+".xml"
    jnt_mech_settings_file = inputs_dir+"\\joint_mechanics_settings_"+results_basename+".xml"
    
    ############################################################################
    ### Configure the COMAKInverseKinematicsTool & run the settling simulations: 
    ############################################################################
    IKtool = osim.COMAKInverseKinematicsTool()
    IKtool.set_model_file(model_file)
    IKtool.set_results_prefix(results_basename)
    IKtool.printToXML(comakIK_settings_file) # Print simulation-specific file
    IKtool.set_results_directory(settling_dir)    
    IKtool.set_perform_secondary_constraint_sim(True)
    IKtool.set_secondary_constraint_sim_integrator_accuracy(sim_accuracy)
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

    # Run the IK tool to determine the settled knee position
    print('Running COMAK-IK Tool...')
    IKtool.run()

    # Remove sweep files:
    for file in os.listdir(settling_dir):
        if 'sweep' in file:
            os.remove(settling_dir+'\\'+file)

    # Delete existing states files:
    for file in os.listdir(settling_dir):
        if results_basename+'_states' in file:
            os.remove(settling_dir+'\\'+file)

    # Rename new states files:
    for file in os.listdir(settling_dir):
        if 'settle' in file:            
            os.rename(settling_dir+'\\'+file,settling_dir+'\\'+results_basename+'_states.sto')

    ###########################################################
    ### Configure the JointMechanicsTool & run the simulation: 
    ###########################################################
    JMtool = osim.JointMechanicsTool()
    JMtool.set_model_file(model_file)
    JMtool.set_input_states_file(settling_dir + '\\' + results_basename + '_states.sto')
    JMtool.set_results_file_basename(results_basename)
    JMtool.printToXML(jnt_mech_settings_file)
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

    print('Running JointMechanicsTool...')
    JMtool.run()
    return (time.time()-start_time)