import numpy as np
import matplotlib.pyplot as plt
import os
import opensim as osim

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

def readH5ContactData(h5file,contact_joint,contact_location,contact_variable):
    data = h5file['model']['forceset']['Smith2018ArticularContactForce'][contact_joint+'_contact'][contact_location][contact_variable][()]
    return data

def readH5LigamentData(h5file,ligament,ligament_variable):
    data = h5file['model']['forceset']['Blankevoort1991Ligament'][ligament][ligament_variable][()]
    return data

def run_settle_sim(ligID,slackOffset,IKtool,JMtool,subjectID,base_model_file,model_dir,inputs_dir,settling_dir):
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
            osim.PropertyHelper.setValueDouble(slackOffset*slack_length,f.getPropertyByName('slack_length'))
            # if print_slacks == 1:
            #    print(f.getName()," - default slack = ",np.round(slack_length*1000,1)," mm - updated slack = ",np.round(slackOffset[idx]*slack_length*1000,1))
    
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

    JMtool.set_model_file(model_file)
    JMtool.set_input_states_file(settling_dir + '\\' + results_basename + '_states.sto')
    JMtool.set_results_file_basename(results_basename)
    JMtool.printToXML(jnt_mech_settings_file)

    print('Running JointMechanicsTool...')
    JMtool.run()
