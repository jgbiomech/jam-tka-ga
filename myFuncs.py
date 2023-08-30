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

def NDsort(Pop,nObj,nMembers,crVal):

    domMat = np.zeros(shape=(len(Pop),len(Pop)))
    for p in range(len(Pop)):
        for q in range(len(Pop)):
            if Pop[p].nViol == 0 and Pop[q].nViol == 0:
                domMat[p,q] = np.sum(np.sum((Pop[q].Obj-Pop[p].Obj)>0) == nObj)
            elif Pop[p].nViol == 0:
                domMat[p,q] = 1
            elif Pop[q].nViol == 0:
                domMat[q,p] = 1
            else:
                if Pop[p].violSum<Pop[q].violSum:
                    domMat[p,q] = 1
                elif Pop[q].violSum<Pop[p].violSum:
                    domMat[q,p] = 1

    domCount = np.sum(domMat,axis=0)
    frUnq,frIdx,frInv = np.unique(domCount,return_index=True,return_inverse=True)

    frIdx = np.linspace(0,len(frUnq),len(frUnq)+1)
    rank = frIdx[frInv]

    # Calculate crowding distance:
    crDist = np.zeros(shape=(len(Pop),1)) # Greater distance == better performance
    for ii in range(0,len(frIdx)):

        idx = [zz for zz, x in enumerate(rank==ii) if x] # Determine members of a given rank
        
        c = 0
        valores = np.zeros(shape=(len(idx),2))
        for jj in idx:                               
            valores[c] = Pop[jj].Obj
            c += 1
        orden = np.argsort(valores,axis=0) # Sort memebers along 1st direction
        crowding = crVal*np.ones(shape=(valores.shape[0],valores.shape[1])) # Assign a large value for extreme solutions of each pareto front --> least crowded

        if len(valores)>2: # Calculate crowding distance for the non-extreme members of a pareto front
            for jj in range(0,nObj):
                crowding[orden[1:len(orden)-1,jj],jj]  = (valores[orden[2:len(orden),jj],jj]-valores[orden[0:len(orden)-2,jj],jj])/(np.max(valores[:,jj])-np.min(valores[:,jj]))

        c = 0 # Assign crowding distance to each member
        for jj in idx:                
            crDist[jj] = np.sum(crowding[c,:])
            c += 1

    # Subselect best members: 
    rankDist = np.column_stack((np.reshape(rank,(len(rank),1)),crDist))

    idx = np.lexsort((-rankDist[:,1],rankDist[:,0])) # Sort first by the rank and then the descending crowding distance (i.e., more distance == better)

    SelPop = []
    for ii in range(0,nMembers):
        mem = Pop[idx[ii]]
        mem.rank = int(rank[idx[ii]])
        mem.dist = crDist[idx[ii]]
        SelPop.append(mem)
    
    return (SelPop)

def BinTour(Pop):

    p1 = 0
    p2 = 0
    while p1 == 0 or p2 == 0:

        tmnt = np.random.randint(0,len(Pop),size=[2,2])

        p = np.zeros(shape=2)
        for jj in range(0,2):
            p[jj] = tmnt[0,jj]
            if Pop[tmnt[0,jj]].rank > Pop[tmnt[1,jj]].rank or (Pop[tmnt[0,jj]].rank == Pop[tmnt[1,jj]].rank and Pop[tmnt[0,jj]].dist < Pop[tmnt[1,jj]].dist):
                p[jj] = tmnt[1,jj]
        
        if p[0] != p[1]:
            p1 = int(p[0])
            p2 = int(p[1])

    return(p1,p2)

def SBX(var,nC):
                
    nInd = var.shape[1]
    b = np.zeros(shape=nInd)
    for jj in range(0,nInd):
        u = np.random.uniform(0,1)
        b[jj] = np.power((2*u),(1/(nC+1)))
        if u > 0.5:
            b[jj] = np.power(2*(1-u),(-1/(nC+1)))

    sbx = np.tile(np.mean(var,axis=0),(2,1))
    sbx = sbx + 0.5*np.row_stack((b,-b))*(np.max(var,axis=0)-np.min(var,axis=0))   

    return(sbx)

def MutateChildren(var,pM,nM,minCIValues,maxCIValues):

    for ii in range(0,len(var)):
        u = np.random.uniform(0,1)
        if u <= pM:
            if u <= 0.5:
                d = np.power((2*u),(1/(nM+1)))-1
                var[ii] = var[ii] + d*(var[ii] - minCIValues[ii])
            else:
                d = 1-np.power(2*(1-u),(1/(nM+1)))
                var[ii] = var[ii] + d*(maxCIValues[ii] - var[ii])
    
    var = np.maximum(var,minCIValues)
    var = np.minimum(var,maxCIValues)

    return(var)