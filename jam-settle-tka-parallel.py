# TO DO:
# Fix XML config of external_loads.xml
# GA code after initial LH sampling
#   - Process LH sampling 

import os
import time
import opensim as osim
import numpy as np
import pandas as pd 
import multiprocessing as mp
# from bs4 import BeautifulSoup as bs 
import xml.etree.ElementTree as ET
from scipy.stats import qmc
import matplotlib.pyplot as plt
import h5py

nPool = int(mp.cpu_count()/2)
LHS_factor = 2

# Import the necessary functions from myFuncs.py:
from myFuncs import run_settle_sim, configure_knee_flex, readH5ContactData, NDsort, BinTour, SBX, MutateChildren

osim.Logger.setLevelString("info")
useVisualizer=False

reset_LHdesign  = 0
run_init_gen    = 0
run_parallel    = 1#1

run_forsim_init = 0
run_debug       = 0
plot_demo       = 1

### Configure inputs:
F_comp           = 50
sim_accuracy     = 1e-3
knee_type        = "TKA"
contact_location = "tibia_implant"
joint            = "knee"
subjectID        = "DM"
base_path        = "C:\\opensim-jam\\jam-tka-ga"
inputs_dir       = base_path+"\\inputs"
results_dir      = base_path+"\\results"
settling_dir     = results_dir+"\\settling\\"+knee_type
forsim_dir       = results_dir+"\\forsim\\"+knee_type
model_dir        = "..\\jam-resources\\models\knee_"+knee_type+"\\grand_challenge\\"+subjectID
forsim_basename  = "Fcomp"+str(F_comp)

### Configure the Member data structure:
class Member():
    def __init__(self):
        self.name    = []
        self.JCFmed  = []
        self.JCFlat  = []
        self.Obj     = []
        self.eref    = []
        self.stiff   = []        
        self.nViol   = 0
        self.violSum = 0
        self.rank    = 0
        self.dist    = 0
        self.nChild  = 0

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
LigBundleNames   = ['MCLd','MCLs','pMC','LCL','PFL','pCAP','ITB','PT','mPFL','lPFL','PCLpm','PCLal']

# Reference strains:
refStrainNominal = [0.03,0.03,0.05,0.06,-0.01,0.08,0.02,0.02,-0.05,0.01,-0.06,0.01]
refStrainMinCI   = [-0.01,-0.01,0.01,0.02,-0.05,0.04,-0.02,-0.02,-0.09,-0.03,-0.10,-0.03]
refStrainMaxCI   = [0.07,0.07,0.09,0.10,0.03,0.12,0.06,0.06,-0.01,0.05,-0.02,0.04]

# Stiffness:
StiffNominal = [2800/10,2200/20,2000/10,1800/10,3000/10,4000/8,4000,14700/30,1000/15,800/15,2400/10,5700//10]
StiffMinCI   = [1120/10,880/20,800/10,720/10,1200/10,1600/8,1600,5880/30,400/15,320/15,960/10,2280/10]
StiffMaxCI   = [4480/10,3520/20,3200/10,2880/10,4800/10,6400/8,6400,23520/30,1600/15,1280/15,3840/10,9120/10]

NominalValues = np.concatenate((refStrainNominal,StiffNominal),axis=0)
minCIValues   = np.concatenate((refStrainMinCI,StiffMinCI),axis=0)
maxCIValues   = np.concatenate((refStrainMaxCI,StiffMaxCI),axis=0)

### Configure the LHS of the ligament input parameters:
if reset_LHdesign == 1:
    LH_sampler = qmc.LatinHypercube(d=len(NominalValues))
    LH_sample  = LH_sampler.random(len(NominalValues)*LHS_factor)
    LH_data    = qmc.scale(LH_sample,minCIValues,maxCIValues)
    LH_data    = np.insert(LH_data,0,NominalValues,axis=0) # Add nominal data
    np.savetxt(settling_dir+'\\LH_design.txt',LH_data)
else:
    LH_data = np.loadtxt(settling_dir+'\\LH_design.txt')

### Optimisation inputs:
nMembers = int(len(LH_data)/2) # Total members of a current population
nGen = 2
nObj = 2 # Number of objective functions
crVal = 1e5
pC    = 0.9
pM    = 0.1
nC    = 2 # Crossover factor
nM    = 50
nChildMax = 3

if plot_demo == 1:
    fig_demo = plt.figure(figsize=(4,4))
    plt.title('Latin Hypercube space-filling design example')
    plt.scatter(100*LH_data[:,0],100*LH_data[:,1],color='black')
    plt.xlabel('MCLs reference strain (%)')
    plt.ylim((-1,7))
    plt.ylabel('MCLd reference strain (%)')
    plt.xlim((-1,7))
    plt.savefig(base_path+"\\overview\\LHS-example.png",dpi=150)

simConfigs = ['Nominal'] # Bake the nominal configuration into the initial simulations (i.e., remove the last LH sample)
for ii in range(1,len(LH_data)):
    simConfigs.append('LHS'+str(ii))

if run_forsim_init == 1:

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

if run_debug == 1:
    idx = 17
    simConfig = simConfigs[idx]
    refStrain = LH_data[idx,0:len(refStrainNominal)]
    Stiffness = LH_data[idx,len(refStrainNominal):2*len(StiffNominal)]
    run_settle_sim(simConfig,refStrain,Stiffness,LigBundleNames,forsim_basename,subjectID,base_model_file,knee_type,model_dir,inputs_dir,results_dir,useVisualizer,sim_accuracy)

### Run settling simulations in parallel
if __name__ == '__main__':

    # Iterate through generations:
    for Gen in range(0,nGen):

        if Gen == 0: # First generation
            
            # Create NewPop of the first generation from the LH sample:
            NewPop = []
            for ii in range(0,len(simConfigs)):
                mem = Member()
                if ii == 0:
                    mem.eref  = np.array(refStrainNominal)
                    mem.stiff = np.array(StiffNominal)
                else:
                    mem.eref  = LH_data[ii-1,0:len(refStrainNominal)]
                    mem.stiff = LH_data[ii-1,len(refStrainNominal):2*len(StiffNominal)] 
                mem.name = simConfigs[ii]
                NewPop.append(mem)

        else: # Subsequent generations
            
            # Create NewPop using the genetic algorithm:
            NewPop = []
            SelectedParents = np.zeros(shape=(nMembers,2),dtype=np.uint8)
            idx_child = 0
            ii = nMembers
            var_old = np.zeros(shape=(len(Pop),2*len(Pop[0].eref)))
            cnt = 0
            while ii != 0:

                cnt += 1

                p1,p2 = BinTour(Pop) # Find the candidate parents of the child using a Binary tournament

                p1_var = np.concatenate((Pop[p1].eref,Pop[p1].stiff),axis=0)
                p2_var = np.concatenate((Pop[p2].eref,Pop[p2].stiff),axis=0)
                var    = np.row_stack((np.reshape(p1_var,(1,len(p1_var))),np.reshape(p2_var,(1,len(p2_var)))))

                # Crossover:            
                if np.random.uniform(0,1) < pC: # Only if sufficient probability, otherwise inherit vars from parent
                    var = SBX(var,nC)

                var_order = np.random.permutation(2) # Randomise the order of the children
                for idx in var_order:
                    
                    # Mutation:                    
                    var[idx,:] = MutateChildren(var[idx,:],pM,nM,minCIValues,maxCIValues)
                    
                    # Check for limit in parents number of children:
                    if ii > 0 and Pop[p1].nChild < nChildMax and Pop[p2].nChild < nChildMax:

                        if ii == nMembers: # Collate the var data for the current population
                            for jj in range(0,len(Pop)):
                                var_old[jj,:] = np.concatenate((Pop[jj].eref,Pop[jj].stiff))
                        
                        var_check = np.zeros(shape=(len(Pop))) 
                        for jj in range(0,len(Pop)):
                            var_check[jj] = np.isin(var[idx,:],var_old[jj,:]).sum()
                        
                        # Check for any repetition --> if none, create child:
                        if ~np.any(var_check==var[idx,:].shape[0]): 

                            mem = Member()
                            mem.name  = 'Gen'+str(Gen)+'_Child'+str(idx_child)
                            mem.eref  = var[idx,0:int(nMembers/2)]
                            mem.stiff = var[idx,int(nMembers/2):nMembers+1]
                            NewPop.append(mem)

                            # Increase child count of parent:
                            Pop[p1].nChild = Pop[p1].nChild + 1
                            Pop[p2].nChild = Pop[p2].nChild + 1

                            # Debug:
                            SelectedParents[idx_child] = [p1,p2]

                            # Update counters:
                            ii -= 1
                            idx_child += 1

        # Run settling simulations in parallel for NewPopulation (nMembers):
        if run_parallel == 1 and (Gen>0 or run_init_gen == 1):
            pool = mp.Pool(processes=nPool)   
            for ii in range(len(NewPop)):
                print(ii)
                simConfig     = NewPop[ii].name 
                refStrainData = NewPop[ii].eref 
                StiffnessData = NewPop[ii].stiff 
                pool.apply_async(run_settle_sim,args=(simConfig,refStrainData,StiffnessData,LigBundleNames,forsim_basename,subjectID,base_model_file,knee_type,model_dir,inputs_dir,results_dir,useVisualizer,sim_accuracy))
            pool.close()
            pool.join()

            # Add check for any missing simulations:
            idxMissing = [1] # Initialise
            while len(idxMissing) != 0:
                idxMissing = [] # Reset
                for ii in range(len(NewPop)):            
                    if Gen == 0: 
                        if ii == 0:
                            results_basename = "settling_Nominal"
                        else:
                            results_basename = "settling_LHS"+str(ii)
                    else:
                        results_basename = "settling_"+NewPop[ii].name

                    fileID = settling_dir+'\\'+results_basename
                    if os.path.exists(fileID+'.h5') is False or os.path.exists(fileID+'_states.sto') is False:
                        idxMissing.append(ii)
                
                if len(idxMissing) != 0: # Run missing simulations
                    pool = mp.Pool(processes=nPool)   
                    for idx in idxMissing:
                        simConfig     = NewPop[idx].name 
                        refStrainData = NewPop[idx].eref 
                        StiffnessData = NewPop[idx].stiff 
                        pool.apply_async(run_settle_sim,args=(simConfig,refStrainData,StiffnessData,LigBundleNames,forsim_basename,subjectID,base_model_file,knee_type,model_dir,inputs_dir,results_dir,useVisualizer,sim_accuracy))
                    pool.close()
                    pool.join()

        # Process simulation data --> Extract joint contact forces & determine objective function:
        for ii in range(len(NewPop)):            
            if ii == 0:
                results_basename = "settling_Nominal"
            else:
                if Gen == 0: 
                    results_basename = "settling_LHS"+str(ii)
                else:
                    results_basename = "settling_"+NewPop[ii].name

            h5file    = h5py.File(settling_dir+'\\'+results_basename+'.h5','r')
            JCFl,JCFm = readH5ContactData(h5file,joint,contact_location,'regional_contact_force',[4,5])

            if ii == 0 and Gen == 0: # Find nominal joint contact forces
                JCFnet_nom = np.linalg.norm(JCFm[-1,:]) + np.linalg.norm(JCFl[-1,:])

            JCFmed = np.linalg.norm(JCFm[-1,:])
            JCFlat = np.linalg.norm(JCFl[-1,:])
            JCFnet = JCFmed + JCFlat

            NewPop[ii].JCFmed = JCFmed
            NewPop[ii].JCFlat = JCFlat
            NewPop[ii].Obj    = np.array((np.power((JCFmed-JCFlat),4),np.power((JCFnet-JCFnet_nom),4)))

        if Gen == 0:
            Pop = NewPop # len(Pop) = 2*nMembers + 1
        else:
            for ii in range(nMembers):
                Pop.append(NewPop[ii]) # len(Pop) = 2*nMembers

        # Non-dominated sort to rank the best nMembers in the population:
        Pop = NDsort(Pop,nObj,nMembers,crVal) # len(Pop) = nMembers

        # Reset child count of current gener
        for ii in range(0,len(Pop)):
            Pop[ii].nChild = 0

        if Gen == 1:
            t = []

        # Evaluate GA performance
            # Terminate if conditions are met
    