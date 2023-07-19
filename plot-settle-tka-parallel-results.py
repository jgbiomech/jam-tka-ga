import os
import opensim as osim
import numpy as np
import h5py
import matplotlib.pyplot as plt
from matplotlib import gridspec
from sklearn.linear_model import LinearRegression as LR

plt.close('all')

# Import the necessary functions from myFuncs.py:
from myFuncs import readOpenSimMotFile 
from myFuncs import readH5ContactData 
from myFuncs import readH5LigamentData 

extract_data = 0
data_types   = ['RefStrain','Stiffness']

# Ligament properties:
LigBundleNames = ['MCLd','MCLs','pMC','LCL','PFL','pCAP','ITB','PT','mPFL','lPFL','PCLpm','PCLal']
LigBundlesPlot = ['MCLd','MCLs','LCL','ITB','PCLpm','PCLal']
nLigs          = len(LigBundlesPlot)

# Knee properties:
knee_type = "TKA"
joint     = "knee"
coordID   = "knee_flex"
limb      = "r"

# Contact properties
contact_location = 'tibia_implant' # location in joint (model-specific definition)

# Pathing:
base_path        = "C:\\opensim-jam\\jam-tka-ga"
results_dir      = base_path+"\\results"
settling_dir     =  results_dir+"\\settling\\"+knee_type

# LHS design:
LH_data  = np.loadtxt(settling_dir+'\\LH_design.txt')
n_sample = len(LH_data)

refStrainData = np.array(LH_data[:,0:12])
StiffnessData = np.multiply(LH_data[:,12:24],[10,20,10,10,10,8,1,30,15,15,10,10])

### Extract simulation data: ###
if extract_data == 1:
    JCFmed_o = []
    JCFlat_o = []
    for ii in range(n_sample):

        if ii == 0:
            results_basename = "settling_Nominal"
        else:
            results_basename = "settling_LHS"+str(ii)

        # Load joint mechanics data (.h5 file):
        jnt_mech_filename = settling_dir + '\\' + results_basename + '.h5'
        h5file = h5py.File(jnt_mech_filename,'r')

        # Extract compartment joint contact forces:
        JCFlat,JCFmed = readH5ContactData(h5file,joint,contact_location,'regional_contact_force',[4,5])
        JCFmed_o = np.append(JCFmed_o,JCFmed[-1,1])
        JCFlat_o = np.append(JCFlat_o,JCFlat[-1,1])

    np.savetxt(settling_dir+'\\JCFmed.txt',JCFmed_o.reshape(-1,1))
    np.savetxt(settling_dir+'\\JCFlat.txt',JCFlat_o.reshape(-1,1))
else:
    JCFmed_o = np.loadtxt(settling_dir+'\\JCFmed.txt')
    JCFlat_o = np.loadtxt(settling_dir+'\\JCFlat.txt')

y = -1*JCFmed_o.reshape(-1,1)

fig = plt.figure(figsize=(10,8))
plt.subplots_adjust(hspace=0.5,wspace=0.5)
plt.suptitle('Reference strain vs. Medial contact force',fontsize=18,y=0.95)
fig_name = "ref-strain-vs-medial-jcf"
for jj in range(nLigs):

    LigPlot = LigBundlesPlot[jj]

    x = 100*refStrainData[:,LigBundleNames.index(LigPlot)].reshape(-1,1)
    
    linReg = LR()
    linReg.fit(x,y)
    y_pred = linReg.predict(x)

    # Plot LHS data vs. JCF data:
    plt.subplot(3,2,jj+1)
    plt.title(LigPlot)
    plt.scatter(x,y,color='black')
    plt.plot(x,y_pred,color='red')
    plt.ylabel('Force (N)')
    plt.xlabel('Ref. Strain (%)')
# plt.show()
plt.savefig(settling_dir+"\\plots\\"+fig_name+".png",dpi=150)






# # Plot LHS data vs. JCF data:
# plt.subplot(1,2,1)
# plt.title('Medial')


# plt.subplot(1,2,2)
# plt.title('Lateral')
# plt.scatter(x,y2) 
# plt.plot(x,y2_pred,color='red')
# plt.show()
# # plt.title('AP force')
# # plt.subplot(1,3,2)
# # plt.plot(time,tf_JCF[:,1]) 
# # plt.title('SI force')
# # plt.subplot(1,3,3)
# # plt.plot(time,tf_JCF[:,2]) 
# # plt.title('ML force')
# # plt.show()

# # time   = h5file['time'][()]

