#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  7 13:43:19 2019

@author: rega0051
"""

#%%
# Import Libraries
import numpy as np
import json

import Loader
import OpenData
import KinematicTransforms
import AirData

import matplotlib.pyplot as plt

fileLog = '/Users/louismueller/Documents/UAV_Lab/FlightArchive/Thor/ThorFLT125/ThorFLT125.h5'
fileTestDef = '/Users/louismueller/Documents/UAV_Lab/FlightArchive/Thor/ThorFLT125/FLT_Def.json'
fileSysConfig = '/Users/louismueller/Documents/UAV_Lab/FlightArchive/Thor/ThorFLT125/thor.json'

#%%
# Read in raw h5 data into dictionary and data
oData, h5Data = Loader.Log_RAPTRS(fileLog, fileSysConfig)

# Plot Overview of flight
OpenData.PlotOverview(oData)

#%% Find Excitation Times
excList = OpenData.FindExcite(oData)

print('\n\nFlight Excitation Times:\n')
for iExc in range(0, len(excList)):
    print('Excitiation: ', excList[iExc][0], ', Time: [', excList[iExc][1][0], ',', excList[iExc][1][1], ']')


#%% Save _init.json file
# Open init json file
testDef = {}
#json_init = JsonRead(fileTestDef)

# Open flight json file
sysConfig = Loader.JsonRead(fileSysConfig)
testPointList = sysConfig['Mission-Manager']['Test-Points']

testDef['Test-Points'] = OpenData.TestPointOut(excList, testPointList)

if False:
    Loader.JsonWrite(fileTestDef, testDef)
    print('Init File Updated:\n', fileTestDef)
else:
    json.dumps(testDef, indent = 4, ensure_ascii=False)
    print('\nInit file NOT updated\n\n')

#%% Looking at plots to get wind circ

time_s = oData['time_s']

latGps_deg = oData['rGps_D_ddm'][0]
lonGps_deg = oData['rGps_D_ddm'][1]
latB_deg = oData['rB_D_ddm'][0]
lonB_deg = oData['rB_D_ddm'][1]
fig2, ax2 = plt.subplots()
ax2.plot(lonGps_deg, latGps_deg, '.', label='GPS')
#ax2.plot(lonB_deg, latB_deg, label='Ekf')
ax2.grid()
ax2.axis('equal')
ax2.set_xlabel('Longitude (deg)')
ax2.set_ylabel('Latitude (deg)')
ax2.set_title('Latitude and Longitude')
ax2.legend()

plt.show()

#%% Find turns using excitations









#%% Wind/Air Cal
windSeg = ('time_s', [524, 530])
oDataWind = OpenData.Segment(oData, windSeg)

#OpenData.PlotOverview(oDataWind)

# #Convert GPS and Nav solution position to L frame
# Change Nav Solution to L frame
rB_L_m = KinematicTransforms.D2L(oDataWind['rB_D_ddm'][:, 0], oDataWind['rB_D_ddm'], degrees = True)

# Change GPS Solution to L frame
rGps_L_m = KinematicTransforms.D2L(oDataWind['rGps_D_ddm'][:, 0], oDataWind['rGps_D_ddm'], degrees = True)

plt.figure()
plt.plot(rB_L_m[1], rB_L_m[0], '-', rGps_L_m[1], rGps_L_m[0], '.')
plt.figure()
plt.plot(oDataWind['time_s'], rB_L_m[2], '-', oDataWind['time_s'], rGps_L_m[2], '.')


#%%
def Airspeed2NED(v_PA_P_mps, s_BL_rad, param):

    from scipy.spatial.transform import Rotation as R
        
    # Assume the rotation rate of the atmosphere is negligible
    w_AL_L_rps = np.zeros_like(v_PA_P_mps)
    
    # Compute the Rotation rate of the Probe wrt the Atm
    # w_BA_B_rps = w_BL_B_rps + T_L2B * w_AL_L_rps
    # w_BA_B_rps = w_BL_B_rps + w_AL_L_rps # FIXIT - should have transformation from L to B
    w_BA_B_rps = np.zeros_like(v_PA_P_mps)
    
    # Translate and Rotate the Pitot measurement to the Body frame
    v_BA_B_mps = KinematicTransforms.TransPitot(v_PA_P_mps, w_BA_B_rps, param['s_B_rad'], param['r_B_m']);
    
    # Transform Coordinates from B to L
    v_BA_L_mps = np.zeros_like(v_BA_B_mps)
    numSamp = v_PA_P_mps.shape[-1]
    for iSamp in range(0, numSamp):
#        T_B2L = R.from_euler('XYZ', -s_BL_rad[:,iSamp], degrees = False).as_dcm().T
        T_B2L = R.from_euler('ZYX', s_BL_rad[[2,1,0], iSamp], degrees = False).as_dcm()
    
        # Compute the NED velocity
        v_BA_L_mps[:,iSamp] = T_B2L @ v_BA_B_mps[:,iSamp]

    return v_BA_B_mps, v_BA_L_mps



#%%
## Pre-Optimization, Initial Guess for the Wind
# Over-ride Default Error Model, Optional
pData = {}
pData['Pitot'] = {}
pData['Pitot']['r_B_m'] = np.array([1.0, 0.0, 0.0])
pData['Pitot']['s_B_rad'] = np.array([0.0, 0.0, 0.0]) * 180.0/np.pi

pData['Pitot']['pTip'] = {}
pData['Pitot']['pTip']['errorType'] = 'ScaleBias+'
pData['Pitot']['pTip']['bias'] = 0
pData['Pitot']['pTip']['K'] = 1.0

pData['Pitot']['pStatic'] = pData['Pitot']['pTip'].copy()


calib = AirData.AirDataCal(oDataWind, pData['Pitot'])

v_BA_B_mps, v_BA_L_mps = Airspeed2NED(calib['v_PA_P_mps'], oDataWind['sB_L_rad'], pData['Pitot'])

# Subtract the Estimated Body Airspeed from the Inertial Velocity
#v_AE_L = v_BE_L - v_BA_L
v_AE_L_mps = oDataWind['vB_L_mps'] - v_BA_L_mps

# Compute the Mean of the Wind Estimate, in NED
vMean_AE_L_mps = np.mean(v_AE_L_mps, axis=1)


plt.subplot(3,1,1)
plt.plot(oDataWind['time_s'], oDataWind['vB_L_mps'][0])
plt.plot(oDataWind['time_s'], v_BA_L_mps[0])
plt.subplot(3,1,2)
plt.plot(oDataWind['time_s'], oDataWind['vB_L_mps'][1])
plt.plot(oDataWind['time_s'], v_BA_L_mps[1])
plt.subplot(3,1,3)
plt.plot(oDataWind['time_s'], oDataWind['vB_L_mps'][2])
plt.plot(oDataWind['time_s'], v_BA_L_mps[2])

#plt.plot(oDataWind['time_s'], v_AE_L_mps.T)

    

#%% Optimize
from scipy.optimize import minimize


# Define the Cost Function for the Optimization

def AirspeedCostFunc(xOpt, oData, param):
    
    # Unpack the free parameter vector
    vMean_AE_L_mps = xOpt[0:3]
    param['pTip']['K'] = xOpt[3]
    param['pTip']['bias'] = xOpt[4]
    
    
    # Compute the Airspeed solution with error model parameters
    calib = AirData.AirDataCal(oData, param)
    v_BA_B_mps, v_BA_L_mps = Airspeed2NED(calib['v_PA_P_mps'], oData['sB_L_rad'], param)

    # Compute the Ground Speeds
    # Wind Estimate, assume constant at mean value
    numSamp = v_BA_B_mps.shape[-1]
    v_AE_L_mps = np.repeat([vMean_AE_L_mps], numSamp, axis=0).T
    
    # Compute the Groudspeeds from the Corrected Airspeeds using the Wind Estimate
    vEst_BE_L_mps = v_AE_L_mps + v_BA_L_mps

    # Cost for each segment
    cost = np.linalg.norm(oData['vB_L_mps'] - vEst_BE_L_mps, 2) / numSamp;
    
    return cost

def xPrint(x):
    
    print(x)


# Compute the optimal parameters
optMethod = 'BFGS'
#optOptions = {'maxiter': 100, 'disp': True}
optOptions = {'maxiter': 400, 'disp': True}
#optOptions = {'disp': True}

# Setup the parameter vector
xOpt = vMean_AE_L_mps.tolist()
xOpt.append(pData['Pitot']['pTip']['K'])
xOpt.append(pData['Pitot']['pTip']['bias'])

# Test simple call to CostFunction
#cost = AirspeedCostFunc(xOpt, oDataWind, pData['5Hole'])

optResult = minimize(AirspeedCostFunc, xOpt, args = (oDataWind, pData['Pitot']), method = optMethod, options = optOptions, callback = xPrint)
vOpt_AE_L_mps = np.copy(optResult.x[0:3])
K = np.copy(optResult.x[3])
bias = np.copy(optResult.x[4])

plt.subplot(3,1,1)
plt.plot(oDataWind['time_s'], oDataWind['vB_L_mps'][0])
plt.plot(oDataWind['time_s'], v_BA_L_mps[0] + vOpt_AE_L_mps[0])
plt.subplot(3,1,2)
plt.plot(oDataWind['time_s'], oDataWind['vB_L_mps'][1])
plt.plot(oDataWind['time_s'], v_BA_L_mps[1] + vOpt_AE_L_mps[1])
plt.subplot(3,1,3)
plt.plot(oDataWind['time_s'], oDataWind['vB_L_mps'][2])
plt.plot(oDataWind['time_s'], v_BA_L_mps[2] + vOpt_AE_L_mps[2])



#%% Save _init.json file
# Open init json file
json_init = {}
#json_init = JsonRead(fileTestDef)

# Open flight json file
flightjson = Loader.JsonRead(fileSysConfig)
testPointList = flightjson['Mission-Manager']['Test-Points']

json_init['Test-Points'] = OpenData.TestPointOut(excList, testPointList)

if False:
    Loader.JsonWrite(fileTestDef, json_init)
    print('Init File Updated:\n', fileTestDef)
else:
    json.dumps(json_init, indent = 4, ensure_ascii=False)
    print('\nInit file NOT updated\n\n')
