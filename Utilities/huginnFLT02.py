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


fileLog = '/home/rega0051/FlightArchive/Huginn/HuginnFLT02/HuginnFLT02.h5'
fileTestDef = '/home/rega0051/FlightArchive/Huginn/HuginnFLT02/huginn_def.json'
fileSysConfig = '/home/rega0051/FlightArchive/Huginn/HuginnFLT02/huginn.json'

#%%
# Read in raw h5 data into dictionary and data
oData, h5Data = Loader.Log_RAPTRS(fileLog, fileSysConfig)

# Plot Overview of flight
oData = OpenData.Segment(oData, ('time_s', [445, 695]))
OpenData.PlotOverview(oData)

#%% Find Excitation Times
excList = OpenData.FindExcite(oData)

print('\n\nFlight Excitation Times:\n')
for iExc in range(0, len(excList)):
    print('Excitiation: ', excList[iExc][0], ', Time: [', excList[iExc][1][0], ',', excList[iExc][1][1], ']')

segList = [('time_us', excList[0][1])]

oDataExc = OpenData.Segment(oData, segList)
#OpenData.PlotOverview(oDataExc[0])


#%% Battery calibration
time_s = oData['time_s']
pwrPropLeft_W = oData['pwrPropLeft_A'] * oData['pwrPropLeft_V']
pwrPropRight_W = oData['pwrPropRight_A'] * oData['pwrPropRight_V']

# Voltage
fig0, ax0 = plt.subplots()
ax0.plot(time_s, pwrPropLeft_W, time_s, pwrPropRight_W)
ax0.set_xlabel('Time (s)')
ax0.set_ylabel('Propulsion Power (W)')
ax0.set_title('Power')


pwrPropLeft_mAh = 1000.0 / 3600.0 * np.trapz(oData['pwrPropLeft_A'], time_s)
pwrPropRight_mAh = 1000.0 / 3600.0 * np.trapz(oData['pwrPropRight_A'], time_s)

pwrPropActual_mAh = 1393.0 + 1362.0 # From re-charging batteries

scale = pwrPropActual_mAh / (pwrPropLeft_mAh + pwrPropRight_mAh) 
scale * 54.6448 # Revised calibration Value

#%%
fig, ax1 = plt.subplots()

color = 'tab:red'
ax1.set_xlabel('time (s)')
ax1.set_ylabel('Airspeed (m/s)', color=color)
ax1.plot(time_s, oData['vIas_mps'], color=color)
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
ax2.set_ylabel('Altitude (m)', color=color)  # we already handled the x-label with ax1
ax2.plot(time_s, oData['altBaro_m'], color=color)
ax2.tick_params(axis='y', labelcolor=color)

fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.show()

#%% Analyze Glide
glideSeg = ('time_s', [676, 686])
oDataGlide = OpenData.Segment(oData, glideSeg)

from scipy import signal
b, a = signal.butter(2, 0.25/50)
altBaro_m = signal.filtfilt(b, a, oDataGlide['altBaro_m'])
vIas_mps = signal.filtfilt(b, a, oDataGlide['vIas_mps'])

time_s = oDataGlide['time_s'][50:-75]
altBaro_m = altBaro_m[50:-75]
vIas_mps = vIas_mps[50:-75]

plt.figure()
plt.plot(oDataGlide['time_s'], oDataGlide['altBaro_m'], time_s, altBaro_m)
plt.plot(oDataGlide['time_s'], oDataGlide['vIas_mps'], time_s, vIas_mps)

dIas_m = np.trapz(vIas_mps, time_s)

temp_deg = np.arcsin(altBaro_m / dIas_m) * 180.0 / np.pi

LD = 1/np.tan(temp_deg * np.pi / 180.0)

plt.figure()
plt.plot(vIas_mps, LD, '.')
plt.xlabel('Airspeed (m/s)')
plt.ylabel('Lift / Drag')
plt.grid()


#%% Wind/Air Cal
windSeg = ('time_s', [505, 580])
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
pData['5Hole'] = {}
pData['5Hole']['r_B_m'] = np.array([1.0, 0.0, 0.0])
pData['5Hole']['s_B_rad'] = np.array([0.0, 0.0, 0.0]) * 180.0/np.pi

pData['5Hole']['pTip'] = {}
pData['5Hole']['pTip']['errorType'] = 'ScaleBias+'
pData['5Hole']['pTip']['bias'] = 0
pData['5Hole']['pTip']['K'] = 1.0

pData['5Hole']['pStatic'] = pData['5Hole']['pTip'].copy()
pData['5Hole']['pAlpha1'] = pData['5Hole']['pTip'].copy()
pData['5Hole']['pAlpha2'] = pData['5Hole']['pAlpha1'].copy()
pData['5Hole']['pBeta1'] = pData['5Hole']['pAlpha1'].copy()
pData['5Hole']['pBeta2'] = pData['5Hole']['pBeta1'].copy()

pData['5Hole']['alphaCal'] = 4.8071159
pData['5Hole']['betaCal'] = 4.8071159


calib = AirData.AirDataCal(oDataWind, pData['5Hole'])

v_BA_B_mps, v_BA_L_mps = Airspeed2NED(calib['v_PA_P_mps'], oDataWind['sB_L_rad'], pData['5Hole'])

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
xOpt.append(pData['5Hole']['pTip']['K'])
xOpt.append(pData['5Hole']['pTip']['bias'])

# Test simple call to CostFunction
#cost = AirspeedCostFunc(xOpt, oDataWind, pData['5Hole'])

optResult = minimize(AirspeedCostFunc, xOpt, args = (oDataWind, pData['5Hole']), method = optMethod, options = optOptions, callback = xPrint)
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
