"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Analysis for Huginn (mAEWing2) FLT 02
"""

#%%
# Import Libraries
import numpy as np
import matplotlib.pyplot as plt

# Hack to allow loading the Core package
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), "..")))
    path.insert(0, abspath(join(dirname(argv[0]), "..", 'Core')))
    
    del path, argv, dirname, abspath, join

from Core import Loader
from Core import OpenData


# Constants
hz2rps = 2 * np.pi
rps2hz = 1 / hz2rps

pathFile = '/home/rega0051/FlightArchive/Huginn/HuginnFLT02/'
fileLog = pathFile + 'HuginnFLT02.h5'
fileTestDef = pathFile + 'huginn_def.json'
fileSysConfig = pathFile + 'huginn.json'

#%%
# Read in raw h5 data into dictionary and data
#oData, h5Data = Loader.Log_RAPTRS(fileLog, fileSysConfig)

h5Data = Loader.Load_h5(fileLog) # RAPTRS log data as hdf5

h5Data['Sensors']['uBlox']['DownVelocity_ms'] = h5Data['Sensor-Processing']['DownVelocity_ms'] # Fix for FLT02

for key, val in h5Data['Sensor-Processing']['PostProcess']['INS'].items(): # Fix for FLT02
    h5Data['Sensor-Processing'][key] = val # Fix for FLT02

sysConfig = Loader.JsonRead(fileSysConfig)
oData = Loader.OpenData_RAPTRS(h5Data, sysConfig)

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
if 0:
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


#%% View Launch
if True:
    launchSeg = ('time_s', [448, 458])
    oDataLaunch = OpenData.Segment(oData, launchSeg)
    
    plt.figure()
    plt.plot(oDataLaunch['time_s'], oDataLaunch['Effectors']['cmdMotor_nd'])
    plt.plot(oDataLaunch['time_s'], oDataLaunch['refTheta_rad'] * 180.0 / np.pi, oDataLaunch['time_s'], oDataLaunch['sB_L_rad'][1] * 180.0 / np.pi)
    plt.xlabel('Time (s)')
    plt.grid()
    
    plt.figure()
    plt.plot(oDataLaunch['time_s'], oDataLaunch['altBaro_m'])
    plt.plot(oDataLaunch['time_s'], oDataLaunch['vIas_mps'])
    plt.xlabel('Time (s)')
    plt.grid()

    plt.figure()
    plt.plot(oDataLaunch['time_s'], oDataLaunch['tempProbe_C'])
    plt.plot(oDataLaunch['time_s'], oDataLaunch['vIas_mps'])
    plt.xlabel('Time (s)')
    plt.grid()

#%% Analyze Glide
if 0:
    glideSeg = ('time_s', [678, 685])
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

OpenData.PlotOverview(oDataWind)


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

pData['5Hole']['pTip']['K'] = 0.85

pData['5Hole']['alphaCal'] = 4.8071159
pData['5Hole']['betaCal'] = 4.8071159


#calib = AirData.AirDataCal(oDataWind, pData['5Hole'])
#v_BA_B_mps, v_BA_L_mps = AirData.Airspeed2NED(calib['v_PA_P_mps'], oDataWind['sB_L_rad'], pData['5Hole'])
#
## Subtract the Estimated Body Airspeed from the Inertial Velocity
##v_AE_L = v_BE_L - v_BA_L
#oDataWind['v_AE_L_mps'] = oDataWind['vB_L_mps'] - v_BA_L_mps
#
## Compute the Mean of the Wind Estimate, in NED
#oData['vMean_AE_L_mps'] = np.mean(oDataWind['v_AE_L_mps'], axis=1)
#
## 
#numSamp = oDataWind['v_AE_L_mps'].shape[-1]
#oDataWind['v_AE_L_mps'] = np.repeat([oData['vMean_AE_L_mps']], numSamp, axis=0).T
#
#plt.subplot(3,1,1)
#plt.plot(oDataWind['time_s'], oDataWind['vB_L_mps'][0])
#plt.plot(oDataWind['time_s'], v_BA_L_mps[0])
#plt.subplot(3,1,2)
#plt.plot(oDataWind['time_s'], oDataWind['vB_L_mps'][1])
#plt.plot(oDataWind['time_s'], v_BA_L_mps[1])
#plt.subplot(3,1,3)
#plt.plot(oDataWind['time_s'], oDataWind['vB_L_mps'][2])
#plt.plot(oDataWind['time_s'], v_BA_L_mps[2])

#plt.plot(oDataWind['time_s'], oDataWind['v_AE_L_mps'].T)

#plt.plot(oDataWind['time_s'], oDataWind['alpha_deg'])
    

#%% Optimize
from Core import AirData
from Core import AirDataCalibration

oDataList = [oDataWind]

# Compute the optimal parameters
#opt = {'Method': 'BFGS', 'Options': {'maxiter': 400, 'disp': True}}
opt = {'Method': 'L-BFGS-B', 'Options': {'maxiter': 400, 'disp': True}}

opt['wind'] = []
for seg in oDataList:
    seg['vMean_AE_L_mps'] = np.asarray([0, 0, 0])
    opt['wind'].append({'val': seg['vMean_AE_L_mps'], 'lb': np.asarray([-10, -10, -3]), 'ub': np.asarray([10, 10, 3])})

opt['param'] = []
opt['param'].append({'val': pData['5Hole']['pTip']['K'], 'lb': 0, 'ub': 2})
opt['param'].append({'val': pData['5Hole']['pTip']['bias'], 'lb': -20, 'ub': 20})


#AirDataCalibration.CostFunc(xOpt, optInfo, oDataList, param)
opt['Result'] = AirDataCalibration.EstCalib(opt, oDataList, pData['5Hole'])


#%% Plot the Solution
# Apply the calibration to the whole flight
calib = AirData.ApplyCalibration(oData, pData['5Hole'])
oData.update(calib)

# Re-segment the Wind-circle after applying calibration
oDataWind = OpenData.Segment(oData, windSeg)

v_BA_B_mps, v_BA_L_mps = AirData.Airspeed2NED(oDataWind['v_PA_P_mps'], oDataWind['sB_L_rad'], pData['5Hole'])

oDataWind['vMean_AE_L_mps'] = opt['Result']['x'][0:3]

plt.figure()
plt.subplot(3,1,1)
plt.plot(oDataWind['time_s'], oDataWind['vB_L_mps'][0], label = 'Inertial')
plt.plot(oDataWind['time_s'], v_BA_L_mps[0] + oDataWind['vMean_AE_L_mps'][0], label = 'AirData + Wind')
plt.grid()
plt.legend()
plt.subplot(3,1,2)
plt.plot(oDataWind['time_s'], oDataWind['vB_L_mps'][1])
plt.plot(oDataWind['time_s'], v_BA_L_mps[1] + oDataWind['vMean_AE_L_mps'][1])
plt.grid()
plt.subplot(3,1,3)
plt.plot(oDataWind['time_s'], oDataWind['vB_L_mps'][2])
plt.plot(oDataWind['time_s'], v_BA_L_mps[2] + oDataWind['vMean_AE_L_mps'][2])
plt.grid()

print('Wind (m/s): ', oDataWind['vMean_AE_L_mps'])
print('Tip Gain: ', pData['5Hole']['pTip']['K'])
print('Tip Bias: ', pData['5Hole']['pTip']['bias'])

#%%
from Core import FreqTrans

#rtsmSeg = segList[0]
#rtsmSeg = ('time_us', [632103380, 644103380])
#rtsmSeg = ('time_us', [631103380, 646103380])
rtsmSeg = ('time_us', [632103380, 640103380]) # Const Airspeed

oDataRtsm = OpenData.Segment(oData, rtsmSeg)
oDataRtsm['time_s']

freqExc_rps = np.linspace(0.1, 10.0, 51) * hz2rps
freqRate_rps = 50 * hz2rps
optSpec = FreqTrans.OptSpect(dftType = 'czt', freq = freqExc_rps, freqRate = freqRate_rps, smooth = ('box', 1), detrendType = 'Constant')

t = oDataRtsm['time_s']
#x = oDataRtsm['Excitation']['cmdBend_nd']

#x = oDataRtsm['wB_I_rps'][1]
#x = oDataRtsm['wLeftAftIMU_IMU_rps'][1] - oDataRtsm['wB_I_rps'][1]
x = oDataRtsm['aLeftFwdIMU_IMU_mps2'][2] + oDataRtsm['aLeftAftIMU_IMU_mps2'][2] + oDataRtsm['aRightFwdIMU_IMU_mps2'][2] + oDataRtsm['aRightAftIMU_IMU_mps2'][2] - 2 * (oDataRtsm['aCenterFwdIMU_IMU_mps2'][2] + oDataRtsm['aCenterAftIMU_IMU_mps2'][2])


# Number of time segments and length of overlap, units of samples
#lenSeg = 2**6 - 1
lenSeg = int(1 * optSpec.freqRate * rps2hz)
lenOverlap = 1

# Compute Spectrum over time
tSpec_s, freqSpec_rps, P_mag = FreqTrans.SpectTime(t, x, lenSeg, lenOverlap, optSpec)
freqSpec_hz = freqSpec_rps * rps2hz
P_dB = 20 * np.log10(P_mag)
    
# Plot the Spectrogram
FreqTrans.Spectogram(freqSpec_hz, tSpec_s, P_dB)


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
    import json
    json.dumps(json_init, indent = 4, ensure_ascii=False)
    print('\nInit file NOT updated\n\n')
