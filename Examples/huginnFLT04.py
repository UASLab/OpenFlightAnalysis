"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Analysis for Huginn (mAEWing2) FLT 03
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

pathFile = '/home/rega0051/FlightArchive/Huginn/HuginnFLT04/'
fileLog = pathFile + 'HuginnFLT04.h5'
fileTestDef = pathFile + 'huginn_def.json'
fileSysConfig = pathFile + 'huginn.json'

#%%
# Read in raw h5 data into dictionary and data
oData, h5Data = Loader.Log_RAPTRS(fileLog, fileSysConfig)

# Plot Overview of flight
#oData = OpenData.Segment(oData, ('time_s', [725, 1205]))
#OpenData.PlotOverview(oData)


#%% Find Excitation Times
excList = OpenData.FindExcite(oData)

print('\n\nFlight Excitation Times:\n')
for iExc in range(0, len(excList)):
    print('Excitiation: ', excList[iExc][0], ', Time: [', excList[iExc][1][0], ',', excList[iExc][1][1], ']')

segList = [('time_us', excList[0][1])]

oDataExc = OpenData.Segment(oData, segList)
#OpenData.PlotOverview(oDataExc[0])


#%% Wind/Air Cal
windSegList = [
        ('time_us', [825000000, excList[0][1][0]]),
        ('time_us', [excList[1][1][1], excList[2][1][0]]),
        ('time_us', [excList[3][1][1], excList[4][1][0]]),
        ('time_us', [excList[5][1][1], excList[6][1][0]]),
        ('time_us', [excList[9][1][0], 1188000000])
        ]


oDataWindList = OpenData.Segment(oData, windSegList)


fig, ax = plt.subplots(nrows=2)
for oDataWind in oDataWindList:
#    OpenData.PlotOverview(oDataWind)
    latGps_deg = oDataWind['rGps_D_ddm'][0]
    lonGps_deg = oDataWind['rGps_D_ddm'][1]
    latB_deg = oDataWind['rB_D_ddm'][0]
    lonB_deg = oDataWind['rB_D_ddm'][1]
    ax[0].plot(lonGps_deg, latGps_deg, '.', label='GPS')
#    ax[0].plot(lonB_deg, latB_deg, label='Ekf')
    ax[0].grid()
    ax[1].plot(oDataWind['time_s'], oDataWind['vIas_mps'])
    ax[1].grid()


#%%
## Pre-Optimization, Initial Guess for the Wind
# Over-ride Default Error Model, Optional
pData = {}
pData['5Hole'] = {}
pData['5Hole']['r_B_m'] = np.array([1.0, 0.0, 0.0])
pData['5Hole']['s_B_rad'] = np.array([0.0, 0.0, 0.0]) * 180.0/np.pi

pData['5Hole']['pTip'] = {}
pData['5Hole']['pTip']['errorType'] = 'ScaleBias+'
pData['5Hole']['pTip']['K'] = 1.0
pData['5Hole']['pTip']['bias'] = 0.0

pData['5Hole']['pStatic'] = pData['5Hole']['pTip'].copy()
pData['5Hole']['pAlpha1'] = pData['5Hole']['pTip'].copy()
pData['5Hole']['pAlpha2'] = pData['5Hole']['pAlpha1'].copy()
pData['5Hole']['pBeta1'] = pData['5Hole']['pAlpha1'].copy()
pData['5Hole']['pBeta2'] = pData['5Hole']['pBeta1'].copy()

pData['5Hole']['alphaCal'] = 4.8071159
pData['5Hole']['betaCal'] = 4.8071159

#
pData['5Hole']['pTip']['K'] = 0.85

#%% Optimize
from Core import AirData
from Core import AirDataCalibration

oDataList = oDataWindList

# Compute the optimal parameters
#opt = {'Method': 'BFGS', 'Options': {'maxiter': 10, 'disp': True}}
opt = {'Method': 'L-BFGS-B', 'Options': {'maxfun': 400, 'disp': True}}

opt['wind'] = []
for seg in oDataList:
    seg['vMean_AE_L_mps'] = np.asarray([-5, 0, 0])
    opt['wind'].append({'val': seg['vMean_AE_L_mps'], 'lb': np.asarray([-10, -10, -3]), 'ub': np.asarray([10, 10, 3])})

opt['param'] = []
opt['param'].append({'val': pData['5Hole']['pTip']['K'], 'lb': 0.5, 'ub': 2})
opt['param'].append({'val': pData['5Hole']['pTip']['bias'], 'lb': -20, 'ub': 20})


#AirDataCalibration.CostFunc(xOpt, optInfo, oDataList, param)
opt['Result'] = AirDataCalibration.EstCalib(opt, oDataList, pData['5Hole'])

nSegs = len(oDataWindList)
nWinds = nSegs * 3
vWind = opt['Result']['x'][0:nWinds].reshape((nSegs, 3))


#%% Plot the Solution
# Apply the calibration to the whole flight
calib = AirData.ApplyCalibration(oData, pData['5Hole'])
oData.update(calib)

# Re-segment the Wind-circle after applying calibration
oDataWindList = OpenData.Segment(oData, windSegList)


for iSeg, oDataWind in enumerate(oDataWindList):
    v_BA_B_mps, v_BA_L_mps = AirData.Airspeed2NED(oDataWind['v_PA_P_mps'], oDataWind['sB_L_rad'], pData['5Hole'])

    oDataWind['vMean_AE_L_mps'] = vWind[iSeg]

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
    
    v_AE_L_mps = np.repeat([oDataWind['vMean_AE_L_mps']], oDataWind['vB_L_mps'].shape[-1], axis=0).T
    vError_mps = (v_BA_L_mps + v_AE_L_mps) - oDataWind['vB_L_mps']
    
    vErrorMag_mps = np.linalg.norm(vError_mps, axis=0)
    vBMag_mps = np.linalg.norm(oDataWind['vB_L_mps'], axis=0)
    
    plt.figure(11)
    plt.plot(vBMag_mps, vErrorMag_mps, '.')
    
    
    print('Wind (m/s): ', oDataWind['vMean_AE_L_mps'])
    
print('Tip Gain: ', pData['5Hole']['pTip']['K'])
print('Tip Bias: ', pData['5Hole']['pTip']['bias'])



#%% Analyze Glide
if False:
    rad2deg = 180.0 / np.pi
    glideSeg = ('time_s', [950, 975])
    oDataGlide = OpenData.Segment(oData, glideSeg)

    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(oDataGlide['time_s'], oDataGlide['altBaro_m'])
    plt.subplot(3,1,2)
    plt.plot(oDataGlide['time_s'], oDataGlide['vIas_mps'])
    plt.plot(oDataGlide['time_s'], oDataGlide['Effectors']['cmdMotor_nd'])
    plt.subplot(3,1,3)
    plt.plot(oDataGlide['time_s'], oDataGlide['refTheta_rad'] * rad2deg)
    plt.plot(oDataGlide['time_s'], oDataGlide['sB_L_rad'][1] * rad2deg)
    plt.plot(oDataGlide['time_s'], oDataGlide['alpha_rad'] * rad2deg)


if False:
    glideSeg = ('time_s', [1005, 1032])
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



#%%
from Core import FreqTrans

rtsmSeg = ('time_us', [excList[1][1][0], excList[1][1][0] + 12e6]) # 23 m/s
#rtsmSeg = ('time_us', [excList[7][1][0], excList[7][1][0] + 12e6]) # 20 m/s

oDataRtsm = OpenData.Segment(oData, rtsmSeg)

#plt.plot(oDataRtsm['time_s'], oDataRtsm['Excitation']['cmdRoll_rps'])
#plt.plot(oDataRtsm['time_s'], oDataRtsm['Excitation']['cmdPitch_rps'])
#plt.plot(oDataRtsm['time_s'], oDataRtsm['Excitation']['cmdBend_nd'])

freqExc_rps = np.linspace(0.1, 10.0, 51) * hz2rps
freqRate_rps = 50 * hz2rps
optSpec = FreqTrans.OptSpect(dftType = 'czt', freq = freqExc_rps, freqRate = freqRate_rps, smooth = ('box', 1), detrendType = 'Linear')

t = oDataRtsm['time_s']
x = oDataRtsm['Excitation']['cmdBend_nd']

#y = oDataRtsm['wB_I_rps'][1]
#y = oDataRtsm['wLeftAftIMU_IMU_rps'][1] - oDataRtsm['wB_I_rps'][1]
y = oDataRtsm['aLeftFwdIMU_IMU_mps2'][2] + oDataRtsm['aLeftAftIMU_IMU_mps2'][2] + oDataRtsm['aRightFwdIMU_IMU_mps2'][2] + oDataRtsm['aRightAftIMU_IMU_mps2'][2] - 2 * (oDataRtsm['aCenterFwdIMU_IMU_mps2'][2] + oDataRtsm['aCenterAftIMU_IMU_mps2'][2])


# Number of time segments and length of overlap, units of samples
#lenSeg = 2**6 - 1
lenSeg = int(1 * optSpec.freqRate * rps2hz)
lenOverlap = 1

# Compute Spectrum over time
tSpec_s, freqSpec_rps, P_mag = FreqTrans.SpectTime(t, y, lenSeg, lenOverlap, optSpec)
freqSpec_hz = freqSpec_rps * rps2hz
P_dB = 20 * np.log10(P_mag)
    
# Plot the Spectrogram
FreqTrans.Spectogram(freqSpec_hz, tSpec_s, P_dB)

#%%
optSpec.freq = freqExc_rps[2::3]
freq, Txy, Cxy, Pxx, Pyy, Pxy = FreqTrans.FreqRespFuncEst(x, y, optSpec)
gain_dB, phase_deg = FreqTrans.GainPhase(Txy)

freq_hz = freq / (2*np.pi)

plt.figure()
plt.subplot(3,1,1)
plt.semilogx(freq_hz.T, gain_dB.T)
plt.grid()
plt.subplot(3,1,2)
plt.semilogx(freq_hz.T, phase_deg.T)
plt.grid()
plt.subplot(3,1,3)
plt.semilogx(freq_hz.T, Cxy.T)
plt.grid()


#%% Save _init.json file
# Open init json file
json_init = {}
#json_init = JsonRead(fileTestDef)

# Open flight json file
flightjson = Loader.JsonRead(fileSysConfig)
testPointList = flightjson['Mission-Manager']['Test-Points']

json_init['Test-Points'] = OpenData.TestPointOut(excList, testPointList)

if True:
    Loader.JsonWrite(fileTestDef, json_init)
    print('Init File Updated:\n', fileTestDef)
else:
    import json
    json.dumps(json_init, indent = 4, ensure_ascii=False)
    print('\nInit file NOT updated\n\n')
