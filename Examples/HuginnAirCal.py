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

#%% File Lists
import os.path as path

pathBase = path.join('/home', 'rega0051', 'FlightArchive', 'Huginn')

fileList = {}
flt = 'FLT03'
fileList[flt] = {}
fileList[flt]['log'] = path.join(pathBase, 'Huginn' + flt, 'Huginn' + flt + '.h5')
fileList[flt]['config'] = path.join(pathBase, 'Huginn' + flt, 'huginn.json')
fileList[flt]['def'] = path.join(pathBase, 'Huginn' + flt, 'huginn_def.json')

flt = 'FLT04'
fileList[flt] = {}
fileList[flt]['log'] = path.join(pathBase, 'Huginn' + flt, 'Huginn' + flt + '.h5')
fileList[flt]['config'] = path.join(pathBase, 'Huginn' + flt, 'huginn.json')
fileList[flt]['def'] = path.join(pathBase, 'Huginn' + flt, 'huginn_def.json')


#%% Wind/Air Cal
windSegList = [
        {'flt': 'FLT03', 'seg': ('time_us', [410000000, 515000000])},
        {'flt': 'FLT03', 'seg': ('time_us', [610000000, 645277887])},
        {'flt': 'FLT03', 'seg': ('time_us', [669278251, 693458513])},
        {'flt': 'FLT03', 'seg': ('time_us', [722078703, 741298701])},
        {'flt': 'FLT03', 'seg': ('time_us', [757878621, 775518514])},
        {'flt': 'FLT03', 'seg': ('time_us', [803818324, 828338096])},
        {'flt': 'FLT03', 'seg': ('time_us', [839997973, 865877709])},
        {'flt': 'FLT03', 'seg': ('time_us', [893797477, 914157292])},
        {'flt': 'FLT03', 'seg': ('time_us', [999000000, 1032000000])},
        
        {'flt': 'FLT04', 'seg': ('time_us', [825000000, 872601981])},
        {'flt': 'FLT04', 'seg': ('time_us', [896345033, 914627038])},
        {'flt': 'FLT04', 'seg': ('time_us', [940469572, 957651007])},
        {'flt': 'FLT04', 'seg': ('time_us', [981512668, 998733748])},
        {'flt': 'FLT04', 'seg': ('time_us', [1069557816, 1188000000])}
        ]

oDataWindList = []
for windSeg in windSegList:
    fltNum = windSeg['flt']
    
    fileLog = fileList[fltNum]['log']
    fileConfig = fileList[fltNum]['config']
    
    oData, h5Data = Loader.Log_RAPTRS(fileLog, fileConfig)
    oDataWindList.append(OpenData.Segment(oData, windSeg['seg']))


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


#%% Optimize
from Core import AirData
from Core import AirDataCalibration

oDataList = oDataWindList

# Compute the optimal parameters
#opt = {'Method': 'BFGS', 'Options': {'maxiter': 10, 'disp': True}}
opt = {'Method': 'L-BFGS-B', 'Options': {'maxfun': 100, 'disp': True}}

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
    
    plt.figure(111)
    plt.plot(vBMag_mps, vErrorMag_mps, '.')
    
    
    print('Wind (m/s): ', oDataWind['vMean_AE_L_mps'])
    
print('Tip Gain: ', pData['5Hole']['pTip']['K'])
print('Tip Bias: ', pData['5Hole']['pTip']['bias'])

