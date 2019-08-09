"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Analysis for Huginn (mAEWing2) FLT03 and FLT04
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
#pathBase = path.join('G:', 'Shared drives', 'UAVLab', 'Flight Data', 'Huginn')
#pathBase = path.join('D:/', 'Huginn')

fileList = {}
flt = 'FLT05'
fileList[flt] = {}
fileList[flt]['log'] = path.join(pathBase, 'Huginn' + flt, 'Huginn' + flt + '.h5')
fileList[flt]['config'] = path.join(pathBase, 'Huginn' + flt, 'huginn.json')
fileList[flt]['def'] = path.join(pathBase, 'Huginn' + flt, 'huginn_def.json')

flt = 'FLT06'
fileList[flt] = {}
fileList[flt]['log'] = path.join(pathBase, 'Huginn' + flt, 'Huginn' + flt + '.h5')
fileList[flt]['config'] = path.join(pathBase, 'Huginn' + flt, 'huginn.json')
fileList[flt]['def'] = path.join(pathBase, 'Huginn' + flt, 'huginn_def.json')


#%% Wind/Air Cal
windSegList = [
        {'flt': 'FLT05', 'seg': ('time_us', [566483686, 582408497])},
        {'flt': 'FLT05', 'seg': ('time_us', [602534178, 622279236])},
        {'flt': 'FLT05', 'seg': ('time_us', [637362791, 654286351])},
        {'flt': 'FLT05', 'seg': ('time_us', [666668777, 687832534])},
        {'flt': 'FLT05', 'seg': ('time_us', [703115100, 766364351])}, # Long!!
        {'flt': 'FLT05', 'seg': ('time_us', [788467105, 799488311])},
        {'flt': 'FLT05', 'seg': ('time_us', [811669552, 831211361])},
        {'flt': 'FLT05', 'seg': ('time_us', [844412511, 861513899])},
        {'flt': 'FLT05', 'seg': ('time_us', [873694795, 887575754])},
        {'flt': 'FLT05', 'seg': ('time_us', [899096534, 909897237])},
        {'flt': 'FLT05', 'seg': ('time_us', [927000000, 950000000])}, # Landing Approach
        {'flt': 'FLT06', 'seg': ('time_us', [940358346, 955822061])},
        {'flt': 'FLT06', 'seg': ('time_us', [982747328, 1000069848])},
        {'flt': 'FLT06', 'seg': ('time_us', [1010491142, 1026492809])},
        {'flt': 'FLT06', 'seg': ('time_us', [1036733749, 1054855133])},
        {'flt': 'FLT06', 'seg': ('time_us', [1065295790, 1087597269])}, # Slowing Turn
        {'flt': 'FLT06', 'seg': ('time_us', [1103958408, 1122539650])},
        {'flt': 'FLT06', 'seg': ('time_us', [1140000000, 1165401057])},
        {'flt': 'FLT06', 'seg': ('time_us', [1165401057, 1189143263])},
        {'flt': 'FLT06', 'seg': ('time_us', [1189143263, 1225000000])}, # Landing Approach
        {'flt': 'FLT06', 'seg': ('time_us', [1225000000, 1260000000])}, # Landing Approach

        
        ]


oDataWindList = []
for windSeg in windSegList:
    fltNum = windSeg['flt']

    fileLog = fileList[fltNum]['log']
    fileConfig = fileList[fltNum]['config']

    oData, h5Data = Loader.Log_RAPTRS(fileLog, fileConfig)
    
    for key in h5Data['Sensor-Processing']['PostProcess']['INS'].keys():
        oData[key] = h5Data['Sensor-Processing']['PostProcess']['INS'][key]
    
    oData = OpenData.Decimate(oData, 10)
    oDataWindList.append(OpenData.Segment(oData, windSeg['seg']))


fig, ax = plt.subplots(nrows=2)
for oDataWind in oDataWindList:

    latGps_deg = oDataWind['rGps_D_ddm'][0]
    lonGps_deg = oDataWind['rGps_D_ddm'][1]
    latB_deg = oDataWind['rB_D_ddm'][0]
    lonB_deg = oDataWind['rB_D_ddm'][1]
    ax[0].plot(lonGps_deg, latGps_deg, '.', label='GPS')
    ax[0].plot(lonB_deg, latB_deg, label='Ekf')
    ax[0].grid()
    ax[1].plot(oDataWind['time_s'], oDataWind['vIas_mps'])
    ax[1].plot(oDataWind['time_s'], oDataWind['sB_L_rad'][0]*180.0/np.pi)
    ax[1].grid()


#%%
## Pre-Optimization, Initial Guess for the Wind
# Over-ride Default Error Model, Optional
pData = {}
pData['5Hole'] = {}
pData['5Hole']['r_B_m'] = np.array([1.0, 0.0, 0.0])
pData['5Hole']['s_B_rad'] = np.array([0.0, 0.0, 0.0]) * 180.0/np.pi

pData['5Hole']['v'] = {}
pData['5Hole']['v']['errorType'] = 'ScaleBias+'
pData['5Hole']['v']['K'] = 1.0
pData['5Hole']['v']['bias'] = 0.0

pData['5Hole']['alt'] = pData['5Hole']['v'].copy()
pData['5Hole']['alpha'] = pData['5Hole']['v'].copy()
pData['5Hole']['beta'] = pData['5Hole']['v'].copy()

pData['5Hole']['v']['K'] = 0.95

#%% Optimize
from Core import AirData
from Core import AirDataCalibration

rad2deg = 180.0 / np.pi
deg2rad = 1 / rad2deg

oDataList = oDataWindList

# Compute the optimal parameters
#opt = {'Method': 'BFGS', 'Options': {'disp': True}}
opt = {'Method': 'L-BFGS-B', 'Options': {'disp': True}}
#opt = {'Method': 'L-BFGS-B', 'Options': {'maxiter': 10, 'disp': True}}
#opt = {'Method': 'CG', 'Options': {'disp': True}}

#%% First Phase - Airspeed and Wind only
opt['wind'] = []
for seg in oDataList:
    seg['vMean_AE_L_mps'] = np.asarray([-2.0, 0.0, 0.0])
    opt['wind'].append({'val': seg['vMean_AE_L_mps'], 'lb': np.asarray([-10, -10, -3]), 'ub': np.asarray([10, 10, 3])})

opt['param'] = []
opt['param'].append({'val': pData['5Hole']['v']['K'], 'lb': 0.80, 'ub': 1.20})
opt['param'].append({'val': pData['5Hole']['v']['bias'], 'lb': -3.0, 'ub': 3.0})
opt['param'].append({'val': pData['5Hole']['alpha']['K'], 'lb': 1.00, 'ub': 1.00})
opt['param'].append({'val': pData['5Hole']['alpha']['bias'], 'lb': -0.0 * deg2rad, 'ub': 0.0 * deg2rad})
opt['param'].append({'val': pData['5Hole']['beta']['K'], 'lb': 1.00, 'ub': 1.00})
opt['param'].append({'val': pData['5Hole']['beta']['bias'], 'lb': -0.0 * deg2rad, 'ub': 0.0 * deg2rad})


#AirDataCalibration.CostFunc(xOpt, optInfo, oDataList, param)
opt['Result'] = AirDataCalibration.EstCalib(opt, oDataList, pData['5Hole'])

nSegs = len(oDataWindList)
nWinds = nSegs * 3
vWind = opt['Result']['x'][0:nWinds].reshape((nSegs, 3))

#%% Second Phase - add alpha and beta
if False:
    for iSeg, seg in enumerate(oDataList):
        seg['vMean_AE_L_mps'] = vWind[iSeg]
        opt['wind'][iSeg]['val'] = seg['vMean_AE_L_mps']
        opt['wind'][iSeg]['lb'] = seg['vMean_AE_L_mps'] - np.asarray([0.0, 0.0, 0.0])
        opt['wind'][iSeg]['ub'] = seg['vMean_AE_L_mps'] + np.asarray([0.0, 0.0, 0.0])

    opt['param'][0] = {'val': pData['5Hole']['v']['K'], 'lb': 1.0 * pData['5Hole']['v']['K'], 'ub': 1.0 * pData['5Hole']['v']['K']}
    opt['param'][1] = {'val': pData['5Hole']['v']['bias'], 'lb': pData['5Hole']['v']['bias']-0.0, 'ub': pData['5Hole']['v']['bias']+0.0}
    opt['param'][2] = {'val': pData['5Hole']['alpha']['K'], 'lb': 0.8, 'ub': 1.2}
    opt['param'][3] = {'val': pData['5Hole']['alpha']['bias'], 'lb': -4.0 * deg2rad, 'ub': 4.0 * deg2rad}
    opt['param'][4] = {'val': pData['5Hole']['beta']['K'], 'lb': 0.8, 'ub': 1.2}
    opt['param'][5] = {'val': pData['5Hole']['beta']['bias'], 'lb': -4.0 * deg2rad, 'ub': 4.0 * deg2rad}


    #AirDataCalibration.CostFunc(xOpt, optInfo, oDataList, param)
    opt['Result'] = AirDataCalibration.EstCalib(opt, oDataList, pData['5Hole'])

    nSegs = len(oDataWindList)
    nWinds = nSegs * 3
    vWind = opt['Result']['x'][0:nWinds].reshape((nSegs, 3))


#%% Plot the Solution
from SensorModel import SensorErrorModel

for iSeg, oDataWind in enumerate(oDataWindList):

    # Update the Flight data with the new calibrations
    calib = AirData.ApplyCalibration(oDataWind, pData['5Hole'])
    oDataWind.update(calib)

    v_BA_B_mps, v_BA_L_mps = AirData.Airspeed2NED(oDataWind['v_PA_P_mps'], oDataWind['sB_L_rad'], pData['5Hole'])

    oDataWind['vMean_AE_L_mps'] = vWind[iSeg]

    if True:
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
    vAirUncalMag_mps = oDataWind['vIas_mps']
    vAirMag_mps = np.linalg.norm((v_BA_L_mps + v_AE_L_mps), axis=0)
    vInertMag_mps = np.linalg.norm(oDataWind['vB_L_mps'], axis=0)

    plt.figure(0)
#    plt.plot(vAirMag_mps, vInertMag_mps - vAirMag_mps, '.')
    plt.plot(vAirMag_mps, vInertMag_mps, '.')
    plt.grid()

    print('Wind (m/s): ', oDataWind['vMean_AE_L_mps'])


plt.figure(0)
vA_mps = np.linspace(15, 35, 5)
vAcal_mps = SensorErrorModel(vA_mps, pData['5Hole']['v'])
plt.plot(vA_mps, vA_mps, 'k:')
#plt.plot(vA_mps, vA_mps - vG_mps, 'k:')

print('Velocity Gain: ', pData['5Hole']['v']['K'])
print('Velocity Bias: ', pData['5Hole']['v']['bias'])
print('Alpha Gain: ', pData['5Hole']['alpha']['K'])
print('Alpha Bias: ', pData['5Hole']['alpha']['bias'])
print('Beta Gain: ', pData['5Hole']['beta']['K'])
print('Beta Bias: ', pData['5Hole']['beta']['bias'])

