"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Analysis for Huginn (mAEWing2)
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
flt = 'FLT02'
fileList[flt] = {}
fileList[flt]['log'] = path.join(pathBase, 'Huginn' + flt, 'Huginn' + flt + '.h5')
fileList[flt]['config'] = path.join(pathBase, 'Huginn' + flt, 'huginn.json')
fileList[flt]['def'] = path.join(pathBase, 'Huginn' + flt, 'huginn_def.json')

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
landSegList = [
#        {'flt': 'FLT03', 'seg': ('time_us', [950000000 - 1000.0e3, 970000000 - 1000.0e3], 'FLT03 - Approach')},
        {'flt': 'FLT03', 'seg': ('time_us', [1016000000, 1036000000], 'FLT03')},
        {'flt': 'FLT04', 'seg': ('time_us', [1180000000 + 20.0e3, 1200000000 + 20.0e3], 'FLT04')},
        {'flt': 'FLT05', 'seg': ('time_us', [953000000, 973000000], 'FLT05')},
        {'flt': 'FLT06', 'seg': ('time_us', [1248000000 - 60e3, 1268000000 - 60e3], 'FLT06')},
        ]


oDataLandList = []
for landSeg in landSegList:
    fltNum = landSeg['flt']

    fileLog = fileList[fltNum]['log']
    fileConfig = fileList[fltNum]['config']

    oData, h5Data = Loader.Log_RAPTRS(fileLog, fileConfig)
    oDataLandList.append(OpenData.Segment(oData, landSeg['seg']))


#%%
fig, ax = plt.subplots(nrows=5, sharex=True)
for oDataLand in oDataLandList:

    latGps_deg = oDataLand['rGps_D_ddm'][0]
    lonGps_deg = oDataLand['rGps_D_ddm'][1]
    latB_deg = oDataLand['rB_D_ddm'][0]
    lonB_deg = oDataLand['rB_D_ddm'][1]
#    ax[0].plot(lonGps_deg, latGps_deg, '.', label='GPS')
#    ax[0].plot(lonB_deg, latB_deg, label='Ekf')
    ax[0].plot(oDataLand['time_s'] - oDataLand['time_s'][0], oDataLand['altBaro_m'], label = oDataLand['Desc'])
    ax[0].set_ylabel('Altitude (m)')
    ax[0].grid(True)
    
    ax[1].plot(oDataLand['time_s'] - oDataLand['time_s'][0], oDataLand['vIas_mps'], label = oDataLand['Desc'])
    ax[1].plot(oDataLand['time_s'] - oDataLand['time_s'][0], 15.7 * np.ones_like(oDataLand['vIas_mps']), 'r:', label = oDataLand['Desc'])
    ax[1].set_ylabel('Airspeed (m/s)')
    ax[1].grid(True)
    
    ax[2].plot(oDataLand['time_s'] - oDataLand['time_s'][0], oDataLand['Effectors']['cmdMotor_nd'], label = oDataLand['Desc'])
    ax[2].set_ylabel('Throttle (nd)')
    ax[2].grid(True)

    ax[3].plot(oDataLand['time_s'] - oDataLand['time_s'][0], oDataLand['refTheta_rad'] * 180.0/np.pi, label = oDataLand['Desc'])
    ax[3].set_ylabel('Theta Cmd (deg)')
    ax[3].grid(True)
    
    ax[4].plot(oDataLand['time_s'] - oDataLand['time_s'][0], oDataLand['sB_L_rad'][1] * 180.0/np.pi, label = oDataLand['Desc'])
    ax[4].set_ylabel('Theta Meas (deg)')
    ax[4].grid(True)
    ax[4].legend(loc = 'center left')
