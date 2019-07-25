#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 17 10:51:47 2019

@author: rega0051
"""

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
#flt = 'FLT03'
#fileList[flt] = {}
#fileList[flt]['log'] = path.join(pathBase, 'Huginn' + flt, 'Huginn' + flt + '.h5')
#fileList[flt]['config'] = path.join(pathBase, 'Huginn' + flt, 'huginn.json')
#fileList[flt]['def'] = path.join(pathBase, 'Huginn' + flt, 'huginn_def.json')
#
#flt = 'FLT04'
#fileList[flt] = {}
#fileList[flt]['log'] = path.join(pathBase, 'Huginn' + flt, 'Huginn' + flt + '.h5')
#fileList[flt]['config'] = path.join(pathBase, 'Huginn' + flt, 'huginn.json')
#fileList[flt]['def'] = path.join(pathBase, 'Huginn' + flt, 'huginn_def.json')

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


#%% Bending

ExcName = 'Bend'
segList = []
oDataList = []

for flt in fileList.keys():
    fileLog = fileList[flt]['log']
    fileConfig = fileList[flt]['config']
    fileDef = fileList[flt]['def']
    
    fltDef = Loader.JsonRead(fileDef)
    
    for testPt in fltDef['Test-Points']:
        if testPt['Excitation'] == ExcName:
    
            # Load Flight Log
            oData, h5Data = Loader.Log_RAPTRS(fileLog, fileConfig)
            t0 = testPt['time_us'][0] * 1e-6
            tf = t0 + (2*np.pi) + 2.0
            
            ODataSeg = OpenData.Segment(oData, ('time_s', [t0, tf]))
            oDataList.append(ODataSeg)
            
            seg = {'flt': flt, 'seg': ('time_us', testPt['time_us']), 'Desc': '{:.2f}'.format(ODataSeg['vIas_mps'].mean()) + ' m/s'}
            
            segList.append(seg)
            
            
# Add Description to each segment
#segList = 
#    [{'flt': 'FLT03', 'seg': ('time_us', [645277887, 669278251]), 'Desc': '23.54 m/s'},
#     {'flt': 'FLT03', 'seg': ('time_us', [828338096, 839997973]), 'Desc': '19.78 m/s'},
#     {'flt': 'FLT04', 'seg': ('time_us', [872601981, 882883356]), 'Desc': '25.76 m/s'},
#     {'flt': 'FLT05', 'seg': ('time_us', [551298711, 566483686]), 'Desc': '25.83 m/s'},
#     {'flt': 'FLT05', 'seg': ('time_us', [766364351, 788467105]), 'Desc': '28.64 m/s'},
#     {'flt': 'FLT06', 'seg': ('time_us', [929895366, 940358346]), 'Desc': '32.47 m/s'},
#     {'flt': 'FLT06', 'seg': ('time_us', [1087597269, 1103958408]), 'Desc': '23.50 m/s'}]


#%%
from scipy.signal import filtfilt, butter
b, a = butter(2, 0.1)

            
fig, ax = plt.subplots(nrows=6, sharex=True)
fig2, ax2 = plt.subplots(nrows=1, sharex=True)
for oData in oDataList:

    pwrBat_w = oData['pwrPropLeft_V'] * oData['pwrPropLeft_A'] + oData['pwrPropRight_V'] * oData['pwrPropRight_A']
    eBat_J = np.cumsum(pwrBat_w) * np.median(np.diff(oData['time_s']))

    pwrBatFilt_w = filtfilt(b, a, pwrBat_w)
    eBatFilt_J = filtfilt(b, a, eBat_J)
    
    vGrnd_mps = np.linalg.norm(oData['vB_L_mps'], axis=0)
    aGrnd_mps2 = filtfilt(b, a, np.linalg.norm(oData['aB_I_mps2'], axis=0))
    TE = 18 * (0.5 * vGrnd_mps**2 + oData['rB_D_ddm'][2])
    
    hDot_mps = oData['vB_L_mps'][2]
    dTE = 18 * (9.81 * hDot_mps + aGrnd_mps2 * vGrnd_mps)
    
    nu = 1.0
    

    D = (dTE + nu * pwrBatFilt_w) / oData['vIas_mps']
    
    
    ax[0].plot(oData['time_s'] - oData['time_s'][0], oData['Control']['cmdBend_nd'])
    ax[0].set_ylabel('Bend Cmd (nd)')
    ax[0].grid(True)
    
    ax[1].plot(oData['time_s'] - oData['time_s'][0], oData['altBaro_m'])
    ax[1].set_ylabel('Altitude (m)')
    ax[1].grid(True)
    
    ax[2].plot(oData['time_s'] - oData['time_s'][0], oData['vIas_mps'])
#    ax[2].plot(oData['time_s'] - oData['time_s'][0], 15.0 * np.ones_like(oData['vIas_mps']), 'r:')
    ax[2].set_ylabel('Airspeed (m/s)')
    ax[2].grid(True)
    
    ax[3].plot(oData['time_s'] - oData['time_s'][0], TE - TE[0] + eBatFilt_J)
    ax[3].set_ylabel('Energy (J)')
    ax[3].grid(True)
    
    ax[4].plot(oData['time_s'] - oData['time_s'][0], D - D[0])
    ax[4].set_ylabel('delDrag Est (N)')
    ax[4].grid(True)
    
    ax[5].plot(oData['time_s'] - oData['time_s'][0], pwrBatFilt_w)
    ax[5].set_ylabel('Power Bat (Watt)')
    ax[5].grid(True)
    ax[5].legend(loc = 'center left')
    
    ax2.plot(oData['Control']['cmdBend_nd'], TE - TE[0], '*')
