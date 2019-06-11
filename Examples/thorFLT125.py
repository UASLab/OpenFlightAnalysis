"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Analysis for Huginn (mAEWing2) FLT 02
"""

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


fileLog = '/home/rega0051/FlightArchive/Thor/ThorFLT125/ThorFLT125.h5'
fileTestDef = '/home/rega0051/FlightArchive/Thor/ThorFLT125/FLT_Def.json'
fileSysConfig = '/home/rega0051/FlightArchive/Thor/ThorFLT125/thor.json'

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


#%% RTSM
from Core import FreqTrans
    
segList = [('time_us', excList[0][1]), 
           ('time_us', excList[1][1]), 
           ('time_us', excList[2][1]), 
           ('time_us', excList[4][1]), 
           ('time_us', excList[5][1]), 
           ('time_us', excList[6][1]), 
           ('time_us', excList[8][1]), 
           ('time_us', excList[9][1]), 
           ('time_us', excList[10][1])]

oDataSegs = OpenData.Segment(oData, segList)

sigExcList = ['cmdRoll_rps', 'cmdPitch_rps', 'cmdYaw_rps']

freqExc_rps = []
freqExc_rps.append( np.array(sysConfig['Excitation']['OMS_RTSM_1']['Frequency']))
freqExc_rps.append( np.array(sysConfig['Excitation']['OMS_RTSM_2']['Frequency']))
freqExc_rps.append( np.array(sysConfig['Excitation']['OMS_RTSM_3']['Frequency']))

vCmdList = []
vExcList = []
vInList = []
for iSeg, seg in enumerate(oDataSegs):
    vCmd = np.zeros((len(sigExcList), len(seg['time_s'])))
    vExc = np.zeros((len(sigExcList), len(seg['time_s'])))
    vIn= np.zeros((len(sigExcList), len(seg['time_s'])))
    for iSig, sigName in enumerate(sigExcList):
        vCmd[iSig] = seg[sigName]
        vExc[iSig] = seg['Excitation'][sigName]
        vIn[iSig] = vCmd[iSig] - vExc[iSig]
    vCmdList.append(vCmd)
    vExcList.append(vExc)
    vInList.append(vIn)

#iSeg = 7
#plt.plot(oDataSegs[iSeg]['time_s'], vExcList[iSeg][0])


#%% Estimate the frequency response function
# Define the excitation frequencies

hz2rps = 2*np.pi
rps2hz = 1/hz2rps



freqRate_rps = 50 * hz2rps
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3))
optSpecN = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps)

# Excited Frequencies per input channel
optSpec.freq = np.asarray(freqExc_rps)

# Null Frequencies
freqGap_rps = optSpec.freq.flatten()[0:-1] + 0.5 * np.diff(optSpec.freq.flatten())
optSpecN.freq = freqGap_rps

# FRF Estimate

for iSeg, seg in enumerate(oDataSegs):
    
#    freq_rps, Teb, Ceb, Pee, Pbb, Peb, TebUnc = FreqTrans.FreqRespFuncEstNoise(vExcList[iSeg], fb, optSpec, optSpecN)
    freq_rps, Tev, Cev, Pee, Pvv, Pev, TevUnc = FreqTrans.FreqRespFuncEstNoise(vExcList[iSeg], vCmdList[iSeg], optSpec, optSpecN)

nFreq = freq_rps.shape[-1]

freq_hz = freq_rps * rps2hz
#eye = np.tile(np.eye(3), (nFreq, 1, 1)).T

# Form the Frequency Response
T = Teb / (Tev + TevUnc)
TUnc = TebUnc / (Tev + TevUnc)

T_InputNames = exc_names
T_OutputNames = fb_names

gain_dB, phase_deg = FreqTrans.GainPhase(T)
phase_deg = np.unwrap(phase_deg * deg2rad) * rad2deg
sigma_mag = np.abs(T - (0 - 1j)) # Distance from (0,-1j)

