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


# Constants
hz2rps = 2 * np.pi
rps2hz = 1 / hz2rps

rad2deg = 180.0 / np.pi
deg2rad = 1 / rad2deg

pathFile = '/home/rega0051/FlightArchive/Thor/ThorFLT126/'
fileLog = pathFile + 'ThorFLT126.h5'
fileTestDef = pathFile + 'FLT_Def.json'
fileSysConfig = pathFile + 'thor.json'

#%%
# Read in raw h5 data into dictionary and data
oData, h5Data = Loader.Log_RAPTRS(fileLog, fileSysConfig)

# Plot Overview of flight
oDataPlot = OpenData.Segment(oData, ('time_s', [425, 1100]))
#OpenData.PlotOverview(oDataPlot)

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
    import json
    json.dumps(testDef, indent = 4, ensure_ascii=False)
    print('\nInit file NOT updated\n\n')


#%% RTSM
from Core import FreqTrans

oData['cmdRoll_pidFF'] = h5Data['Control']['cmdRoll_pidFF']
oData['cmdRoll_pidFB'] = h5Data['Control']['cmdRoll_pidFB']
oData['cmdPitch_pidFF'] = h5Data['Control']['cmdPitch_pidFF']
oData['cmdPitch_pidFB'] = h5Data['Control']['cmdPitch_pidFB']
oData['cmdYaw_pidFF'] = h5Data['Control']['refPsi_rad']
oData['cmdYaw_pidFB'] = h5Data['Control']['cmdYaw_damp_rps']

segList = [('time_us', [excList[10][1][0], excList[10][1][0]+12e6], 'RTSM - Nominal Gain, 4 deg amp'),
#           ('time_us', [excList[4][1][0], excList[4][1][0]+12e6], 'RTSM - High Gain, 4 deg amp'),
#           ('time_us', [excList[12][1][0], excList[12][1][0]+12e6], 'RTSM - Nominal Gain, 8 deg amp'),
#           ('time_us', [excList[6][1][0], excList[6][1][0]+12e6], 'RTSM - High Gain, 8 deg amp'),
           ('time_us', [excList[8][1][0], excList[8][1][0]+12e6], 'RTSM Route - Nominal Gain, 4 deg amp'),
           ('time_us', [excList[9][1][0], excList[9][1][0]+12e6], 'RTSM Route - Nominal Gain, 4 deg amp')]

oDataSegs = OpenData.Segment(oData, segList)

sigExcList = ['cmdRoll_rps', 'cmdPitch_rps', 'cmdYaw_rps']
sigFbList = ['cmdRoll_pidFB', 'cmdPitch_pidFB', 'cmdYaw_pidFB']

freqExc_rps = []
freqExc_rps.append( np.array(sysConfig['Excitation']['OMS_RTSM_1']['Frequency']))
freqExc_rps.append( np.array(sysConfig['Excitation']['OMS_RTSM_2']['Frequency']))
freqExc_rps.append( np.array(sysConfig['Excitation']['OMS_RTSM_3']['Frequency']))

vCmdList = []
vExcList = []
vFbList = []
vFfList = []
for iSeg, seg in enumerate(oDataSegs):
    vCmd = np.zeros((len(sigExcList), len(seg['time_s'])))
    vExc = np.zeros((len(sigExcList), len(seg['time_s'])))
    vFb = np.zeros((len(sigExcList), len(seg['time_s'])))
    vFf = np.zeros((len(sigExcList), len(seg['time_s'])))
    
    for iSig, sigExc in enumerate(sigExcList):
        sigFb = sigFbList[iSig]
        
        vCmd[iSig] = seg['Control'][sigExc]
        vExc[iSig] = seg['Excitation'][sigExc]
        vFb[iSig] = seg[sigFb]
        vFf[iSig] = vCmd[iSig] - vExc[iSig] - vFb[iSig]
        
    vCmdList.append(vCmd)
    vExcList.append(vExc)
    vFbList.append(vFb)
    vFfList.append(vFf)

#    plt.plot(oDataSegs[iSeg]['time_s'], vExcList[iSeg][0])
#    plt.plot(oDataSegs[iSeg]['time_s'], vExcList[iSeg][1])
#    plt.plot(oDataSegs[iSeg]['time_s'], vExcList[iSeg][2])
#    plt.plot(oDataSegs[iSeg]['time_s'], vFbList[iSeg][0])
#    plt.plot(oDataSegs[iSeg]['time_s'], vFbList[iSeg][1])
#    plt.plot(oDataSegs[iSeg]['time_s'], vFbList[iSeg][2])


#%% Estimate the frequency response function
# Define the excitation frequencies
freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.2), detrendType = 'Linear')
optSpecN = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 1), winType = ('tukey', 0.1), detrendType = 'Linear')

# Excited Frequencies per input channel
optSpec.freq = np.asarray(freqExc_rps)

# Null Frequencies
freqGap_rps = optSpec.freq.flatten()[0:-1] + 0.5 * np.diff(optSpec.freq.flatten())
optSpecN.freq = freqGap_rps

# FRF Estimate
T = []
TUnc = []
C = []
for iSeg, seg in enumerate(oDataSegs):
    
    freq_rps, Teb, Ceb, Pee, Pbb, Peb, TebUnc = FreqTrans.FreqRespFuncEstNoise(vExcList[iSeg], vFbList[iSeg], optSpec, optSpecN)
    _       , Tev, Cev, _  , Pvv, Pev, TevUnc = FreqTrans.FreqRespFuncEstNoise(vExcList[iSeg], vCmdList[iSeg], optSpec, optSpecN)
    
    freq_hz = freq_rps * rps2hz
    
    # Form the Frequency Response
    T.append( Teb / (Tev + TevUnc) )
    TUnc.append( TebUnc / (Tev + TevUnc) )
    
#    C.append(Ceb)
    C.append(Cev)


T_InputNames = sigExcList
T_OutputNames = sigFbList

# Generate Gain, Phase, and distance to critital point (-1, 0j)
gain_dB = []
phase_deg = []
rCrit_mag = []
rCritUnc_mag = []
for iSeg in range(0, len(oDataSegs)):
    
    gain_dB.append(FreqTrans.Gain(T[iSeg], magUnit = 'dB'))
    phase_deg.append(FreqTrans.Phase(T[iSeg], phaseUnit = 'deg', unwrap = True))
    
    nom_mag, unc_mag, _ = FreqTrans.DistCritCirc(T[iSeg], TUnc[iSeg], magUnit='mag') # Distance from (-1,0j)
    
    rCrit_mag.append(nom_mag)
    rCritUnc_mag.append(unc_mag)


#%% Disk Margin Plots
inPlot = sigExcList # Elements of sigExcList
outPlot = sigFbList # Elements of sigFbList

if True:
    for iIn, inName in enumerate(inPlot):
        for iOut, outName in enumerate(outPlot):
    
            fig = 60 + 3*iIn + iOut
            #fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotDistCrit(freq_hz[iOut, 0], rCrit_mag[iSeg][iOut, iIn], unc = rCritUnc_mag[iSeg][iOut, iIn], coher_nd = C[iSeg][iOut, iIn], fig = fig, fmt = '*:', label = oDataSegs[iSeg]['Desc'])
            
            #fig = FreqTrans.PlotDistCrit(freq_hz[iOut, 0], 0.4 * np.ones_like(freq_hz[iOut, 0]), fig = fig, fmt = 'r--', label = 'Critical Limit')
            fig.suptitle(inName + ' to ' + outName, size=20)
            
            ax = fig.get_axes()
            ax[0].set_ylim(0, 2)


#%% Nyquist Plots
if False:
    for iIn, inName in enumerate(inPlot):
        for iOut, outName in enumerate(outPlot):
            
            fig = 80 + 3*iIn + iOut
            #fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotNyquist(T[iSeg][iOut, iIn], TUnc[iSeg][iOut, iIn], fig = fig, fmt = '*', label = oDataSegs[iSeg]['Desc'])
            
            fig = FreqTrans.PlotNyquist(np.asarray([-1+ 0j]), TUnc = np.asarray([0.4 + 0.4j]), fig = fig, fmt = '*r', label = 'Critical Region')            
            fig.suptitle(inName + ' to ' + outName, size=20)
            
            ax = fig.get_axes()
            ax[0].set_xlim(-3, 1)
            ax[0].set_ylim(-2, 2)


#%% Bode Plots
if False:

    for iIn, inName in enumerate(inPlot):
        for iOut, outName in enumerate(outPlot):
            
            fig = 100 + 3*iIn + iOut
            #fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotBode(freq_hz[iOut, 0], gain_dB[iSeg][iOut, iIn], phase_deg[iSeg][iOut, iIn], C[iSeg][iOut, iIn], fig = fig, fmt = '*--', label = oDataSegs[iSeg]['Desc'])
            

