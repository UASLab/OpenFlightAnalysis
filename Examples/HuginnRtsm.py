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


#%%
from Core import FreqTrans

rtsmSegList = [
        {'flt': 'FLT03', 'seg': ('time_us', [693458513 , 705458513], 'FLT03 - 23 m/s')}, # 23 m/s
        {'flt': 'FLT03', 'seg': ('time_us', [865877709 , 877877709], 'FLT03 - 20 m/s')}, # 20 m/s
        
        {'flt': 'FLT04', 'seg': ('time_us', [884683583, 896683583], 'FLT04 - 26 m/s')}, # 26 m/s
        {'flt': 'FLT04', 'seg': ('time_us', [998733748, 1010733748], 'FLT04 - 20 m/s')} # 20 m/s
        ]


oDataSegs = []
for rtsmSeg in rtsmSegList:
    fltNum = rtsmSeg['flt']
    
    fileLog = fileList[fltNum]['log']
    fileConfig = fileList[fltNum]['config']
    
    # Load
    h5Data = Loader.Load_h5(fileLog) # RAPTRS log data as hdf5
    sysConfig = Loader.JsonRead(fileConfig)
    oData = Loader.OpenData_RAPTRS(h5Data, sysConfig)
    
    # Create Signal for Bending Measurement
    aZ = np.array([oData['aCenterFwdIMU_IMU_mps2'][2], 
    oData['aCenterAftIMU_IMU_mps2'][2], 
    oData['aLeftMidIMU_IMU_mps2'][2], 
    oData['aLeftFwdIMU_IMU_mps2'][2], 
    oData['aLeftAftIMU_IMU_mps2'][2], 
    oData['aRightMidIMU_IMU_mps2'][2], 
    oData['aRightFwdIMU_IMU_mps2'][2], 
    oData['aRightAftIMU_IMU_mps2'][2]])
    
#    aCoefEta1 = [456.1, -565.1, -289.9, 605.4, 472.4, -292, 605.6, 472.2] * 0.3048
    aCoefEta1dt = np.array([2.956, -2.998, -1.141, 5.974, 5.349, -1.149, 5.974, 5.348]) * 0.3048
    measEta1dt = aCoefEta1dt @ (aZ - np.mean(aZ, axis = 0)) * 1/1000
    
    # Added signals
    oData['cmdRoll_FF'] = h5Data['Control']['cmdRoll_PID_rpsFF']
    oData['cmdRoll_FB'] = h5Data['Control']['cmdRoll_PID_rpsFB']
    oData['cmdPitch_FF'] = h5Data['Control']['cmdPitch_PID_rpsFF']
    oData['cmdPitch_FB'] = h5Data['Control']['cmdPitch_PID_rpsFB']
    oData['cmdBend_FF'] = h5Data['Control']['refBend_nd'] # h5Data['Excitation']['Bend']['cmdBend_nd']
    oData['cmdBend_FB'] = measEta1dt
        
    # Segments
    oDataSegs.append(OpenData.Segment(oData, rtsmSeg['seg']))

#%%

#    from scipy.signal import detrend
#    estEta1 = detrend(np.cumsum(measEta1dt) * 1/50, type = 'linear')
#    
#    plt.plot(h5Data['Control']['cmdBend_nd'])
#    plt.plot(measEta1dt)
#    plt.plot(estEta1)

#%%


sigExcList = ['cmdRoll_rps', 'cmdPitch_rps', 'cmdBend_nd']
sigFbList = ['cmdRoll_FB', 'cmdPitch_FB', 'cmdBend_FB']

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
    
    plt.plot(oDataSegs[iSeg]['time_s'], oDataSegs[iSeg]['vIas_mps'])

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

# Compute Gain, Phase, Crit Distance
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


#%% Spectrograms
if True:
    
    iSgnl = 2
    
    freqRate_rps = 50 * hz2rps
    freqExc_rps = np.linspace(0.1, 50/2, 151) * hz2rps
    optSpec = FreqTrans.OptSpect(dftType = 'czt', freq = freqExc_rps, freqRate = freqRate_rps, winType = ('tukey', 0.2), smooth = ('box', 1), detrendType = 'Linear')
    
    
    for iSeg in range(0, len(oDataSegs)):
        t = oDataSegs[iSeg]['time_s']
        y = vFbList[iSeg][iSgnl]
        
        # Number of time segments and length of overlap, units of samples
        #lenSeg = 2**6 - 1
        lenSeg = int(1 * optSpec.freqRate * rps2hz)
        lenOverlap = 5
        
        # Compute Spectrum over time
        tSpec_s, freqSpec_rps, P_mag = FreqTrans.SpectTime(t, y, lenSeg, lenOverlap, optSpec)
        
        # Plot the Spectrogram
        fig = FreqTrans.Spectogram(tSpec_s, freqSpec_rps * rps2hz, 20 * np.log10(P_mag))
        fig.suptitle(oDataSegs[iSeg]['Desc'] + ': Spectrogram - ' + sigFbList[iSgnl])


#%% Disk Margin Plots
inPlot = sigExcList # Elements of sigExcList
outPlot = sigFbList # Elements of sigFbList

if False:
    for iIn, inName in enumerate(inPlot):
        for iOut, outName in enumerate(outPlot):
    
            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotDistCrit(freq_hz[iOut, 0], rCrit_mag[iSeg][iOut, iIn], unc = rCritUnc_mag[iSeg][iOut, iIn], coher_nd = C[iSeg][iOut, iIn], fig = fig, fmt = '*:', label = oDataSegs[iSeg]['Desc'])
            
            fig = FreqTrans.PlotDistCrit(freq_hz[iOut, 0], 0.4 * np.ones_like(freq_hz[iOut, 0]), fig = fig, fmt = 'r--', label = 'Critical Limit')
            fig.suptitle(inName + ' to ' + outName, size=20)
            
            ax = fig.get_axes()
            ax[0].set_ylim(0, 2)


#%% Nyquist Plots
if False:
    for iIn, inName in enumerate(inPlot):
        for iOut, outName in enumerate(outPlot):
            
            fig = None
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
            
            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotBode(freq_hz[iOut, 0], gain_dB[iSeg][iOut, iIn], phase_deg[iSeg][iOut, iIn], C[iSeg][iOut, iIn], fig = fig, fmt = '*--', label = oDataSegs[iSeg]['Desc'])
            
            fig.suptitle(inName + ' to ' + outName, size=20)
