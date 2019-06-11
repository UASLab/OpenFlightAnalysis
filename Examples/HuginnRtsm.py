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
import copy

rtsmSegList = [
        {'flt': 'FLT03', 'seg': ('time_us', [693458513 , 705458513])}, # 23 m/s
        {'flt': 'FLT03', 'seg': ('time_us', [865877709 , 877877709])} # 20 m/s
        
#        {'flt': 'FLT04', 'seg': ('time_us', [884683583, 896683583])}, # 26 m/s
#        {'flt': 'FLT04', 'seg': ('time_us', [998733748, 1010733748])} # 20 m/s
        ]


oDataSegs = []
for rtsmSeg in rtsmSegList:
    fltNum = rtsmSeg['flt']
    
    fileLog = fileList[fltNum]['log']
    fileConfig = fileList[fltNum]['config']
    
    # Load
    h5Data = []
    oData = []
    
    h5Data = copy.deepcopy(Loader.Load_h5(fileLog)) # RAPTRS log data as hdf5
    sysConfig = Loader.JsonRead(fileConfig)
    oData = Loader.OpenData_RAPTRS(h5Data, sysConfig)
    
    # Added signals
    oData['cmdRoll_FF'] = h5Data['Control']['cmdRoll_PID_rpsFF']
    oData['cmdRoll_FB'] = h5Data['Control']['cmdRoll_PID_rpsFB']
    oData['cmdPitch_FF'] = h5Data['Control']['cmdPitch_PID_rpsFF']
    oData['cmdPitch_FB'] = h5Data['Control']['cmdPitch_PID_rpsFB']
    oData['cmdBend_FF'] = h5Data['Control']['refBend_nd']
    oData['cmdBend_FB'] = oData['aLeftFwdIMU_IMU_mps2'][2] + oData['aLeftAftIMU_IMU_mps2'][2] + oData['aRightFwdIMU_IMU_mps2'][2] + oData['aRightAftIMU_IMU_mps2'][2] - 2 * (oData['aCenterFwdIMU_IMU_mps2'][2] + oData['aCenterAftIMU_IMU_mps2'][2])

    # Segments
    seg = OpenData.Segment(oData, rtsmSeg['seg'])
    oDataSegs.append(seg)


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
        
        vCmd[iSig] = seg[sigExc]
        vExc[iSig] = seg['Excitation'][sigExc]
        vFb[iSig] = seg[sigFb]
        vFf[iSig] = vCmd[iSig] - vExc[iSig] - vFb[iSig]
        
    vCmdList.append(vCmd)
    vExcList.append(vExc)
    vFbList.append(vFb)
    vFfList.append(vFf)

    plt.plot(oDataSegs[iSeg]['time_s'], vExcList[iSeg][0])
    plt.plot(oDataSegs[iSeg]['time_s'], vExcList[iSeg][1])
    plt.plot(oDataSegs[iSeg]['time_s'], vExcList[iSeg][2])
    plt.plot(oDataSegs[iSeg]['time_s'], vFbList[iSeg][0])
    plt.plot(oDataSegs[iSeg]['time_s'], vFbList[iSeg][1])
    plt.plot(oDataSegs[iSeg]['time_s'], vFbList[iSeg][2])


#%% Estimate the frequency response function
# Define the excitation frequencies
freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.2))
optSpecN = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.2))

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
    
    C.append(Ceb)


T_InputNames = sigExcList
T_OutputNames = sigFbList


#%% Disk Margin Plots
inPlot = sigExcList # Elements of sigExcList
outPlot = sigFbList # Elements of sigFbList

fig, ax = plt.subplots(len(outPlot), len(inPlot), sharex=True)

sigmaList_mag = []
sigmaUncList_mag = []
for iSeg in range(0, len(oDataSegs)):
    
    sigmaList_mag.append(np.abs(T[iSeg] - (0 - 1j))) # Distance from (0,-1j)
    sigmaUncList_mag = np.abs(TUnc[iSeg])
    
    for iIn, inName in enumerate(inPlot):
        inElem = sigExcList.index(inName)
        
        for iOut, outName in enumerate(outPlot):
            outElem = sigFbList.index(outName)
            
            sigma_mag = np.abs(T[iSeg][iOut, iIn] - (0 - 1j)) # Distance from (0,-1j)
            uncDisk_mag = np.abs(TUnc[iSeg][iOut, iIn])
            
            ax[iOut, iIn].errorbar(freq_hz[iOut, 0], sigma_mag, yerr = uncDisk_mag, fmt = '.-', elinewidth = 0, capsize = 2, label = oDataSegs[iSeg]['Desc'])
            ax[iOut, iIn].plot([min(freq_hz.flatten()), max(freq_hz.flatten())], [0.4, 0.4], 'r--')
#            ax[iOut, iIn].set_xlim(left = 0.0, right = max(freq_hz.flatten()))
            ax[iOut, iIn].set_ylim(bottom = 0.0, top = 5.0)
            ax[iOut, iIn].grid()
            
    ax[iOut, iIn].legend()


#%% Nyquist Plots
if True:
    import matplotlib.patches as patch
    
    inPlot = sigExcList # Elements of sigExcList
    outPlot = sigFbList # Elements of sigFbList
    
    for iSeg in range(0, len(oDataSegs)):
        
        fig, ax = plt.subplots(len(outPlot), len(inPlot))
        for iIn, inName in enumerate(inPlot):
            inElem = sigExcList.index(inName)
        
            for iOut, outName in enumerate(outPlot):
                outElem = sigFbList.index(outName)
            
                ax[iOut, iIn].plot(T[iSeg][iOut, iIn].imag, T[iSeg][iOut, iIn].real, '-b*', label = oDataSegs[iSeg]['Desc'])
                ax[iOut, iIn].grid()
                critPatch = patch.Ellipse((-1, 0), 2*0.4, 2*0.4, color='r', alpha=0.25)
                ax[iOut, iIn].add_artist(critPatch)
                
                for iNom, nom in enumerate(T[iSeg][iOut, iIn]):
                    unc = TUnc[iSeg][iOut, iIn][iNom]
                    uncPatch = patch.Ellipse((nom.imag, nom.real), 2*unc.imag, 2*unc.real, color='b', alpha=0.25)
                    ax[iOut, iIn].add_artist(uncPatch)
        
                ax[iOut, iIn].set_xlim(-3, 1)
                ax[iOut, iIn].set_ylim(-2, 2)
        

#%% Bode Plots
if True:
    gain_dB = []
    phase_deg = []
    for iSeg in range(0, len(oDataSegs)):
        
        g_dB, p_deg = FreqTrans.GainPhase(T[iSeg])
#        p_deg = np.unwrap(p_deg * deg2rad) * rad2deg
        
        gain_dB.append(g_dB)
        phase_deg.append(p_deg)
        
    
    for iIn in range(0, 3):
        for iOut in range(0, 3):
            plt.figure()
            for iSeg in range(0, len(oDataSegs)):
                
                ax1 = plt.subplot(3, 1, 1)
                ax1.semilogx(freq_hz[iOut, 0], gain_dB[iSeg][iOut, iIn], '-*', label = oDataSegs[iSeg]['Desc'])
                ax1.grid(); ax1.set_ylabel('Gain (dB)')
                
                ax2 = plt.subplot(3, 1, 2, sharex = ax1)
                ax2.semilogx(freq_hz[iOut, 0], phase_deg[iSeg][iOut, iIn], '-*', label = oDataSegs[iSeg]['Desc'])
                ax2.grid(); ax2.set_ylabel('Phase (deg)')
                
                ax3 = plt.subplot(3, 1, 3, sharex = ax1)
                ax3.semilogx(freq_hz[iOut, 0], C[iSeg][iOut, iIn], '-*', label = oDataSegs[iSeg]['Desc'])
                ax3.grid(); ax3.set_xlabel('Freq (Hz)'); ax3.set_ylabel('Coherence (nd)')
                ax3.set_ylim(0, 1)
                
            ax1.legend()

