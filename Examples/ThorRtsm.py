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

#pathBase = path.join('/home', 'rega0051', 'FlightArchive', 'Thor')
pathBase = path.join('G:', 'Shared drives', 'UAVLab', 'Flight Data', 'Thor')

fileList = {}
flt = 'FLT126'
fileList[flt] = {}
fileList[flt]['log'] = path.join(pathBase, 'Thor' + flt, 'Thor' + flt + '.h5')
fileList[flt]['config'] = path.join(pathBase, 'Thor' + flt, 'thor.json')
fileList[flt]['def'] = path.join(pathBase, 'Thor' + flt, 'thor_def.json')

flt = 'FLT127'
fileList[flt] = {}
fileList[flt]['log'] = path.join(pathBase, 'Thor' + flt, 'Thor' + flt + '.h5')
fileList[flt]['config'] = path.join(pathBase, 'Thor' + flt, 'thor.json')
fileList[flt]['def'] = path.join(pathBase, 'Thor' + flt, 'thor_def.json')

flt = 'FLT128'
fileList[flt] = {}
fileList[flt]['log'] = path.join(pathBase, 'Thor' + flt, 'Thor' + flt + '.h5')
fileList[flt]['config'] = path.join(pathBase, 'Thor' + flt, 'thor.json')
fileList[flt]['def'] = path.join(pathBase, 'Thor' + flt, 'thor_def.json')


#%%
from Core import FreqTrans

rtsmSegList = [
        {'flt': 'FLT126', 'seg': ('time_us', [875171956 , 887171956], 'FLT126 - RTSM - Nominal Gain, 4 deg amp'), 'fmt': 'k'},
        {'flt': 'FLT126', 'seg': ('time_us', [829130591 , 841130591], 'FLT126 - RTSM Route - Nominal Gain, 4 deg amp'), 'fmt': 'k'},
        {'flt': 'FLT127', 'seg': ('time_us', [641655909 , 653655909], 'FLT127 - RTSM Route - Nominal Gain, 4 deg amp'), 'fmt': 'k'},
        {'flt': 'FLT128', 'seg': ('time_us', [700263746 , 712263746 ], 'FLT128 - RTSM Route - Nominal Gain, 4 deg amp'), 'fmt': 'k'},
        {'flt': 'FLT128', 'seg': ('time_us', [831753831 , 843753831 ], 'FLT128 - RTSM Route - Nominal Gain, 4 deg amp'), 'fmt': 'k'},
#        {'flt': 'FLT128', 'seg': ('time_us', [ 959859721 , 971859721 ], 'FLT128 - RTSM Route - Nominal Gain, 4 deg amp'), 'fmt': 'k'},

        {'flt': 'FLT126', 'seg': ('time_us', [928833763 , 940833763], 'FLT126 - RTSM Large - Nominal Gain, 8 deg amp'), 'fmt': 'r'},
        {'flt': 'FLT127', 'seg': ('time_us', [698755386 , 707255278], 'FLT127 - RTSM Large Route - Nominal Gain, 8 deg amp'), 'fmt': 'r'},
        {'flt': 'FLT128', 'seg': ('time_us', [779830919 , 791830919 ], 'FLT128 - RTSM Large Route - Nominal Gain, 8 deg amp'), 'fmt': 'r'},
        {'flt': 'FLT128', 'seg': ('time_us', [900237086 , 912237086 ], 'FLT128 - RTSM Large Route - Nominal Gain, 8 deg amp'), 'fmt': 'r'},

#        {'flt': 'FLT126', 'seg': ('time_us', [902952886 , 924952886], 'FLT126 - RTSM Long - Nominal Gain, 4 deg amp'), 'fmt': 'b'},
        {'flt': 'FLT127', 'seg': ('time_us', [657015836 , 689015836], 'FLT127 - RTSM Long Route - Nominal Gain, 4 deg amp'), 'fmt': 'b'},
        {'flt': 'FLT128', 'seg': ('time_us', [714385469 , 746385469 ], 'FLT128 - RTSM Long Route - Nominal Gain, 4 deg amp'), 'fmt': 'b'}, # Not good
        {'flt': 'FLT128', 'seg': ('time_us', [847254621 , 879254621 ], 'FLT128 - RTSM Long Route - Nominal Gain, 4 deg amp'), 'fmt': 'b'},

#        {'flt': 'FLT127', 'seg': ('time_us', [1209355236 , 1221535868], 'FLT127 - RTSM LongLarge Route - Nominal Gain, 8 deg amp'), 'fmt': 'm'},
#        {'flt': 'FLT128', 'seg': ('time_us', [794251787 , 826251787 ], 'FLT128 - RTSM LongLarge Route - Nominal Gain, 8 deg amp'), 'fmt': 'm'},
#        {'flt': 'FLT128', 'seg': ('time_us', [921438015 , 953438015 ], 'FLT128 - RTSM LongLarge Route - Nominal Gain, 8 deg amp'), 'fmt': 'm'},

#        {'flt': 'FLT126', 'seg': ('time_us', [981115495 , 993115495], 'FLT126 - RTSM - High Gain, 4 deg amp')},
#        {'flt': 'FLT126', 'seg': ('time_us', [689907125 , 711907125], 'FLT126 - RTSM Long - High Gain, 4 deg amp')},
#        {'flt': 'FLT126', 'seg': ('time_us', [728048050 , 740048050], 'FLT126 - RTSM Large - High Gain, 8 deg amp')},
#
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

    oData['cmdRoll_FF'] = h5Data['Control']['cmdRoll_pidFF']
    oData['cmdRoll_FB'] = h5Data['Control']['cmdRoll_pidFB']
    oData['cmdPitch_FF'] = h5Data['Control']['cmdPitch_pidFF']
    oData['cmdPitch_FB'] = h5Data['Control']['cmdPitch_pidFB']
    oData['cmdYaw_FF'] = h5Data['Control']['refPsi_rad']
    oData['cmdYaw_FB'] = h5Data['Control']['cmdYaw_damp_rps']

    # Segments
    oDataSegs.append(OpenData.Segment(oData, rtsmSeg['seg']))


#%%

sigExcList = ['cmdRoll_rps', 'cmdPitch_rps', 'cmdYaw_rps']
sigFbList = ['cmdRoll_FB', 'cmdPitch_FB', 'cmdYaw_FB']

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
rCritNom_mag = []
rCritUnc_mag = []
rCrit_mag = []
for iSeg in range(0, len(oDataSegs)):

    gain_dB.append(FreqTrans.Gain(T[iSeg], magUnit = 'dB'))
    phase_deg.append(FreqTrans.Phase(T[iSeg], phaseUnit = 'deg', unwrap = True))

    nom_mag, unc_mag, diff_mag = FreqTrans.DistCrit(T[iSeg], TUnc[iSeg], pCrit = -1+0j, type = 'ellipse', magUnit = 'mag')

    rCritNom_mag.append(nom_mag)
    rCritUnc_mag.append(unc_mag)
    rCrit_mag.append(diff_mag)


#%% Spectrograms
if False:

    iSgnl = 1

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

if True:
    for iIn, inName in enumerate(inPlot):
        for iOut, outName in enumerate(outPlot):

            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotDistCrit(freq_hz[iIn, 0], rCritNom_mag[iSeg][iIn, iOut], unc = rCrit_mag[iSeg][iIn, iOut], coher_nd = C[iSeg][iIn, iOut], fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + '*-', label = oDataSegs[iSeg]['Desc'])

            fig = FreqTrans.PlotDistCrit(freq_hz[iIn, 0], 0.4 * np.ones_like(freq_hz[iIn, 0]), fig = fig, fmt = 'r--', label = 'Critical Limit')
            fig.suptitle(inName + ' to ' + outName, size=20)

            ax = fig.get_axes()
            ax[0].set_ylim(0, 2)


#%% Nyquist Plots
if False:
    for iIn, inName in enumerate(inPlot):
        for iOut, outName in enumerate(outPlot):

            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotNyquist(T[iSeg][iIn, iOut], TUnc[iSeg][iIn, iOut], fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + '*', label = oDataSegs[iSeg]['Desc'])

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
                fig = FreqTrans.PlotBode(freq_hz[iIn, 0], gain_dB[iSeg][iIn, iOut], phase_deg[iSeg][iIn, iOut], C[iSeg][iIn, iOut], fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + '*-', label = oDataSegs[iSeg]['Desc'])

            fig.suptitle(inName + ' to ' + outName, size=20)
