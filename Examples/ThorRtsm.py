"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Analysis for Thor RTSM
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
#        {'flt': 'FLT126', 'seg': ('time_us', [875171956 , 887171956], 'FLT126 - RTSM - Nominal Gain, 4 deg amp'), 'fmt': 'k'},
#        {'flt': 'FLT126', 'seg': ('time_us', [829130591 , 841130591], 'FLT126 - RTSM Route - Nominal Gain, 4 deg amp'), 'fmt': 'k'},
#        {'flt': 'FLT127', 'seg': ('time_us', [641655909 , 653655909], 'FLT127 - RTSM Route - Nominal Gain, 4 deg amp'), 'fmt': 'k'}, # Yaw controller in-op??
#        {'flt': 'FLT128', 'seg': ('time_us', [700263746 , 712263746 ], 'FLT128 - RTSM Route - Nominal Gain, 4 deg amp'), 'fmt': 'k'}, # Interesting Roll Margin vs. Uncertainty
#        {'flt': 'FLT128', 'seg': ('time_us', [831753831 , 843753831 ], 'FLT128 - RTSM Route - Nominal Gain, 4 deg amp'), 'fmt': 'k'},
#        {'flt': 'FLT128', 'seg': ('time_us', [ 959859721 , 971859721 ], 'FLT128 - RTSM Route - Nominal Gain, 4 deg amp'), 'fmt': 'k'}, # Not good

#        {'flt': 'FLT126', 'seg': ('time_us', [928833763 , 940833763], 'FLT126 - RTSM Large - Nominal Gain, 8 deg amp'), 'fmt': 'r'},
#        {'flt': 'FLT127', 'seg': ('time_us', [698755386 , 707255278], 'FLT127 - RTSM Large Route - Nominal Gain, 8 deg amp'), 'fmt': 'r'}, # Yaw controller in-op??
#        {'flt': 'FLT128', 'seg': ('time_us', [779830919 , 791830919 ], 'FLT128 - RTSM Large Route - Nominal Gain, 8 deg amp'), 'fmt': 'r'},
#        {'flt': 'FLT128', 'seg': ('time_us', [900237086 , 912237086 ], 'FLT128 - RTSM Large Route - Nominal Gain, 8 deg amp'), 'fmt': 'r'},
#
#        {'flt': 'FLT126', 'seg': ('time_us', [902952886 , 924952886], 'FLT126 - RTSM Long - Nominal Gain, 4 deg amp'), 'fmt': 'b'},
#        {'flt': 'FLT127', 'seg': ('time_us', [657015836 , 689015836], 'FLT127 - RTSM Long Route - Nominal Gain, 4 deg amp'), 'fmt': 'b'}, # Yaw controller in-op??
#        {'flt': 'FLT128', 'seg': ('time_us', [714385469 , 746385469 ], 'FLT128 - RTSM Long Route - Nominal Gain, 4 deg amp'), 'fmt': 'b'},
        {'flt': 'FLT128', 'seg': ('time_us', [847254621 , 879254621 ], 'FLT128 - RTSM Long Route - Nominal Gain, 4 deg amp'), 'fmt': 'b'}, # Best

#        {'flt': 'FLT127', 'seg': ('time_us', [1209355236 , 1221535868], 'FLT127 - RTSM LongLarge Route - Nominal Gain, 8 deg amp'), 'fmt': 'm'}, # Yaw controller in-op??
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
    rtsmSeg['seg'][1][0] += 1e6
    rtsmSeg['seg'][1][1] += -1e6 + 50e3

    oDataSegs.append(OpenData.Segment(oData, rtsmSeg['seg']))


#%%

sigExcList = ['cmdRoll_rps', 'cmdPitch_rps', 'cmdYaw_rps']
sigFbList = ['cmdRoll_FB', 'cmdPitch_FB', 'cmdYaw_FB']
sigFfList = ['cmdRoll_FF', 'cmdPitch_FF', 'cmdYaw_FF']
#sigSensList = ['wB_I_rps', 'cmdPitch_FF', 'cmdYaw_FF']

freqExc_rps = []
freqExc_rps.append( np.array(sysConfig['Excitation']['OMS_RTSM_1']['Frequency']))
freqExc_rps.append( np.array(sysConfig['Excitation']['OMS_RTSM_2']['Frequency']))
freqExc_rps.append( np.array(sysConfig['Excitation']['OMS_RTSM_3']['Frequency']))

vCmdList = []
vExcList = []
vFbList = []
vFfList = []
ySensList = []
for iSeg, seg in enumerate(oDataSegs):
    vCmd = np.zeros((len(sigExcList), len(seg['time_s'])))
    vExc = np.zeros((len(sigExcList), len(seg['time_s'])))
    vFb = np.zeros((len(sigExcList), len(seg['time_s'])))
    vFf = np.zeros((len(sigExcList), len(seg['time_s'])))
    ySens = np.zeros((len(sigExcList), len(seg['time_s'])))

    for iSig, sigExc in enumerate(sigExcList):
        sigFb = sigFbList[iSig]
        sigFf = sigFfList[iSig]

        vCmd[iSig] = seg['Control'][sigExc]
        vExc[iSig] = seg['Excitation'][sigExc]
#        vFb[iSig] = seg[sigFb]
        vFb[iSig][1:-1] = seg[sigFb][0:-2] # Shift the time of the output into next frame
        vFf[iSig] = seg[sigFf]

        ySens[iSig] = seg['wB_I_rps'][iSig]


    vCmdList.append(vCmd)
    vExcList.append(vExc)
    vFbList.append(vFb)
    vFfList.append(vFf)
    ySensList.append(ySens)

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
optSpecN = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.2), detrendType = 'Linear')

# Excited Frequencies per input channel
optSpec.freq = np.asarray(freqExc_rps)

# Null Frequencies
freqNull_rps = optSpec.freq.flatten()[0:-1] + 0.5 * np.diff(optSpec.freq.flatten())
optSpecN.freq = freqNull_rps

# FRF Estimate
T = []
TUnc = []
TSens = []
C = []
sNom = []
sUnc = []
for iSeg, seg in enumerate(oDataSegs):

    freq_rps, Teb, Ceb, Pee, Pbb, Peb, TebUnc, Pbb_N = FreqTrans.FreqRespFuncEstNoise(vExcList[iSeg], vFbList[iSeg], optSpec, optSpecN)
    _       , Tev, Cev, _  , Pvv, Pev, TevUnc, Pvv_N = FreqTrans.FreqRespFuncEstNoise(vExcList[iSeg], vCmdList[iSeg], optSpec, optSpecN)

    _       , Tey, Cey, _  , Pyy, Pey, TeyUnc, Pyy_N = FreqTrans.FreqRespFuncEstNoise(vExcList[iSeg], ySensList[iSeg], optSpec, optSpecN) # Just for Turbulence

    freq_hz = freq_rps * rps2hz

    # Form the Frequency Response
    T_seg = np.empty_like(Tev)
    TUnc_seg = np.empty_like(Tev)
    TSens_seg = np.empty_like(Tev)

    for i in range(T_seg.shape[-1]):
        T_seg[...,i] = (Teb[...,i].T @ np.linalg.inv(Tev[...,i].T)).T
        TUnc_seg[...,i] = (TebUnc[...,i].T @ np.linalg.inv(Tev[...,i].T)).T

        TSens_seg[...,i] = (TeyUnc[...,i].T @ np.linalg.inv(Tev[...,i].T)).T


    T.append( T_seg )
    TUnc.append( np.abs(TUnc_seg) )
    TSens.append( np.abs(TSens_seg) )

#    C.append(Cev)
    C.append(Ceb)

    sNom_seg, sUnc_seg = FreqTrans.Sigma(T_seg, TUnc_seg, typeSigma = 2) # Singular Value Decomp, U @ S @ Vh == T[...,i]
    sNom.append(sNom_seg)
    sUnc.append(sUnc_seg)


T_InputNames = sigExcList
T_OutputNames = sigFbList

# Compute Gain, Phase, Crit Distance
gain_mag = []
gainUnc_mag = []
phase_deg = []
rCritNom_mag = []
rCritUnc_mag = []
rCrit_mag = []
for iSeg in range(0, len(oDataSegs)):

    gainElem_mag, gainElemUnc_mag, gainElemDiff_mag = FreqTrans.DistCritCirc(T[iSeg], TUnc[iSeg])

    gain_mag.append(gainElem_mag)
    gainUnc_mag.append(gainElemUnc_mag)

    phase_deg.append(FreqTrans.Phase(T[iSeg], phaseUnit = 'deg', unwrap = True))

#    nom_mag, unc_mag, diff_mag = FreqTrans.DistCrit(T[iSeg], TUnc[iSeg], typeUnc = 'ellipse')
    nom_mag, unc_mag, diff_mag = FreqTrans.DistCritCirc(T[iSeg], TUnc[iSeg])

    rCritNom_mag.append(nom_mag)
    rCritUnc_mag.append(unc_mag)
    rCrit_mag.append(diff_mag)


#%% Sigma Plot
fig = None
for iSeg in range(0, len(oDataSegs)):
    Cmin = np.min(np.min(C[iSeg], axis = 0), axis = 0)
    sNomMin = np.min(sNom[iSeg], axis=0)
    sCritMin = np.min(sUnc[iSeg], axis=0)
    sNomMinErr = sNomMin - sCritMin

    fig = FreqTrans.PlotSigma(freq_hz[0], sNomMin, err = sNomMinErr, coher_nd = Cmin, fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + '*-', label = oDataSegs[iSeg]['Desc'])
#    fig = FreqTrans.PlotSigma(freq_hz[0], sNomMin, coher_nd = Cmin, fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + '*-', label = oDataSegs[iSeg]['Desc'])
#    fig = FreqTrans.PlotSigma(freq_hz[0], sCritMin, coher_nd = Cmin, fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + 'o', label = oDataSegs[iSeg]['Desc'])
    fig = FreqTrans.PlotSigma(freq_hz[0], sNomMinErr, coher_nd = Cmin, fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + ':', label = oDataSegs[iSeg]['Desc'])

fig = FreqTrans.PlotSigma(freq_hz[0], 0.4 * np.ones_like(freq_hz[0]), fmt = '--r', fig = fig)

ax = fig.get_axes()
ax[0].set_xlim(0, 10)
ax[0].set_ylim(0, 1)


#%% Disk Margin Plots
inPlot = sigExcList # Elements of sigExcList
outPlot = sigFbList # Elements of sigFbList

if False:
    #%%
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):

            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotSigma(freq_hz[0], rCritNom_mag[iSeg][iOut, iIn], err = rCritUnc_mag[iSeg][iOut, iIn], coher_nd = C[iSeg][iOut, iIn], fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + '*-', label = oDataSegs[iSeg]['Desc'])

            fig = FreqTrans.PlotSigma(freq_hz[0], 0.4 * np.ones_like(freq_hz[0]), fig = fig, fmt = 'r--')
            fig.suptitle(inName + ' to ' + outName, size=20)

            ax = fig.get_axes()
            ax[0].set_ylim(0, 2)


#%% Nyquist Plots
if False:
    #%%
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):

            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotNyquist(T[iSeg][iOut, iIn], TUnc[iSeg][iOut, iIn], fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + '*', label = oDataSegs[iSeg]['Desc'])

            fig = FreqTrans.PlotNyquist(np.asarray([-1+ 0j]), TUnc = np.asarray([0.4 + 0.4j]), fig = fig, fmt = '*r', label = 'Critical Region')
            fig.suptitle(inName + ' to ' + outName, size=20)

            ax = fig.get_axes()
            ax[0].set_xlim(-3, 1)
            ax[0].set_ylim(-2, 2)


#%% Bode Plots
if False:
    #%%
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):

            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotBode(freq_hz[0], gain_mag[iSeg][iOut, iIn], phase_deg[iSeg][iOut, iIn], C[iSeg][iOut, iIn], gainUnc_mag = gainUnc_mag[iSeg][iOut, iIn], fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + '*-', label = oDataSegs[iSeg]['Desc'])

            fig.suptitle(inName + ' to ' + outName, size=20)


#%% Noise Estimation
if False:
    #%%
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):

            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotBodeMag(freq_hz[0], gain_mag[iSeg][iOut, iIn], C[iSeg][iOut, iIn], fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + '*-', label = oDataSegs[iSeg]['Desc'])
                fig = FreqTrans.PlotBodeMag(freq_hz[0], gainUnc_mag[iSeg][iOut, iIn], fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + ':', label = oDataSegs[iSeg]['Desc'] + '_Noise')

            fig.suptitle(inName + ' to ' + outName, size=20)


#%% Turbulence
from Core import Environment

ft2m = 0.3049
m2ft = 1/ft2m

b_ft = 4
levelList = ['light', 'moderate', 'severe']
freqTurb_rps = np.sort(freqNull_rps)


if False:
    #%%
    for iOut, outName in enumerate(outPlot):
        plt.figure()

        for iSeg in range(0, len(oDataSegs)):
            plt.loglog(freq_hz[0], TSens[iSeg][iOut, 0], '*', label = oDataSegs[iSeg]['Desc'])

            V_mps = np.mean(seg['vIas_mps'])
            V_fps = V_mps * m2ft

            h_m = np.mean(seg['altBaro_m'])
            h_ft = h_m * m2ft

        for iLevel, level in enumerate(levelList):
            sigma = Environment.TurbIntensity(h_ft, level = level)
            L_ft = Environment.TurbLengthScale(h_ft)

            Puvw_VonKarman =  Environment.TurbSpectVonKarman(sigma, L_ft, freqTurb_rps / V_fps) * V_fps
            Ppqr_VonKarman =  Environment.TurbSpectRate(Puvw_VonKarman, sigma, L_ft, freqTurb_rps, V_fps, b_ft)

            plt.loglog(freqTurb_rps*rps2hz, np.abs(Ppqr_VonKarman[iOut]), label = "VonKarman - Level: " + level)

        plt.grid(True)
        plt.xlim([0.397, 10])
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Power Spectrum')
        plt.legend()

#%% Spectrograms of Output and Disturbances
if False:
#%%
    iSgnlExc = 0
    iSgnlOut = 0

    freqRate_rps = 50 * hz2rps
    optSpec = FreqTrans.OptSpect(dftType = 'dftmat', freq = freqExc_rps[iSgnlExc], freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear')
    optSpecN = FreqTrans.OptSpect(dftType = 'dftmat', freq = freqNull_rps, freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear')


    for iSeg in range(0, len(oDataSegs)):
        t = oDataSegs[iSeg]['time_s']
        x = vExcList[iSeg][iSgnlExc]
        y = vFbList[iSeg][iSgnlOut]

        # Number of time segments and length of overlap, units of samples
        #lenSeg = 2**6 - 1
        lenSeg = int(1.0 * optSpec.freqRate * rps2hz)
        lenOverlap = 1

        # Compute Spectrum over time
        tSpecY_s, freqSpecY_rps, P_Y_mag = FreqTrans.SpectTime(t, y, lenSeg, lenOverlap, optSpec)
        tSpecN_s, freqSpecN_rps, P_N_mag = FreqTrans.SpectTime(t, y, lenSeg, lenOverlap, optSpecN)

        # Plot the Spectrogram
        fig = FreqTrans.Spectogram(tSpecY_s, freqSpecY_rps * rps2hz, 20 * np.log10(P_Y_mag))
        fig.suptitle(oDataSegs[iSeg]['Desc'] + ': Spectrogram - ' + sigFbList[iSgnlOut])

        fig = FreqTrans.Spectogram(tSpecN_s, freqSpecN_rps * rps2hz, 20 * np.log10(P_N_mag))
        fig.suptitle(oDataSegs[iSeg]['Desc'] + ': Spectrogram Null - ' + sigFbList[iSgnlOut])

