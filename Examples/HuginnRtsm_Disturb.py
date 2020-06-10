"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Analysis for Huginn (mAEWing2) FLT 03-06
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


#%%
from Core import FreqTrans

rtsmSegList = [
    {'flt': 'FLT03', 'seg': ('time_us', [693458513 , 705458513], 'FLT03 - 23 m/s'), 'fmt': 'k'}, # 23 m/s
    {'flt': 'FLT03', 'seg': ('time_us', [865877709 , 877877709], 'FLT03 - 20 m/s'), 'fmt': 'k'}, # 20 m/s

    {'flt': 'FLT04', 'seg': ('time_us', [884683583, 896683583], 'FLT04 - 26 m/s'), 'fmt': 'k'}, # 26 m/s
    {'flt': 'FLT04', 'seg': ('time_us', [998733748, 1010733748], 'FLT04 - 20 m/s'), 'fmt': 'k'}, # 20 m/s

    {'flt': 'FLT06', 'seg': ('time_us', [1122539650, 1134539650], 'FLT06 - 23 m/s'), 'fmt': 'k'}, # 23 m/s

    {'flt': 'FLT05', 'seg': ('time_us', [582408497, 594408497], 'FLT05 - 26 m/s'), 'fmt': 'b'}, # 26 m/s
    {'flt': 'FLT05', 'seg': ('time_us', [799488311, 811488311], 'FLT05 - 29 m/s'), 'fmt': 'g'}, # 29 m/s

    {'flt': 'FLT06', 'seg': ('time_us', [955822061, 967822061], 'FLT06 - 32 m/s'), 'fmt': 'm'}, # 32 m/s
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
    aZ = np.array([
            oData['aCenterFwdIMU_IMU_mps2'][2] - oData['aCenterFwdIMU_IMU_mps2'][2][0],
            oData['aCenterAftIMU_IMU_mps2'][2] - oData['aCenterAftIMU_IMU_mps2'][2][0],
            oData['aLeftMidIMU_IMU_mps2'][2] - oData['aLeftMidIMU_IMU_mps2'][2][0],
            oData['aLeftFwdIMU_IMU_mps2'][2] - oData['aLeftFwdIMU_IMU_mps2'][2][0],
            oData['aLeftAftIMU_IMU_mps2'][2] - oData['aLeftAftIMU_IMU_mps2'][2][0],
            oData['aRightMidIMU_IMU_mps2'][2] - oData['aRightMidIMU_IMU_mps2'][2][0],
            oData['aRightFwdIMU_IMU_mps2'][2] - oData['aRightFwdIMU_IMU_mps2'][2][0],
            oData['aRightAftIMU_IMU_mps2'][2] - oData['aRightAftIMU_IMU_mps2'][2][0]
            ])

    aCoefEta1 = np.array([456.1, -565.1, -289.9, 605.4, 472.4, -292, 605.6, 472.2]) * 0.3048
    measEta1 = aCoefEta1 @ (aZ - np.mean(aZ, axis = 0)) * 1/50 * 1e-3


    aCoefEta1dt = np.array([2.956, -2.998, -1.141, 5.974, 5.349, -1.149, 5.974, 5.348]) * 0.3048
    measEta1dt = aCoefEta1dt @ (aZ - np.mean(aZ, axis = 0)) * 1e-3

    # Added signals
    oData['cmdRoll_FF'] = h5Data['Control']['cmdRoll_PID_rpsFF']
    oData['cmdRoll_FB'] = h5Data['Control']['cmdRoll_PID_rpsFB']
    oData['cmdPitch_FF'] = h5Data['Control']['cmdPitch_PID_rpsFF']
    oData['cmdPitch_FB'] = h5Data['Control']['cmdPitch_PID_rpsFB']
    oData['cmdBend_FF'] = h5Data['Control']['refBend_nd'] # h5Data['Excitation']['Bend']['cmdBend_nd']
    oData['cmdBendDt_FB'] = measEta1dt
    oData['cmdBend_FB'] = measEta1

    # Segments
    seg = OpenData.Segment(oData, rtsmSeg['seg'])
    oDataSegs.append(seg)

#    plt.plot(seg['time_s'], seg['Control']['cmdBend_nd'], seg['time_s'], seg['cmdBendDt_FB'], seg['time_s'], seg['cmdBend_FB'])


#%%

sigExcList = ['cmdRoll_rps', 'cmdPitch_rps', 'cmdBend_nd']
sigFbList = ['cmdRoll_FB', 'cmdPitch_FB', 'cmdBend_FB']
sigFfList = ['cmdRoll_FF', 'cmdPitch_FF', 'cmdBend_FF']

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
        sigFf = sigFfList[iSig]

        vCmd[iSig] = seg['Control'][sigExc]
        vExc[iSig] = seg['Excitation'][sigExc]
        vFb[iSig][1:-1] = seg[sigFb][0:-2] # Shift the time of the output into next frame
        vFf[iSig] = seg[sigFf]

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

# Excited Frequencies per input channel
optSpec.freq = np.asarray(freqExc_rps)
optSpec.freqInterp = np.sort(optSpec.freq.flatten())

# Null Frequencies
freqGap_rps = optSpec.freq.flatten()[0:-1] + 0.5 * np.diff(optSpec.freq.flatten())
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
T = []
TUnc = []
C = []
for iSeg, seg in enumerate(oDataSegs):

    freq_rps, Teb, Ceb, Pee, Pbb, Peb, TebUnc, Pee_N, Pbb_N = FreqTrans.FreqRespFuncEstNoise(vExcList[iSeg], vFbList[iSeg], optSpec)
    _       , Tev, Cev, _  , Pvv, Pev, TevUnc, _    , Pvv_N = FreqTrans.FreqRespFuncEstNoise(vExcList[iSeg], vCmdList[iSeg], optSpec)

    freq_hz = freq_rps * rps2hz

    # Form the Frequency Response
    T_seg = np.empty_like(Tev)
    TUnc_seg = np.empty_like(Tev)

    for i in range(T_seg.shape[-1]):
        T_seg[...,i] = Teb[...,i] @ np.linalg.inv(Tev[...,i])
        TUnc_seg[...,i] = TebUnc[...,i] @ np.linalg.inv(Tev[...,i])


    T.append( T_seg )
    TUnc.append( np.abs(TUnc_seg) )

#    C.append(Cev)
    C.append(Ceb)


T_InputNames = sigExcList
T_OutputNames = sigFbList

# Compute Gain, Phase, Crit Distance
gain_mag = []
gainUnc_mag = []
phase_deg = []
rCritNom_mag = []
rCritUnc_mag = []
rCrit_mag = []
sNom = []
sUnc = []
for iSeg in range(0, len(oDataSegs)):

    gainElem_mag, gainElemUnc_mag, gainElemDiff_mag = FreqTrans.DistCritCirc(T[iSeg], TUnc[iSeg], pCrit = 0+0j, typeNorm = 'RSS')

    gain_mag.append(gainElem_mag)
    gainUnc_mag.append(gainElemUnc_mag)

    phase_deg.append(FreqTrans.Phase(T[iSeg], phaseUnit = 'deg', unwrap = True))

    nom_mag, unc_mag, diff_mag = FreqTrans.DistCritCirc(T[iSeg], TUnc[iSeg], pCrit = -1+0j, typeNorm = 'RSS')

    rCritNom_mag.append(nom_mag)
    rCritUnc_mag.append(unc_mag)
    rCrit_mag.append(diff_mag)

    sNom_seg, sUnc_seg = FreqTrans.Sigma(T[iSeg], TUnc[iSeg]) # Singular Value Decomp, U @ S @ Vh == T[...,i]
    sNom.append(sNom_seg)
    sUnc.append(sUnc_seg)


#%% Spectrograms
if False:
#%%
    iSgnlExc = 0
    iSgnlOut = 0

    freqRate_rps = 50 * hz2rps
    optSpec = FreqTrans.OptSpect(dftType = 'dftmat', freq = freqExc_rps[iSgnlExc], freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear')
    optSpecN = FreqTrans.OptSpect(dftType = 'dftmat', freq = freqGap_rps, freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear')


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


#%% Sigma Plot
fig = None
for iSeg in range(0, len(oDataSegs)):
    Cmin = np.min(np.min(C[iSeg], axis = 0), axis = 0)
    sNomMin = np.min(sNom[iSeg], axis=0)
    sCritMin = np.min(sUnc[iSeg], axis=0)
    sNomMinErr = sNomMin - sCritMin

    fig = FreqTrans.PlotSigma(freq_hz[0], sNomMin, err = sNomMinErr, coher_nd = Cmin, fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + '*-', label = oDataSegs[iSeg]['Desc'])

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

            fig = FreqTrans.PlotSigma(freq_hz[0], 0.4 * np.ones_like(freq_hz[0]), fig = fig, fmt = 'r--', label = 'Critical Limit')
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
#                fig = FreqTrans.PlotBode(freq_hz[0], gain_mag[iSeg][iOut, iIn], phase_deg[iSeg][iOut, iIn], C[iSeg][iOut, iIn], gainUnc_mag = gainUnc_mag[iSeg][iOut, iIn], fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + '*-', label = oDataSegs[iSeg]['Desc'])
                fig = FreqTrans.PlotBode(freq_hz[0], gain_mag[iSeg][iOut, iIn], phase_deg[iSeg][iOut, iIn], C[iSeg][iOut, iIn], fig = fig, fmt = rtsmSegList[iSeg]['fmt'] + '*-', label = oDataSegs[iSeg]['Desc'])

            fig.suptitle(inName + ' to ' + outName, size=20)
