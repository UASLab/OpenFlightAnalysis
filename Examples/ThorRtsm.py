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


# plt.rcParams.update({
#     "text.usetex": True,
#     "font.family": "serif",
#     "font.serif": ["Palatino"],
#     "font.size": 10
# })

# Constants
hz2rps = 2 * np.pi
rps2hz = 1 / hz2rps


#%% File Lists
import os.path as path

# pathBase = path.join('/home', 'rega0051', 'FlightArchive', 'Thor')
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
#        {'flt': 'FLT126', 'seg': ('time_us', [875171956 , 887171956], 'FLT126 - RTSM - Nominal Gain, 4 deg amp'), 'color': 'k'},
#        {'flt': 'FLT126', 'seg': ('time_us', [829130591 , 841130591], 'FLT126 - RTSM Route - Nominal Gain, 4 deg amp'), 'color': 'k'},
#        {'flt': 'FLT127', 'seg': ('time_us', [641655909 , 653655909], 'FLT127 - RTSM Route - Nominal Gain, 4 deg amp'), 'color': 'k'}, # Yaw controller in-op??
#        {'flt': 'FLT128', 'seg': ('time_us', [700263746 , 712263746 ], 'FLT128 - RTSM Route - Nominal Gain, 4 deg amp'), 'color': 'k'}, # Interesting Roll Margin vs. Uncertainty
#        {'flt': 'FLT128', 'seg': ('time_us', [831753831 , 843753831 ], 'FLT128 - RTSM Route - Nominal Gain, 4 deg amp'), 'color': 'k'},
#        {'flt': 'FLT128', 'seg': ('time_us', [ 959859721 , 971859721 ], 'FLT128 - RTSM Route - Nominal Gain, 4 deg amp'), 'color': 'k'}, # Not good

#        {'flt': 'FLT126', 'seg': ('time_us', [928833763 , 940833763], 'FLT126 - RTSM Large - Nominal Gain, 8 deg amp'), 'color': 'r'},
#        {'flt': 'FLT127', 'seg': ('time_us', [698755386 , 707255278], 'FLT127 - RTSM Large Route - Nominal Gain, 8 deg amp'), 'color': 'r'}, # Yaw controller in-op??
#        {'flt': 'FLT128', 'seg': ('time_us', [779830919 , 791830919 ], 'FLT128 - RTSM Large Route - Nominal Gain, 8 deg amp'), 'color': 'r'},
#        {'flt': 'FLT128', 'seg': ('time_us', [900237086 , 912237086 ], 'FLT128 - RTSM Large Route - Nominal Gain, 8 deg amp'), 'color': 'r'},
#
#        {'flt': 'FLT126', 'seg': ('time_us', [902952886 , 924952886], 'FLT126 - RTSM Long - Nominal Gain, 4 deg amp'), 'color': 'b'},
#        {'flt': 'FLT127', 'seg': ('time_us', [657015836 , 689015836], 'FLT127 - RTSM Long Route - Nominal Gain, 4 deg amp'), 'color': 'b'}, # Yaw controller in-op??
#        {'flt': 'FLT128', 'seg': ('time_us', [714385469 , 746385469 ], 'FLT128 - RTSM Long Route - Nominal Gain, 4 deg amp'), 'color': 'b'},
        {'flt': 'FLT128', 'seg': ('time_us', [847254621 , 879254621 ], 'FLT128 - RTSM Long Route - Nominal Gain, 4 deg amp'), 'color': 'b'}, # Best

#        {'flt': 'FLT127', 'seg': ('time_us', [1209355236 , 1221535868], 'FLT127 - RTSM LongLarge Route - Nominal Gain, 8 deg amp'), 'color': 'm'}, # Yaw controller in-op??
#        {'flt': 'FLT128', 'seg': ('time_us', [794251787 , 826251787 ], 'FLT128 - RTSM LongLarge Route - Nominal Gain, 8 deg amp'), 'color': 'm'},
#        {'flt': 'FLT128', 'seg': ('time_us', [921438015 , 953438015 ], 'FLT128 - RTSM LongLarge Route - Nominal Gain, 8 deg amp'), 'color': 'm'},

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

# Excited Frequencies per input channel
optSpec.freq = np.asarray(freqExc_rps)

# FRF Estimate
LiEstNomList = []
LiEstCohList = []
svLiEstNomList = []
for iSeg, seg in enumerate(oDataSegs):

    freq_rps, Teb, Ceb, Pee, Pbb, Peb = FreqTrans.FreqRespFuncEst(vExcList[iSeg], vExcList[iSeg] + vFbList[iSeg], optSpec)
    # _       , Tev, Cev, _  , Pvv, Pev = FreqTrans.FreqRespFuncEst(vExcList[iSeg], vCmdList[iSeg], optSpec)

    freq_hz = freq_rps * rps2hz

    I3 = np.repeat([np.eye(3)], Teb.shape[-1], axis=0).T
    SaEstNom = Teb # Sa = I + Teb
    SaEstCoh = Ceb # Cxy = np.abs(Sxy)**2 / (Sxx * Syy) = (np.abs(Sxy) / Sxx) * (np.abs(Sxy) / Syy)

    # T = TNom = (uCtrl + uExc) / uExc - uNull / uExc
    # Li = inv(TNom + TUnc) - I = LiEstNom + LiEstUnc
    # LiEstNom = -I + TNom^-1
    # LiEstUnc = -(I + TNom^-1 * TUnc)^-1 * TNom^-1 * TUnc * TNom^-1
    LiEstNom = np.zeros_like(SaEstNom, dtype = complex)
    LiEstCoh = np.zeros_like(SaEstCoh)

    inv = np.linalg.inv

    for i in range(SaEstNom.shape[-1]):
        SaEstNomElem = SaEstNom[...,i]
        SaEstNomInvElem = inv(SaEstNomElem)

        LiEstNom[...,i] = -np.eye(3) + SaEstNomInvElem
        # LiEstCoh[...,i] = -np.eye(3) + inv(SaEstCoh[...,i])
        LiEstCoh[...,i] = SaEstCoh[...,i]

    LiEstNomList.append( LiEstNom )
    LiEstCohList.append( LiEstCoh )

    svLiEstNomList_seg = FreqTrans.Sigma( LiEstNom ) # Singular Value Decomp
    svLiEstNomList.append(svLiEstNomList_seg)


T_InputNames = sigExcList
T_OutputNames = sigFbList

# Compute Gain, Phase, Crit Distance
gainLiEstNomList_mag = []
phaseLiEstNomList_deg = []
rCritLiEstNomList_mag = []
for iSeg in range(0, len(oDataSegs)):

    gain_mag, phase_deg = FreqTrans.GainPhase(LiEstNomList[iSeg], magUnit = 'mag', phaseUnit = 'deg', unwrap = True)

    gainLiEstNomList_mag.append(gain_mag)
    phaseLiEstNomList_deg.append(phase_deg)

    # rCritLiEstNom_mag, _, _ = FreqTrans.DistCrit(LiEstNomList[iSeg], typeUnc = 'ellipse')
    rCritLiEstNom_mag, _, _ = FreqTrans.DistCritCirc(LiEstNomList[iSeg])

    rCritLiEstNomList_mag.append(rCritLiEstNom_mag)


#%% Sigma Plot
fig = None
for iSeg in range(0, len(oDataSegs)):
    Cmin = np.min(np.min(LiEstCohList[iSeg], axis = 0), axis = 0)
    sNomMin = np.min(svLiEstNomList[iSeg], axis=0)

    fig = FreqTrans.PlotSigma(freq_hz[0], svLiEstNomList[iSeg], coher_nd = Cmin, fig = fig, color = rtsmSegList[iSeg]['color'], linestyle = '-', label = oDataSegs[iSeg]['Desc'])

fig = FreqTrans.PlotSigma(freq_hz[0], 0.4 * np.ones_like(freq_hz[0]), color = 'r', linestyle = '--', fig = fig)

ax = fig.get_axes()
ax[0].set_xlim(0, 10)
# ax[0].set_ylim(0, 1)


#%% Disk Margin Plots
inPlot = sigExcList # Elements of sigExcList
outPlot = sigFbList # Elements of sigFbList

if False:
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):

            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotSigma(freq_hz[0], rCritLiEstNomList_mag[iSeg][iOut, iIn], coher_nd = LiEstCohList[iSeg][iOut, iIn], fig = fig, color = rtsmSegList[iSeg]['color'], linestyle = '-', label = oDataSegs[iSeg]['Desc'])

            fig = FreqTrans.PlotSigma(freq_hz[0], 0.4 * np.ones_like(freq_hz[0]), fig = fig, color = 'r', linestyle = '--')
            fig.suptitle(inName + ' to ' + outName, size=20)

            ax = fig.get_axes()
            # ax[0].set_ylim(0, 2)


#%% Nyquist Plots
if False:
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):

            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotNyquist(LiEstNomList[iSeg][iOut, iIn], fig = fig, color = rtsmSegList[iSeg]['color'], label = oDataSegs[iSeg]['Desc'])

            fig = FreqTrans.PlotNyquist(np.asarray([-1+ 0j]), TUnc = np.asarray([0.4 + 0.4j]), fig = fig, fmt = '*r', label = 'Critical Region')
            fig.suptitle(inName + ' to ' + outName, size=20)

            ax = fig.get_axes()
            ax[0].set_xlim(-3, 1)
            ax[0].set_ylim(-2, 2)


#%% Bode Plots
if False:
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):

            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotBode(freq_hz[0], gainLiEstNomList_mag[iSeg][iOut, iIn], phaseLiEstNomList_deg[iSeg][iOut, iIn], LiEstCohList[iSeg][iOut, iIn], fig = fig, color = rtsmSegList[iSeg]['color'], linestyle = '-', label = oDataSegs[iSeg]['Desc'])

            fig.suptitle(inName + ' to ' + outName, size=20)
