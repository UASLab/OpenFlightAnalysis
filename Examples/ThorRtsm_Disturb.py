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


plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Palatino"],
    "font.size": 10
})

# Constants
hz2rps = 2 * np.pi
rps2hz = 1 / hz2rps


#%% File Lists
import os.path as path

pathBase = path.join('/home', 'rega0051', 'FlightArchive', 'Thor')
#pathBase = path.join('G:', 'Shared drives', 'UAVLab', 'Flight Data', 'Thor')

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
        # {'flt': 'FLT126', 'seg': ('time_us', [875171956 , 887171956], 'FLT126 - RTSM - Nominal Gain, 4 deg amp'), 'color': 'k'},
        # {'flt': 'FLT126', 'seg': ('time_us', [829130591 , 841130591], 'FLT126 - RTSM Route - Nominal Gain, 4 deg amp'), 'color': 'k'},
        # {'flt': 'FLT127', 'seg': ('time_us', [641655909 , 653655909], 'FLT127 - RTSM Route - Nominal Gain, 4 deg amp'), 'color': 'k'}, # Yaw controller in-op??
        # {'flt': 'FLT128', 'seg': ('time_us', [700263746 , 712263746 ], 'FLT128 - RTSM Route - Nominal Gain, 4 deg amp'), 'color': 'k'}, # Interesting Roll Margin vs. Uncertainty
        # {'flt': 'FLT128', 'seg': ('time_us', [831753831 , 843753831 ], 'FLT128 - RTSM Route - Nominal Gain, 4 deg amp'), 'color': 'k'},
        # {'flt': 'FLT128', 'seg': ('time_us', [ 959859721 , 971859721 ], 'FLT128 - RTSM Route - Nominal Gain, 4 deg amp'), 'color': 'k'}, # Not good

        # {'flt': 'FLT126', 'seg': ('time_us', [928833763 , 940833763], 'FLT126 - RTSM Large - Nominal Gain, 8 deg amp'), 'color': 'r'},
        # {'flt': 'FLT127', 'seg': ('time_us', [698755386 , 707255278], 'FLT127 - RTSM Large Route - Nominal Gain, 8 deg amp'), 'color': 'r'}, # Yaw controller in-op??
        # {'flt': 'FLT128', 'seg': ('time_us', [779830919 , 791830919 ], 'FLT128 - RTSM Large Route - Nominal Gain, 8 deg amp'), 'color': 'r'},
        # {'flt': 'FLT128', 'seg': ('time_us', [900237086 , 912237086 ], 'FLT128 - RTSM Large Route - Nominal Gain, 8 deg amp'), 'color': 'r'},

        # {'flt': 'FLT126', 'seg': ('time_us', [902952886 , 924952886], 'FLT126 - RTSM Long - Nominal Gain, 4 deg amp'), 'color': 'b'},
        # {'flt': 'FLT127', 'seg': ('time_us', [657015836 , 689015836], 'FLT127 - RTSM Long Route - Nominal Gain, 4 deg amp'), 'color': 'b'}, # Yaw controller in-op??
        {'flt': 'FLT128', 'seg': ('time_us', [714385469 , 746385469 ], 'FLT128 - RTSM Long Route - Nominal Gain, 4 deg amp'), 'color': 'b'},
        {'flt': 'FLT128', 'seg': ('time_us', [847254621 , 879254621 ], 'FLT128 - RTSM Long Route - Nominal Gain, 4 deg amp'), 'color': 'g'}, # Best

        # {'flt': 'FLT127', 'seg': ('time_us', [1209355236 , 1221535868], 'FLT127 - RTSM LongLarge Route - Nominal Gain, 8 deg amp'), 'color': 'm'}, # Yaw controller in-op??
        {'flt': 'FLT128', 'seg': ('time_us', [794251787 , 826251787 ], 'FLT128 - RTSM LongLarge Route - Nominal Gain, 8 deg amp'), 'color': 'r'},
        {'flt': 'FLT128', 'seg': ('time_us', [921438015 , 953438015 ], 'FLT128 - RTSM LongLarge Route - Nominal Gain, 8 deg amp'), 'color': 'm'},

        # {'flt': 'FLT126', 'seg': ('time_us', [981115495 , 993115495], 'FLT126 - RTSM - High Gain, 4 deg amp')},
        # {'flt': 'FLT126', 'seg': ('time_us', [689907125 , 711907125], 'FLT126 - RTSM Long - High Gain, 4 deg amp')},
        # {'flt': 'FLT126', 'seg': ('time_us', [728048050 , 740048050], 'FLT126 - RTSM Large - High Gain, 8 deg amp')},
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
        vFb[iSig] = -seg[sigFb]
        # vFb[iSig][1:-1] = -seg[sigFb][0:-2] # Shift the time of the output into next frame
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
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear')

# Excited Frequencies per input channel
optSpec.freq = np.asarray(freqExc_rps)
optSpec.freqInterp = np.sort(optSpec.freq.flatten())

# Null Frequencies
freqNull_rps = optSpec.freqInterp[0:-1] + 0.5 * np.diff(optSpec.freqInterp)
optSpec.freqNull = freqNull_rps
optSpec.freqNullInterp = True

# FRF Estimate
TaEstNomList = []
TaEstUncList = []
TaEstCohList = []
SaEstNomList = []
SaEstUncList = []
SaEstCohList = []
LaEstNomList = []
LaEstUncList = []
LaEstCohList = []

for iSeg, seg in enumerate(oDataSegs):

    freq_rps, Txy, Cxy, Sxx, Syy, Sxy, TxyUnc, SxxNull, Snn = FreqTrans.FreqRespFuncEstNoise(vExcList[iSeg], vFbList[iSeg], optSpec)
    freq_hz = freq_rps * rps2hz

    TaEstNom = -Txy # Sa = I - Ta
    TaEstUnc = TxyUnc # TxuUnc = np.abs(Sxu / Sxx)
    TaEstCoh = Cxy # Cxy = np.abs(Sxu)**2 / (Sxx * Suu)

    SaEstNom, SaEstUnc, SaEstCoh = FreqTrans.TtoS(TaEstNom, TaEstUnc, TaEstCoh)
    LaEstNom, LaEstUnc, LaEstCoh = FreqTrans.StoL(SaEstNom, SaEstUnc, SaEstCoh)

    TaEstNomList.append( TaEstNom )
    TaEstUncList.append( TaEstUnc )
    TaEstCohList.append( TaEstCoh )
    SaEstNomList.append( SaEstNom )
    SaEstUncList.append( SaEstUnc )
    SaEstCohList.append( SaEstCoh )
    LaEstNomList.append( LaEstNom )
    LaEstUncList.append( LaEstUnc )
    LaEstCohList.append( LaEstCoh )

    print(np.sum(SxxNull, axis = -1) / np.sum(Sxx, axis = -1))

T_InputNames = sigExcList
T_OutputNames = sigFbList

# Compute Gain, Phase, Crit Distance

#%% Sigma Plot
svLaEstNomList = []
svLaEstUncList = []

for iSeg in range(0, len(oDataSegs)):
    # I3 = np.repeat([np.eye(3)], SaEstNomList.shape[-1], axis=0).T
    # svLaEstNom_mag = FreqTrans.Sigma( I3 + LaEstNomList[iSeg] ) # Singular Value Decomp
    svLaEstNom_mag = 1 / FreqTrans.Sigma(SaEstNomList[iSeg]) # sv(I + La) = 1 / sv(Sa)
    svLaEstUnc_mag = FreqTrans.Sigma( LaEstUncList[iSeg] ) # Singular Value Decomp
    
    svLaEstNomList.append(svLaEstNom_mag)
    svLaEstUncList.append(svLaEstUnc_mag)
    
if True:
    fig = None
    for iSeg in range(0, len(oDataSegs)):
        cohLaEst = LaEstCohList[iSeg]
        # cohLaEstMin = np.min(cohLaEst, axis = (0,1))
        cohLaEstMin = np.mean(cohLaEst, axis = (0,1))

        svNom = svLaEstNomList[iSeg]
        svNomMin = np.min(svNom, axis=0)
        svUnc = svLaEstUncList[iSeg]
        svUncMax = np.max(svUnc, axis=0)

        svUncLower = svNomMin - svUncMax
        svUncLower[svUncLower < 0] = svNomMin[svUncLower < 0]

        fig = FreqTrans.PlotSigma(freq_hz[0], svNomMin, svUnc_mag = svUncLower, coher_nd = cohLaEstMin, fig = fig, color = rtsmSegList[iSeg]['color'], linestyle = '-', label = oDataSegs[iSeg]['Desc'])

    fig = FreqTrans.PlotSigma(freq_hz[0], 0.4 * np.ones_like(freq_hz[0]), color = 'r', linestyle = '--', fig = fig)

    ax = fig.get_axes()
    ax[0].set_xlim(0, 10)
    ax[0].set_ylim(0, 1)


#%% Vector Margin Plots
inPlot = ['$p_{ex}$', '$q_{ex}$', '$r_{ex}$'] # Elements of sigExcList
outPlot = ['$p_{fb}$', '$q_{fb}$', '$r_{fb}$'] # Elements of sigFbList

vmLaEstNomList_mag = []
vmLaEstUncList_mag = []

for iSeg in range(0, len(oDataSegs)):

    vm_mag, vmUnc_mag, vmMin_mag = FreqTrans.VectorMargin(LaEstNomList[iSeg], LaEstUncList[iSeg], typeUnc = 'circle')

    vmLaEstNomList_mag.append(vm_mag)
    vmLaEstUncList_mag.append(vmUnc_mag)
    # vm_mag.append(vmMin_mag)

numOut = len(outPlot); numIn = len(inPlot)
ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

if False:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 10 + iPlot
        for iSeg in range(0, len(oDataSegs)):
            
            vm_mag = vmLaEstNomList_mag[iSeg][iOut, iIn]
            vmUnc_mag = vmLaEstUncList_mag[iSeg][iOut, iIn]
            
            fig = FreqTrans.PlotVectorMargin(freq_hz[iIn], vm_mag, vmUnc_mag = vmUnc_mag, coher_nd = LaEstCohList[iSeg][iOut, iIn], fig = fig, color = rtsmSegList[iSeg]['color'], label = oDataSegs[iSeg]['Desc'])

        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn], 0.4 * np.ones_like(freq_hz[iIn]), fig = fig, color = 'r', linestyle = '--', label = 'Critical')
        fig.suptitle(inPlot[iIn] + ' to ' + outPlot[iOut])

        ax = fig.get_axes()
        ax[0].set_ylim(0, 2)


#%% Nyquist Plots
if False:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 20 + iPlot
        for iSeg in range(0, len(oDataSegs)):
            Tnom = LaEstNomList[iSeg][iOut, iIn]
            Tunc = np.abs(LaEstUncList[iSeg][iOut, iIn])
            fig = FreqTrans.PlotNyquist(Tnom, Tunc, fig = fig, color = rtsmSegList[iSeg]['color'], marker = '.', label = oDataSegs[iSeg]['Desc'])

        fig = FreqTrans.PlotNyquist(np.asarray([-1+ 0j]), TUnc = np.asarray([0.4 + 0.4j]), fig = fig, color = 'r', marker = '+', label = 'Critical Region')
        fig.suptitle(inPlot[iIn] + ' to ' + outPlot[iOut])

        ax = fig.get_axes()
        ax[0].set_xlim(-3, 1)
        ax[0].set_ylim(-2, 2)


#%% Bode Plots
gainLaEstNomList_mag = []
gainLaEstUncList_mag = []
phaseLaEstNomList_deg = []
for iSeg in range(0, len(oDataSegs)):
    gainLaEstNom_mag, phaseLaEstNom_deg = FreqTrans.GainPhase(LaEstNomList[iSeg], magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
    gainLaEstUnc_mag = FreqTrans.Gain(LaEstNomList[iSeg], magUnit = 'mag')

    gainLaEstNomList_mag.append(gainLaEstNom_mag)
    phaseLaEstNomList_deg.append(phaseLaEstNom_deg)
    gainLaEstUncList_mag.append(gainLaEstUnc_mag)
    
if False:
    
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 20 + iPlot
        for iSeg in range(0, len(oDataSegs)):
            gain_mag = gainLaEstNomList_mag[iSeg][iOut, iIn]
            phase_deg = phaseLaEstNomList_deg[iSeg][iOut, iIn]
            coher_nd = LaEstCohList[iSeg][iOut, iIn]
            gainUnc_mag = gainLaEstUncList_mag[iSeg][iOut, iIn]

            fig = FreqTrans.PlotBode(freq_hz[iIn], gain_mag, phase_deg, coher_nd, gainUnc_mag, fig = fig, dB = True, color = rtsmSegList[iSeg]['color'], label = oDataSegs[iSeg]['Desc'])

#        fig.suptitle(inName + ' to ' + outName, size=20)


#%% Turbulence
optSpecE = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 7), winType=('tukey', 0.0))
optSpecE.freq = np.asarray(freqExc_rps)
optSpecE.freqInterp = np.sort(optSpecE.freq.flatten())

optSpecN = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 7), winType=('tukey', 0.0))
optSpecN.freq = freqNull_rps


SyyList = []
for iSeg in range(0, len(oDataSegs)):
    _, _, SyyNull = FreqTrans.Spectrum(ySensList[iSeg], optSpecN)
    SyyList.append(SyyNull)
    
    _, _, Sxx = FreqTrans.Spectrum(vExcList[iSeg], optSpec)
    _, _, SxxNull = FreqTrans.Spectrum(vExcList[iSeg], optSpecN)
    
    print(np.sum(SxxNull, axis = -1) / np.sum(Sxx, axis = -1))


from Core import Environment

ft2m = 0.3049
m2ft = 1/ft2m

b_ft = 4
# levelList = ['light', 'moderate', 'severe']
levelList = ['light', 'moderate']
freqTurb_rps = np.sort(freqNull_rps)


if False:
    # for iOut, outName in enumerate(outPlot):
    for iOut, outName in enumerate(outPlot[0:1]):
        plt.figure()

        for iSeg in range(0, len(oDataSegs)):
            plt.loglog(freqTurb_rps*rps2hz, SyyList[iSeg][iOut], marker='.', linestyle='None', color = rtsmSegList[iSeg]['color'], label = oDataSegs[iSeg]['Desc'])

            V_mps = np.mean(seg['vIas_mps'])
            V_fps = V_mps * m2ft

            h_m = np.mean(seg['altBaro_m'])
            h_ft = h_m * m2ft

        for iLevel, level in enumerate(levelList):
            sigma = Environment.TurbIntensity(h_ft, level = level)
            L_ft = Environment.TurbLengthScale(h_ft)

            Puvw_Dryden =  Environment.TurbSpectDryden(sigma, L_ft, freqTurb_rps / V_fps) * V_fps
            Ppqr_Dryden =  Environment.TurbSpectRate(Puvw_Dryden, sigma, L_ft, freqTurb_rps, V_fps, b_ft)

#            plt.loglog(freqTurb_rps*rps2hz, np.abs(Ppqr_Dryden[iOut]), label = "Dryden - Level: " + level)

            Puvw_VonKarman =  Environment.TurbSpectVonKarman(sigma, L_ft, freqTurb_rps / V_fps) * V_fps
            Ppqr_VonKarman =  Environment.TurbSpectRate(Puvw_VonKarman, sigma, L_ft, freqTurb_rps, V_fps, b_ft)

            plt.loglog(freqTurb_rps*rps2hz, np.abs(Ppqr_VonKarman[iOut]), label = "VonKarman - Level: " + level)

        plt.grid(True)
        plt.xlim([0.1, 10])
#        plt.title('Disturbance Estimate - ' + outName)
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Power Spectrum (dB)')
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

      
        
#%% Estimate the frequency response function time history
iSeg = 3
        
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.2), detrendType = 'Linear')

# Excited Frequencies per input channel
optSpec.freq = np.asarray(freqExc_rps)
optSpec.freqInterp = np.sort(optSpec.freq.flatten())

# Null Frequencies
freqNull_rps = optSpec.freqInterp[0:-1] + 0.5 * np.diff(optSpec.freqInterp)
optSpec.freqNull = freqNull_rps
optSpec.freqNullInterp = True

# FRF Estimate
time_s = seg['time_s']
lenX = len(time_s)
lenFreq = optSpec.freqInterp.shape[-1]
lenStep = 1

numSec = int((lenX) / lenStep)
numOut = 3
numIn = 3


TaEstNomHist = np.zeros((numSec, numOut, numIn, lenFreq), dtype=complex)
TaEstUncHist = np.zeros((numSec, numOut, numIn, lenFreq), dtype=complex)
TaEstCohHist = np.zeros((numSec, numOut, numIn, lenFreq))
SaEstNomHist = np.zeros((numSec, numOut, numIn, lenFreq), dtype=complex)
SaEstUncHist = np.zeros((numSec, numOut, numIn, lenFreq), dtype=complex)
SaEstCohHist = np.zeros((numSec, numOut, numIn, lenFreq))
LaEstNomHist = np.zeros((numSec, numOut, numIn, lenFreq), dtype=complex)
LaEstUncHist = np.zeros((numSec, numOut, numIn, lenFreq), dtype=complex)
LaEstCohHist = np.zeros((numSec, numOut, numIn, lenFreq))
SxxHist = np.zeros((numSec, numIn, lenFreq))
SzzHist = np.zeros((numSec, numOut, numIn, lenFreq))
SxxNullHist = np.zeros((numSec, numIn, lenFreq))
SnnHist = np.zeros((numSec, numOut, numIn, lenFreq))


iStart = 0
#lenCyc = int(len(time_s) / 3)
for iSec in range(0, numSec):
    print(100 * iSec/numSec)
    iEnd = iSec
    
    if iEnd > lenFreq-1:
        
        # Move the Start index once the End index has reached 2* cycle
#        if iEnd > (2 * lenCyc):
#            iStart = iEnd - lenCyc
                
        x =  vExcList[iSeg][:, iStart:iEnd+1]
        z = vFbList[iSeg][:, iStart:iEnd+1]
        
        freq_rps, Txy, Cxy, Sxx, Szz, Suz, TxyUnc, SxxNull, Snn = FreqTrans.FreqRespFuncEstNoise(x, z, optSpec)
        
        TaEstNom = -Txy # Sa = I - Ta
        TaEstUnc = TxyUnc # TxuUnc = np.abs(Sxu / Sxx)
        TaEstCoh = Cxy # Cxy = np.abs(Sxu)**2 / (Sxx * Sxx)
        
        SaEstNom, SaEstUnc, SaEstCoh = FreqTrans.TtoS(TaEstNom, TaEstUnc, TaEstCoh)
        LaEstNom, LaEstUnc, LaEstCoh = FreqTrans.StoL(SaEstNom, SaEstUnc, SaEstCoh)
        
        TaEstNomHist[iSec, ] = TaEstNom
        TaEstUncHist[iSec, ] = TaEstUnc
        TaEstCohHist[iSec, ] = TaEstCoh
        SaEstNomHist[iSec, ] = SaEstNom
        SaEstUncHist[iSec, ] = SaEstUnc
        SaEstCohHist[iSec, ] = SaEstCoh
        LaEstNomHist[iSec, ] = LaEstNom
        LaEstUncHist[iSec, ] = LaEstUnc
        LaEstCohHist[iSec, ] = LaEstCoh
        SxxHist[iSec, ] = Sxx
        SzzHist[iSec, ] = Szz
        SxxNullHist[iSec, ] = SxxNull
        SnnHist[iSec, ] = Snn
    
    
    freq_hz = freq_rps * rps2hz
    
    
#%%

FreqTrans.Sigma(SaEstNomHist[100,...])


def SigmaTemporal(THist):
    numSec, numOut, numIn, numFreq = THist.shape
    
    sHist = np.zeros((numSec, numOut, numFreq))
    
    for iSec in range(numSec):
        sHist[iSec, ...] = FreqTrans.Sigma(THist[iSec, ...])
    
    return sHist


svLaEstNom_mag = 1 / SigmaTemporal(SaEstNomHist)
svLaEstUnc_mag = SigmaTemporal(SaEstUncHist)

svLaEstNomMin = np.min(svLaEstNom_mag, axis=1)
svLaEstUncMax = np.max(svLaEstUnc_mag, axis=1)
    
if True:
    svLaEstNomMean = np.mean(svLaEstNomMin, axis=-1)
    svLaEstNomMin = np.min(svLaEstNomMin, axis=-1)
    svLaEstNomDiff = svLaEstNomMean - svLaEstNomMin
    
    svLaEstUncMean = np.mean(svLaEstUncMax, axis=-1)
    svLaEstUncMax = np.max(svLaEstUncMax, axis=-1)
    
    svLaEstLower = svLaEstNomMin - svLaEstUncMax
    svLaEstLower[svLaEstLower < 0] = svLaEstNomMin[svLaEstLower < 0]
    
    cohEst = np.abs(LaEstCohHist)
    cohEst[cohEst < 0] = 0
    cohEst[cohEst > 1] = 1
    cohEstMean = np.mean(cohEst, axis=(1,2,3))
    cohEstStd = np.std(cohEst, axis=(1,2,3))
    cohEstMin = np.min(cohEst, axis=(1,2,3))
    
    ones = np.ones_like(time_s)
    
    fig = None
    fig = FreqTrans.PlotSigmaTemporal(time_s, svLaEstNomMin, svUnc_mag = svLaEstLower, coher_nd = cohEstMin, fig = fig, linestyle='-', color='b', label = 'Estimate - Lowest Singular Value')
#    fig = FreqTrans.PlotSigmaTemporal(time_s, svLaEstUncMax, coher_nd = cohEstMin, fig = fig, linestyle='-', color='r', label = 'Estimate - Maximum Uncertainty')

#    fig = FreqTrans.PlotSigmaTemporal(time_s, svLaEstNomMean, coher_nd = cohEstMean, svUnc_mag = svLaEstNomDiff, fig = fig, linestyle='-', color='b', label = 'Estimate - Lowest Singular Value')
#    fig = FreqTrans.PlotSigmaTemporal(time_s, svLaEstUncMean, coher_nd = cohEstMin, svUnc_mag = svLaEstUncMax, fig = fig, linestyle='-', color='r', label = 'Estimate - Maximum Uncertainty')

    ax = fig.get_axes()
#    ax[0].set_ylim(bottom = -1, top = 3)

#        fig.suptitle('$u$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')
        
#%%

vmLaEstNom_mag, vmLaEstUnc_mag, vmLaEstMin_mag = FreqTrans.VectorMargin(LaEstNomHist, LaEstUncHist, typeUnc = 'circle')

if True:
    for iPlot, [iOut, iIn] in enumerate(ioArray):

        vmLaEstNomMin = np.min(vmLaEstNom_mag[:,iOut,iIn,:], axis=-1)
        vmLaEstUncMax = np.max(vmLaEstUnc_mag[:,iOut,iIn,:], axis=-1)
        vmLaEstLower = vmLaEstNomMin - vmLaEstUncMax
        vmLaEstLower[vmLaEstLower < 0] = vmLaEstNomMin[vmLaEstLower < 0]
        
        cohEst = np.abs(LaEstCohHist[:,iOut,iIn,:])
        cohEst[cohEst < 0] = 0
        cohEst[cohEst > 1] = 1
        cohEstMean = np.mean(cohEst, axis=-1)
        cohEstStd = np.std(cohEst, axis=-1)
        cohEstMin = np.min(cohEst, axis=-1)
        
        ones = np.ones_like(time_s)
        
        fig = None
        fig = FreqTrans.PlotVectorMarginTemporal(time_s, vmLaEstNomMin, coher_nd = cohEstMin, vmUnc_mag = vmLaEstLower, fig = fig, linestyle='-', color='r', label = 'Estimate')
        
        ax = fig.get_axes()
        ax[0].set_ylim(bottom = 0, top = 1.5)

        fig.suptitle('$u$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')
        
#    handles, labels = ax[0, 0].get_legend_handles_labels()
#    handles = [handles[0], handles[3], handles[1], handles[4], handles[5], handles[2]]
#    labels = [labels[0], labels[3], labels[1], labels[4], labels[5], labels[2]]
#    ax[0, 0].legend(handles, labels)

#%%
if True:
    for iPlot, [iOut, iIn] in enumerate(ioArray):

        # Best Case SNR can be estimated as the Null input to Excited input
        uSNR = np.abs(SxxHist[:,iIn,:]) / np.abs(SxxNullHist[:,iIn,:])
        uSNRMean = np.mean(uSNR, axis=-1)
        uSNRMin = np.min(uSNR, axis=-1)
        
        zSNR = np.abs(SzzHist[:,iOut, iIn,:]) / np.abs(SnnHist[:,iOut, iIn,:])
        zSNRMean = np.mean(zSNR, axis=-1)
        zSNRMin = np.min(zSNR, axis=-1)
#        zSNRMin[zSNRMin < 0] = 0
        
        cohEst = np.abs(TaEstCohHist[:,iOut,iIn,:])
        cohEst[cohEst < 0] = 0
        cohEst[cohEst > 1] = 1
        cohEstMean = np.mean(cohEst, axis=-1)
        cohEstMin = np.min(cohEst, axis=-1)
        
        fig = None
        ones = np.ones_like(time_s)
        fig = FreqTrans.PlotGainTemporal(time_s, zSNRMean, None, None, zSNRMin, fig = fig, dB = True, UncSide = 'Min', linestyle='-', color='r', label = 'Estimate of Output')
        fig = FreqTrans.PlotGainTemporal(time_s, uSNRMean, None, None, uSNRMin, fig = fig, dB = True, UncSide = 'Min', linestyle='-', color='k', label = 'Estimate of Input')
#        fig = FreqTrans.PlotGainTemporal(time_s, zSNRMean, None, cohEstMin, zSNRMin, fig = fig, dB = True, UncSide = 'Min', linestyle='-', color='r', label = 'Estimate of Output')
#        fig = FreqTrans.PlotGainTemporal(time_s, uSNRMean, None, cohEstMean, uSNRMin, fig = fig, dB = True, UncSide = 'Min', linestyle='-', color='k', label = 'Estimate of Input')

        ax = fig.get_axes()
#        ax[0].set_ylim(bottom = 0, top = 3)
        ax[0].set_ylabel("Signal/Noise [dB]")

        fig.suptitle('$u$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')
        
    