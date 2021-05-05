"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Example script for testing Frequency Response Estimation - MIMO.
"""

import numpy as np
import matplotlib.pyplot as plt
import control

# Hack to allow loading the Core package
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), "..")))
    path.insert(0, abspath(join(dirname(argv[0]), "..", 'Core')))

    del path, argv, dirname, abspath, join

from Core import GenExcite
from Core import FreqTrans
from Core.Systems import ConnectName

# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps

rad2deg = 180/np.pi
deg2rad = 1/rad2deg


def weighting(wb, m, a):
    """weighting(wb,m,a) -> wf
    wb - design frequency (where |wf| is approximately 1)
    m - high frequency gain of 1/wf; should be > 1
    a - low frequency gain of 1/wf; should be < 1
    wf - SISO LTI object
    """
    s = control.tf([1, 0], [1])
    return (s/m + wb) / (s + wb*a)


#%% Define a linear plant systems
if True:
    freqLin_hz = np.linspace(1e-1, 1e1, 200)
    freqRate_hz = 50

    plantK11 = 1.0 ; plantWn11 = 3 * hz2rps; plantD11 = 0.2;
    plantK12 = 0.5 ; plantWn12 = 5 * hz2rps; plantD12 = 0.3;
    plantK21 = 0.25; plantWn21 = 4 * hz2rps; plantD21 = 0.1;
    plantK22 = 1.0 ; plantWn22 = 6 * hz2rps; plantD22 = 0.4;
    
    sysPlant = control.tf([[[0, 0, plantK11 * plantWn11**2], [0, 0, plantK21 * plantWn21**2]],
                           [[0, 0, plantK12 * plantWn12**2], [0, 0, plantK22 * plantWn22**2]]],
                          [[[1, 2.0*plantD11*plantWn11, plantWn11**2], [1, 2.0*plantD21*plantWn21, plantWn21**2]],
                           [[1, 2.0*plantD12*plantWn12, plantWn12**2], [1, 2.0*plantD22*plantWn22, plantWn22**2]]])

    sysK = control.ss([], [], [], [[0.75, 0.25], [0.25, 0.5]])

elif False: # SP Example #3.8
    freqLin_hz = np.linspace(1e-2 * rps2hz, 1e2 * rps2hz, 200)
    freqRate_hz = 50

    plantK11 = 1.0; plantWn11 = np.sqrt(5);      plantD11 = 6 / (2 * plantWn11);
    plantK12 = 1.0; plantWn12 = 1.0 * plantWn11; plantD12 = 1.0 * plantD11;
    plantK21 = 1.0; plantWn21 = 1.0 * plantWn11; plantD21 = 1.0 * plantD11;
    plantK22 = 1.0; plantWn22 = 1.0 * plantWn11; plantD22 = 1.0 * plantD11;

    sysPlant = control.tf([[[0, 0, plantK11 * plantWn11**2], [0, 0, plantK12 * plantWn12**2]],
                           [[0, 0, plantK21 * plantWn21**2], [0, 0, plantK22 * plantWn22**2]]],
                          [[[1, 2.0*plantD11*plantWn11, plantWn11**2], [1, 2.0*plantD21*plantWn21, plantWn21**2]],
                           [[1, 2.0*plantD21*plantWn21, plantWn21**2], [1, 2.0*plantD22*plantWn22, plantWn22**2]]])

    # Based on Example 3.8 from Multivariable Feedback Control, Skogestad and Postlethwaite, 2nd Edition.
    wu = control.ss([], [], [], np.eye(2))
    wp1 = control.ss(weighting(wb = 0.25, m = 1.5, a = 1e-4))
    wp2 = control.ss(weighting(wb = 0.25, m = 1.5, a = 1e-4))
    wp = wp1.append(wp2)

    sysK, sysCL, (gam, rcond) = control.mixsyn(sysPlant, wp, wu)

else: # SP Section #3.7.1
    freqLin_hz = np.linspace(1e-1, 1e1, 200)
    freqRate_hz = 50

    plantWn = 10
    den = [1, 0.0, plantWn**2]
    sysPlant = control.tf([[[1, -1.0 * plantWn**2], [1.0 * plantWn, 1.0 * plantWn]],
                           [[-1.0 * plantWn, -1.0 * plantWn], [1, -1.0 * plantWn**2]]],
                          [[den, den], [den, den]])

    sysK = control.ss([], [], [], 1.0 * np.eye(2))

freqRate_rps = freqRate_hz * hz2rps
freqLin_rps = freqLin_hz * hz2rps

sysPlant.InputName = ['u1', 'u2']
sysPlant.OutputName = ['y1', 'y2']

sysK.InputName = ['e1', 'e2']
sysK.OutputName = ['uCtrl1', 'uCtrl2']


#%% Plant Connection with Disturbances
# Add the disturbances to the Plant model
sysSumD = control.ss([], [], [], [[1, 0, 1, 0], [0, 1, 0, 1]])
sysSumD.InputName = ['uCtrl1', 'uCtrl2', 'uExc1', 'uExc2']
sysSumD.OutputName = ['u1', 'u2']

sysSumN = control.ss([], [], [], [[1, 0, 1, 0], [0, 1, 0, 1]])
sysSumN.InputName = ['y1', 'y2', 'n1', 'n2']
sysSumN.OutputName = ['z1', 'z2']

# z = n + P * (u + uExc)
connectNames = sysPlant.InputName + sysPlant.OutputName
inKeep = sysSumD.InputName + sysSumN.InputName[2:]
outKeep = sysSumN.OutputName
sysPlantDist = ConnectName([sysSumD, sysPlant, sysSumN], connectNames, inKeep, outKeep)


#%% Control System Connection
# Reference Input: e = r - z
sysSumR = control.ss([], [], [], [[1, 0, -1, 0], [0, 1, 0, -1]])
sysSumR.InputName = ['r1', 'r2', 'z1', 'z2']
sysSumR.OutputName = ['e1', 'e2']

connectNames = sysK.InputName
inKeep = sysSumR.InputName
outKeep = sysK.OutputName
sysCtrl = ConnectName([sysSumR, sysK], connectNames, inKeep, outKeep)


# Reference Inputs
refK11 = (2/8) * np.abs(sysPlant.dcgain())[0, 0]; refWn11 = 6 * hz2rps; refD11 = 1.0;
refK12 = (0/8) * np.abs(sysPlant.dcgain())[0, 1]; refWn12 = 6 * hz2rps; refD12 = 1.0;
refK21 = (0/8) * np.abs(sysPlant.dcgain())[1, 0]; refWn21 = 6 * hz2rps; refD21 = 1.0;
refK22 = (2/8) * np.abs(sysPlant.dcgain())[1, 1]; refWn22 = 6 * hz2rps; refD22 = 1.0;
sysR = control.tf([[[refK11 * refWn11**2], [refK12 * refWn12**2]],
                   [[refK21 * refWn21**2], [refK22 * refWn22**2]]],
                  [[[1, 2*refD11*refWn11, refWn11**2], [1, 2*refD12*refWn12, refWn12**2]],
                   [[1, 2*refD21*refWn21, refWn21**2], [1, 2*refD22*refWn22, refWn22**2]]])


# Plant-Output Noise
noiseK11 = (2/8); noiseWn11 = 6 * hz2rps; noiseD11 = 0.1;
noiseK22 = (8/8); noiseWn22 = 4 * hz2rps; noiseD22 = 1.0;

sysN = control.tf([[[-noiseK11, 0, noiseK11 * noiseWn11**2], [0]],
                   [[0], [-noiseK22, 0, noiseK22 * noiseWn22**2]]],
                  [[[1, 2.0*noiseD11*noiseWn11, noiseWn11**2], [1]],
                   [[1], [1, 2.0*noiseD22*noiseWn22, noiseWn22**2]]])


#%% Linear Models
# Li = KG (connect via Plant Output [z], loop is open at [u])
connectNames = sysPlantDist.OutputName
inKeep = sysCtrl.InputName[0:2] + sysPlantDist.InputName
outKeep = sysCtrl.OutputName + sysPlantDist.OutputName
sysLoopIn = ConnectName([sysPlantDist, sysCtrl], connectNames, inKeep, outKeep)

# Closed-Loop (connect [uCntrl])
connectNames = sysLoopIn.OutputName[0:2]
inKeep = sysLoopIn.InputName[:2] + sysLoopIn.InputName[4:]
outKeep = sysLoopIn.OutputName
sysCL = ConnectName([sysLoopIn], connectNames, inKeep, outKeep)


# Loop Li - (uExc -> uCtrl)
sysLi = -sysLoopIn[0:2, 4:6] # Li = KG, u_e to u # FIXIT - YES Negative : sysLi.dcgain() = array([[0.5, 0.25], [0.125, 0.5]])
sysLi.InputName = sysLoopIn.InputName[4:6]
sysLi.OutputName = sysLoopIn.OutputName[0:2]

LiLinNom = FreqTrans.FreqResp(sysLi, freqLin_rps)

# Loop Ti - (uExc -> uCtrl)
sysTi = -sysCL[0:2, 2:4] # FIXIT - YES Negative : sysTi.dcgain() = array([[0.32394366, 0.11267606], [0.05633803, 0.32394366]])
sysTi.InputName = sysCL.InputName[2:4]
sysTi.OutputName = sysCL.OutputName[0:2]

TiLinNom = FreqTrans.FreqResp(sysTi, freqLin_rps)

sysSi = (np.eye(2) - sysTi)

pStd = [0.0, 0.0]
mStd = [1.0, 1.0]

pIn = 0.5 * control.ss([],[],[],np.diag(pStd))
mIn = 0.5 * control.ss([],[],[],np.diag(mStd))
sysTiUnc = sysSi * sysK * (sysR * pIn - sysN * mIn)


#%% Excitation
numExc = 2
numCycles = 3
ampInit = 1
ampFinal = 1
freqMinDes_rps = 0.1 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 10.1 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (10 / freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)

# Generate Schroeder MultiSine Signal
ampExcit_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
uExc, phaseElem_rad, sigExcit = GenExcite.MultiSine(freqExc_rps, ampExcit_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')
uStd = np.std(uExc)
uExc = uExc / uStd
uPeak = np.mean(GenExcite.PeakFactor(uExc) * np.std(uExc))**2

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]
freqChan_hz = freqChan_rps * rps2hz

# Null Frequencies
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)
freqGap_hz = freqGap_rps * rps2hz


#%% Simulate the excitation through the system
np.random.seed(0)

# Time Response
p = np.random.normal([0.0, 0.0], pStd, size = (len(time_s), 2)).T
_, r, _ = control.forced_response(sysR, T = time_s, U = p)

m = np.random.normal([0.0, 0.0], mStd, size = (len(time_s), 2)).T
_, n, _ = control.forced_response(sysN, T = time_s, U = m)

inCL = np.concatenate((r, uExc, n))

_, outCL, _ = control.forced_response(sysCL, T = time_s, U = inCL)
uCtrl = outCL[0:2]
z = outCL[2:4]


#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = 'bartlett', detrendType = 'linear')

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps
optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
freq_rps, Txu, Cxu, Sxx, Syy, Sxy, TxuUnc, SxxNull, Snn = FreqTrans.FreqRespFuncEstNoise(uExc, uCtrl, optSpec)
freq_hz = freq_rps * rps2hz

I2 = np.repeat([np.eye(2)], Txu.shape[-1], axis=0).T
TiEstNom = -Txu # Si = I - Ti
TiEstUnc = TxuUnc # TxuUnc = np.abs(Sxu / Sxx)
TiEstCoh = Cxu # Cxy = np.abs(Sxu)**2 / (Sxx * Syy)

SiEstNom, SiEstUnc, SiEstCoh = FreqTrans.TtoS(TiEstNom, TiEstUnc, TiEstCoh)
LiEstNom, LiEstUnc, LiEstCoh = FreqTrans.StoL(SiEstNom, SiEstUnc, SiEstCoh)

print(np.sum(SxxNull, axis = -1) / np.sum(Sxx, axis = -1))

TiEstSNR = np.abs(TiEstNom / TiEstUnc)**2
SiEstSNR = np.abs(SiEstNom / SiEstUnc)**2
LiEstSNR = np.abs(LiEstNom / LiEstUnc)**2


#%%
numFreq = len(freqExc_rps)
numTime = len(time_s)
scaleLinUnc = np.sqrt(numFreq / numTime)
TiLinUnc = scaleLinUnc * FreqTrans.FreqResp(sysTiUnc, freqLin_rps) # Noise STD scaled to Uncertainty by sqrt(numFreq)

I2 = np.repeat([np.eye(2)], TiLinNom.shape[-1], axis=0).T
SiLinNom = I2 - TiLinNom
SiLinUnc = TiLinUnc

LiLinUnc = np.zeros_like(TiLinNom, dtype = complex)
inv = np.linalg.inv
for i in range(TiLinNom.shape[-1]):
    SiLinNomElem = SiLinNom[...,i]
    SiLinUncElem = SiLinUnc[...,i]
    SiLinNomInvElem = inv(SiLinNomElem)
    
#    LiLinNom[...,i] = -np.eye(2) + SiLinNomElem
    LiLinUnc[...,i] = -inv(np.eye(2) + SiLinNomInvElem @ SiLinUncElem) @ SiLinNomInvElem @ SiLinUncElem @ SiLinNomInvElem

    

#%% Nyquist Plot - Output Complimentary Sensitivity Function
TiLinUncMag = np.abs(TiLinUnc)
TiEstUncMag = np.abs(TiEstUnc)

numOut, numIn = TiLinNom.shape[0:-1]
ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)
    
if True:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 10 + iPlot
        fig = FreqTrans.PlotNyquist(TiLinNom[iOut, iIn], TiLinUncMag[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear')
        fig = FreqTrans.PlotNyquist(TiEstNom[iOut, iIn], TiEstUncMag[iOut, iIn], fig = fig, fillType = 'circle', marker='.', color = 'b', linestyle='None', label = 'Estimate (MIMO)')
#        fig = FreqTrans.PlotNyquist(TiEstNom[iOut, iIn, sigIndx[iIn]], TiEstUncMag[iOut, iIn, sigIndx[iIn]], fig = fig, fillType = 'circle', marker='.', color = 'g', linestyle='None', label = 'Estimate (SIMO)')
        fig = FreqTrans.PlotNyquist(np.array([-1+0j]), np.array([0.4]), fig = fig, fillType = 'circle', marker='+', color = 'r', linestyle='None')

        ax = fig.get_axes()
#        handles, labels = ax[0].get_legend_handles_labels()
#        handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
#        labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
#        ax[0].legend(handles, labels)
    
        fig.suptitle('$T_i$ : ' + '$r_{Exc}$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')


#%% Bode Plot - Output Complimentary Sensitivity Function
# Linear Model Gain and Phase
gainTiLinNom_mag, phaseTiLinNom_deg = FreqTrans.GainPhase(TiLinNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTiLinUnc_mag = FreqTrans.Gain(TiLinUnc, magUnit = 'mag')
cohTiLin_mag = np.ones_like(TiLinUnc)

# Estimated
gainTiEstNom_mag, phaseTiEstNom_deg = FreqTrans.GainPhase(TiEstNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTiEstUnc_mag = FreqTrans.Gain(TiEstUnc, magUnit = 'mag') # Estimation Uncertain Response
cohTiEst_mag = TiEstCoh # Estimation Coherence
cohTiEstMin = np.min(cohTiEst_mag, axis = (0, 1))


if True:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 20 + iPlot
        fig = FreqTrans.PlotBode(freqLin_hz, gainTiLinNom_mag[iOut, iIn], phaseTiLinNom_deg[iOut, iIn], coher_nd = cohTiLin_mag[iOut, iIn], gainUnc_mag = None, fig = fig, dB = True, color='k', label='Linear Model')
        fig = FreqTrans.PlotBode(freqLin_hz, gainTiLinUnc_mag[iOut, iIn], None, coher_nd = None, gainUnc_mag = None, fig = fig, dB = True, color='k', linestyle = '--', label='Linear Model')

        fig = FreqTrans.PlotBode(freq_hz[iIn], gainTiEstNom_mag[iOut, iIn], phaseTiEstNom_deg[iOut, iIn], coher_nd = cohTiEst_mag[iOut, iIn], gainUnc_mag = None, fig = fig, dB = True, marker = '.', color='b', label='Estimate Nominal [MIMO]')
        fig = FreqTrans.PlotBode(freq_hz[iIn], gainTiEstUnc_mag[iOut, iIn], None, coher_nd = None, gainUnc_mag = None, fig = fig, dB = True, marker = '.', color='r', label='Estimate Uncertainty[MIMO]')

#        fig = FreqTrans.PlotBode(freq_hz[iIn, sigIndx[iIn]], gainTiEstNom_mag[iOut, iIn, sigIndx[iIn]], phaseTiEstNom_deg[iOut, iIn, sigIndx[iIn]], coher_nd = cohTiEst_mag[iOut, iIn, sigIndx[iIn]], gainUnc_mag = gainTiEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, dB = True, color='g', label='Estimate [SIMO]')
        
        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [(handles[0], handles[2]), (handles[1], handles[3])]
        labels = [labels[0], labels[2]]
        ax[0].legend(handles, labels)
    
        fig.suptitle('$T_i$ : ' + '$r_{Exc}$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')


#%% Sigma - Output Stability
I2 = np.repeat([np.eye(2)], LiLinNom.shape[-1], axis=0).T

# Linear Model SVD magnitude
svLiLinNom_mag = FreqTrans.Sigma(I2 + LiLinNom) # sigma(I + Lo)
svLiLinNomMin_mag = np.min(svLiLinNom_mag, axis=0)

svLiLinUnc_mag = FreqTrans.Sigma(LiLinUnc)
svLiLinUncMax_mag = np.max(svLiLinUnc_mag, axis=0) # Overly Conservative

cohLiLin_mag = np.ones_like(LiLinUnc)

# Estimate SVD magnitude
# svLiEstNom_mag = FreqTrans.Sigma(I2 + LiEstNom)
svLiEstNom_mag = 1 / FreqTrans.Sigma(SiEstNom) # sigma(I + Li) = 1 / sigma(Si)
svLiEstNomMin_mag = np.min(svLiEstNom_mag, axis=0)

svLiEstUnc_mag = FreqTrans.Sigma(LiEstUnc) # Uncertain SVD magnitude
svLiEstUncMax_mag = np.max(svLiEstUnc_mag, axis=0) # Overly Conservative

cohLiEst_mag = LiEstCoh
cohLiEstMin = np.min(cohLiEst_mag, axis = (0, 1)) # Estimation Coherence


if True:
    fig = 1
    fig = FreqTrans.PlotSigma(freqLin_hz, svLiLinNomMin_mag, np.ones_like(freqLin_hz), svLiLinUncMax_mag, fig = fig, color = 'k', label = 'Linear')
    fig = FreqTrans.PlotSigma(freq_hz[0], svLiEstNomMin_mag, cohLiEstMin, svLiEstUncMax_mag, marker='.', color = 'b', fig = fig, label = 'Estimate (MIMO)')
    fig = FreqTrans.PlotSigma(freqLin_hz, (0.4) * np.ones_like(freqLin_hz), linestyle = '--', color = 'r', fig = fig, label = 'Critical Limit')
    
    ax = fig.get_axes()
    handles, labels = ax[0].get_legend_handles_labels()
    handles = [handles[0], handles[3], handles[1], handles[4], handles[2]]
    labels = [labels[0], labels[3], labels[1], labels[4], labels[2]]
    ax[0].legend(handles, labels)
    
    fig.suptitle('$L_i$ : ' + '$r_{Exc}$' + ' to ' + '$z$')


#%% Vector Margin Plots - Output Stability
vmLiLinNom_mag, vmLiLinUnc_mag, vmLiLinMin_mag = FreqTrans.VectorMargin(LiLinNom, LiLinUnc, typeUnc = 'circle')
vmLiEstNom_mag, vmLiEstUnc_mag, vmLiEstMin_mag = FreqTrans.VectorMargin(LiEstNom, LiEstUnc, typeUnc = 'circle')

numOut, numIn = LiLinNom.shape[0:-1]
ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

if True:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 30 + iPlot
        fig = FreqTrans.PlotVectorMargin(freqLin_hz, vmLiLinNom_mag[iOut, iIn], cohLiLin_mag[iOut, iIn], vmLiLinUnc_mag[iOut, iIn], fig = fig, linestyle='-', color='k', label='Linear Model')
        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn], vmLiEstNom_mag[iOut, iIn], cohLiEst_mag[iOut, iIn], vmLiEstUnc_mag[iOut, iIn], fig = fig, linestyle='-', marker='.', color='b', label='Estimate [MIMO]')
#        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn, sigIndx[iIn]], vmLiEstNom_mag[iOut, iIn, sigIndx[iIn]], cohLiEst_mag[iOut, iIn, sigIndx[iIn]], vmLiEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, linestyle='None', marker='.', color='g', label='Estimate [SIMO]')
        
        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [(handles[0], handles[2]), (handles[1], handles[3])]
        labels = [labels[0], labels[2]]
        ax[0].legend(handles, labels)
    
        fig.suptitle('$L_i$ : ' + '$r_{Exc}$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')


#%% Nyquist Plot - Output Stability
LiLinUncMag = np.abs(LiLinUnc)
LiEstUncMag = np.abs(LiEstUnc)

if False:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 50 + iPlot
        fig = FreqTrans.PlotNyquist(LiLinNom[iOut, iIn], LiLinUncMag[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear')
        fig = FreqTrans.PlotNyquist(LiEstNom[iOut, iIn], LiEstUncMag[iOut, iIn], fig = fig, fillType = 'circle', marker='.', color = 'b', linestyle='None', label = 'Estimate (MIMO)')
#        fig = FreqTrans.PlotNyquist(LiEstNom[iOut, iIn, sigIndx[iIn]], LiEstUncMag[iOut, iIn, sigIndx[iIn]], fig = fig, fillType = 'circle', marker='.', color = 'g', linestyle='None', label = 'Estimate (SIMO)')
        fig = FreqTrans.PlotNyquist(np.array([-1+0j]), np.array([0.4]), fig = fig, fillType = 'circle', marker='+', color = 'r', linestyle='None')

        ax = fig.get_axes()
#        handles, labels = ax[0].get_legend_handles_labels()
#        handles = [(handles[0], handles[2]), (handles[1], handles[3])]
#        labels = [labels[0], labels[2]]
#        ax[0].legend(handles, labels)
    
        fig.suptitle('$L_i$ : ' + '$r_{Exc}$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')


#%% Bode Plot - Output Stability
# Linear Model Gain and Phase
gainLiLinNom_mag, phaseLiLinNom_deg = FreqTrans.GainPhase(LiLinNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainLiLinUnc_mag = FreqTrans.Gain(LiLinUnc, magUnit = 'mag')
cohLiLin_mag = np.ones_like(LiLinUnc)

# Estimated
gainLiEstNom_mag, phaseLiEstNom_deg = FreqTrans.GainPhase(LiEstNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainLiEstUnc_mag = FreqTrans.Gain(LiEstUnc, magUnit = 'mag') # Estimation Uncertain Response
cohLiEst_mag = LiEstCoh # Estimation Coherence
cohLiEstMin = np.min(cohLiEst_mag, axis = (0, 1))


if False:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 40 + iPlot
        fig = FreqTrans.PlotBode(freqLin_hz, gainLiLinNom_mag[iOut, iIn], phaseLiLinNom_deg[iOut, iIn], coher_nd = cohLiLin_mag[iOut, iIn], gainUnc_mag = gainLiLinUnc_mag[iOut, iIn], fig = fig, dB = False, color='k', label='Linear Model')
        fig = FreqTrans.PlotBode(freq_hz[iIn], gainLiEstNom_mag[iOut, iIn], phaseLiEstNom_deg[iOut, iIn], coher_nd = cohLiEst_mag[iOut, iIn], gainUnc_mag = gainLiEstUnc_mag[iOut, iIn], fig = fig, dB = False, color='g', label='Estimate')
#        fig = FreqTrans.PlotBode(freq_hz[iIn, sigIndx[iIn]], gainLiEstNom_mag[iOut, iIn, sigIndx[iIn]], phaseLiEstNom_deg[iOut, iIn, sigIndx[iIn]], coher_nd = cohLiEst_mag[iOut, iIn, sigIndx[iIn]], gainUnc_mag = gainLiEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, dB = True, color='g', label='Estimate [SIMO]')
        
        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [(handles[0], handles[2]), (handles[1], handles[3])]
        labels = [labels[0], labels[2]]
        ax[0].legend(handles, labels)
    
        fig.suptitle('$L_i$ : ' + '$r_{Exc}$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')

