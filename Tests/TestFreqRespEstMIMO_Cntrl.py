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
freqLin_hz = np.linspace(1e-1, 1e1, 200)
freqRate_hz = 50

plantK11 = 1.0 ; plantWn11 = 3 * hz2rps; plantD11 = 0.2;
plantK12 = 0.5 ; plantWn12 = 5 * hz2rps; plantD12 = 0.3;
plantK21 = 0.25; plantWn21 = 4 * hz2rps; plantD21 = 0.1;
plantK22 = 1.0 ; plantWn22 = 6 * hz2rps; plantD22 = 0.4;

sysPlant = control.tf([[[0, 0, plantK11 * plantWn11**2], [0, 0, plantK12 * plantWn12**2]],
                       [[0, 0, plantK21 * plantWn21**2], [0, 0, plantK22 * plantWn22**2]]],
                      [[[1, 2.0*plantD11*plantWn11, plantWn11**2], [1, 2.0*plantD12*plantWn12, plantWn12**2]],
                       [[1, 2.0*plantD21*plantWn21, plantWn21**2], [1, 2.0*plantD22*plantWn22, plantWn22**2]]])

sysK = control.ss([], [], [], np.diag([0.75, 0.5]))


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
noiseK11 = (1/8) * np.abs(sysPlant.dcgain())[0, 0]; noiseWn11 = 6 * hz2rps; noiseD11 = 0.1;
noiseK12 = (1/8) * np.abs(sysPlant.dcgain())[0, 1]; noiseWn12 = 6 * hz2rps; noiseD12 = 0.7;
noiseK21 = (4/8) * np.abs(sysPlant.dcgain())[1, 0]; noiseWn21 = 4 * hz2rps; noiseD21 = 0.7;
noiseK22 = (4/8) * np.abs(sysPlant.dcgain())[1, 1]; noiseWn22 = 4 * hz2rps; noiseD22 = 1.0;
sysN = control.tf([[[-noiseK11, 0, noiseK11 * noiseWn11**2], [-noiseK12, 0, noiseK12 * noiseWn12**2]],
                   [[-noiseK21, 0, noiseK21 * noiseWn21**2], [-noiseK22, 0, noiseK22 * noiseWn22**2]]],
                  [[[1, 2.0*noiseD11*noiseWn11, noiseWn11**2], [1, 2.0*noiseD12*noiseWn12, noiseWn12**2]],
                   [[1, 2.0*noiseD21*noiseWn21, noiseWn21**2], [1, 2.0*noiseD22*noiseWn22, noiseWn22**2]]])


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

gainLiLinNom_mag, phaseLiLinNom_rad, _ = control.freqresp(sysLi, omega = freqLin_rps)
LiLinNom = gainLiLinNom_mag * np.exp(1j * phaseLiLinNom_rad)
gainLiLinNom_dB = 20 * np.log10(gainLiLinNom_mag)
phaseLiLinNom_deg = np.unwrap(phaseLiLinNom_rad) * rad2deg


# Loop Ti - (uExc -> uCtrl)
sysTi = -sysCL[0:2, 2:4] # FIXIT - YES Negative : sysTi.dcgain() = array([[0.32394366, 0.11267606], [0.05633803, 0.32394366]])
sysTi.InputName = sysCL.InputName[2:4]
sysTi.OutputName = sysCL.OutputName[0:2]

gainTiLinNom_mag, phaseTiLinNom_rad, _ = control.freqresp(sysTi, omega = freqLin_rps)
TiLinNom = gainTiLinNom_mag * np.exp(1j * phaseTiLinNom_rad)
gainTiLinNom_dB = 20 * np.log10(gainTiLinNom_mag)
phaseTiLinNom_deg = np.unwrap(phaseTiLinNom_rad) * rad2deg

sysSi = np.eye(2) - sysTi

pSigma = [0.0, 0.0]
mSigma = [1.0, 1.0]

pIn = 0.5 * control.ss([],[],[],np.diag(pSigma))
mIn = 0.5 * control.ss([],[],[],np.diag(mSigma))
sysTiUnc = sysSi * sysK * (sysR * pIn - sysN * mIn)
gainTiLinUnc_mag, phaseTiLinUnc_rad, _ = control.freqresp(sysTiUnc, omega = freqLin_rps)
TiLinUnc = gainTiLinUnc_mag * np.exp(1j * phaseTiLinUnc_rad * deg2rad)

# Ti to Si, Si to Li
SiLinNom, SiLinUnc, _ = FreqTrans.TtoS(TiLinNom, TiLinUnc)
LiLinNom2, LiLinUnc, _ = FreqTrans.StoL(SiLinNom, SiLinUnc)
# Verification: LiLinNom2 - LiLinNom


# Sampling
nSamp = 50
numOut, numIn, nFreq = TiLinNom.shape

# Unstructured
shapeSamp = (numOut, numIn, nFreq, nSamp)

rSamp = np.sqrt(np.random.uniform(0, 1, shapeSamp))
phSamp = np.random.uniform(-np.pi, np.pi, shapeSamp)
samp = rSamp * np.exp(1j * phSamp)

TiLinSamp = np.zeros(shapeSamp, dtype='complex')
SiLinSamp = np.zeros(shapeSamp, dtype='complex')
LiLinSamp = np.zeros(shapeSamp, dtype='complex')
for iSamp in range(nSamp):
  TiLinSamp[..., iSamp] = TiLinNom + TiLinUnc * samp[..., iSamp]
  SiLinSamp[..., iSamp], _, _ = FreqTrans.TtoS(TiLinSamp[..., iSamp])
  LiLinSamp[..., iSamp], _, _ = FreqTrans.StoL(SiLinSamp[..., iSamp])


# Structured Sampling
shapeSampStruc = (nFreq, nSamp)

rSampStruc = np.sqrt(np.random.uniform(0, 1, shapeSampStruc))
phSampStruc = np.random.uniform(-np.pi, np.pi, shapeSampStruc)
sampStruc = rSampStruc * np.exp(1j * phSampStruc)

TiLinSampStruc = np.zeros(shapeSamp, dtype='complex')
SiLinSampStruc = np.zeros(shapeSamp, dtype='complex')
LiLinSampStruc = np.zeros(shapeSamp, dtype='complex')
for iSamp in range(nSamp):
  TiLinSampStruc[..., iSamp] = TiLinNom + TiLinUnc * sampStruc[..., iSamp]
  SiLinSampStruc[..., iSamp], _, _ = FreqTrans.TtoS(TiLinSampStruc[..., iSamp])
  LiLinSampStruc[..., iSamp], _, _ = FreqTrans.StoL(SiLinSampStruc[..., iSamp])



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
uExc = uExc / np.std(uExc)
uPeak = np.mean(GenExcite.PeakFactor(uExc) * np.std(uExc))**2

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]
freqChan_hz = freqChan_rps * rps2hz

# Null Frequencies
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)
freqGap_hz = freqGap_rps * rps2hz


#%% Simulate the excitation through the system
#np.random.seed(0)

# Time Response
p = np.random.normal([0.0, 0.0], pSigma, size = (len(time_s), 2)).T
_, r, _ = control.forced_response(sysR, T = time_s, U = p)

m = np.random.normal([0.0, 0.0], mSigma, size = (len(time_s), 2)).T
_, n, _ = control.forced_response(sysN, T = time_s, U = m)

inCL = np.concatenate((r, uExc, n))

_, outCL, _ = control.forced_response(sysCL, T = time_s, U = inCL)
uCtrl = outCL[0:2]
z = outCL[2:4]


#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'linear')

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps
optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
freq_rps, Txu, Cxu, Sxx, Suu, Sxy, TxuUnc, SxxNull, Snn = FreqTrans.FreqRespFuncEstNoise(uExc, uCtrl, optSpec)
freq_hz = freq_rps * rps2hz

I2 = np.repeat([np.eye(2)], Txu.shape[-1], axis=0).T
TiEstNom = -Txu # Si = I - Ti
TiEstUnc = TxuUnc # TxuUnc = np.abs(Sxu / Sxx)
TiEstCoh = Cxu # Cxy = np.abs(Sxu)**2 / (Sxx * Suu)

# Ti to Si, Si to Li
SiEstNom, SiEstUnc, SiEstCoh = FreqTrans.TtoS(TiEstNom, TiEstUnc, TiEstCoh)
LiEstNom, LiEstUnc, LiEstCoh = FreqTrans.StoL(SiEstNom, SiEstUnc, SiEstCoh)

# print(np.sum(SxxNull, axis = -1) / np.sum(Sxx, axis = -1))
# TiEstSNR = np.abs(TiEstNom / TiEstUnc)**2
# SiEstSNR = np.abs(SiEstNom / SiEstUnc)**2
# LiEstSNR = np.abs(LiEstNom / LiEstUnc)**2



#%% Nyquist Plot - Compl Sens Function
TiLinUncMag = np.abs(TiLinUnc)
TiEstUncMag = np.abs(TiEstUnc)

if False:
    numOut, numIn = TiLinNom.shape[0:-1]

    ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

    for iPlot, io in enumerate(ioArray):
        [iOut, iIn] = io

        fig = 10 + iPlot
        # Linear - Unstructured
        fig = FreqTrans.PlotNyquist(TiLinNom[iOut, iIn], TiLinUncMag[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear')
        # for iSamp in range(nSamp):
        #   fig = FreqTrans.PlotNyquist(TiLinSamp[iOut, iIn, :, iSamp], fig = fig, marker='.', color = 'gray', linestyle='None', label = 'Linear Unstructured Sample')

        # Linear - Structured (Same as unstructured)
        # fig = FreqTrans.PlotNyquist(TiLinNom[iOut, iIn], TiLinUncMag[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear')
        # for iSamp in range(nSamp):
        #   fig = FreqTrans.PlotNyquist(TiLinSampStruc[iOut, iIn, :, iSamp], fig = fig, marker='.', color = 'b', linestyle='None', label = 'Linear Structured Sample')

        fig = FreqTrans.PlotNyquist(TiEstNom[iOut, iIn], TiEstUncMag[iOut, iIn], fig = fig, fillType = 'circle', marker='.', color = 'b', linestyle='None', label = 'Estimate (MIMO)')
        fig = FreqTrans.PlotNyquist(TiEstNom[iOut, iIn, sigIndx[iIn]], TiEstUncMag[iOut, iIn, sigIndx[iIn]], fig = fig, fillType = 'circle', marker='.', color = 'g', linestyle='None', label = 'Estimate (SIMO)')

        fig = FreqTrans.PlotNyquist(np.array([-1+0j]), np.array([0.4]), fig = fig, fillType = 'circle', marker='+', color = 'r', linestyle='None', label = 'Critical')

        # ax = fig.get_axes()
        # handles, labels = ax[0].get_legend_handles_labels()
        # handles = [handles[0], handles[3], handles[1], handles[2], handles[3], handles[5]]
        # labels = [labels[0], labels[3], labels[1], labels[2], labels[3], labels[5]]
        # ax[0].legend(handles, labels)

        fig.suptitle('$T_i$ : ' + '$u_{Exc}$[' + str(iIn) + '] to ' + 'u[' + str(iOut) + ']')


#%% Bode Plot - Compl Sens Function
# Linear Model Gain and Phase
gainTiLinUnc_mag, phaseTiLinUnc_deg = FreqTrans.GainPhase(TiLinUnc, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
cohTiLin_mag = np.ones_like(TiLinUnc)

# Estimated
gainTiEstNom_mag, phaseTiEstNom_deg = FreqTrans.GainPhase(TiEstNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTiEstUnc_mag = FreqTrans.Gain(TiEstUnc, magUnit = 'mag') # Estimation Uncertain Response
cohTiEst_mag = TiEstCoh # Estimation Coherence
cohTiEstMin = np.min(cohTiEst_mag, axis = (0, 1))


if False:
    numOut, numIn = TiLinNom.shape[0:-1]
    ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

    for iPlot, io in enumerate(ioArray):
        [iOut, iIn] = io

        fig = 20 + iPlot
        fig = FreqTrans.PlotBode(freqLin_hz, gainTiLinNom_mag[iOut, iIn], phaseTiLinNom_deg[iOut, iIn], coher_nd = cohTiLin_mag[iOut, iIn], gainUnc_mag = gainTiLinUnc_mag[iOut, iIn], fig = fig, dB = True, color='k', label='Linear Model')
        fig = FreqTrans.PlotBode(freq_hz[iIn], gainTiEstNom_mag[iOut, iIn], phaseTiEstNom_deg[iOut, iIn], coher_nd = cohTiEst_mag[iOut, iIn], gainUnc_mag = gainTiEstUnc_mag[iOut, iIn], fig = fig, dB = True, color='b', label='Estimate [MIMO]')
        fig = FreqTrans.PlotBode(freq_hz[iIn, sigIndx[iIn]], gainTiEstNom_mag[iOut, iIn, sigIndx[iIn]], phaseTiEstNom_deg[iOut, iIn, sigIndx[iIn]], coher_nd = cohTiEst_mag[iOut, iIn, sigIndx[iIn]], gainUnc_mag = gainTiEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, dB = True, color='g', label='Estimate [SIMO]')

        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
        labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
        ax[0].legend(handles, labels)

        fig.suptitle('$T_i$ : ' + '$u_{Exc}$[' + str(iIn) + '] to ' + 'u[' + str(iOut) + ']')


#%% Sigma Plot - Compl Sens Function
I2 = np.repeat([np.eye(2)], len(freqLin_rps), axis=0).T

# Unstructured Singular Values
svTiLinNom_mag = FreqTrans.Sigma(TiLinNom)
svTiLinNomMin_mag = np.min(svTiLinNom_mag, axis=0)
svTiLinNomMax_mag = np.max(svTiLinNom_mag, axis=0)

svTiLinUnc_mag = FreqTrans.Sigma(TiLinUnc)
svTiLinUncMax_mag = np.max(svTiLinUnc_mag, axis=0) # Overly Conservative

cohTiLin_mag = np.ones_like(TiLinUnc)

# Structured Singular Values
MiLin = TiLinNom * TiLinUnc
DeltaLin = np.eye(2)
muMiLin_mag, MiLin_info = FreqTrans.SigmaStruct(MiLin, DeltaLin, bound = 'upper')
muMiLinMin_mag = svTiLinNomMin_mag - (muMiLin_mag * svTiLinUncMax_mag)

# Structured Singular Values (Additive)
svTiLinStructMin_mag, TiLinCritStruct = FreqTrans.SigmaStruct_Add(TiLinNom, TiLinUnc)


# Linear Sampled
svTiLinSampMin_mag = np.zeros((nFreq, nSamp))
svTiLinSampStrucMin_mag = np.zeros((nFreq, nSamp))
for iSamp in range(nSamp):
  svTiLinSampMin_mag[..., iSamp]      = svTiLinNomMin_mag - np.max(FreqTrans.Sigma(TiLinSamp[..., iSamp] - TiLinNom), axis=0)
  svTiLinSampStrucMin_mag[..., iSamp] = np.min(FreqTrans.Sigma(TiLinSampStruc[..., iSamp]), axis=0)

# Estimated
I2 = np.repeat([np.eye(2)], len(freq_hz[0]), axis=0).T

# Max singular value
svTiEstNom_mag = FreqTrans.Sigma(TiEstNom)
svTiEstNomMin_mag = np.min(svTiEstNom_mag, axis=0)

svTiEstUnc_mag = FreqTrans.Sigma(TiEstUnc)
svTiEstUncMax_mag = np.max(svTiEstUnc_mag, axis=0) # Overly Conservative

# Structured Singular Values (Additive)
svTiEstStructMin_mag, TiEstCritStruct = FreqTrans.SigmaStruct_Add(TiEstNom, TiEstUnc)

if False:
    fig = 1
    # Linear
    fig = FreqTrans.PlotSigma(freqLin_hz, svTiLinNomMin_mag, np.ones_like(freqLin_hz), svTiLinUncMax_mag, fig = fig, color = 'k', label = 'Linear w/Unstructured SV')
    # for iSamp in range(nSamp):
    #   fig = FreqTrans.PlotSigma(freqLin_hz, svTiLinSampMin_mag[..., iSamp], fig = fig, marker='.', color = 'gray', linestyle='None', label = 'Unstructured Samples')
    fig = FreqTrans.PlotSigma(freqLin_hz, np.min(svTiLinSampMin_mag, axis = -1), fig = fig, marker='*', color = 'gray', linestyle='None', label = 'Linear w/Unstructured Samples')

    fig = FreqTrans.PlotSigma(freqLin_hz, svTiLinStructMin_mag, marker='None', color = 'k', linestyle = ':', fig = fig, label = 'Linear w/Structured SV')
    # for iSamp in range(nSamp):
    #   fig = FreqTrans.PlotSigma(freqLin_hz, svTiLinSampStrucMin_mag[..., iSamp], fig = fig, marker='.', color = 'b', linestyle='None', label = 'Unstructured Samples')
    fig = FreqTrans.PlotSigma(freqLin_hz, np.min(svTiLinSampStrucMin_mag, axis = -1), fig = fig, marker='*', color = 'm', linestyle='None', label = 'Linear w/Structured Samples')

    # Estimated
    fig = FreqTrans.PlotSigma(freq_hz[0], svTiEstNomMin_mag, cohTiEstMin, svTiEstUncMax_mag, fig = fig, color = 'b', label = 'Estimate w/Unstructured SV')
    fig = FreqTrans.PlotSigma(freq_hz[0], svTiEstStructMin_mag, marker='None', color = 'b', linestyle = ':', fig = fig, label = 'Estimate w/Structured SV')

    ax = fig.get_axes()
    # handles, labels = ax[0].get_legend_handles_labels()
    # handles = [handles[0], handles[3], handles[1], handles[4], handles[2]]
    # labels = [labels[0], labels[3], labels[1], labels[4], labels[2]]
    # ax[0].legend(handles, labels)

    fig.suptitle('$T_i$ : ' + '$u_{Exc}$' + ' to ' + '$u$')

#%% Sigma - Stability
I2 = np.repeat([np.eye(2)], len(freqLin_rps), axis=0).T

# Unstructured Singular Values
# svLiLinNom_mag = 1/FreqTrans.Sigma(SiLinNom)
svLiLinNom_mag = FreqTrans.Sigma(I2 + LiLinNom)
# Validation: np.allclose(np.min(1/FreqTrans.Sigma(SiLinNom), axis=0), np.min(FreqTrans.Sigma(I2 + LiLinNom), axis=0))
svLiLinNomMin_mag = np.min(svLiLinNom_mag, axis=0)
svLiLinNomMax_mag = np.max(svLiLinNom_mag, axis=0)

# svLiLinUnc_mag = FreqTrans.Sigma(SiLinUnc)
svLiLinUnc_mag = FreqTrans.Sigma(LiLinUnc)
# Validation: np.allclose(FreqTrans.Sigma(SiLinUnc), FreqTrans.Sigma(LiLinUnc)) # FIXIT -
svLiLinUncMax_mag = np.max(svLiLinUnc_mag, axis=0) # Overly Conservative

# Structured Singular Values (Additive)
svLiLinStructMin_mag, LiLinCritStruct = FreqTrans.SigmaStruct_Add(I2 + LiLinNom, LiLinUnc, minmax = 'min')

LiLinCoh = np.ones_like(LiLinUnc)


# Estimated
I2 = np.repeat([np.eye(2)], len(freq_hz[0]), axis=0).T

# Max singular value
svLiEstNom_mag = 1/FreqTrans.Sigma(SiEstNom) # 1/sigma(S) = sigma(I + L)
# svLiEstNom_mag = FreqTrans.Sigma(I2 + LiEstNom)
# Validation: np.allclose(1/np.max(FreqTrans.Sigma(SiEstNom), axis=0), np.min(FreqTrans.Sigma(I2 + LiEstNom), axis=0))
svLiEstNomMin_mag = np.min(svLiEstNom_mag, axis=0)


svLiEstUnc_mag = FreqTrans.Sigma(SiEstUnc)
# Validation: np.allclose(FreqTrans.Sigma(SiEstUnc), FreqTrans.Sigma(LiEstUnc)) # FIXIT -
svLiEstUncMax_mag = np.max(svLiEstUnc_mag, axis=0) # Overly Conservative

# Structured Singular Values (Additive)
svLiEstStructMin_mag, LiEstCritStruct = FreqTrans.SigmaStruct_Add(SiEstNom, SiEstUnc, minmax = 'max')
# LiEstCritStruct -= I2

LiEstCohMin = np.min(LiEstCoh, axis = (0, 1)) # Estimation Coherence

if True:
    fig = 1
    # Linear
    fig = FreqTrans.PlotSigma(freqLin_hz, svLiLinNomMin_mag, np.ones_like(freqLin_hz), svLiLinUncMax_mag, fig = fig, color = 'k', label = 'Linear w/Unstructured SV')
    fig = FreqTrans.PlotSigma(freqLin_hz, svLiLinStructMin_mag, marker='None', color = 'k', linestyle = ':', fig = fig, label = 'Linear w/Structured SV')

    # Estimated
    fig = FreqTrans.PlotSigma(freq_hz[0], svLiEstNomMin_mag, LiEstCohMin, svLiEstUncMax_mag, fig = fig, color = 'b', label = 'Estimate w/Unstructured SV')
    fig = FreqTrans.PlotSigma(freq_hz[0], svLiEstStructMin_mag, marker='None', color = 'b', linestyle = ':', fig = fig, label = 'Estimate w/Structured SV')

    fig = FreqTrans.PlotSigma(freqLin_hz, (0) * np.ones_like(freqLin_hz), linestyle = '-', color = 'r', fig = fig, label = 'Critical Limit = 0.0')
    fig = FreqTrans.PlotSigma(freqLin_hz, (0.4) * np.ones_like(freqLin_hz), linestyle = ':', color = 'r', fig = fig, label = 'Critical Limit = 0.4')

    ax = fig.get_axes()
    handles, labels = ax[0].get_legend_handles_labels()
    # handles = [(handles[0], handles[6]), handles[1], (handles[2], handles[7]), handles[3]]
    # labels = labels[0:3]
    # ax[0].legend(handles, labels)

    fig.suptitle('$L_i$ : ' + '$u_{Exc}$' + ' to ' + '$u$')


#%% Vector Margin Plots
# Linear Model
rCritLiLinNom_mag, rCritLiLinUnc_mag, rCritLiLinMin_mag = FreqTrans.VectorMargin(LiLinNom, LiLinUnc, typeUnc = 'circle')

# Estimated Model
rCritLiEstNom_mag, rCritLiEstUnc_mag, rCritLiEstMin_mag = FreqTrans.VectorMargin(LiEstNom, LiEstUnc, typeUnc = 'circle')


if False:
    numOut, numIn = LiLinNom.shape[0:-1]
    ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

    for iPlot, io in enumerate(ioArray):
        [iOut, iIn] = io

        fig = 30 + iPlot
        fig = FreqTrans.PlotVectorMargin(freqLin_hz, rCritLiLinNom_mag[iOut, iIn], LiLinCoh[iOut, iIn], rCritLiLinUnc_mag[iOut, iIn], fig = fig, linestyle='-', color='k', label='Linear Model')

        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn], rCritLiEstNom_mag[iOut, iIn], LiEstCoh[iOut, iIn], rCritLiEstUnc_mag[iOut, iIn], fig = fig, linestyle='-', marker='.', color='b', label='Estimate [MIMO]')
        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn, sigIndx[iIn]], rCritLiEstNom_mag[iOut, iIn, sigIndx[iIn]], LiEstCoh[iOut, iIn, sigIndx[iIn]], rCritLiEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, linestyle='None', marker='.', color='g', label='Estimate [SIMO]')

        fig = FreqTrans.PlotVectorMargin(freqLin_hz, 0.0 * np.ones_like(freqLin_hz), fig = fig, linestyle='-', color='r', label='Critical Limit = 0.0')
        fig = FreqTrans.PlotVectorMargin(freqLin_hz, 0.4 * np.ones_like(freqLin_hz), fig = fig, linestyle=':', color='r', label='Critical Limit = 0.4')

        ax = fig.get_axes()
        # handles, labels = ax[0].get_legend_handles_labels()
        # handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
        # labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
        # ax[0].legend(handles, labels)

        fig.suptitle('$L_i$ : ' + '$u_{Exc}$[' + str(iIn) + '] to ' + '$u$ [' + str(iOut) + ']')


#%% Bode Plot
# Linear Model Gain and Phase
gainLiLinUnc_mag, phaseLiLinUnc_deg = FreqTrans.GainPhase(LiLinUnc, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)

# Estimated Nominal Response
gainLiEstNom_mag, phaseLiEstNom_deg = FreqTrans.GainPhase(LiEstNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)

# Estimation Uncertain Response
gainLiEstUnc_mag = FreqTrans.Gain(LiEstUnc, magUnit = 'mag')


if False:
    numOut, numIn = LiLinNom.shape[0:-1]
    ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

    for iPlot, io in enumerate(ioArray):
        [iOut, iIn] = io

        fig = 40 + iPlot
        fig = FreqTrans.PlotBode(freqLin_hz, gainLiLinNom_mag[iOut, iIn], phaseLiLinNom_deg[iOut, iIn], coher_nd = LiLinCoh[iOut, iIn], fig = fig, dB = True, color='k', label='Linear Model')
        fig = FreqTrans.PlotBode(freq_hz[iIn], gainLiEstNom_mag[iOut, iIn], phaseLiEstNom_deg[iOut, iIn], coher_nd = LiEstCoh[iOut, iIn], fig = fig, dB = True, color='b', label='Estimate [MIMO]')
        fig = FreqTrans.PlotBode(freq_hz[iIn, sigIndx[iIn]], gainLiEstNom_mag[iOut, iIn, sigIndx[iIn]], phaseLiEstNom_deg[iOut, iIn, sigIndx[iIn]], coher_nd = LiEstCoh[iOut, iIn, sigIndx[iIn]], fig = fig, dB = True, color='g', label='Estimate [SIMO]')

        fig = FreqTrans.PlotGainType(freqLin_hz, gainLiLinUnc_mag[iOut, iIn], fig = fig, dB = True, color='k', linestyle = '--', label='Linear Model')
        fig = FreqTrans.PlotGainType(freq_hz[iIn], gainLiEstUnc_mag[iOut, iIn], fig = fig, dB = True, color='b', linestyle = '--', label='Estimate [MIMO]')
        fig = FreqTrans.PlotGainType(freq_hz[iIn, sigIndx[iIn]], gainLiEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, dB = True, color='g', linestyle = '--', label='Estimate [SIMO]')


        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [(handles[0], handles[3]), (handles[1], handles[4]), (handles[2], handles[5])]
        labels = labels[0:3]
        ax[0].legend(handles, labels)

        fig.suptitle('$L_i$ : ' + '$u_{Exc}$[' + str(iIn) + '] to ' + '$u$ [' + str(iOut) + ']')


#%% Nyquist Plot
LiLinUncMag = np.abs(LiLinUnc)
LiEstUncMag = np.abs(LiEstUnc)

LiEstCrit = LiEstCritStruct

if True:
    numOut, numIn = LiLinNom.shape[0:-1]

    ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

    for iPlot, io in enumerate(ioArray):
        [iOut, iIn] = io

        fig = 50 + iPlot
        # fig = FreqTrans.PlotNyquist(LiLinNom[iOut, iIn], fig = fig, color = 'k', label = 'Linear')
        fig = FreqTrans.PlotNyquist(LiLinNom[iOut, iIn], LiLinUncMag[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear')

        fig = FreqTrans.PlotNyquist(LiEstNom[iOut, iIn], LiEstUncMag[iOut, iIn], fig = fig, fillType = 'circle', marker='.', color = 'b', linestyle='None', label = 'Estimate (MIMO)')
        fig = FreqTrans.PlotNyquist(LiEstNom[iOut, iIn, sigIndx[iIn]], LiEstUncMag[iOut, iIn, sigIndx[iIn]], fig = fig, fillType = 'circle', marker='.', color = 'g', linestyle='None', label = 'Estimate (SIMO)')

        fig = FreqTrans.PlotNyquist(LiEstCritStruct[iOut, iIn], fig = fig, linestyle='None', marker='*', color='b', label='Estimate Critical SSV')

        fig = FreqTrans.PlotNyquist(np.array([-1+0j]), np.array([0.4]), fig = fig, fillType = 'circle', marker='+', color = 'r', linestyle=':')


        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [(handles[0], handles[3]), handles[1], handles[2]]
        labels = labels[0:3]
        ax[0].legend(handles, labels)

        fig.suptitle('$L_i$ : ' + '$u_{Exc}$[' + str(iIn) + '] to ' + '$u$ [' + str(iOut) + ']')
