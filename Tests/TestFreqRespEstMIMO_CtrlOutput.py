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


plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Palatino"],
    "font.size": 10
})

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
noiseK11 = (1/8) * np.abs(sysPlant.dcgain())[0, 0]; noiseWn11 = 6 * hz2rps; noiseD11 = 0.1;
noiseK12 = (4/8) * np.abs(sysPlant.dcgain())[0, 1]; noiseWn12 = 6 * hz2rps; noiseD12 = 0.7;
noiseK21 = (4/8) * np.abs(sysPlant.dcgain())[1, 0]; noiseWn21 = 4 * hz2rps; noiseD21 = 0.7;
noiseK22 = (4/8) * np.abs(sysPlant.dcgain())[1, 1]; noiseWn22 = 4 * hz2rps; noiseD22 = 1.0;

sysN = control.tf([[[-noiseK11, 0, noiseK11 * noiseWn11**2], [-noiseK21, 0, noiseK21 * noiseWn21**2]],
                   [[-noiseK12, 0, noiseK12 * noiseWn12**2], [-noiseK22, 0, noiseK22 * noiseWn22**2]]],
                  [[[1, 2.0*noiseD11*noiseWn11, noiseWn11**2], [1, 2.0*noiseD21*noiseWn21, noiseWn21**2]],
                   [[1, 2.0*noiseD12*noiseWn12, noiseWn12**2], [1, 2.0*noiseD22*noiseWn22, noiseWn22**2]]])


#%% Linear Models
# Lo = GK (connect via Control Output [u], loop is open at [z])
connectNames = sysCtrl.OutputName
inKeep = sysCtrl.InputName + sysPlantDist.InputName[2:]
outKeep = sysCtrl.OutputName + sysPlantDist.OutputName
sysLoopOut = ConnectName([sysCtrl, sysPlantDist], connectNames, inKeep, outKeep) # Lo = GK

# Closed-Loop
connectNames = sysLoopOut.OutputName[2:]
inKeep = sysLoopOut.InputName[:2] + sysLoopOut.InputName[4:]
outKeep = sysLoopOut.OutputName
sysCL = ConnectName([sysLoopOut], connectNames, inKeep, outKeep)

# Loop Lo - (e -> z)
sysLo = sysLoopOut[2:4, 0:2]
sysLo.InputName = sysLoopOut.InputName[0:2]
sysLo.OutputName = sysLoopOut.OutputName[2:4]

LoLinNom = FreqTrans.FreqResp(sysLo, freqLin_rps)

# Loop To - (r -> z)
sysTo = sysCL[2:4, 0:2]
sysTo.InputName = sysCL.InputName[0:2]
sysTo.OutputName = sysCL.OutputName[2:4]

ToLinNom = FreqTrans.FreqResp(sysTo, freqLin_rps)

# Loop So
sysSo = np.eye(2) - sysTo

pSigma = [0.0, 0.0]
mSigma = [1.0, 1.0]

pIn = 0.5 * control.ss([],[],[],np.diag(pSigma))
mIn = 0.5 * control.ss([],[],[],np.diag(mSigma))
sysToUnc = sysSo * (sysN * mIn + sysLo * sysR * pIn)
#sysToUnc = sysSo * sysN * np.diag(mSigma)

ToLinUnc = FreqTrans.FreqResp(sysToUnc, freqLin_rps)

I2 = np.repeat([np.eye(2)], ToLinNom.shape[-1], axis=0).T
SoLinNom = I2 - ToLinNom
SoLinUnc = ToLinUnc


LoLinUnc = np.zeros_like(ToLinNom, dtype = complex)
inv = np.linalg.inv
for i in range(ToLinNom.shape[-1]):
    SoLinNomElem = SoLinNom[...,i]
    SoLinUncElem = SoLinUnc[...,i]
    SoLinNomInvElem = inv(SoLinNomElem)
    
#    LoLinNom[...,i] = -np.eye(2) + SoLinNomElem
    LoLinUnc[...,i] = -inv(np.eye(2) + SoLinNomInvElem @ SoLinUncElem) @ SoLinNomInvElem @ SoLinUncElem @ SoLinNomInvElem
    
    
#%% Excitation
numExc = 2
numCycles = 6
ampInit = 1
ampFinal = 1
freqMinDes_rps = 0.1 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 10.3 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (20 / freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)

# Generate Schroeder MultiSine Signal
ampExcit_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
rExc, phaseElem_rad, sigExcit = GenExcite.MultiSine(freqExc_rps, ampExcit_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')
rExc = rExc / np.std(rExc)
rPeak = np.mean(GenExcite.PeakFactor(rExc) * np.std(rExc))**2

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]
freqChan_hz = freqChan_rps * rps2hz

# Null Frequencies
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)
freqGap_hz = freqGap_rps * rps2hz


#%% Simulate the excitation through the system
np.random.seed(0)

# Time Response
p = np.random.normal([0.0, 0.0], pSigma, size = (len(time_s), 2)).T
_, r, _ = control.forced_response(sysR, T = time_s, U = p)
r = r + rExc

uExc = np.zeros_like(rExc)

m = np.random.normal([0.0, 0.0], mSigma, size = (len(time_s), 2)).T
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
freq_rps, Txy, Cxy, Sxx, Syy, Sxy, TxyUnc, SxxNull, Snn = FreqTrans.FreqRespFuncEstNoise(rExc, z, optSpec)
freq_hz = freq_rps * rps2hz

I2 = np.repeat([np.eye(2)], Txy.shape[-1], axis=0).T
ToEstNom = Txy # Txy = Sxy / Sxx
ToEstUnc = -TxyUnc # TxyUnc = np.abs(Sxn / Sxx)
ToEstCoh = Cxy # Cxy = np.abs(Sxy)**2 / (Sxx * Syy)


SoEstNom, SoEstUnc, SoEstCoh = FreqTrans.TtoS(ToEstNom, ToEstUnc, ToEstCoh)
LoEstNom, LoEstUnc, LoEstCoh = FreqTrans.StoL(SoEstNom, SoEstUnc, SoEstCoh)


print(np.sum(SxxNull, axis = -1) / np.sum(Sxx, axis = -1))
print(np.sum(Snn, axis = -1) / np.sum(Syy, axis = -1))
ToEstSNR = np.abs(ToEstNom / ToEstUnc)**2
SoEstSNR = np.abs(SoEstNom / SoEstUnc)**2
LoEstSNR = np.abs(LoEstNom / LoEstUnc)**2


#%% Nyquist Plot - Output Complimentary Sensitivity Function
ToLinUncMag = np.abs(ToLinUnc)
ToEstUncMag = np.abs(ToEstUnc)

numOut, numIn = ToLinNom.shape[0:-1]
ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)
    
if False:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 10 + iPlot
        fig = FreqTrans.PlotNyquist(ToLinNom[iOut, iIn], ToLinUncMag[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear')
        fig = FreqTrans.PlotNyquist(ToEstNom[iOut, iIn], ToEstUncMag[iOut, iIn], fig = fig, fillType = 'circle', marker='.', color = 'b', linestyle='None', label = 'Estimate (MIMO)')
        fig = FreqTrans.PlotNyquist(ToEstNom[iOut, iIn, sigIndx[iIn]], ToEstUncMag[iOut, iIn, sigIndx[iIn]], fig = fig, fillType = 'circle', marker='.', color = 'g', linestyle='None', label = 'Estimate (SIMO)')
        fig = FreqTrans.PlotNyquist(np.array([-1+0j]), np.array([0.4]), fig = fig, fillType = 'circle', marker='+', color = 'r', linestyle='None')

        ax = fig.get_axes()
#        handles, labels = ax[0].get_legend_handles_labels()
#        handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
#        labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
#        ax[0].legend(handles, labels)
    
        fig.suptitle('$T_o$ : ' + '$r_{Exc}$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')


#%% Bode Plot - Output Complimentary Sensitivity Function
# Linear Model Gain and Phase
gainToLinNom_mag, phaseToLinNom_deg = FreqTrans.GainPhase(ToLinNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainToLinUnc_mag = FreqTrans.Gain(ToLinUnc, magUnit = 'mag')
cohToLin_mag = np.ones_like(ToLinUnc)

# Estimated
gainToEstNom_mag, phaseToEstNom_deg = FreqTrans.GainPhase(ToEstNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainToEstUnc_mag = FreqTrans.Gain(ToEstUnc, magUnit = 'mag') # Estimation Uncertain Response
cohToEst_mag = ToEstCoh # Estimation Coherence
cohToEstMin = np.min(cohToEst_mag, axis = (0, 1))


if False:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 20 + iPlot
        fig = FreqTrans.PlotBode(freqLin_hz, gainToLinNom_mag[iOut, iIn], phaseToLinNom_deg[iOut, iIn], coher_nd = cohToLin_mag[iOut, iIn], gainUnc_mag = gainToLinUnc_mag[iOut, iIn], fig = fig, dB = True, color='k', label='Linear Model')
        fig = FreqTrans.PlotBode(freq_hz[iIn], gainToEstNom_mag[iOut, iIn], phaseToEstNom_deg[iOut, iIn], coher_nd = cohToEst_mag[iOut, iIn], gainUnc_mag = gainToEstUnc_mag[iOut, iIn], fig = fig, dB = True, color='b', label='Estimate [MIMO]')
        fig = FreqTrans.PlotBode(freq_hz[iIn, sigIndx[iIn]], gainToEstNom_mag[iOut, iIn, sigIndx[iIn]], phaseToEstNom_deg[iOut, iIn, sigIndx[iIn]], coher_nd = cohToEst_mag[iOut, iIn, sigIndx[iIn]], gainUnc_mag = gainToEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, dB = True, color='g', label='Estimate [SIMO]')
        
        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
        labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
        ax[0].legend(handles, labels)
    
        fig.suptitle('$T_o$ : ' + '$r_{Exc}$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')


#%% Sigma - Output Stability
I2 = np.repeat([np.eye(2)], LoLinNom.shape[-1], axis=0).T

# Linear Model SVD magnitude
svLoLinNom_mag = FreqTrans.Sigma(I2 + LoLinNom) # sigma(I + Lo)
svLoLinNomMin_mag = np.min(svLoLinNom_mag, axis=0)

svLoLinUnc_mag = FreqTrans.Sigma(LoLinUnc)
svLoLinUncMax_mag = np.max(svLoLinUnc_mag, axis=0) # Overly Conservative

cohLoLin_mag = np.ones_like(LoLinUnc)

# Estimate SVD magnitude
# svLiEstNom_mag = FreqTrans.Sigma(I2 + LoEstNom)
svLoEstNom_mag = 1 / FreqTrans.Sigma(SoEstNom) # sigma(I + Lo) = 1 / sigma(So)
svLoEstNomMin_mag = np.min(svLoEstNom_mag, axis=0)

svLoEstUnc_mag = FreqTrans.Sigma(LoEstUnc) # Uncertain SVD magnitude
svLoEstUncMax_mag = np.max(svLoEstUnc_mag, axis=0) # Overly Conservative

cohLoEst_mag = LoEstCoh
cohLoEstMin = np.min(cohLoEst_mag, axis = (0, 1)) # Estimation Coherence


if True:
    fig = 1
    fig = FreqTrans.PlotSigma(freqLin_hz, svLoLinNomMin_mag, np.ones_like(freqLin_hz), svLoLinUncMax_mag, fig = fig, color = 'k', label = 'Linear')
    fig = FreqTrans.PlotSigma(freq_hz[0], svLoEstNomMin_mag, cohLoEstMin, svLoEstUncMax_mag, marker='.', color = 'b', fig = fig, label = 'Estimate (MIMO)')
    fig = FreqTrans.PlotSigma(freqLin_hz, (0.4) * np.ones_like(freqLin_hz), linestyle = '--', color = 'r', fig = fig, label = 'Critical Limit')
    
    ax = fig.get_axes()
    handles, labels = ax[0].get_legend_handles_labels()
    handles = [handles[0], handles[3], handles[1], handles[4], handles[2]]
    labels = [labels[0], labels[3], labels[1], labels[4], labels[2]]
    ax[0].legend(handles, labels)
    
    fig.suptitle('$L_o$ : ' + '$r_{Exc}$' + ' to ' + '$z$')


#%% Vector Margin Plots - Output Stability
vmLoLinNom_mag, vmLoLinUnc_mag, vmLoLinMin_mag = FreqTrans.VectorMargin(LoLinNom, LoLinUnc, typeUnc = 'circle')
vmLoEstNom_mag, vmLoEstUnc_mag, vmLoEstMin_mag = FreqTrans.VectorMargin(LoEstNom, LoEstUnc, typeUnc = 'circle')

numOut, numIn = LoLinNom.shape[0:-1]
ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

if True:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 30 + iPlot
        fig = FreqTrans.PlotVectorMargin(freqLin_hz, vmLoLinNom_mag[iOut, iIn], cohLoLin_mag[iOut, iIn], vmLoLinUnc_mag[iOut, iIn], fig = fig, linestyle='-', color='k', label='Linear Model')
        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn], vmLoEstNom_mag[iOut, iIn], cohLoEst_mag[iOut, iIn], vmLoEstUnc_mag[iOut, iIn], fig = fig, linestyle='-', marker='.', color='b', label='Estimate [MIMO]')
        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn, sigIndx[iIn]], vmLoEstNom_mag[iOut, iIn, sigIndx[iIn]], cohLoEst_mag[iOut, iIn, sigIndx[iIn]], vmLoEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, linestyle='None', marker='.', color='g', label='Estimate [SIMO]')
        
        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
        labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
        ax[0].legend(handles, labels)
    
        fig.suptitle('$L_o$ : ' + '$r_{Exc}$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')


#%% Nyquist Plot - Output Stability
LoLinUncMag = np.abs(LoLinUnc)
LoEstUncMag = np.abs(LoEstUnc)

if False:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 50 + iPlot
        fig = FreqTrans.PlotNyquist(LoLinNom[iOut, iIn], LoLinUncMag[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear')
        fig = FreqTrans.PlotNyquist(LoEstNom[iOut, iIn], LoEstUncMag[iOut, iIn], fig = fig, fillType = 'circle', marker='.', color = 'b', linestyle='None', label = 'Estimate (MIMO)')
        fig = FreqTrans.PlotNyquist(LoEstNom[iOut, iIn, sigIndx[iIn]], LoEstUncMag[iOut, iIn, sigIndx[iIn]], fig = fig, fillType = 'circle', marker='.', color = 'g', linestyle='None', label = 'Estimate (SIMO)')
        fig = FreqTrans.PlotNyquist(np.array([-1+0j]), np.array([0.4]), fig = fig, fillType = 'circle', marker='+', color = 'r', linestyle='None')

        ax = fig.get_axes()
#        handles, labels = ax[0].get_legend_handles_labels()
#        handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
#        labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
#        ax[0].legend(handles, labels)
    
        fig.suptitle('$L_o$ : ' + '$r_{Exc}$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')


#%% Bode Plot - Output Stability
# Linear Model Gain and Phase
gainLoLinNom_mag, phaseLoLinNom_deg = FreqTrans.GainPhase(LoLinNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainLoLinUnc_mag = FreqTrans.Gain(LoLinUnc, magUnit = 'mag')
cohLoLin_mag = np.ones_like(LoLinUnc)

# Estimated
gainLoEstNom_mag, phaseLoEstNom_deg = FreqTrans.GainPhase(LoEstNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainLoEstUnc_mag = FreqTrans.Gain(LoEstUnc, magUnit = 'mag') # Estimation Uncertain Response
cohLoEst_mag = LoEstCoh # Estimation Coherence
cohLoEstMin = np.min(cohLoEst_mag, axis = (0, 1))


if False:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = 40 + iPlot
        fig = FreqTrans.PlotBode(freqLin_hz, gainLoLinNom_mag[iOut, iIn], phaseLoLinNom_deg[iOut, iIn], coher_nd = cohLoLin_mag[iOut, iIn], gainUnc_mag = gainLoLinUnc_mag[iOut, iIn], fig = fig, dB = True, color='k', label='Linear Model')
        fig = FreqTrans.PlotBode(freq_hz[iIn], gainLoEstNom_mag[iOut, iIn], phaseLoEstNom_deg[iOut, iIn], coher_nd = cohLoEst_mag[iOut, iIn], gainUnc_mag = gainLoEstUnc_mag[iOut, iIn], fig = fig, dB = True, color='b', label='Estimate [MIMO]')
        fig = FreqTrans.PlotBode(freq_hz[iIn, sigIndx[iIn]], gainLoEstNom_mag[iOut, iIn, sigIndx[iIn]], phaseLoEstNom_deg[iOut, iIn, sigIndx[iIn]], coher_nd = cohLoEst_mag[iOut, iIn, sigIndx[iIn]], gainUnc_mag = gainLoEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, dB = True, color='g', label='Estimate [SIMO]')
        
        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
        labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
        ax[0].legend(handles, labels)
    
        fig.suptitle('$L_o$ : ' + '$r_{Exc}$[' + str(iIn) + '] to ' + '$z$[' + str(iOut) + ']')

