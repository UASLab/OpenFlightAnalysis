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

    sysPlant = control.tf([[[0, 0, plantK11 * plantWn11**2], [0, 0, plantK12 * plantWn12**2]],
                           [[0, 0, plantK21 * plantWn21**2], [0, 0, plantK22 * plantWn22**2]]],
                          [[[1, 2.0*plantD11*plantWn11, plantWn11**2], [1, 2.0*plantD12*plantWn12, plantWn12**2]],
                           [[1, 2.0*plantD21*plantWn21, plantWn21**2], [1, 2.0*plantD22*plantWn22, plantWn22**2]]])

    sysK = control.ss([], [], [], np.diag([0.75, 0.5]))

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

sysSi = (np.eye(2) - sysTi)

pSigma = [0.0, 0.0]
mSigma = [1.0, 1.0]

pIn = 0.5 * control.ss([],[],[],np.diag(pSigma))
mIn = 0.5 * control.ss([],[],[],np.diag(mSigma))
sysTiUnc = sysSi * sysK * (sysR * pIn - sysN * mIn)
gainTiLinUnc_mag, phaseTiLinUnc_rad, _ = control.freqresp(sysTiUnc, omega = freqLin_rps)
TiLinUnc = gainTiLinUnc_mag * np.exp(1j * phaseTiLinUnc_rad * deg2rad)

# Ti to Si, Si to Li
SiLinNom, SiLinUnc, _ = FreqTrans.TtoS(TiLinNom, TiLinUnc)
LiLinNom, LiLinUnc, _ = FreqTrans.StoL(SiLinNom, SiLinUnc)

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


print(np.sum(SxxNull, axis = -1) / np.sum(Sxx, axis = -1))
TiEstSNR = np.abs(TiEstNom / TiEstUnc)**2
SiEstSNR = np.abs(SiEstNom / SiEstUnc)**2
LiEstSNR = np.abs(LiEstNom / LiEstUnc)**2


#%% Sampling
numOut, numIn, nFreq = TiEstNom.shape

nSamp = 20
shapeSamp = (numOut, numIn, nFreq, nSamp)

rSamp = np.sqrt(np.random.uniform(0, 1, shapeSamp))
phSamp = np.random.uniform(-np.pi, np.pi, shapeSamp)
samp = rSamp * np.exp(1j * phSamp)

TiEstSamp = np.zeros(shapeSamp, dtype='complex')
SiEstSamp = np.zeros_like(TiEstSamp)
LiEstSamp = np.zeros_like(SiEstSamp)
for iSamp in range(nSamp):
  TiEstSamp[..., iSamp] = TiEstNom + TiEstUnc * samp[..., iSamp]
  SiEstSamp[..., iSamp], _, _ = FreqTrans.TtoS(TiEstSamp[..., iSamp])
  LiEstSamp[..., iSamp], _, _ = FreqTrans.StoL(SiEstSamp[..., iSamp])


#%% Nyquist Plot - Compl Sens Function
TiLinUncMag = np.abs(TiLinUnc)
TiEstUncMag = np.abs(TiEstUnc)

if False:
    numOut, numIn = TiLinNom.shape[0:-1]

    ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

    for iPlot, io in enumerate(ioArray):
        [iOut, iIn] = io

        fig = 10 + iPlot
        fig = FreqTrans.PlotNyquist(TiLinNom[iOut, iIn], TiLinUncMag[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear')
        fig = FreqTrans.PlotNyquist(TiEstNom[iOut, iIn], TiEstUncMag[iOut, iIn], fig = fig, fillType = 'circle', marker='.', color = 'b', linestyle='None', label = 'Estimate (MIMO)')
        fig = FreqTrans.PlotNyquist(TiEstNom[iOut, iIn, sigIndx[iIn]], TiEstUncMag[iOut, iIn, sigIndx[iIn]], fig = fig, fillType = 'circle', marker='.', color = 'g', linestyle='None', label = 'Estimate (SIMO)')
        fig = FreqTrans.PlotNyquist(np.array([-1+0j]), np.array([0.4]), fig = fig, fillType = 'circle', marker='+', color = 'r', linestyle='None', label = 'Critical')

        for iSamp in range(nSamp):
          fig = FreqTrans.PlotNyquist(TiEstSamp[iOut, iIn, :, iSamp], fig = fig, marker='.', color = 'gray', linestyle='None', label = 'Sample (MIMO)')

#        ax = fig.get_axes()
#        handles, labels = ax[0].get_legend_handles_labels()
#        handles = [handles[0], handles[3], handles[1], handles[2], handles[3], handles[5]]
#        labels = [labels[0], labels[3], labels[1], labels[2], labels[3], labels[5]]
#        ax[0].legend(handles, labels)

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


#%% Sigma Plot
I2 = np.repeat([np.eye(2)], len(freqLin_rps), axis=0).T

# Linear Model SVD magnitude
svLiLinNom_mag = FreqTrans.Sigma(I2 + LiLinNom) # sigma(I + Li)
svLiLinNomMin_mag = np.min(svLiLinNom_mag, axis=0)

svLiLinUnc_mag = FreqTrans.Sigma(LiLinUnc)
svLiLinUncMax_mag = np.max(svLiLinUnc_mag, axis=0) # Overly Conservative

cohLiLin_mag = np.ones_like(LiLinUnc)

# Compute Structured Singular Values
svLiLinSSV_mag, LiLinCrit = FreqTrans.SigmaStruct(I2 + LiLinNom, np.abs(LiLinUnc))
LiLinCrit = LiLinCrit - I2


# Estimate SVD magnitude
I2 = np.repeat([np.eye(2)], len(freqExc_rps), axis=0).T
# svLiEstNom_mag = FreqTrans.Sigma(I2 + LiEstNom) # sigma(I + Li)
svLiEstNom_mag = 1 / FreqTrans.Sigma(SiEstNom) # sigma(I + Li) = 1 / sigma(Si)
svLiEstNomMin_mag = np.min(svLiEstNom_mag, axis=0)

svLiEstUnc_mag = FreqTrans.Sigma(LiEstUnc) # Uncertain SVD magnitude
svLiEstUncMax_mag = np.max(svLiEstUnc_mag, axis=0) # Overly Conservative

cohLiEst_mag = LiEstCoh
cohLiEstMin = np.min(cohLiEst_mag, axis = (0, 1)) # Estimation Coherence

# Sampled Systems
svLiEstSamp_mag = np.zeros((numOut, nFreq, nSamp), dtype='float')
for iSamp in range(nSamp):
  svLiEstSamp_mag[..., iSamp] = FreqTrans.Sigma(I2 + LiEstSamp[..., iSamp])

svLiEstSampMin_mag = np.min(svLiEstSamp_mag, axis = 0)

# Compute Structured Singular Values
svLiEstSSV_mag, LiEstCrit = FreqTrans.SigmaStruct(I2 + LiEstNom, np.abs(LiEstUnc))
LiEstCrit = LiEstCrit - I2

if True:
    fig = 1
    fig = FreqTrans.PlotSigma(freqLin_hz, svLiLinNomMin_mag, np.ones_like(freqLin_hz), svLiLinUncMax_mag, fig = fig, color = 'k', label = 'Linear')
    fig = FreqTrans.PlotSigma(freqLin_hz, svLiLinSSV_mag, marker='None', color = 'k', linestyle = ':', fig = fig, label = 'Linear SSV')

    fig = FreqTrans.PlotSigma(freq_hz[0], svLiEstNomMin_mag, cohLiEstMin, svLiEstUncMax_mag, marker='.', color = 'b', fig = fig, label = 'Estimate (MIMO)')
    fig = FreqTrans.PlotSigma(freq_hz[0], svLiEstSSV_mag, marker='None', color = 'b', linestyle = ':', fig = fig, label = 'Estimate SSV (MIMO)')

    # for iSamp in range(nSamp):
    #   fig = FreqTrans.PlotSigma(freq_hz[0], svLiEstSampMin_mag[..., iSamp], fig = fig, marker='.', color = 'gray', linestyle='None', label = 'Sample (MIMO)')

    fig = FreqTrans.PlotSigma(freqLin_hz, (0.4) * np.ones_like(freqLin_hz), linestyle = '--', color = 'r', fig = fig, label = 'Critical Limit')

    ax = fig.get_axes()
    handles, labels = ax[0].get_legend_handles_labels()
    handles = [handles[0], handles[3], handles[1], handles[4], handles[2]]
    labels = [labels[0], labels[3], labels[1], labels[4], labels[2]]
    ax[0].legend(handles, labels)

    fig.suptitle('$L_i$ : ' + '$u_{Exc}$' + ' to ' + '$u$')


#%% Vector Margin Plots
rCritLiLinNom_mag, rCritLiLinUnc_mag, rCritLiLinMin_mag = FreqTrans.VectorMargin(LiLinNom, LiLinUnc, typeUnc = 'circle')
rCritLiEstNom_mag, rCritLiEstUnc_mag, rCritLiEstMin_mag = FreqTrans.VectorMargin(LiEstNom, LiEstUnc, typeUnc = 'circle')

rCritLiEstSamp_mag, _, _ = FreqTrans.VectorMargin(LiEstSamp, typeUnc = 'circle')

rCritLiEstCrit_mag, _, _ = FreqTrans.VectorMargin(LiEstCrit, typeUnc = 'circle')

if True:
    numOut, numIn = LiLinNom.shape[0:-1]
    ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

    for iPlot, io in enumerate(ioArray):
        [iOut, iIn] = io

        fig = 30 + iPlot
        fig = FreqTrans.PlotVectorMargin(freqLin_hz, rCritLiLinNom_mag[iOut, iIn], cohLiLin_mag[iOut, iIn], rCritLiLinUnc_mag[iOut, iIn], fig = fig, linestyle='-', color='k', label='Linear Model')
        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn], rCritLiEstNom_mag[iOut, iIn], cohLiEst_mag[iOut, iIn], rCritLiEstUnc_mag[iOut, iIn], fig = fig, linestyle='-', marker='.', color='b', label='Estimate [MIMO]')
        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn, sigIndx[iIn]], rCritLiEstNom_mag[iOut, iIn, sigIndx[iIn]], cohLiEst_mag[iOut, iIn, sigIndx[iIn]], rCritLiEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, linestyle='None', marker='.', color='g', label='Estimate [SIMO]')
        fig = FreqTrans.PlotVectorMargin(freqLin_hz, 0.4 * np.ones_like(freqLin_hz), fig = fig, linestyle=':', color='r', label='Critical')

        for iSamp in range(nSamp):
          fig = FreqTrans.PlotVectorMargin(freq_hz[iIn], rCritLiEstSamp_mag[iOut, iIn, :, iSamp], fig = fig, marker='.', color = 'gray', linestyle='None', label = 'Sample (MIMO)')

        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn], rCritLiEstCrit_mag[iOut, iIn], fig = fig, linestyle='None', marker='.', color='m', label='SSV Crit [SIMO]')

        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
        labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
        ax[0].legend(handles, labels)

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
        fig = FreqTrans.PlotBode(freqLin_hz, gainLiLinNom_mag[iOut, iIn], phaseLiLinNom_deg[iOut, iIn], coher_nd = cohLiLin_mag[iOut, iIn], gainUnc_mag = gainLiLinUnc_mag[iOut, iIn], fig = fig, dB = True, color='k', label='Linear Model')
        fig = FreqTrans.PlotBode(freq_hz[iIn], gainLiEstNom_mag[iOut, iIn], phaseLiEstNom_deg[iOut, iIn], coher_nd = cohLiEst_mag[iOut, iIn], gainUnc_mag = gainLiEstUnc_mag[iOut, iIn], fig = fig, dB = True, color='b', label='Estimate [MIMO]')
        fig = FreqTrans.PlotBode(freq_hz[iIn, sigIndx[iIn]], gainLiEstNom_mag[iOut, iIn, sigIndx[iIn]], phaseLiEstNom_deg[iOut, iIn, sigIndx[iIn]], coher_nd = cohLiEst_mag[iOut, iIn, sigIndx[iIn]], gainUnc_mag = gainLiEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, dB = True, color='g', label='Estimate [SIMO]')

        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
        labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
        ax[0].legend(handles, labels)

        fig.suptitle('$L_i$ : ' + '$u_{Exc}$[' + str(iIn) + '] to ' + '$u$ [' + str(iOut) + ']')


#%% Nyquist Plot
LiLinUncMag = np.abs(LiLinUnc)
LiEstUncMag = np.abs(LiEstUnc)

if True:
    numOut, numIn = LiLinNom.shape[0:-1]

    ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

    for iPlot, io in enumerate(ioArray):
        [iOut, iIn] = io

        fig = 50 + iPlot
        fig = FreqTrans.PlotNyquist(LiLinNom[iOut, iIn], LiLinUncMag[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear')
        fig = FreqTrans.PlotNyquist(LiEstNom[iOut, iIn], LiEstUncMag[iOut, iIn], fig = fig, fillType = 'circle', marker='.', color = 'b', linestyle='None', label = 'Estimate (MIMO)')
        fig = FreqTrans.PlotNyquist(LiEstNom[iOut, iIn, sigIndx[iIn]], LiEstUncMag[iOut, iIn, sigIndx[iIn]], fig = fig, fillType = 'circle', marker='.', color = 'g', linestyle='None', label = 'Estimate (SIMO)')
        fig = FreqTrans.PlotNyquist(np.array([-1+0j]), np.array([0.4]), fig = fig, fillType = 'circle', marker='+', color = 'r', linestyle='None')

        for iSamp in range(nSamp):
          fig = FreqTrans.PlotNyquist(LiEstSamp[iOut, iIn, :, iSamp], fig = fig, fillType = 'circle', marker='.', color = 'gray', linestyle='None')

        fig = FreqTrans.PlotNyquist(LiEstCrit[iOut, iIn], fig = fig, linestyle='None', marker='.', color='m', label='SSV Crit [SIMO]')

        ax = fig.get_axes()
#        handles, labels = ax[0].get_legend_handles_labels()
#        handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
#        labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
#        ax[0].legend(handles, labels)

        fig.suptitle('$L_i$ : ' + '$u_{Exc}$[' + str(iIn) + '] to ' + '$u$ [' + str(iOut) + ']')
