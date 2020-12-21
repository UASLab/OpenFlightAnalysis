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


# plt.rcParams.update({
#     "text.usetex": True,
#     "font.family": "serif",
#     "font.serif": ["Palatino"],
#     "font.size": 10
# })

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
    plantK21 = 0.25; plantWn21 = 4 * hz2rps; plantD21 = 0.1;
    plantK12 = 0.5 ; plantWn12 = 5 * hz2rps; plantD12 = 0.3;
    plantK22 = 1.0 ; plantWn22 = 6 * hz2rps; plantD22 = 0.4;

    sysPlant = control.tf([[[0, 0, plantK11 * plantWn11**2], [0, 0, plantK21 * plantWn21**2]],
                           [[0, 0, plantK12 * plantWn12**2], [0, 0, plantK22 * plantWn22**2]]],
                          [[[1, 2.0*plantD11*plantWn11, plantWn11**2], [1, 2.0*plantD21*plantWn21, plantWn21**2]],
                           [[1, 2.0*plantD12*plantWn12, plantWn12**2], [1, 2.0*plantD22*plantWn22, plantWn22**2]]])

    sysK = control.ss([], [], [], 0.5 * np.eye(2))

elif False: # SP Example #3.8
    freqLin_hz = np.linspace(1e-2 * rps2hz, 1e2 * rps2hz, 200)
    freqRate_hz = 50

    plantK11 = 1.0; plantWn11 = np.sqrt(5);      plantD11 = 6 / (2 * plantWn11);
    plantK21 = 1.0; plantWn21 = 1.0 * plantWn11; plantD21 = 1.0 * plantD11;
    plantK12 = 1.0; plantWn12 = 1.0 * plantWn11; plantD12 = 1.0 * plantD11;
    plantK22 = 1.0; plantWn22 = 1.0 * plantWn11; plantD22 = 1.0 * plantD11;

    sysPlant = control.tf([[[0, 0, plantK11 * plantWn11**2], [0, 0, plantK21 * plantWn21**2]],
                           [[0, 0, plantK12 * plantWn12**2], [0, 0, plantK22 * plantWn22**2]]],
                          [[[1, 2.0*plantD11*plantWn11, plantWn11**2], [1, 2.0*plantD21*plantWn21, plantWn21**2]],
                           [[1, 2.0*plantD12*plantWn12, plantWn12**2], [1, 2.0*plantD22*plantWn22, plantWn22**2]]])

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
refK11 = (2/8) * plantK11; refWn11 = 6 * hz2rps; refD11 = 1.0;
refK12 = (2/8) * plantK12; refWn12 = 6 * hz2rps; refD12 = 1.0;
refK21 = (2/8) * plantK21; refWn21 = 6 * hz2rps; refD21 = 1.0;
refK22 = (2/8) * plantK22; refWn22 = 6 * hz2rps; refD22 = 1.0;
sysR = control.tf([[[refK11 * refWn11**2], [refK12 * refWn12**2]],
                   [[refK21 * refWn21**2], [refK22 * refWn22**2]]],
                  [[[1, 2*refD11*refWn11, refWn11**2], [1, 2*refD12*refWn12, refWn12**2]],
                   [[1, 2*refD21*refWn21, refWn21**2], [1, 2*refD22*refWn22, refWn22**2]]])


# Plant-Output Noise
noiseK11 = (2/8) * plantK11; noiseWn11 = 6 * hz2rps; noiseD11 = 0.1;
noiseK12 = (2/8) * plantK12; noiseWn12 = 6 * hz2rps; noiseD12 = 0.1;
noiseK21 = (4/8) * plantK21; noiseWn21 = 4 * hz2rps; noiseD21 = 1.0;
noiseK22 = (4/8) * plantK22; noiseWn22 = 4 * hz2rps; noiseD22 = 1.0;
sysN = control.tf([[[-noiseK11, 0, noiseK11 * noiseWn11**2], [-noiseK12, 0, noiseK12 * noiseWn12**2]],
                   [[-noiseK21, 0, noiseK21 * noiseWn21**2], [-noiseK22, 0, noiseK22 * noiseWn22**2]]],
                  [[[1, 2.0*noiseD11*noiseWn11, noiseWn11**2], [1, 2.0*noiseD12*noiseWn12, noiseWn12**2]],
                   [[1, 2.0*noiseD21*noiseWn21, noiseWn21**2], [1, 2.0*noiseD22*noiseWn22, noiseWn22**2]]])


#%% Linear Models
# Lo = GK
connectNames = sysCtrl.OutputName
inKeep = sysCtrl.InputName + sysPlantDist.InputName[2:]
outKeep = sysCtrl.OutputName + sysPlantDist.OutputName
sysOL = ConnectName([sysCtrl, sysPlantDist], connectNames, inKeep, outKeep) # Lo = GK
sysLo = sysOL[2:4, 0:2]

# Closed-Loop
connectNames = sysOL.OutputName[2:]
inKeep = sysOL.InputName[:2] + sysOL.InputName[4:]
outKeep = sysOL.OutputName
sysCL = ConnectName([sysOL], connectNames, inKeep, outKeep)

# Loop Lo - (e -> y)
gainLoLinNom_mag, phaseLoLinNom_rad, _ = control.freqresp(sysLo, omega = freqLin_rps)
LoLinNom = gainLoLinNom_mag * np.exp(1j * phaseLoLinNom_rad)
I2 = np.repeat([np.eye(2)], LoLinNom.shape[-1], axis=0).T
svLoLinNom_mag = FreqTrans.Sigma(I2 + LoLinNom)
gainLoLinNom_dB = 20 * np.log10(gainLoLinNom_mag)
phaseLoLinNom_deg = np.unwrap(phaseLoLinNom_rad) * rad2deg

# Loop So - (n -> z)
sysSo = sysCL[2:4, 4:6]
gainSoLinNom_mag, phaseSoLinNom_rad, _ = control.freqresp(sysSo, omega = freqLin_rps)
SoLinNom = gainSoLinNom_mag * np.exp(1j * phaseSoLinNom_rad)
svSoLinNom_mag = FreqTrans.Sigma(SoLinNom)
gainSoLinNom_dB = 20 * np.log10(gainSoLinNom_mag)
phaseSoLinNom_deg = np.unwrap(phaseSoLinNom_rad) * rad2deg

# Loop To - (r -> z)
sysTo = sysCL[2:4, 0:2]
gainToLinNom_mag, phaseToLinNom_rad, _ = control.freqresp(sysTo, omega = freqLin_rps)
ToLinNom = gainToLinNom_mag * np.exp(1j * phaseToLinNom_rad)
svToLinNom_mag = FreqTrans.Sigma(ToLinNom)
gainToLinNom_dB = 20 * np.log10(gainToLinNom_mag)
phaseToLinNom_deg = np.unwrap(phaseToLinNom_rad) * rad2deg


gainLinN_mag, phaseLinN_deg, _ = control.freqresp(sysN, omega = freqLin_rps)
NLin = gainLinN_mag * np.exp(1j * phaseLinN_deg * deg2rad)

gainLinR_mag, phaseLinR_deg, _ = control.freqresp(sysR, omega = freqLin_rps)
RLin = gainLinR_mag * np.exp(1j * phaseLinR_deg * deg2rad)


# LoLinUnc = np.zeros_like(TiLinNom, dtype = complex) # Li = Ti * inv(I - Ti)
# inv = np.linalg.inv
# for i in range(TiLinNom.shape[-1]): # FIXIT
    # LiLinUnc[...,i] = LiLinNom[...,i] * (NLin[...,i] * 1.0 + RLin[...,i] * 1.0)
    # LiLinUnc[...,i] = SiLinNom[...,i] * NLin[...,i]
    # LiLinUnc[...,i] = -inv(np.eye(2) + SiLinNomInvElem * SiLinUncElem) * SiLinNomInvElem * SiLinUncElem * SiLinNomInvElem

    # LiLinUnc[...,i] = TiLinNom[...,i]
    # LiLinUnc[...,i] = TiLinNom[...,i] * inv(np.eye(2) - TiLinNom[...,i])

    # Li = control.series(TiLinNom[...,i], inv(np.eye(2) - TiLinNom[...,i]))
    # LiLinUnc[...,i] = control.series(Li, (NLin[...,i] * 1.0 + RLin[...,i] * 1.0))


ToLinUnc = np.abs(SoLinNom * NLin)# FIXIT


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
pSigma = 0.0
p = np.random.normal([0.0], [pSigma], size = rExc.shape)
_, r, _ = control.forced_response(sysR, T = time_s, U = p)
r = r + rExc

mSigma = 1.0
m = np.random.normal([0.0], [mSigma], size = rExc.shape)
_, n, _ = control.forced_response(sysN, T = time_s, U = m)

uExc = np.zeros_like(rExc)
inCL = np.concatenate((r, uExc, n))

_, outCL, _ = control.forced_response(sysCL, T = time_s, U = inCL)
uCtrl = outCL[0:2]
z = outCL[2:4]


#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.1), detrendType = 'Linear')

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps
optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
freq_rps, Trz, Crz, Srr, Szz, Srz, TrzUnc, SrrNull, Snn = FreqTrans.FreqRespFuncEstNoise(r, z, optSpec)
freq_hz = freq_rps * rps2hz

print(1/np.sum(SrrNull, axis = -1))

I2 = np.repeat([np.eye(2)], Trz.shape[-1], axis=0).T
ToEstNom = Trz # Txy = Sxy / Sxx
ToEstUnc = TrzUnc # TxyUnc = np.abs(Sxn / Sxx)
ToEstCoh = Crz # Cxy = np.abs(Sxy)**2 / (Sxx * Syy) = (np.abs(Sxy) / Sxx) * (np.abs(Sxy) / Syy)

ToEstSNR = np.abs(ToEstNom / ToEstUnc)**2

# T = TNom + TUnc = uCtrl / uExc - uNull / uExc
# Lo = inv(TNom + TUnc) - I = LoEstNom + LoEstUnc
# LoEstNom = -I + TNom^-1
# LoEstUnc = -(I + TNom^-1 * TUnc)^-1 * TNom^-1 * TUnc * TNom^-1
LoEstNom = np.zeros_like(ToEstNom, dtype = complex)
LoEstUnc = np.zeros_like(ToEstUnc, dtype = complex)
LoEstCoh = np.zeros_like(ToEstCoh)

# inv = np.linalg.inv

# for i in range(SiEstNom.shape[-1]):
#     SiEstNomElem = SiEstNom[...,i]
#     SiEstUncElem = SiEstUnc[...,i]
#     SiEstNomInvElem = inv(SiEstNomElem)

#     LiEstNom[...,i] = -np.eye(2) + SiEstNomInvElem
#     LiEstUnc[...,i] = -inv(np.eye(2) + SiEstNomInvElem * SiEstUncElem) * SiEstNomInvElem * SiEstUncElem * SiEstNomInvElem
#     # LiEstCoh[...,i] = -np.eye(2) + inv(SiEstCoh[...,i])
#     LiEstCoh[...,i] = SiEstCoh[...,i]

# LiEstSNR = np.abs(LiEstNom / LiEstUnc)**2


#%% Bode Plot
# Linear Model Gain and Phase
gainToLinUnc_dB, phaseToLinUnc_deg = FreqTrans.GainPhase(ToLinUnc)
phaseToLinUnc_deg = np.unwrap(phaseToLinUnc_deg * deg2rad) * rad2deg

# Estimated Nominal Response
gainToEstNom_dB, phaseToEstNom_deg = FreqTrans.GainPhase(ToEstNom)
phaseToEstNom_deg = np.unwrap(phaseToEstNom_deg * deg2rad) * rad2deg

# Estimation Uncertain Response
gainToEstUnc_mag = np.abs(ToEstUnc)
gainToEstUnc_dB = FreqTrans.Gain(ToEstUnc)


if True:
    numOut, numIn = ToLinNom.shape[0:-1]
    fig, ax = plt.subplots(num=12, ncols=numOut, nrows=2*numIn)
    fig.tight_layout()

    ioArray = np.array(np.meshgrid([0,1], [0,1])).T.reshape(-1, 2)
    gainPlotArray = np.array(np.meshgrid([0,2], [0,1])).T.reshape(-1, 2)
    phasePlotArray = np.array(np.meshgrid([1,3], [0,1])).T.reshape(-1, 2)

    for iPlot, io in enumerate(ioArray):
        [iOut, iIn] = io
        gainPlot = gainPlotArray[iPlot]
        phasePlot = phasePlotArray[iPlot]

        [gOut, gIn] = gainPlot
        [pOut, pIn] = phasePlot

        ax[gOut, gIn].semilogx(freqLin_hz, gainToLinNom_dB[iOut, iIn], '-k', label='Linear Nominal')
        ax[gOut, gIn].semilogx(freqLin_hz, gainToLinUnc_dB[iOut, iIn], '--k', label='Linear Uncertainty')

        ax[gOut, gIn].semilogx(freq_hz[iIn], gainToEstNom_dB[iOut, iIn], '-b', label='Estimate Nominal')
        ax[gOut, gIn].semilogx(freq_hz[iIn], gainToEstUnc_dB[iOut, iIn], '--b', label='Estimate Uncertainty')

        ax[pOut, pIn].semilogx(freqLin_hz, phaseToLinNom_deg[iOut, iIn], '-k', label='Linear Nominal')
        ax[pOut, pIn].semilogx(freq_hz[iIn], phaseToEstNom_deg[iOut, iIn], '-b', label='Estimate Nominal')

        ax[gOut, gIn].grid(True); ax[pOut, pIn].grid(True)
        ax[gOut, gIn].tick_params(axis ='both'); ax[pOut, pIn].tick_params(axis ='both')

        # ax[pOut, pIn].set_ylim(-270, 90); ax.set_yticks([-270,-180,-90,0,90])
        ax[gOut, gIn].set_ylabel('Gain [dB]')
        ax[pOut, pIn].set_xlabel('Frequency [Hz]')
        ax[pOut, pIn].set_ylabel('Phase [deg]')

        ax[0, 0].legend()


#%% Nyquist Plot
# def PlotNyquistMimo(T, TUnc = None, fig = None, linestyle = '-', color = 'k', label='')

if True:
    numOut, numIn = ToLinNom.shape[0:-1]
    fig, ax = plt.subplots(num=13, ncols=numOut, nrows=numIn, sharex=True, sharey=True)
    fig.tight_layout()

    ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

    for io in ioArray:
        [iOut, iIn] = io

        ax[iOut, iIn].plot(ToLinNom[iOut, iIn].real, ToLinNom[iOut, iIn].imag, '-k', label = 'Linear')
        FreqTrans.PlotUncPts(ToLinNom[iOut, iIn], ToLinUnc[iOut, iIn], ax[iOut, iIn], color='k')
        # FreqTrans.PlotUncFill(ToLinNom[iOut, iIn], ToLinUnc[iOut, iIn], ax[iOut, iIn], color = 'k')


        ax[iOut, iIn].plot(ToEstNom[iOut, iIn].real, ToEstNom[iOut, iIn].imag, '.b', label = 'Estimation')
        FreqTrans.PlotUncPts(ToEstNom[iOut, iIn], ToEstUnc[iOut, iIn], ax[iOut, iIn], color='b')

        ax[iOut, iIn].grid(True)
        ax[iOut, iIn].tick_params(axis = 'both')
        # critPatch = patch.Ellipse((-1, 0), 2*0.4, 2*0.4, color='r', alpha=0.25)
        # ax[iOut, iIn].add_artist(critPatch)
        ax[iOut, iIn].plot(-1, 0, '+r')
        ax[iOut, iIn].set_xlabel('Real')
        ax[iOut, iIn].set_ylabel('Imag')

    ax[0,0].legend()

