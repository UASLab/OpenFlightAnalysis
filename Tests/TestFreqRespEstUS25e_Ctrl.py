"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Example script for testing Frequency Response Estimation - US25e with Noise.
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
from Core import Systems


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


# Load the US25e linear model
exec(open("US25e_Lin.py").read())

#%% Define a linear systems
freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps

# freqLin_hz = np.logspace(np.log10(0.01), np.log10(25), 800)
freqLin_hz = np.linspace(1e-1, 1e1, 400)
freqLin_rps = freqLin_hz * hz2rps


# OL : Mixer -> Plant -> SCAS_FB [Remain Open at vFb]
connectName = sysMixer.OutputName + sysScas.InputName[1::3]
inKeep = sysMixer.InputName[1:] + sysPlant.InputName[7:]
outKeep = sysPlant.OutputName + sysScas.OutputName[0::4] + sysScas.OutputName[1::4] + sysScas.OutputName[2::4]
sysOL = Systems.ConnectName([sysMixer, sysPlant, sysScas], connectName, inKeep, outKeep)

if False:
    _ = control.bode_plot(sysOL[7,0], omega_limits = [0.01, 500], Hz = True, dB = True)
    _ = control.bode_plot(sysOL[8,1], omega_limits = [0.01, 500], Hz = True, dB = True)
    _ = control.bode_plot(sysOL[9,2], omega_limits = [0.01, 500], Hz = True, dB = True)

# CL: Ctrl -> Plant [Connect at vCmd]
connectName = sysMixer.OutputName + sysScas.InputName[1::3] + sysMixer.InputName[1:]
inKeep = sysScas.InputName[2::3] + sysScas.InputName[0::3] + sysPlant.InputName[7:]
outKeep = sysScas.OutputName[0::4] + sysPlant.OutputName
sysCL = Systems.ConnectName([sysMixer, sysPlant, sysScas], connectName, inKeep, outKeep)

if False:
    plt.figure(2)
    _, stepPhiOut = control.step_response(sysCL, input=3, T=time_s)
    plt.subplot(4,1,1); plt.plot(time_s, stepPhiOut[0])
    plt.subplot(4,1,2); plt.plot(time_s, stepPhiOut[3])
    plt.subplot(4,1,3); plt.plot(time_s, stepPhiOut[5])
    plt.subplot(4,1,4); plt.plot(time_s, stepPhiOut[7])
    
if False:
    _ = control.bode_plot(sysCL[0,0], omega_limits = [0.01, 500], Hz = True, dB = True)
    _ = control.bode_plot(sysCL[1,1], omega_limits = [0.01, 500], Hz = True, dB = True)
    _ = control.bode_plot(sysCL[2,2], omega_limits = [0.01, 500], Hz = True, dB = True)

    
#%%
#
pSigma = np.array([0.0, 0.0, 0.0])
pIn = 1.0 * control.ss([],[],[],np.diag(pSigma))


pqrSigma = np.array([1.0, 1.0, 1.0]) * 4.0 * deg2rad
mIn = 1.0 * control.ss([], [], [], np.diag(pqrSigma))


# Chunk the OL and CL models into Nominal and Uncertain components of La and Ta
sysLaLinNom = sysOL[7:10, :][:, 0:3]
sysLaLinNom.InputName = sysOL.InputName[0:3]
sysLaLinNom.OutputName = sysOL.OutputName[7:10]
if False:
    _ = control.bode_plot(sysLaLinNom[0,0], omega_limits = [0.01, 500], Hz = True, dB = True)
    _ = control.bode_plot(sysLaLinNom[1,1], omega_limits = [0.01, 500], Hz = True, dB = True)
    _ = control.bode_plot(sysLaLinNom[2,2], omega_limits = [0.01, 500], Hz = True, dB = True)
    
sysLaLinUnc = sysOL[7:10, :][:, [3,4,7]] * mIn.D
sysLaLinUnc.InputName = sysOL.InputName[3:5] + sysOL.InputName[7:8]
sysLaLinUnc.OutputName = sysOL.OutputName[7:10]
if False:
    _ = control.bode_plot(sysLaLinUnc[0,0], omega_limits = [0.01, 500], Hz = True, dB = True)
    _ = control.bode_plot(sysLaLinUnc[1,1], omega_limits = [0.01, 500], Hz = True, dB = True)
    _ = control.bode_plot(sysLaLinUnc[2,2], omega_limits = [0.01, 500], Hz = True, dB = True)

sysTaLinNom = sysCL[0:3, :][:, 0:3]
sysTaLinNom.InputName = sysCL.InputName[0:3]
sysTaLinNom.OutputName = sysCL.OutputName[0:3]
if False:
    _ = control.bode_plot(sysTaLinNom[0,0], omega_limits = [0.01, 500], Hz = True, dB = True)
    _ = control.bode_plot(sysTaLinNom[1,1], omega_limits = [0.01, 500], Hz = True, dB = True)
    _ = control.bode_plot(sysTaLinNom[2,2], omega_limits = [0.01, 500], Hz = True, dB = True)

sysTaLinUnc = sysCL[0:3, :][:, [6,7,10]] * mIn.D
sysTaLinUnc.InputName = sysCL.InputName[6:8] + sysCL.InputName[10:11]
sysTaLinUnc.OutputName = sysCL.OutputName[0:3]

if False:
    _ = control.bode_plot(sysTaLinUnc[0,0], omega_limits = [0.01, 500], Hz = True, dB = True)
    _ = control.bode_plot(sysTaLinUnc[1,1], omega_limits = [0.01, 500], Hz = True, dB = True)
    _ = control.bode_plot(sysTaLinUnc[2,2], omega_limits = [0.01, 500], Hz = True, dB = True)

# Linear System Response
gainLaLinNom_mag, phaseLaLinNom_rad, _ = control.freqresp(sysLaLinNom, omega = freqLin_rps)
LaLinNom = gainLaLinNom_mag * np.exp(1j * phaseLaLinNom_rad)

gainLaLinUnc_mag, phaseLaLinUnc_rad, _ = control.freqresp(sysLaLinUnc, omega = freqLin_rps)
LaLinUnc = gainLaLinUnc_mag * np.exp(1j * phaseLaLinUnc_rad)

gainTaLinNom_mag, phaseTaLinNom_rad, _ = control.freqresp(sysTaLinNom, omega = freqLin_rps)
TaLinNom = gainTaLinNom_mag * np.exp(1j * phaseTaLinNom_rad)

gainTaLinUnc_mag, phaseTaLinUnc_rad, _ = control.freqresp(sysTaLinUnc, omega = freqLin_rps)
TaLinUnc = gainTaLinUnc_mag * np.exp(1j * phaseTaLinUnc_rad)


#%% Excitation
numExc = 3
numCycles = 3
ampInit = 4.0 * deg2rad
ampFinal = ampInit
freqMinDes_rps = 0.1 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 9.7 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (10 / freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
freqNull_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

# Generate Schroeder MultiSine Signal
ampExc_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
vExc, _, sigExc = GenExcite.MultiSine(freqExc_rps, ampExc_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')
vExcNames = ['excP', 'excQ', 'excR']

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]

# Null Frequencies
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

# Reference Inputs
ref_names = ['refPhi', 'refTheta', 'refYaw']
sigma = 0.0 * ampInit * deg2rad
r = 0.0 * np.random.normal(0.0, sigma, size = (len(ref_names), len(time_s)))

# Generate Sensor Noise (Processes Output Disturbance)
noise_names = ['phiDist', 'thetaDist', 'pDist', 'qDist', 'rDist', 'VDist', 'hDist']

angNoise = np.random.normal(np.array([0.0, 0.0]), [0.0, 0.0], size = (len(time_s), 2))
pqrNoise = np.random.normal(np.array([0.0, 0.0, 0.0]), pqrSigma, size = (len(time_s), 3))
airNoise = np.random.normal(np.array([0.0, 0.0]), [0.0, 0.0], size = (len(time_s), 2))
n = np.hstack((angNoise, pqrNoise, airNoise)).T


# Simulate the excitation through the system, with noise
u = np.concatenate((vExc, r, n))
_, out, stateSim = control.forced_response(sysCL, T = time_s, U = u, X0 = 0.0, transpose = False)

vCmdName = sysCL.OutputName[:3]
vCmd = out[:3]

zName = sysCL.OutputName[-7:]
z = out[-7:]

#v = -(vCmd - vExc)
v = vCmd # FIXIT


#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear', interpType = 'linear')

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps
optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
freq_rps, Txy, Cxy, Pxx, Pyy, Pxy, TxyUnc, PxxNull, Pnn = FreqTrans.FreqRespFuncEstNoise(vExc, v, optSpec)
# _       , Txy, Cxy, _  , Pvv, Pev = FreqTrans.FreqRespFuncEst(vExc, v, optSpec)

freq_hz = freq_rps * rps2hz

print(np.sum(PxxNull, axis = -1) / np.sum(Pxx, axis = -1))

I3 = np.repeat([np.eye(3)], Txy.shape[-1], axis=0).T
TaEstNom = -Txy
TaEstUnc = TxyUnc
TaEstCoh = Cxy

SaEstNom = I3 - TaEstNom # Sa = I - Ta
SaEstUnc = -TaEstUnc
SaEstCoh = TaEstCoh

SaEstSNR = np.abs(SaEstNom / SaEstUnc)**2
SaEstMeanSNR = np.mean(SaEstSNR, axis = -1)

# T = TNom + TUnc = vCtrl / vExc - vNull / vExc
# La = inv(TNom + TUnc) - I = LaEstNom + LaEstUnc
# LaEstNom = -I + SaEstNom^-1
# LaEstUnc = -(I + SaEstNom^-1 * SaEstUnc)^-1 * SaEstNom^-1 * SaEstUnc * SaEstNom^-1
LaEstNom = np.zeros_like(SaEstNom, dtype = complex)
LaEstUnc = np.zeros_like(SaEstUnc, dtype = complex)
LaEstCoh = np.zeros_like(SaEstCoh)

inv = np.linalg.inv
for i in range(SaEstNom.shape[-1]):
    SaEstNomElem = SaEstNom[...,i]
    SaEstUncElem = SaEstUnc[...,i]
    SaEstNomInvElem = inv(SaEstNomElem)

    LaEstNom[...,i] = -np.eye(3) + SaEstNomInvElem
    LaEstUnc[...,i] = -inv(np.eye(3) + SaEstNomInvElem @ SaEstUncElem) @ (SaEstNomInvElem @ SaEstUncElem @ SaEstNomInvElem)
    # LaEstCoh[...,i] = -np.eye(3) + inv(SaEstCoh[...,i])
    LaEstCoh[...,i] = SaEstCoh[...,i]

LaEstSNR = np.abs(LaEstNom / LaEstUnc)**2
LaEstMeanSNR = np.mean(LaEstSNR, axis = -1)


#%% Sigma Plot
I3 = np.repeat([np.eye(3)], LaLinNom.shape[-1], axis=0).T
svLaLinNom_mag = FreqTrans.Sigma(I3 + LaLinNom) # sigma(I + Li) = 1 / sigma(Si)
svLaLinNomMin_mag = np.min(svLaLinNom_mag, axis = 0)

svLaLinUnc_mag = FreqTrans.Sigma(LaLinUnc)
svLaLinUncMax_mag = np.max(svLaLinUnc_mag, axis = 0)
svLaLinLower_mag = svLaLinNomMin_mag - svLaLinUncMax_mag

# I3 = np.repeat([np.eye(3)], SaEstNom.shape[-1], axis=0).T
# svLaEstNom_mag = FreqTrans.Sigma(I3 + LaEstNom)
svLaEstNom_mag = 1 / FreqTrans.Sigma(SaEstNom) # sv(I + La) = 1 / sv(Sa)
svLaEstNomMin_mag = np.min(svLaEstNom_mag, axis = 0)

svLaEstUnc_mag = FreqTrans.Sigma(LaEstUnc)
svLaEstUncMax_mag = np.max(svLaEstUnc_mag, axis = 0)
svLaEstLower_mag = svLaEstNomMin_mag - svLaEstUncMax_mag

LaEstCohMin = np.min(np.min(SaEstCoh, axis = 0), axis = 0)

if False:
    fig = 20
    # fig = FreqTrans.PlotSigma(freqLin_hz, svLaLinNom_mag, coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
    # fig = FreqTrans.PlotSigma(freqLin_hz, svLaLinNomMin_mag, svUnc_mag = svLaEstUncMax_mag, coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
    fig = FreqTrans.PlotSigma(freqLin_hz, svLaLinNomMin_mag, coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
    # fig = FreqTrans.PlotSigma(freq_hz, svLaEstNom_mag, coher_nd = LaEstCohMin, color = 'b', fig = fig, label = 'Estimation (MIMO)')
    fig = FreqTrans.PlotSigma(freq_hz[0], svLaEstNomMin_mag, svUnc_mag = svLaEstLower_mag, coher_nd = LaEstCohMin, color = 'b', fig = fig, label = 'Estimation with Uncertainty')
    # fig = FreqTrans.PlotSigma(freqLin_hz, 0.4 * np.ones_like(freqLin_hz), color = 'r', linestyle = '--', fig = fig, label = 'Critical Limit')
    ax = fig.get_axes()
    ax[0].set_xlim(0, 10)
    # ax[0].set_yscale('log')
    ax[0].set_ylim(0, 2)


#%% Vector Margin Plots
inPlot = sysCL.InputName[:3]
outPlot = [inName.replace('exc', 'fb') for inName in inPlot] 

numOut = len(outPlot); numIn = len(inPlot)
ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

#
rCritLaLinNom_mag, rCritLaLinUnc_mag, rCritLaLinLower_mag = FreqTrans.DistCrit(LaLinNom, LaLinUnc, typeUnc = 'circle')
rCritLaEstNom_mag, rCritLaEstUnc_mag, rCritLaEstLower_mag = FreqTrans.DistCrit(LaEstNom, LaEstUnc, typeUnc = 'circle')

if False:
    for iPlot, io in enumerate(ioArray):
        iOut, iIn = io

        outName = outPlot[iOut]
        inName = inPlot[iIn]

        fig = 40 + iPlot
#        fig = FreqTrans.PlotVectorMargin(freqLin_hz, rCritLaLinNom_mag[iOut, iIn], coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
        fig = FreqTrans.PlotVectorMargin(freqLin_hz, rCritLaLinNom_mag[iOut, iIn], vmUnc_mag = rCritLaLinUnc_mag[iOut, iIn], coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
        fig = FreqTrans.PlotVectorMargin(freq_hz[0], rCritLaEstNom_mag[iOut, iIn], vmUnc_mag = rCritLaEstUnc_mag[iOut, iIn], coher_nd = LaEstCoh[iOut, iIn], color = 'b', fig = fig, label = 'Excitation (MIMO)')
        fig = FreqTrans.PlotVectorMargin(freq_hz[0, sigIndx[iIn]], rCritLaEstNom_mag[iOut, iIn, sigIndx[iIn]], vmUnc_mag = rCritLaEstUnc_mag[iOut, iIn, sigIndx[iIn]], coher_nd = LaEstCoh[iOut, iIn, sigIndx[iIn]], color = 'g', fig = fig, label = 'Excitation (SIMO)')
        fig = FreqTrans.PlotVectorMargin(freqLin_hz, 0.4 * np.ones_like(freqLin_hz), color = 'r', linestyle = '--', fig = fig, label = 'Critical Limit')

        fig.suptitle(inName + ' to ' + outName, size = 20)
        ax = fig.get_axes()
        ax[0].set_xlim(0, 10)
        ax[0].set_ylim(0, 2)


#%% Nyquist Plots - Ta
TaLinUncMag = abs(TaLinUnc);
TaEstUncMag = abs(TaEstUnc);

if False:
    for iPlot, io in enumerate(ioArray):
        iOut, iIn = io

        outName = outPlot[iOut]
        inName = inPlot[iIn]

        fig = 60 + iPlot

        fig = FreqTrans.PlotNyquist(TaLinNom[iOut, iIn], TaLinUncMag[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear')
        fig = FreqTrans.PlotNyquist(TaEstNom[iOut, iIn], TaEstUncMag[iOut, iIn], fig = fig, color = 'b', label = 'Estimate (MIMO)')
        fig = FreqTrans.PlotNyquist(TaEstNom[iOut, iIn, sigIndx[iIn]], TaEstUncMag[iOut, iIn, sigIndx[iIn]], fig = fig, color = 'g', label = 'Estimate (SIMO)')

        fig = FreqTrans.PlotNyquist(np.asarray([-1 + 0j]), TUnc = np.asarray([0.4 + 0.4j]), fig = fig, color = 'r', linestyle = ':', label = 'Critical Region')

        fig.suptitle(inName + ' to ' + outName, size = 20)
        ax = fig.get_axes()
        ax[0].set_xlim(-3, 1)
        ax[0].set_ylim(-2, 2)

#%% Bode Plots - Ta
gainTaLinNom_mag, phaseTaLinNom_deg = FreqTrans.GainPhase(TaLinNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTaLinUnc_mag = FreqTrans.Gain(TaLinUnc, magUnit = 'mag')

gainTaEstNom_mag, phaseTaEstNom_deg = FreqTrans.GainPhase(TaEstNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTaEstUnc_mag = FreqTrans.Gain(TaEstUnc, magUnit = 'mag')

if False:
    for iPlot, io in enumerate(ioArray):
        iOut, iIn = io

        outName = outPlot[iOut]
        inName = inPlot[iIn]

        fig = 80 + iPlot

        fig = FreqTrans.PlotBode(freqLin_hz, gainTaLinNom_mag[iOut, iIn], phaseTaLinNom_deg[iOut, iIn], coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
        fig = FreqTrans.PlotBode(freq_hz[0], gainTaEstNom_mag[iOut, iIn], phaseTaEstNom_deg[iOut, iIn], TaEstCoh[iOut, iIn], gainUnc_mag = gainTaEstUnc_mag[iOut, iIn], fig = fig, color = 'b', label = 'Estimation (MIMO)')
        fig = FreqTrans.PlotBode(freq_hz[0, sigIndx[iIn]], gainTaEstNom_mag[iOut, iIn, sigIndx[iIn]], phaseTaEstNom_deg[iOut, iIn, sigIndx[iIn]], TaEstCoh[iOut, iIn, sigIndx[iIn]], gainUnc_mag = gainTaEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, color = 'g', label = 'Estimation (SIMO)')

        fig.suptitle(inName + ' to ' + outName, size = 20)
        # ax[0].set_ylim(-2, 2)


#%% Nyquist Plots - La
LaLinUncMag = abs(LaLinUnc);
LaEstUncMag = abs(LaEstUnc);

if True:
    for iPlot, io in enumerate(ioArray):
        iOut, iIn = io

        outName = outPlot[iOut]
        inName = inPlot[iIn]

        fig = 60 + iPlot

        fig = FreqTrans.PlotNyquist(LaLinNom[iOut, iIn], LaLinUncMag[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear')
        fig = FreqTrans.PlotNyquist(LaEstNom[iOut, iIn], LaEstUncMag[iOut, iIn], fig = fig, color = 'b', label = 'Estimate (MIMO)')
        fig = FreqTrans.PlotNyquist(LaEstNom[iOut, iIn, sigIndx[iIn]], LaEstUncMag[iOut, iIn, sigIndx[iIn]], fig = fig, color = 'g', label = 'Estimate (SIMO)')

        fig = FreqTrans.PlotNyquist(np.asarray([-1 + 0j]), TUnc = np.asarray([0.4 + 0.4j]), fig = fig, color = 'r', linestyle = ':', label = 'Critical Region')

        fig.suptitle(inName + ' to ' + outName, size = 20)
        ax = fig.get_axes()
        ax[0].set_xlim(-3, 1)
        ax[0].set_ylim(-2, 2)


#%% Bode Plots
gainLaLinNom_mag, phaseLaLinNom_deg = FreqTrans.GainPhase(LaLinNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainLaLinUnc_mag = FreqTrans.Gain(LaLinUnc, magUnit = 'mag')

gainLaEstNom_mag, phaseLaEstNom_deg = FreqTrans.GainPhase(LaEstNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainLaEstUnc_mag = FreqTrans.Gain(LaEstUnc, magUnit = 'mag')


if False:
    for iPlot, io in enumerate(ioArray):
        iOut, iIn = io

        outName = outPlot[iOut]
        inName = inPlot[iIn]

        fig = 80 + iPlot

        fig = FreqTrans.PlotBode(freqLin_hz, gainLaLinNom_mag[iOut, iIn], phaseLaLinNom_deg[iOut, iIn], coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
        fig = FreqTrans.PlotBode(freq_hz[0], gainLaEstNom_mag[iOut, iIn], phaseLaEstNom_deg[iOut, iIn], LaEstCoh[iOut, iIn], gainUnc_mag = gainLaEstUnc_mag[iOut, iIn], fig = fig, color = 'b', label = 'Estimation (MIMO)')
        fig = FreqTrans.PlotBode(freq_hz[0, sigIndx[iIn]], gainLaEstNom_mag[iOut, iIn, sigIndx[iIn]], phaseLaEstNom_deg[iOut, iIn, sigIndx[iIn]], LaEstCoh[iOut, iIn, sigIndx[iIn]], gainUnc_mag = gainLaEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, color = 'g', label = 'Estimation (SIMO)')

        fig.suptitle(inName + ' to ' + outName, size = 20)
        # ax[0].set_ylim(-2, 2)


#%% Bode Mag - Noise Estimation Validation
if False:
    for iPlot, io in enumerate(ioArray):
        iOut, iIn = io

        outName = outPlot[iOut]
        inName = inPlot[iIn]

        fig = 100 + iPlot

        fig = FreqTrans.PlotGainType(freqLin_hz, gainLaLinNom_mag[iOut, iIn], coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
        fig = FreqTrans.PlotGainType(freq_hz[0], gainLaEstNom_mag[iOut, iIn], LaEstCoh[iOut, iIn], fig = fig, color = 'b', label = 'Estimation (MIMO)')
        fig = FreqTrans.PlotGainType(freq_hz[0, sigIndx[iIn]], gainLaEstNom_mag[iOut, iIn, sigIndx[iIn]], SaEstCoh[iOut, iIn, sigIndx[iIn]], fig = fig, color = 'g', label = 'Estimation (SIMO)')

        fig = FreqTrans.PlotGainType(freqLin_hz, gainLaLinUnc_mag[iOut, iIn], fig = fig, linestyle = ':', color = 'k', label = 'Linear Uncertainty')
        fig = FreqTrans.PlotGainType(freq_hz[0], gainLaEstUnc_mag[iOut, iIn], SaEstCoh[iOut, iIn], fig = fig, linestyle = ':', color = 'b', label = 'Estimation Uncertainty (MIMO)')
        fig = FreqTrans.PlotGainType(freq_hz[0, sigIndx[iIn]], gainLaEstUnc_mag[iOut, iIn, sigIndx[iIn]], SaEstCoh[iOut, iIn, sigIndx[iIn]], fig = fig, linestyle = ':', color = 'g', label = 'Estimation Uncertainty (SIMO)')

        fig.suptitle(inName + ' to ' + outName, size = 20)
        ax = fig.get_axes()
        ax[0].set_xlim(0.1, 10)
