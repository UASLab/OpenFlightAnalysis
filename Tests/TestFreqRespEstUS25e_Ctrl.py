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
import matplotlib.patches as patch
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


# Load the US25e linear model
exec(open("US25e_Lin.py").read())

#%% Define a linear systems
freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps

# freqLin_hz = np.logspace(np.log10(0.01), np.log10(25), 800)
freqLin_hz = np.linspace(1e-1, 1e1, 400)
freqLin_rps = freqLin_hz * hz2rps


# OL : Mixer -> Plant -> SCAS_FB
connectName = sysPlant.InputName[:7] + sysScas.InputName[1::3]
inKeep = sysMixer.InputName + sysPlant.InputName[-7:]
outKeep = sysScas.OutputName[2::4]

sysOL = Systems.ConnectName([sysMixer, sysPlant, sysScas], connectName, inKeep, outKeep)
sysOL.InputName = inKeep
sysOL.OutputName = outKeep

# CL: Ctrl -> Plant
connectName = ['cmdMotor', 'cmdElev', 'cmdRud', 'cmdAilL', 'cmdAilR', 'cmdFlapL', 'cmdFlapR', 'sensPhi', 'sensTheta', 'sensR']
inName = sysCtrl.InputName + sysPlant.InputName
inKeep = [inName[i - 1] for i in [1, 2, 3, 7, 8, 9, 17, 18, 19, 20, 21, 22, 23]]
outName = sysCtrl.OutputName + sysPlant.OutputName
outKeep = [outName[i - 1] for i in [8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]]

sysCL = Systems.ConnectName([sysCtrl, sysPlant], connectName, inKeep, outKeep)
sysCL.InputName = inKeep
sysCL.OutputName = outKeep

# Look at only the in-out of the OL
inList = [sysOL.InputName.index(s) for s in ['cmdP', 'cmdQ', 'cmdR']]
outList = [sysOL.OutputName.index(s) for s in ['fbP', 'fbQ', 'fbR']]

sysLaLinNom = sysOL[:, inList]


# Linear System Response
gainLaLinNom_mag, phaseLaLinNom_rad, _ = control.freqresp(sysLaLinNom, omega = freqLin_rps)
LaLinNom = gainLaLinNom_mag * np.exp(1j * phaseLaLinNom_rad)

gainLaLinNom_dB = 20 * np.log10(gainLaLinNom_mag)
phaseLaLinNom_deg = np.unwrap(phaseLaLinNom_rad) * rad2deg

I3 = np.repeat([np.eye(3)], LaLinNom.shape[-1], axis=0).T
svLaLinNom_mag = FreqTrans.Sigma(I3 + LaLinNom) # sigma(I + Li) = 1 / sigma(Si)
# svLaLinUnc_mag = FreqTrans.Sigma(LaLinUnc)


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
sigma = 1.0 * ampInit * deg2rad
r = 0.0 * np.random.normal(0.0, sigma, size = (len(ref_names), len(time_s)))

# Generate Sensor Noise (Processes Output Disturbance)
noise_names = ['phiDist', 'thetaDist', 'pDist', 'qDist', 'rDist', 'VDist', 'hDist']
sigma =  0.0 * ampInit * deg2rad
angleNoise = np.random.normal(0.0, sigma, size = (2, len(time_s)))
sigma =  1.0 * ampInit * deg2rad
pqrNoise = np.random.normal(0.0, sigma, size = (3, len(time_s)))
sigma =  0.0 * ampInit * deg2rad
airNoise = np.random.normal(0.0, sigma, size = (2, len(time_s)))
n = 1.0 * np.concatenate((angleNoise, pqrNoise, airNoise))


# Simulate the excitation through the system, with noise
u = np.concatenate((r, vExc, n))
_, out, stateSim = control.forced_response(sysCL, T = time_s, U = u, X0 = 0.0, transpose = False)

vFbName = sysCL.OutputName[:3]
vFb = -out[:3]

vFfName = sysCL.OutputName[3:6]
vFf = out[3:6]

zName = sysCL.OutputName[-7:]
z = out[-7:]


#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.2), detrendType = 'Linear', interpType = 'linear')

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps
optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
freq_rps, Teb, Ceb, Pee, Pbb, Peb, TebUnc, Pee_N, Pbb_N = FreqTrans.FreqRespFuncEstNoise(vExc, vFb + vExc, optSpec)
# _       , Tev, Cev, _  , Pvv, Pev = FreqTrans.FreqRespFuncEst(vExc, v, optSpec)

print(np.sum(Pee_N, axis = -1))

freq_hz = freq_rps * rps2hz

SaEstNom = Teb # Sa = Teb
SaEstUnc = TebUnc # TxyUnc = np.abs(Sxn / Sxx)
SaEstCoh = Ceb # Cxy = np.abs(Sxy)**2 / (Sxx * Syy) = (np.abs(Sxy) / Sxx) * (np.abs(Sxy) / Syy)

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
    LaEstUnc[...,i] = -inv(np.eye(3) + SaEstNomInvElem * SaEstUncElem) * (SaEstNomInvElem * SaEstUncElem * SaEstNomInvElem)
    # LaEstCoh[...,i] = -np.eye(3) + inv(SaEstCoh[...,i])
    LaEstCoh[...,i] = SaEstCoh[...,i]

LaEstSNR = np.abs(LaEstNom / LaEstUnc)**2
LaEstMeanSNR = np.mean(LaEstSNR, axis = -1)

# Estimated Nominal Response
gainLaEstNom_mag, phaseLaEstNom_deg = FreqTrans.GainPhase(LaEstNom, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)

I3 = np.repeat([np.eye(3)], Teb.shape[-1], axis=0).T
# svLaEstNom_mag = FreqTrans.Sigma(I3 + LaEstNom)
svLaEstNom_mag = 1 / FreqTrans.Sigma(SaEstNom) # sv(I + La) = 1 / sv(Sa)

# Estimation Uncertain Response
gainLaEstUnc_mag = np.abs(LaEstUnc)
gainLaEstUnc_dB = FreqTrans.Gain(LaEstUnc)

svLaEstUnc_mag = FreqTrans.Sigma(LaEstUnc)

# Estimation Coherence
# SaEstCohMin = np.min(np.min(SaEstCoh, axis = 0), axis = 0)


#%% Sigma Plot
svLaLinNomMin_mag = np.min(svLaLinNom_mag, axis = 0)
# svLaLinUncMax_mag = np.max(svLaLinUnc_mag, axis = 0)
# svLaLinLower_mag = svLaLinNomMin_mag - svLaLinUncMax_mag

svLaEstNomMin_mag = np.min(svLaEstNom_mag, axis = 0)
svLaEstUncMax_mag = np.max(svLaEstUnc_mag, axis = 0)
svLaEstLower_mag = svLaEstNomMin_mag - svLaEstUncMax_mag

LaEstCohMin = np.min(np.min(SaEstCoh, axis = 0), axis = 0)

if True:
    fig = 20
    # fig = FreqTrans.PlotSigma(freqLin_hz, svLaLinNom_mag, coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
    # fig = FreqTrans.PlotSigma(freqLin_hz, svLaLinNomMin_mag, lower = svLaEstLower_mag, coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
    fig = FreqTrans.PlotSigma(freqLin_hz, svLaLinNomMin_mag, coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
    # fig = FreqTrans.PlotSigma(freq_hz, svLaEstNom_mag, coher_nd = LaEstCohMin, color = 'b', fig = fig, label = 'Estimation (MIMO)')
    fig = FreqTrans.PlotSigma(freq_hz[0], svLaEstNomMin_mag, lower = svLaEstLower_mag, coher_nd = LaEstCohMin, color = 'b', fig = fig, label = 'Estimation with Uncertainty')
    # fig = FreqTrans.PlotSigma(freqLin_hz, 0.4 * np.ones_like(freqLin_hz), color = 'r', linestyle = '--', fig = fig, label = 'Critical Limit')
    ax = fig.get_axes()
    ax[0].set_xlim(0, 10)
    # ax[0].set_yscale('log')
    ax[0].set_ylim(0, 2)


#%% Vector Margin Plots
inPlot = vExcNames # Elements of vExcNames
outPlot = vFbName # Elements of vFbName

numOut = len(outPlot); numIn = len(inPlot)
ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

#
rCritLaLinNom_mag = np.abs(LaLinNom - (-1 + 0j))
# rCritLaLinNom_mag, rCritLaLinUnc_mag, rCritLaLinLower_mag = FreqTrans.DistCrit(LaLinNom, LaLinUnc, typeUnc = 'circle')

rCritLaEstNom_mag, rCritLaEstUnc_mag, rCritLaEstLower_mag = FreqTrans.DistCrit(LaEstNom, LaEstUnc, typeUnc = 'circle')
#rCritLaEstNom_mag, rCritLaEstUnc_mag, rCritLaEstLower_mag = FreqTrans.DistCrit(LaEstNom, LaEstUnc, typeUnc = 'ellipse')
#rCritLaEstNom_mag, rCritLaEstUnc_mag, rCritLaEstLower_mag, pContLaEst_mag = FreqTrans.DistCritEllipse(LaEstNom, LaEstUnc) # Returns closest approach points

if True:
    for iPlot, io in enumerate(ioArray):
        iOut, iIn = io

        outName = outPlot[iOut]
        inName = inPlot[iIn]

        fig = 40 + iPlot
        fig = FreqTrans.PlotSigma(freqLin_hz, rCritLaLinNom_mag[iOut, iIn], coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
        # fig = FreqTrans.PlotSigma(freqLin_hz, rCritLaLinNom_mag[iOut, iIn], lower = rCritLaLinLower_mag[iOut, iIn], coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
        fig = FreqTrans.PlotSigma(freq_hz[0], rCritLaEstNom_mag[iOut, iIn], lower = rCritLaEstLower_mag[iOut, iIn], coher_nd = LaEstCoh[iOut, iIn], color = 'b', fig = fig, label = 'Excitation (MIMO)')
        fig = FreqTrans.PlotSigma(freq_hz[0, sigIndx[iIn]], rCritLaEstNom_mag[iOut, iIn, sigIndx[iIn]], lower = rCritLaEstLower_mag[iOut, iIn, sigIndx[iIn]], coher_nd = LaEstCoh[iOut, iIn, sigIndx[iIn]], color = 'g', fig = fig, label = 'Excitation (SIMO)')
        fig = FreqTrans.PlotSigma(freq_hz[0], 0.4 * np.ones_like(freq_hz[0]), color = 'r', linestyle = '--', fig = fig, label = 'Critical Limit')

        fig.suptitle(inName + ' to ' + outName, size = 20)
        ax = fig.get_axes()
        ax[0].set_xlim(0, 10)
        ax[0].set_ylim(0, 2)


#%% Nyquist Plots
LaEstUncMag = abs(LaEstUnc);

if False:
    for iPlot, io in enumerate(ioArray):
        iOut, iIn = io

        outName = outPlot[iOut]
        inName = inPlot[iIn]

        fig = 60 + iPlot

        fig = FreqTrans.PlotNyquist(LaLinNom[iOut, iIn], fig = fig, color = 'k', label = 'Linear')
        fig = FreqTrans.PlotNyquist(LaEstNom[iOut, iIn], LaEstUncMag[iOut, iIn], fig = fig, color = 'b', label = 'Estimation (MIMO)')
        fig = FreqTrans.PlotNyquist(LaEstNom[iOut, iIn, sigIndx[iIn]], LaEstUncMag[iOut, iIn, sigIndx[iIn]], fig = fig, color = 'g', label = 'Estimation (SIMO)')

        fig = FreqTrans.PlotNyquist(np.asarray([-1 + 0j]), TUnc = np.asarray([0.4 + 0.4j]), fig = fig, color = 'r', linestyle = ':', label = 'Critical Region')

        fig.suptitle(inName + ' to ' + outName, size = 20)
        ax = fig.get_axes()
        ax[0].set_xlim(-3, 1)
        ax[0].set_ylim(-2, 2)


#%% Bode Plots
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

        fig = FreqTrans.PlotBodeMag(freqLin_hz, gainLaLinNom_mag[iOut, iIn], coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
        fig = FreqTrans.PlotBodeMag(freq_hz[0], gainLaEstNom_mag[iOut, iIn], LaEstCoh[iOut, iIn], fig = fig, color = 'b', label = 'Estimation (MIMO)')
        fig = FreqTrans.PlotBodeMag(freq_hz[0, sigIndx[iIn]], gainLaEstNom_mag[iOut, iIn, sigIndx[iIn]], SaEstCoh[iOut, iIn, sigIndx[iIn]], fig = fig, color = 'g', label = 'Estimation (SIMO)')

        # fig = FreqTrans.PlotBodeMag(freqLin_hz, gainLaLinUnc_mag[iOut, iIn], fig = fig, color = 'k', label = 'Linear')
        fig = FreqTrans.PlotBodeMag(freq_hz[0], gainLaEstUnc_mag[iOut, iIn], SaEstCoh[iOut, iIn], fig = fig, color = 'b', label = 'Estimation Uncertainty (MIMO)')
        fig = FreqTrans.PlotBodeMag(freq_hz[0, sigIndx[iIn]], gainLaEstUnc_mag[iOut, iIn, sigIndx[iIn]], SaEstCoh[iOut, iIn, sigIndx[iIn]], fig = fig, color = 'g', label = 'Estimation Uncertainty (SIMO)')

        fig.suptitle(inName + ' to ' + outName, size = 20)
        ax = fig.get_axes()
        ax[0].set_xlim(0.1, 10)
