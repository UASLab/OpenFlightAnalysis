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

freqSys_hz = np.logspace(np.log10(0.01), np.log10(25), 800)
freqSys_rps = freqSys_hz * hz2rps


# OL : Mixer -> Plant -> SCAS_FB
inNames = sysMixer_InputNames + sysPlant_InputNames + sysScas_InputNames
outNames = sysMixer_OutputNames + sysPlant_OutputNames + sysScas_OutputNames

sysOL_ConnectNames = sysPlant_InputNames[:7] + sysScas_InputNames[1::3]
sysOL_InputNames = sysMixer_InputNames + sysPlant_InputNames[-7:]
sysOL_OutputNames = sysScas_OutputNames[2::4]

sysOL = Systems.ConnectName(control.append(sysMixer, sysPlant, sysScas), inNames, outNames, sysOL_ConnectNames, sysOL_InputNames, sysOL_OutputNames)


# CL: Ctrl -> Plant
inNames = sysCtrl_InputNames + sysPlant_InputNames
outNames = sysCtrl_OutputNames + sysPlant_OutputNames

sysCL_ConnectNames = ['cmdMotor', 'cmdElev', 'cmdRud', 'cmdAilL', 'cmdAilR', 'cmdFlapL', 'cmdFlapR', 'sensPhi', 'sensTheta', 'sensR']
sysCL_InputNames = [inNames[i-1] for i in [1, 2, 3, 7, 8, 9, 17, 18, 19, 20, 21, 22, 23]]
sysCL_OutputNames = [outNames[i-1] for i in [8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]]

sysCL = Systems.ConnectName(control.append(sysCtrl, sysPlant), inNames, outNames, sysCL_ConnectNames, sysCL_InputNames, sysCL_OutputNames)


# Look at only the in-out of the OL
nFreq = len(freqSys_rps)
sysSimOL_InputNames = ['cmdP',  'cmdQ', 'cmdR']; nIn = len(sysSimOL_InputNames)
sysSimOL_OutputNames = ['fbP', 'fbQ', 'fbR']; nOut = len(sysSimOL_OutputNames)

inList = [sysOL_InputNames.index(s) for s in sysSimOL_InputNames]
outList = [sysOL_OutputNames.index(s) for s in sysSimOL_OutputNames]
sysSimOL_gain_nd = np.zeros([nIn, nOut, nFreq])
sysSimOL_phase_rad = np.zeros([nIn, nOut, nFreq])
sysSimOL = np.zeros([nIn, nOut, nFreq], dtype=complex)

for iOut, outEntry in enumerate(outList):
    for iIn, inEntry in enumerate(inList):
        sysSimOL_gain_nd[iOut, iIn], sysSimOL_phase_rad[iOut, iIn], _ = control.bode_plot(sysOL[outEntry, inEntry], omega = freqSys_rps, Plot = False)
        Treal, Timag, _ = control.nyquist_plot(sysOL[outEntry, inEntry], omega = freqSys_rps, Plot = False)
        sysSimOL[iOut, iIn, :] = Treal + 1j*Timag

#sysSimOL_gain_nd[sysSimOL_gain_nd == 0] = 1e-6
sysSimOL_gain_dB = 20*np.log10(sysSimOL_gain_nd)
sysSimOL_phase_deg = sysSimOL_phase_rad * rad2deg
sysSimOL_rCrit_mag = np.abs(sysSimOL - (-1 + 0j))

sysSimOL_sigma, _ = FreqTrans.Sigma(sysSimOL, typeSigma = 2)


#%% Excitation
numExc = 3
numCycles = 1
ampInit = 4.0 * deg2rad
ampFinal = ampInit
freqMinDes_rps = 0.1 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 10 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (10 / freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
freqNull_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

# Generate Schroeder MultiSine Signal
ampExc_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
exc, _, sigExc = GenExcite.MultiSine(freqExc_rps, ampExc_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')
exc_names = ['excP', 'excQ', 'excR']

# Generate Noise
dist_names = ['phiDist', 'thetaDist', 'pDist', 'qDist', 'rDist', 'VDist', 'hDist']
pqrDist = np.random.normal(0, 0.25 * ampInit, size = (3, len(time_s)))
angleDist = np.cumsum(pqrDist[[0,1],:], axis = -1)
airDist = np.random.normal(0, 0.0, size = (2, len(time_s)))
dist = np.concatenate((angleDist, pqrDist, airDist))
dist = 1.0 * dist

# Reference Inputs
ref_names = ['refPhi', 'refTheta', 'refYaw']
shapeRef = (len(ref_names), len(time_s))
ref = np.random.normal(0, 1.0 * ampInit, size = (3, len(time_s)))
ref[1] = 2.0 * deg2rad + ref[1]
ref = 0.0 * ref

# Simulate the excitation through the system, with noise
sysExc_InputNames = sysCL_InputNames
sysExc_OutputNames = sysCL_OutputNames
sysExc = control.StateSpace(sysCL.A, sysCL.B, sysCL.C, sysCL.D)

u = np.concatenate((ref, exc, dist))
_, out, stateSim = control.forced_response(sysExc, T = time_s, U = u, X0 = 0.0, transpose = False)

# shift output time to represent the next frame, pad t=0 with 0.0
#out = np.concatenate((np.zeros((out.shape[0],1)), out[:,1:-1]), axis=1) # this is handled by a time delay on sensors in the linear simulation

fb_names = sysExc_OutputNames[:3]
fb = out[:3]

v_names = sysExc_OutputNames[3:6]
v = out[3:6]

sens_names = sysExc_OutputNames[-7:]
sens = out[-7:]

#plt.plot(time_s, exc[1], time_s, v[1], time_s, fb[1], time_s, pqrDist[1])



#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear', interpType = 'linear')
optSpecN = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear', interpType = 'linear')

# Excited Frequencies per input channel
optSpec.freq = []
for iChan in range(0, numExc):
    optSpec.freq.append(freqExc_rps[sigIndx[iChan]])
optSpec.freq = np.asarray(optSpec.freq)

# Null Frequencies
optSpecN.freq = freqNull_rps

# FRF Estimate
freq_rps, Teb, Ceb, Pee, Pbb, Peb, TebUnc, Pbb_N = FreqTrans.FreqRespFuncEstNoise(exc, fb, optSpec, optSpecN)
_       , Tev, Cev, _  , Pvv, Pev = FreqTrans.FreqRespFuncEst(exc, v, optSpec)

freq_hz = freq_rps * rps2hz


# Form the Frequency Response, T = Teb @ Tev^-1
T = np.zeros_like(Tev, dtype=complex)
TUnc = np.zeros_like(Tev, dtype=float)
C = np.zeros_like(Tev, dtype=float)

for i in range(T.shape[-1]):
    T[...,i] = (Teb[...,i].T @ np.linalg.inv(Tev[...,i].T)).T
    TUnc[...,i] = np.abs(TebUnc[...,i].T @ np.linalg.inv(Tev[...,i].T)).T

sNom, sCrit = FreqTrans.Sigma(T, TUnc, typeSigma = 2) # Singular Value Decomp, U @ S @ Vh == T[...,i]


C = Ceb

T_InputNames = exc_names
T_OutputNames = fb_names

gain_mag, phase_deg = FreqTrans.GainPhase(T, magUnit='mag', phaseUnit='deg', unwrap=True)
rCritNom_mag, rCritUnc_mag, rCrit_mag = FreqTrans.DistCrit(T, TUnc, typeUnc = 'circle')
#rCritNom_mag, rCritUnc_mag, rCrit_mag = FreqTrans.DistCrit(T, TUnc, typeUnc = 'ellipse')
#rCritNom_mag, rCritUnc_mag, rCrit_mag, pCont_mag = FreqTrans.DistCritEllipse(T, TUnc) # Returns closest approach points


#%% Sigma Plot

#numOut = 3
#TLin = np.copy(np.moveaxis(sysSimOL, -1, 0))
#sysSimOL_sigma = np.moveaxis(np.linalg.svd(np.eye(numOut) + TLin, compute_uv = False), 0, -1)
#
#TNom = np.copy(np.moveaxis(T, -1, 0))
#sNom = np.moveaxis(np.linalg.svd(np.eye(numOut) + TNom, compute_uv = False), 0, -1)
#
#TCrit = (np.eye(numOut) + TNom) - np.moveaxis(TUnc, -1, 0) * (np.eye(numOut) + TNom) / np.abs(np.eye(numOut) + TNom)
#sCrit = -1 + np.moveaxis(np.linalg.svd(np.eye(numOut) + TCrit, compute_uv = False), 0, -1)


Cmin = np.min(np.min(C, axis = 0), axis = 0)
sNomMin = np.min(sNom, axis=0)
sCritMin = np.min(sCrit, axis=0)
sNomMinErr = np.abs(sNomMin - sCritMin)

fig = 20
fig = FreqTrans.PlotSigma(freqSys_hz.transpose(), sysSimOL_sigma.transpose(), coher_nd = np.ones_like(freqSys_hz), fig = fig, fmt = 'k', label='Linear')
fig = FreqTrans.PlotSigma(freq_hz[0], sNomMin, err = sNomMinErr, coher_nd = Cmin, fmt = 'bo', fig = fig, label = 'Excitation with Error')
#fig = FreqTrans.PlotSigma(freq_hz.transpose(), sNom.transpose(), coher_nd = Cmin, fmt = 'bo', fig = fig, label = 'Excitation Nominal')
#fig = FreqTrans.PlotSigma(freq_hz.transpose(), sCrit.transpose(), coher_nd = Cmin, fmt = 'ro', fig = fig, label = 'Excitation Uncertain')
fig = FreqTrans.PlotSigma(freq_hz[0], 0.4 * np.ones_like(freq_hz[0]), fmt = '--r', fig = fig, label = 'Critical Limit')
ax = fig.get_axes()
ax[0].set_xlim(0, 10)
ax[0].set_ylim(0, 5)


#%% Disk Margin Plots
inPlot = exc_names # Elements of exc_names
outPlot = fb_names # Elements of fb_names

if False:
    #%%
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):
            fig = 40 + 3*iOut + iIn
            fig = FreqTrans.PlotSigma(freqSys_hz, sysSimOL_rCrit_mag[iOut, iIn], coher_nd = np.ones_like(freqSys_hz), fig = fig, fmt = 'k', label='Linear')
            fig = FreqTrans.PlotSigma(freq_hz[0, sigIndx[iIn]], rCritNom_mag[iOut, iIn, sigIndx[iIn]], err = rCritUnc_mag[iOut, iIn, sigIndx[iIn]], coher_nd = C[iOut, iIn, sigIndx[iIn]], fmt = 'bo', fig = fig, label = 'Excitation')
            fig = FreqTrans.PlotSigma(freq_hz[0], rCritNom_mag[iOut, iIn], err = rCritUnc_mag[iOut, iIn], coher_nd = C[iOut, iIn], fmt = 'g.:', fig = fig, label = 'MIMO')
            fig = FreqTrans.PlotSigma(freq_hz[0], 0.4 * np.ones_like(freq_hz[0]), fmt = '--r', fig = fig, label = 'Critical Limit')
            ax = fig.get_axes()
            ax[0].set_xlim(0, 10)
            ax[0].set_ylim(0, 2)
            fig.suptitle(inName + ' to ' + outName, size=20)

#%% Nyquist Plots
if False:
    #%%
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):
            fig = 60 + 3*iOut + iIn

            fig = FreqTrans.PlotNyquist(sysSimOL[iOut, iIn], fig = fig, fmt = 'k', label='Linear')
            fig = FreqTrans.PlotNyquist(T[iOut, iIn, sigIndx[iIn]], TUnc[iOut, iIn, sigIndx[iIn]], fig = fig, fmt = 'bo', label='Excitation')
            fig = FreqTrans.PlotNyquist(T[iOut, iIn], TUnc[iOut, iIn], fig = fig, fmt = 'g.:', label='MIMO')

            fig = FreqTrans.PlotNyquist(np.asarray([-1 + 0j]), TUnc = np.asarray([0.4 + 0.4j]), fig = fig, fmt = 'r+', label='Critical Region')
            fig.suptitle(inName + ' to ' + outName, size=20)

            ax = fig.get_axes()
            ax[0].set_xlim(-3, 1)
            ax[0].set_ylim(-2, 2)

#%% Bode Plots
if False:
    #%%
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):
            fig = 80 + 3*iOut + iIn
            fig = FreqTrans.PlotBode(freqSys_hz, sysSimOL_gain_nd[iOut, iIn], sysSimOL_phase_deg[iOut, iIn], coher_nd = np.ones_like(freqSys_hz), fig = fig, fmt = 'k', label = 'Linear')
            fig = FreqTrans.PlotBode(freq_hz[0, sigIndx[iIn]], gain_mag[iOut, iIn, sigIndx[iIn]], phase_deg[iOut, iIn, sigIndx[iIn]], C[iOut, iIn, sigIndx[iIn]], gainUnc_mag = rCritUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, fmt = 'bo', label = 'Estimated SIMO')
            fig = FreqTrans.PlotBode(freq_hz[0], gain_mag[iOut, iIn], phase_deg[iOut, iIn], C[iOut, iIn], gainUnc_mag = rCritUnc_mag[iOut, iIn], fig = fig, fmt = 'g.', label = 'Estimated MIMO')
            fig.suptitle(inName + ' to ' + outName, size=20)


#%% Noise Estimation Validation
if False:
    #%%
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):
            fig = 120 + 3*iOut + iIn
            fig = FreqTrans.PlotBodeMag(freqSys_hz, sysSimOL_gain_nd[iOut, iIn], coher_nd = np.ones_like(freqSys_hz), fig = fig, fmt = 'k', label = 'Linear')
            fig = FreqTrans.PlotBodeMag(freq_hz[0, sigIndx[iIn]], gain_mag[iOut, iIn, sigIndx[iIn]], C[iOut, iIn, sigIndx[iIn]], fig = fig, fmt = 'bo', label = 'Estimated SIMO')
            fig = FreqTrans.PlotBodeMag(freq_hz[0], gain_mag[iOut, iIn], C[iOut, iIn], fig = fig, fmt = 'g.', label = 'Estimated MIMO')

            fig = FreqTrans.PlotBodeMag(freq_hz[0, sigIndx[iIn]], rCritUnc_mag[iOut, iIn, sigIndx[iIn]], C[iOut, iIn, sigIndx[iIn]], fig = fig, fmt = 'b--', label = 'Estimated Noise SIMO')
            fig = FreqTrans.PlotBodeMag(freq_hz[0], rCritUnc_mag[iOut, iIn], C[iOut, iIn], fig = fig, fmt = 'g:', label = 'Estimated Noise MIMO')

            fig.suptitle(inName + ' to ' + outName, size=20)
