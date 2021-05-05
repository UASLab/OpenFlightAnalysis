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

freqLin_hz = np.logspace(np.log10(0.01), np.log10(25), 800)
freqLin_rps = freqLin_hz * hz2rps


# OL : Mixer -> Plant -> SCAS_FB
connectName = sysPlant.InputName[:7] + sysScas.InputName[1::3]
inKeep = sysMixer.InputName + sysPlant.InputName[-7:]
outKeep = sysScas.OutputName[2::4]

sysOL = Systems.ConnectName([sysMixer, sysPlant, sysScas], connectName, inKeep, outKeep)


# CL: Ctrl -> Plant
inName = sysCtrl.InputName + sysPlant.InputName
outName = sysCtrl.OutputName + sysPlant.OutputName
connectName = ['cmdMotor', 'cmdElev', 'cmdRud', 'cmdAilL', 'cmdAilR', 'cmdFlapL', 'cmdFlapR', 'sensPhi', 'sensTheta', 'sensR']
inKeep = [inName[i-1] for i in [1, 2, 3, 7, 8, 9, 17, 18, 19, 20, 21, 22, 23]]
outKeep = [outName[i-1] for i in [8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]]

sysCL = Systems.ConnectName([sysCtrl, sysPlant], connectName, inKeep, outKeep)

# Look at only the in-out of the OL
inList = [sysOL.InputName.index(s) for s in ['cmdP',  'cmdQ', 'cmdR']]
outList = [sysOL.OutputName.index(s) for s in ['fbP', 'fbQ', 'fbR']]

sysSimOL = sysOL[outList, :]
sysSimOL = sysOL[:, inList]


# Linear System Response
gainLin_mag, phaseLin_rad, _ = control.freqresp(sysSimOL, omega = freqLin_rps)

TxyLin = gainLin_mag * np.exp(1j*phaseLin_rad)

gainLin_dB = 20 * np.log10(gainLin_mag)
phaseLin_deg = np.unwrap(phaseLin_rad) * rad2deg
rCritLin_mag = np.abs(TxyLin - (-1 + 0j))

sigmaLin_mag, _ = FreqTrans.Sigma(TxyLin)


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
uExc, _, sigExc = GenExcite.MultiSine(freqExc_rps, ampExc_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')
exc_names = ['excP', 'excQ', 'excR']

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]

# Null Frequencies
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

# Generate Noise
dist_names = ['phiDist', 'thetaDist', 'pDist', 'qDist', 'rDist', 'VDist', 'hDist']
pqrDist = np.random.normal(0, 0.25 * ampInit, size = (3, len(time_s)))
angleDist = np.cumsum(pqrDist[[0,1],:], axis = -1)
airDist = np.random.normal(0, 0.0, size = (2, len(time_s)))
dist = np.concatenate((angleDist, pqrDist, airDist))
dist = 0.0 * dist

# Reference Inputs
ref_names = ['refPhi', 'refTheta', 'refYaw']
shapeRef = (len(ref_names), len(time_s))
ref = np.random.normal(0, 1.0 * ampInit, size = (3, len(time_s)))
ref[1] = 2.0 * deg2rad + ref[1]
ref = 0.0 * ref

# Simulate the excitation through the system, with noise
u = np.concatenate((ref, uExc, dist))
_, out, stateSim = control.forced_response(sysCL, T = time_s, U = u, X0 = 0.0, transpose = False)

# shift output time to represent the next frame, pad t=0 with 0.0
#out = np.concatenate((np.zeros((out.shape[0],1)), out[:,1:-1]), axis=1) # this is handled by a time delay on sensors in the linear simulation

fbName = sysCL.OutputName[:3]
fb = out[:3]

vName = sysCL.OutputName[3:6]
v = out[3:6]

sensName = sysCL.OutputName[-7:]
sens = out[-7:]

#plt.plot(time_s, uExc[1], time_s, v[1], time_s, fb[1], time_s, pqrDist[1])


#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear', interpType = 'linear')

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps

# FRF Estimate
freq_rps, Teb, Ceb, Pee, Pbb, Peb = FreqTrans.FreqRespFuncEst(uExc, fb, optSpec)
_       , Tev, Cev, _  , Pvv, Pev = FreqTrans.FreqRespFuncEst(uExc, v, optSpec)

freq_hz = freq_rps * rps2hz


# Form the Frequency Response, T = Teb @ Tev^-1
T = np.zeros_like(Tev, dtype=complex)
C = np.zeros_like(Tev, dtype=float)

for i in range(T.shape[-1]):
    T[...,i] = (Teb[...,i].T @ np.linalg.inv(Tev[...,i].T)).T

sigmaNom_mag, _ = FreqTrans.Sigma(T) # Singular Value Decomp

# Coherence
C = Ceb

T_InputName = exc_names
T_OutputName = fbName

gain_mag, phase_deg = FreqTrans.GainPhase(T, magUnit='mag', phaseUnit='deg', unwrap=True)
rCritNom_mag, _, _ = FreqTrans.DistCrit(T, typeUnc = 'ellipse')
#rCritNom_mag, rCritUnc_mag, rCrit_mag = FreqTrans.DistCrit(T, TUnc, typeUnc = 'ellipse')
#rCritNom_mag, rCritUnc_mag, rCrit_mag, pCont_mag = FreqTrans.DistCritEllipse(T, TUnc) # Returns closest approach points


#%% Sigma Plot
Cmin = np.min(np.min(C, axis = 0), axis = 0)
sigmaNom_magMin = np.min(sigmaNom_mag, axis=0)

fig = 20
fig = FreqTrans.PlotSigma(freqLin_hz, sigmaLin_mag, coher_nd = np.ones_like(freqLin_hz), fig = fig, fmt = 'k', label='Linear')
fig = FreqTrans.PlotSigma(freq_hz, sigmaNom_mag, coher_nd = Cmin, fmt = 'bo', fig = fig, label = 'Excitation Nominal')
fig = FreqTrans.PlotSigma(freq_hz[0], 0.4 * np.ones_like(freq_hz[0]), fmt = '--r', fig = fig, label = 'Critical Limit')
ax = fig.get_axes()
ax[0].set_xlim(0, 10)
ax[0].set_yscale('log')
ax[0].set_ylim(1e-2, 1e2)


#%% Disk Margin Plots
inPlot = exc_names # Elements of exc_names
outPlot = fbName # Elements of fbName

if False:
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):
            fig = 40 + 3*iOut + iIn
            fig = FreqTrans.PlotSigma(freqLin_hz, rCritLin_mag[iOut, iIn], coher_nd = np.ones_like(freqLin_hz), fig = fig, fmt = 'k', label='Linear')
            fig = FreqTrans.PlotSigma(freq_hz[0, sigIndx[iIn]], rCritNom_mag[iOut, iIn, sigIndx[iIn]], coher_nd = C[iOut, iIn, sigIndx[iIn]], fmt = 'bo', fig = fig, label = 'Excitation')
            fig = FreqTrans.PlotSigma(freq_hz[0], rCritNom_mag[iOut, iIn], coher_nd = C[iOut, iIn], fmt = 'g.:', fig = fig, label = 'MIMO')
            fig = FreqTrans.PlotSigma(freq_hz[0], 0.4 * np.ones_like(freq_hz[0]), fmt = '--r', fig = fig, label = 'Critical Limit')
            ax = fig.get_axes()
            ax[0].set_xlim(0, 10)
            ax[0].set_ylim(0, 2)
            fig.suptitle(inName + ' to ' + outName, size=20)

#%% Nyquist Plots
if False:
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):
            fig = 60 + 3*iOut + iIn

            fig = FreqTrans.PlotNyquist(TxyLin[iOut, iIn], fig = fig, fmt = 'k', label='Linear')
            fig = FreqTrans.PlotNyquist(T[iOut, iIn, sigIndx[iIn]], fig = fig, fmt = 'bo', label='Excitation')
            fig = FreqTrans.PlotNyquist(T[iOut, iIn], fig = fig, fmt = 'g.:', label='MIMO')

            fig = FreqTrans.PlotNyquist(np.asarray([-1 + 0j]), TUnc = np.asarray([0.4 + 0.4j]), fig = fig, fmt = 'r+', label='Critical Region')
            fig.suptitle(inName + ' to ' + outName, size=20)

            ax = fig.get_axes()
            ax[0].set_xlim(-3, 1)
            ax[0].set_ylim(-2, 2)

#%% Bode Plots
if True:
    for iOut, outName in enumerate(outPlot):
        for iIn, inName in enumerate(inPlot):
            fig = 80 + 3*iOut + iIn
            fig = FreqTrans.PlotBode(freqLin_hz, gainLin_mag[iOut, iIn], phaseLin_deg[iOut, iIn], coher_nd = np.ones_like(freqLin_hz), fig = fig, fmt = 'k', label = 'Linear')
            fig = FreqTrans.PlotBode(freq_hz[0, sigIndx[iIn]], gain_mag[iOut, iIn, sigIndx[iIn]], phase_deg[iOut, iIn, sigIndx[iIn]], C[iOut, iIn, sigIndx[iIn]], fig = fig, fmt = 'bo', label = 'Estimated SIMO')
            fig = FreqTrans.PlotBode(freq_hz[0], gain_mag[iOut, iIn], phase_deg[iOut, iIn], C[iOut, iIn], fig = fig, fmt = 'g.', label = 'Estimated MIMO')
            fig.suptitle(inName + ' to ' + outName, size=20)

