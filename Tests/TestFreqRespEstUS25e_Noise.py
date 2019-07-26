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
freqSys_rps = freqSys_hz*hz2rps


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
        sysSimOL_gain_nd[iIn, iOut], sysSimOL_phase_rad[iIn, iOut], _ = control.bode_plot(sysOL[outEntry, inEntry], omega = freqSys_rps, Plot = False)
        Treal, Timag, _ = control.nyquist_plot(sysOL[outEntry, inEntry], omega = freqSys_rps, Plot = False)
        sysSimOL[iIn, iOut, :] = Treal + 1j*Timag

#sysSimOL_gain_nd[sysSimOL_gain_nd == 0] = 1e-6
sysSimOL_gain_dB = 20*np.log10(sysSimOL_gain_nd)
sysSimOL_phase_deg = sysSimOL_phase_rad * rad2deg
sysSimOL_rCrit_mag = np.abs(sysSimOL - (-1 + 0j))


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
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

freqNull_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

# Generate Schroeder MultiSine Signal
ampExc_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
exc, _, sigExc = GenExcite.MultiSine(freqExc_rps, ampExc_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')
exc_names = ['excP', 'excQ', 'excR']

# Generate Noise
dist_names = ['phiDist', 'thetaDist', 'pDist', 'qDist', 'rDist', 'VDist', 'hDist']
angleDist = np.random.normal(0, 0.0 * ampInit, size = (2, len(time_s)))
pqrDist = np.random.normal(0, 0.5, size = (3, len(time_s)))
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
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear')
optSpecN = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 1), winType = ('tukey', 0.0), detrendType = 'Linear')

# Excited Frequencies per input channel
optSpec.freq = []
for iChan in range(0, numExc):
    optSpec.freq.append(freqExc_rps[sigIndx[iChan]])
optSpec.freq = np.asarray(optSpec.freq)

# Null Frequencies
optSpecN.freq = freqGap_rps

# FRF Estimate
freq_rps, Teb, Ceb, Pee, Pbb, Peb, TebUnc, Pbb_N = FreqTrans.FreqRespFuncEstNoise(exc, fb, optSpec, optSpecN)
_       , Tev, Cev, _  , Pvv, Pev = FreqTrans.FreqRespFuncEst(exc, v, optSpec)


nFreq = freq_rps.shape[-1]

freq_hz = freq_rps * rps2hz
eye = np.tile(np.eye(3), (nFreq, 1, 1)).T

# Form the Frequency Response, T = Teb @ Tev^-1
T = np.empty_like(Tev)
TUnc = np.empty_like(Tev)
C = np.empty_like(Tev)

for i in range(T.shape[-1]):  
    T[...,i] = Teb[...,i] @ np.linalg.inv(Tev[...,i])
    TUnc[...,i] = TebUnc[...,i] @ np.linalg.inv(Tev[...,i])

C = Ceb

T_InputNames = exc_names
T_OutputNames = fb_names

gain_dB, phase_deg = FreqTrans.GainPhase(T, magUnit='dB', phaseUnit='deg', unwrap=True)
#rCritNom_mag, rCritUnc_mag, rCrit_mag = FreqTrans.DistCrit(T, TUnc, pCrit = -1+0j, typeUnc = 'circle', magUnit = 'mag')
#rCritNom_mag, rCritUnc_mag, rCrit_mag = FreqTrans.DistCrit(T, TUnc, pCrit = -1+0j, typeUnc = 'ellipse', magUnit = 'mag')
rCritNom_mag, rCritUnc_mag, rCrit_mag, pCont_mag = FreqTrans.DistCritEllipse(T, TUnc, pCrit = -1+0j, magUnit = 'mag') # Returns closest approach points


#%% Disk Margin Plots
inPlot = exc_names # Elements of exc_names
outPlot = fb_names # Elements of fb_names

if True:
    for iIn, inName in enumerate(inPlot):
        for iOut, outName in enumerate(outPlot):
            fig = 60 + 3*iIn + iOut
            fig = FreqTrans.PlotDistCrit(freqSys_hz, sysSimOL_rCrit_mag[iIn, iOut], fig = fig, fmt = 'k', label='Linear')
            fig = FreqTrans.PlotDistCrit(freq_hz[iOut, 0], rCritNom_mag[iIn, iOut], unc = rCritUnc_mag[iIn, iOut], coher_nd = C[iIn, iOut], fmt = '.b', fig = fig, label = 'Excitation')
            fig = FreqTrans.PlotDistCrit(freq_hz[iOut, 0], 0.4 * np.ones_like(freq_hz[iOut, 0]), fmt = '--r', fig = fig, label = 'Critical Limit')
            ax = fig.get_axes()
            ax[0].set_xlim(0, 10)
            ax[0].set_ylim(0, 2)
            fig.suptitle(inName + ' to ' + outName, size=20)

#%% Nyquist Plots
if False:
    for iIn, inName in enumerate(inPlot):
        for iOut, outName in enumerate(outPlot):
            fig = 80 + 3*iIn + iOut

            fig = FreqTrans.PlotNyquist(sysSimOL[iIn, iOut], fig = fig, fmt = 'k', label='Linear')
            fig = FreqTrans.PlotNyquist(T[iIn, iOut], TUnc[iIn, iOut], fig = fig, fmt = 'b*:', label='Excitation')
            fig = FreqTrans.PlotNyquist(pCont_mag[iIn, iOut],  fig = fig, fmt = 'b.', label='Min Critical Dist')
            
            fig = FreqTrans.PlotNyquist(np.asarray([-1+ 0j]), TUnc = np.asarray([0.4 + 0.4j]), fig = fig, fmt = 'r*', label='Critical Region')
            fig.suptitle(inName + ' to ' + outName, size=20)
            
            ax = fig.get_axes()
            ax[0].set_xlim(-3, 1)
            ax[0].set_ylim(-2, 2)
            

#%% Bode Plots
if False:
    for iIn, inName in enumerate(inPlot):
        for iOut, outName in enumerate(outPlot):
            fig = 100 + 3*iIn + iOut
            fig = FreqTrans.PlotBode(freqSys_hz, sysSimOL_gain_dB[iIn, iOut], sysSimOL_phase_deg[iIn, iOut], fig = fig, fmt = 'k', label = 'Linear')
            fig = FreqTrans.PlotBode(freq_hz[iOut, 0], gain_dB[iIn, iOut], phase_deg[iIn, iOut], C[iIn, iOut], fig = fig, fmt = 'b.', label = 'Excitation')
            fig.suptitle(inName + ' to ' + outName, size=20)

