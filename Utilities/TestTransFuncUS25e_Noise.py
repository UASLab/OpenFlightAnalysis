"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import scipy.interpolate as interp
import scipy.signal as signal
import control

import FreqTrans
import GenExcite
import Systems

# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps

rad2deg = 180/np.pi
deg2rad = 1/rad2deg


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
sysSimOL_gain_nd = np.zeros([nOut, nIn, nFreq])
sysSimOL_phase_rad = np.zeros([nOut, nIn, nFreq])
sysSimOL = np.zeros([nOut, nIn, nFreq], dtype=complex)

for iOut, outEntry in enumerate(outList):
    for iIn, inEntry in enumerate(inList):
        sysSimOL_gain_nd[iOut, iIn], sysSimOL_phase_rad[iOut, iIn], _ = control.bode_plot(sysOL[outEntry, inEntry], omega = freqSys_rps, Plot = False)
        Treal, Timag, _ = control.nyquist_plot(sysOL[outEntry, inEntry], omega = freqSys_rps, Plot = False)
        sysSimOL[iOut, iIn, :] = Treal + 1j*Timag

#sysSimOL_gain_nd[sysSimOL_gain_nd == 0] = 1e-6
sysSimOL_gain_dB = 20*np.log10(sysSimOL_gain_nd)
sysSimOL_phase_deg = sysSimOL_phase_rad * rad2deg
sysSimOL_sigma_mag = np.abs(sysSimOL - (0 - 1j))


#%% Excitation
numExc = 3
numCycles = 1
ampInit = 4 * deg2rad
ampFinal = ampInit
freqMinDes_rps = 0.1 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 15 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (10 / freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

freqNull_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

# Generate Schroeder MultiSine Signal
ampExc_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
exc, _, sigExc = GenExcite.Schroeder(freqExc_rps, ampExc_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak');
exc_names = ['excP', 'excQ', 'excR']

# Generate Noise
dist_names = ['phiDist', 'thetaDist', 'pDist', 'qDist', 'rDist', 'VDist', 'hDist']
angleDist = np.random.normal(0, 0.0 * ampInit, size = (2, len(time_s)))
pqrDist = np.random.normal(0, 0.0 * ampInit, size = (3, len(time_s)))
airDist = np.random.normal(0, 0.0 * ampInit, size = (2, len(time_s)))
dist = np.concatenate((angleDist, pqrDist, airDist))


# Reference Inputs
ref_names = ['refPhi', 'refTheta', 'refYaw']
shapeRef = (len(ref_names), len(time_s))
ref = np.random.normal(0, 0.0 * ampInit, size = (3, len(time_s)))
ref[1] = 0.0 * deg2rad + ref[1]

# Simulate the excitation through the system, with noise
sysExc_InputNames = sysCL_InputNames
sysExc_OutputNames = sysCL_OutputNames
sysExc = control.StateSpace(sysCL.A, sysCL.B, sysCL.C, sysCL.D)

u = np.concatenate((ref, exc, dist))
_, out, stateSim = control.forced_response(sysExc, T = time_s, U = u, X0 = 0.0, transpose = False)

# Time shift
#time_s = time_s[0:-1]
#ref = ref[:,0:-1]
#exc = exc[:,0:-1]
#dist = dist[:,0:-1]
#u = u[:,0:-1]
#out = out[:, 1:]

fb_names = sysExc_OutputNames[:3]
fb = out[:3]

v_names = sysExc_OutputNames[3:6]
v = out[3:6]

sens_names = sysExc_OutputNames[-7:]
sens = out[-7:]

#plt.plot(time_s, exc[1], time_s, v[1], time_s, fb[1])

#%% Estimate the transfer functions

nIn = len(exc)
nSens = len(sens)
nB = len(fb)
nV = len(v)
nFreq = int(len(freqExc_rps) / numExc)

freq_rps = np.zeros([nB, nIn, nFreq])

Ceb = np.zeros([nB, nIn, nFreq])
Teb = np.zeros([nB, nIn, nFreq], dtype=complex)
TebUnc = np.zeros([nB, nIn, nFreq], dtype=complex)

Cev = np.zeros([nB, nIn, nFreq])
Tev = np.zeros([nB, nIn, nFreq], dtype=complex)
TevUnc = np.zeros([nB, nIn, nFreq], dtype=complex)

optSpect = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3))
optSpectN = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps)

for iExc in range(0, numExc):
    optSpect.freq = freqExc_rps[sigIndx[iExc]]
    optSpectN.freq = freqGap_rps
    freq_rps[:, iExc, :], Teb[:, iExc, :], Ceb[:, iExc, :], _, _, _, TebUnc[:, iExc, :] = FreqTrans.FreqRespFuncEstNoise(exc[np.newaxis, iExc], fb, optSpect, optSpectN)
    _                   , Tev[:, iExc, :], Cev[:, iExc, :], _, _, _, TevUnc[:, iExc, :] = FreqTrans.FreqRespFuncEstNoise(exc[np.newaxis, iExc], v, optSpect, optSpectN)
    

freq_hz = freq_rps * rps2hz
eye = np.tile(np.eye(3), (nFreq, 1, 1)).T

# Form the Frequency Response
T = Teb / (Tev + TevUnc)
TUnc = TebUnc / (Tev + TevUnc)

T_InputNames = exc_names
T_OutputNames = fb_names

gain_dB, phase_deg = FreqTrans.GainPhase(T)
phase_deg = np.unwrap(phase_deg * deg2rad) * rad2deg
sigma_mag = np.abs(T - (0 - 1j)) # Distance from (0,-1j)


#%% Disk Margin Plots
inPlot = exc_names # Elements of exc_names
outPlot = fb_names # Elements of fb_names

fig, ax = plt.subplots(len(outPlot), len(inPlot), sharex=True, sharey=True, num = 1)
for iIn, inName in enumerate(inPlot):
    inElem = exc_names.index(inName)

    for iOut, outName in enumerate(outPlot):
        outElem = fb_names.index(outName)
        
        uncDisk = np.abs(TUnc[iOut, iIn])
        
        ax[iOut, iIn].semilogx(freqSys_hz, sysSimOL_sigma_mag[iOut, iIn], 'k')
        ax[iOut, iIn].errorbar(freq_hz[iOut, iIn], sigma_mag[iOut, iIn], yerr = uncDisk, fmt = 'b.')
        ax[iOut, iIn].grid()
        ax[iOut, iIn].set_xlim(left = 0.05, right = freqRate_hz/2)
        ax[iOut, iIn].set_ylim(bottom = 0.0, top = 2.0)
        
        ax[iOut, iIn].plot(freqSys_hz, 0.4 * np.ones_like(freqSys_hz), 'r-')
        

#%% Nyquist Plots
inPlot = exc_names # Elements of exc_names
outPlot = fb_names # Elements of fb_names

fig, ax = plt.subplots(len(outPlot), len(inPlot), num = 2)
for iIn, inName in enumerate(inPlot):
    inElem = exc_names.index(inName)

    for iOut, outName in enumerate(outPlot):
        outElem = fb_names.index(outName)
    
        ax[iOut, iIn].plot(sysSimOL[iOut, iIn].imag, sysSimOL[iOut, iIn].real, 'k')
        ax[iOut, iIn].plot(T[iOut, iIn].imag, T[iOut, iIn].real, 'b.')
        ax[iOut, iIn].grid()
        critPatch = patch.Ellipse((-1, 0), 2*0.4, 2*0.4, color='r', alpha=0.25)
        ax[iOut, iIn].add_artist(critPatch)
        
        for iNom, nom in enumerate(T[iOut, iIn]):
            unc = TUnc[iOut, iIn][iNom]
            uncPatch = patch.Ellipse((nom.imag, nom.real), 2*unc.imag, 2*unc.real, color='b', alpha=0.25)
            ax[iOut, iIn].add_artist(uncPatch)

        ax[iOut,iIn].set_xlim(-3, 1)
        ax[iOut,iIn].set_ylim(-2, 2)
        

#%% Bode Plots
for iIn in range(0, 3):
    for iOut in range(0, 3):
        
        plt.figure()
        
        ax1 = plt.subplot(3, 1, 1)
        ax1.semilogx(freqSys_hz, sysSimOL_gain_dB[iOut, iIn], 'k')
        ax1.semilogx(freq_hz[iOut, iIn], gain_dB[iOut, iIn], '.')
        ax1.grid(); ax1.set_ylabel('Gain (dB)')
        
        ax2 = plt.subplot(3, 1, 2, sharex = ax1)
        ax2.semilogx(freqSys_hz, sysSimOL_phase_deg[iOut, iIn], 'k')
        ax2.semilogx(freq_hz[iOut, iIn], phase_deg[iOut, iIn], '.')
        ax2.grid(); ax2.set_ylabel('Phase (deg)')
        
        ax3 = plt.subplot(3, 1, 3, sharex = ax1)
        ax3.semilogx(freqSys_hz, np.ones_like(freqSys_hz), 'k')
        ax3.semilogx(freq_hz[iOut, iIn], Ceb[iOut, iIn], '.')
        ax3.semilogx(freq_hz[iOut, iIn], Cev[iOut, iIn], '.')
        ax3.grid(); ax3.set_xlabel('Freq (Hz)'); ax3.set_ylabel('Coherence (nd)')
        ax3.set_ylim(0, 1)

