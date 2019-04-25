"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as interp
import scipy.signal as signal
import control


import FreqTrans
import GenExcite

# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps
d2r = np.pi/180
r2d = 1/d2r


#%% Define a linear systems
freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps

freqSys_hz = np.linspace(0.1, 15, 300)
freqSys_rps = freqSys_hz*hz2rps

nFreq = len(freqSys_rps)
linInNames = ['excP',  'excQ', 'excR']; nIn = len(linInNames)
linOutNames = sysOL_OutputNames; nOut = len(linOutNames)

inList = [sysOL_InputNames.index(s) for s in linInNames]
outList = [sysOL_OutputNames.index(s) for s in linOutNames]
sysOL_gain_nd = np.zeros([nOut, nIn, nFreq])
sysOL_phase_rad = np.zeros([nOut, nIn, nFreq])
sysOL_T = np.zeros([nOut, nIn, nFreq], dtype=complex)

for iOut, outEntry in enumerate(outList):
    for iIn, inEntry in enumerate(inList):
        sysOL_gain_nd[iOut, iIn, :], sysOL_phase_rad[iOut, iIn], _ = control.bode_plot(sysOL[outEntry, inEntry], omega = freqSys_rps, Plot = False)
        Treal, Timag, _ = control.nyquist_plot(sysOL[outEntry, inEntry], omega = freqSys_rps, Plot = False)
        sysOL_T[iOut, iIn, :] = Treal + 1j*Timag

sysOL_gain_nd[sysOL_gain_nd == 0] = 1e-6
sysOL_gain_dB = 20*np.log10(sysOL_gain_nd)
sysOL_phase_deg = sysOL_phase_rad * r2d


#%% Excitation
numExc = 3
numCycles = 1
ampInit = 1
ampFinal = 1
freqMinDes_rps = 0.1 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 15 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (10/freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)


freqNull_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

# Generate Schroeder MultiSine Signal
ampExc_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
exc, _, sigExc = GenExcite.Schroeder(freqExc_rps, ampExc_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak');


# Generate Noise
dist_names = ['phiDist', 'thetaDist', 'pDist', 'qDist', 'rDist', 'VDist', 'hDist']
sigmaNoise = 0.1 * ampInit
shapeDist = (len(dist_names), len(time_s))
dist = np.random.normal(0, sigmaNoise, size = shapeDist)


# Simulate the excitation through the system, with noise
exc_names = ['excP', 'excQ', 'excR']
indxIn = [sysCL_InputNames.index(s) for s in exc_names + dist_names]

sysExc = sysCL[:,indxIn]
sysExc_InputNames = exc_names + dist_names
sysExc_OutputNames = sysCL_OutputNames

_, out, stateSim = control.forced_response(sysExc, T = time_s, U=np.concatenate((exc, dist)), X0=0.0, transpose=False)

sens_names = sysExc_OutputNames[:7]
sens = out[:7]

fb_names = sysExc_OutputNames[7:10]
fb = out[7:10]

v_names = sysExc_OutputNames[10:]
v = out[10:]

#plt.plot(time_s, ySim[8])

#%% Estimate the transfer functions
dftType = 'czt'
scaleType = 'spectrum'
detrendType = 'constant'
winType = ('tukey', 0.0)
smooth = ('box', 1)

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

for iExc in range(0, numExc):
    freqChan_rps = freqExc_rps[sigIndx[iExc]]
    freqNull_rps = freqGap_rps
    freq_rps[:, iExc, :], _, _, Ceb[:, iExc, :], Teb[:, iExc, :], _, _, _, TebUnc[:, iExc, :] = FreqTrans.FreqRespFuncEstNoise(exc[np.newaxis, iExc], fb, freqRate_rps, freqChan_rps, freqNull_rps, dftType, winType, detrendType, smooth, scaleType)

    _, _, _, Cev[:, iExc, :], Tev[:, iExc, :], _, _, _ = FreqTrans.FreqRespFuncEst(exc[np.newaxis, iExc], v, freqRate_rps, freqChan_rps, dftType, winType, detrendType, smooth, scaleType)
    

freq_hz = freq_rps * rps2hz
T = Teb / (Tev + 1)
TUnc = TebUnc / (Tev + 1)

gain_dB, phase_deg = FreqTrans.GainPhase(T)
phase_deg = np.unwrap(phase_deg * deg2rad) * rad2deg


#%% Plot
plt.figure(1)

iIn = 0
iOut = 0
ax1 = plt.subplot(4,2,1)
ax1.semilogx(freqSys_hz, sysOL_gain_dB[iOut,iIn], 'k')
ax1.semilogx(freq_hz[iOut,iIn], gain_dB[iOut,iIn], '.')

ax3 = plt.subplot(4,2,3, sharex = ax1)
ax3.semilogx(freqSys_hz, sysOL_phase_deg[iOut,iIn], 'k')
ax3.semilogx(freq_hz[iOut,iIn], phase_deg[iOut,iIn], '.')

iIn = 0
iOut = 1
ax2 = plt.subplot(4,2,2)
ax2.semilogx(freqSys_hz, sysOL_gain_dB[iOut,iIn], 'k')
ax2.semilogx(freq_hz[iOut,iIn], gain_dB[iOut,iIn], '.')

ax4 = plt.subplot(4,2,4, sharex = ax2)
ax4.semilogx(freqSys_hz, sysOL_phase_deg[iOut,iIn], 'k')
ax4.semilogx(freq_hz[iOut,iIn], phase_deg[iOut,iIn], '.')

iIn = 1
iOut = 0
ax5 = plt.subplot(4,2,5);
ax5.semilogx(freqSys_hz, sysOL_gain_dB[iOut,iIn], 'k')
ax5.semilogx(freq_hz[iOut,iIn], gain_dB[iOut,iIn], '.')

ax7 = plt.subplot(4,2,7, sharex = ax5);
ax7.semilogx(freqSys_hz, sysOL_phase_deg[iOut,iIn], 'k')
ax7.semilogx(freq_hz[iOut,iIn], phase_deg[iOut,iIn], '.')

iIn = 1
iOut = 1
ax6 = plt.subplot(4,2,6);
ax6.semilogx(freqSys_hz, sysOL_gain_dB[iOut,iIn], 'k')
ax6.semilogx(freq_hz[iOut,iIn], gain_dB[iOut,iIn], '.')

ax8 = plt.subplot(4,2,8, sharex = ax6);
ax8.semilogx(freqSys_hz, sysOL_phase_deg[iOut,iIn], 'k')
ax8.semilogx(freq_hz[iOut,iIn], phase_deg[iOut,iIn], '.')


    
#%%
inPlot = exc_names # Elements of exc_names
outPlot = fb_names # Elements of fb_names


fig, ax = plt.subplots(len(outPlot), len(inPlot), num = 2)
for iIn, inName in enumerate(inPlot):
    inElem = exc_names.index(inName)

    for iOut, outName in enumerate(outPlot):
        outElem = fb_names.index(outName)
    
        ax[iOut, iIn].plot(sysOL_T[outElem, inElem].imag, sysOL_T[outElem, inElem].real, 'k')
        ax[iOut, iIn].plot(T[outElem, inElem].imag, T[outElem, inElem].real, '.')
        ax[iOut, iIn].grid()
        critCirc = plt.Circle((-1, 0), 0.4, color='r', alpha=0.5)
        ax[iOut, iIn].add_artist(critCirc)
        
        for iNom, nom in enumerate(T[outElem, inElem]):
            unc = np.abs(TUnc[outElem, inElem][iNom])
            uncCirc = plt.Circle((nom.imag, nom.real), unc, color='b', alpha=0.5)
            ax[iOut, iIn].add_artist(uncCirc)

        ax[iOut,iIn].set_xlim(-2, 0)
        ax[iOut,iIn].set_ylim(-1, 1)
        
