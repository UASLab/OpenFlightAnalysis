#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan  6 16:48:31 2021

@author: rega0051
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


# Constants
pi = np.pi

hz2rps = 2*pi
rps2hz = 1/hz2rps

rad2deg = 180/pi
deg2rad = 1/rad2deg

mag2db = control.mag2db
db2mag = control.db2mag


#%%
W_dB = -20.0
W = db2mag(W_dB)

fBinSel = FreqTrans.LeakageGoal(W, winType = 'Dirichlet')[0]


#%%
freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps
dt = 1/freqRate_hz

freqMinDes_hz = 0.1
numCyclesDes = 3 # Plan for 3

freqStepDes_hz = 0.1


#%% Excitation
numCycles = 6 # Need 6!
numChan = 2
ampInit = 1
ampFinal = 1
freqMinDes_rps = freqMinDes_hz * hz2rps * np.ones(numChan)
freqMaxDes_rps = 10.3 * hz2rps *  np.ones(numChan)
freqStepDes_rps = (freqStepDes_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

#time_s = time_s[0:-5]

# Generate Schroeder MultiSine Signal
ampExcit_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
exc, phaseElem_rad, sigExcit = GenExcite.MultiSine(freqExc_rps, ampExcit_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')
exc = exc / np.std(exc)
uPeak = np.mean(GenExcite.PeakFactor(exc) * np.std(exc, axis = -1))**2

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]
freqChan_hz = freqChan_rps * rps2hz


#%%
N = 2**8

fBin = np.linspace(0, 10, num = N)

D = FreqTrans.Dirichlet(fBin, N)
Dmag = np.abs(D)
Dmax = np.nanmax(Dmag)
Dnorm_mag = Dmag / Dmax
Dnorm_dB = mag2db(Dnorm_mag)

Dapprox_mag = FreqTrans.DirichletApprox(fBin)
Dapprox_dB = mag2db( Dapprox_mag )


B = FreqTrans.Bartlett(fBin, N)
Bmag = np.abs(B)
Bmax = np.nanmax(Bmag)
Bnorm_mag = Bmag / Bmax
Bnorm_dB = mag2db(Bnorm_mag)

Bapprox_mag = FreqTrans.BartlettApprox(fBin)
Bapprox_dB = mag2db( Bapprox_mag )


H = FreqTrans.Hann(fBin, N)
Hmag = np.abs(H)
Hmax = np.nanmax(Hmag)
Hnorm_mag = Hmag / Hmax
Hnorm_dH = mag2db(Hnorm_mag)

Happrox_mag = FreqTrans.HannApprox(fBin)
Happrox_dH = mag2db( Happrox_mag )

fig = 1
fig = FreqTrans.PlotGainType(fBin, Dnorm_mag, None, None, None, fig = fig, dB = True, linestyle='-', color='b', label = 'Dirichlet Function')
fig = FreqTrans.PlotGainType(fBin, Dapprox_mag, None, None, None, fig = fig, dB = True, linestyle=':', color='b', label = 'Dirichlet Approximation')
fig = FreqTrans.PlotGainType(fBin, Bnorm_mag, None, None, None, fig = fig, dB = True, linestyle='-', color='g', label = 'Bartlett Function')
fig = FreqTrans.PlotGainType(fBin, Bapprox_mag, None, None, None, fig = fig, dB = True, linestyle=':', color='g', label = 'Bartlett Approximation')
fig = FreqTrans.PlotGainType(fBin, Hnorm_mag, None, None, None, fig = fig, dB = True, linestyle='-', color='r', label = 'Hann Function')
fig = FreqTrans.PlotGainType(fBin, Happrox_mag, None, None, None, fig = fig, dB = True, linestyle=':', color='r', label = 'Hann Approximation')

ax = fig.get_axes()

ax[0].set_xscale('linear')
ax[0].set_xlim(left = 0.0, right = 10)
ax[0].set_xlabel('Normalized Bin')
ax[0].set_ylim(bottom = -70, top = 10)
ax[0].set_ylabel('Normalized Power Magnitude [dB]')
ax[0].grid(True)
ax[0].legend()


#%% Validation with Spectrum
optTemp = FreqTrans.OptSpect(dftType = 'dftMat', scaleType = 'density', freqRate = freqRate_rps, smooth = ('box', 1))
optTemp.winType = ('tukey', 0.0)
#optTemp.winType = 'bartlett'
optTemp.freq = freqChan_rps
optTemp.freqInterp = freqExc_rps


optTempN = FreqTrans.OptSpect(dftType = 'dftMat', scaleType = 'density', freqRate = freqRate_rps, smooth = ('box', 1))
optTempN.winType = ('tukey', 0.0)
#optTempN.winType = 'bartlett'
optTempN.freq = np.vstack((freqGap_rps,freqGap_rps,freqGap_rps))
optTemp.freqInterp = freqExc_rps


# FRF Estimate
stride = 10
lenX = len(time_s) / stride
lenFreq = optTemp.freq.shape[-1]
lenFreqN = optTempN.freq.shape[-1]

numSeg = int(lenX)


t_s = np.zeros(numSeg)
PwwList = np.zeros((numSeg, numChan, lenFreq))
PwwNList = np.zeros((numSeg, numChan, lenFreqN))
for iSeg in range(0, numSeg):
    iEnd = iSeg * stride
    print ( 100 * iSeg / numSeg )
    _, _, Pww = FreqTrans.Spectrum(exc[:, 0:iEnd+1], optTemp)
    _, _, PwwN = FreqTrans.Spectrum(exc[:, 0:iEnd+1], optTempN)

    t_s[iSeg] = time_s[iEnd+1]
    PwwList[iSeg, ] = Pww
    PwwNList[iSeg, ] = PwwN


#%% Plot
N = len(time_s)

#PwwList[:,] = np.nan
#PwwNList[:,] = np.nan

PwwListMag = np.abs(PwwList[:,0,:])
PwwNListMag = np.abs(PwwNList[:,0,:])

PwwListMean = np.mean(PwwListMag, axis=-1)
PwwListStd = np.std(PwwListMag, axis=-1)
PwwListMin = np.min(PwwListMag, axis=-1)
PwwListMax = np.max(PwwListMag, axis=-1)

PwwNListMean = np.mean(PwwNListMag, axis=-1)
PwwNListStd = np.std(PwwNListMag, axis=-1)
PwwNListMin = np.min(PwwNListMag, axis=-1)
PwwNListMax = np.max(PwwNListMag, axis=-1)

freqStep_rps = freqExc_rps[1] - freqExc_rps[0]

freqStep_hz = freqStep_rps * rps2hz
freqMinDes_rps = freqMinDes_hz * hz2rps
tBinWidth = FreqTrans.FreqStep2TimeBin(freqStepDes_rps)

numCycles / freqMinDes_hz
tBinD = np.linspace(0, time_s[-1] / tBinWidth, 80)
timeD_s = tBinWidth * tBinD


D = FreqTrans.Dirichlet(tBinD, N+1)
Dmag = np.abs(D)
Dmax = np.nanmax(Dmag)
Dnorm_mag = Dmag / Dmax


tBinDapprox = np.linspace(0.0, time_s[-1] / tBinWidth, 80)
timeDapprox_s = tBinWidth * tBinDapprox
Dapprox_mag = 10**-0.5 * 1/tBinDapprox


tBinB = np.linspace(0, numCycles, 80)
timeB_s = tBinWidth * tBinB

B = FreqTrans.Bartlett(tBinB, N+1)
Bmag = np.abs(B)
Bmax = np.nanmax(Bmag)
Bnorm_mag = Bmag / Bmax

tBinBapprox = np.linspace(0.0, time_s[-1] / tBinWidth, 80)
timeBapprox_s = tBinWidth * tBinBapprox
Bapprox_mag = 10**-(2/5) * (1/tBinBapprox)**2

fig = 2
fig = FreqTrans.PlotGainTemporal(t_s, PwwNListMean / PwwNListMean.max(), None, None, None, fig = fig, dB = False, linestyle='-', color='b', label = 'Null Estimate at Input')

fig = FreqTrans.PlotGainTemporal(timeD_s, Dnorm_mag, None, None, None, fig = fig, dB = False, UncSide = 'Max', linestyle='-', color='r', label = 'Dirichlet Function')
fig = FreqTrans.PlotGainTemporal(timeDapprox_s, Dapprox_mag, None, None, None, fig = fig, dB = False, UncSide = 'Max', linestyle=':', color='r', label = 'Dirichlet Approximation')

#fig = FreqTrans.PlotGainTemporal(timeB_s, Bnorm_mag, None, None, None, fig = fig, dB = False, UncSide = 'Max', linestyle='-', color='r', label = 'Bartlett Function')
#fig = FreqTrans.PlotGainTemporal(timeBapprox_s, Bapprox_mag, None, None, None, fig = fig, dB = False, UncSide = 'Max', linestyle=':', color='r', label = 'Bartlett Approximation')

ax = fig.get_axes()
ax[0].set_xlim(0, time_s[-1])
#ax[0].set_ylim(-40, 0)
ax[0].set_ylim(0, 1.2)
ax[0].set_ylabel(str.replace(ax[0].get_ylabel(), 'Gain', 'Normalized Power Magnitude'))


#%%
PexcD = (PwwListMean.max()) * timeD_s / timeD_s[-1]
PexcDApp = (PwwListMean.max()) * timeDapprox_s / timeDapprox_s[-1]

fig = 3
fig = FreqTrans.PlotGainTemporal(t_s, PwwListMean, None, None, PwwListStd, fig = fig, dB = False, linestyle='-', color='k', label = 'Excitation Estimate at Input')
fig = FreqTrans.PlotGainTemporal(timeD_s, PexcD, None, None, None, fig = fig, dB = False, linestyle='-', color='r', label = 'Excitation Estimate at Input')
fig = FreqTrans.PlotGainTemporal(timeDapprox_s, PexcDApp, None, None, None, fig = fig, dB = False, linestyle=':', color='r', label = 'Excitation Estimate at Input')


#%%
uENRMean = PwwNListMean / PwwListMean
uENRStd = np.std(PwwListMag, axis=-1) / PwwListMean
uENRMax = PwwNListMax / PwwListMin * uENRMean
uENRMax[uENRMax>uENRMean] = uENRMean[uENRMax>uENRMean]

dENR = (Dnorm_mag / PexcD) * PwwNListMean.max()
dENRapprox = (Dapprox_mag / PexcDApp) * PwwNListMean.max()

fig = 4
fig = FreqTrans.PlotGainTemporal(t_s, uENRMean, None, None, uENRMax, fig = fig, dB = True, UncSide = 'Max', linestyle='-', color='k', label = 'Leakage Estimate')
fig = FreqTrans.PlotGainTemporal(timeD_s, dENR, None, None, None, fig = fig, dB = True, UncSide = 'Max', linestyle='-', color='r', label = 'Dirichlet Function')
fig = FreqTrans.PlotGainTemporal(timeDapprox_s, dENRapprox, None, None, None, fig = fig, dB = True, UncSide = 'Max', linestyle=':', color='r', label = 'Dirichlet Approximation')

ax = fig.get_axes()
# ax[0].set_xlim(0, time_s[-1])
ax[0].set_xlim(right = 3 * tBinWidth)
ax[0].set_ylim(bottom = -60)
ax[0].set_ylabel(str.replace(ax[0].get_ylabel(), 'Gain [dB]', 'Null / Excitation [mag]'))

ax[0].legend()
FreqTrans.FixLegend(ax[0])
fig.set_size_inches([6.4,2.4])
if False:
    FreqTrans.PrintPrettyFig(fig, 'PowerLeakage.pgf')


#%%

fBin = np.linspace(0, 5, num = 300)

freqExc_rps / freqExc_rps[0]
freqExc_rps / freqRate_rps

freqStep_rps / freqExc_rps[0]
freqStep_rps / freqRate_rps

Pww = PwwListMean.sum()

# Pexc = Pww * t / T
# Pexc = uPeak**2 *

PwwListMag = np.abs(PwwList[:,0,:])
PwwNListMag = np.abs(PwwNList[:,0,:])

PwwListMean = np.sum(PwwListMag, axis=-1)


Dapprox_mag = FreqTrans.DirichletApprox(fBin)
Dapprox_dB = mag2db( Dapprox_mag )

Bapprox_mag = FreqTrans.BartlettApprox(fBin)
Bapprox_dB = mag2db( Bapprox_mag )

Happrox_mag = FreqTrans.HannApprox(fBin)
Happrox_dB = mag2db( Happrox_mag )


DapproxBest_mag = FreqTrans.DirichletApprox(fBin)
BapproxBest_mag = FreqTrans.BartlettApprox(fBin)
HapproxBest_mag = FreqTrans.HannApprox(fBin)

WapproxBest_mag = np.nanmin(np.array([DapproxBest_mag,BapproxBest_mag,HapproxBest_mag]), axis=0)
WapproxBest_dB = mag2db( WapproxBest_mag )

fBinInt = np.arange(1, 5)

DBest_mag = np.abs(FreqTrans.Dirichlet(fBinInt, len(fBinInt)))
BBest_mag = np.abs(FreqTrans.Bartlett(fBinInt, len(fBinInt)))
HBest_mag = np.abs(FreqTrans.Hann(fBinInt, len(fBinInt)))

WBest_mag = np.nanmin(np.array([DBest_mag,BBest_mag,HBest_mag]), axis=0)
WBest_dB = mag2db( WBest_mag )


fig = plt.figure()
plt.semilogx(fBin, Dapprox_dB, color = 'b', label = 'Dirichlet Approximation')
plt.semilogx(fBin, Bapprox_dB, color = 'g', label = 'Bartlett Approximation')
plt.semilogx(fBin, Happrox_dB, color = 'r', label = 'Hann Approximation')
plt.semilogx(fBin, WapproxBest_dB, ':k', label = 'Best Choice')
# plt.semilogx(fBinInt, WBest_dB, '*k', label = 'Best Choice')
plt.grid(True)
plt.xlabel('Normalized Bin')
plt.ylabel('Null / Excitation [dB]')
plt.legend()

fig.set_size_inches([6.4,3.6])
if False:
    FreqTrans.PrintPrettyFig(fig, 'WindFuncBest.pgf')

