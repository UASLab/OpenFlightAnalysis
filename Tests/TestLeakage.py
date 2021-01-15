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
mDesire = db2mag(-20)
fbinSel = 10**-0.5 * 1/mDesire
#fbinBSel = 10**-(1/5) * (1/mDesire)**0.5

freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps
dt = 1/freqRate_hz

freqMinDes_hz = 0.1
numCyclesDes = 3 # Plan for 3

freqStepDes_hz = fbinSel / (numCyclesDes / freqMinDes_hz)


#%% Excitation
numCycles = 9 # Need 6!
numExc = 2
ampInit = 1
ampFinal = 1
freqMinDes_rps = freqMinDes_hz * hz2rps * np.ones(numExc)
freqMaxDes_rps = 10.3 * hz2rps *  np.ones(numExc)
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
def Dirichlet(fBin, N):
#    theta = (2*pi) * fBin / N
#    W = np.exp(-1j * (N-1)/2 * theta) * np.sin(N*theta/2) / np.sin(theta/2)
    W = np.exp(-1j * (1-1/N) * pi * fBin) * np.sin(pi * fBin) / np.sin(pi * fBin / N)
    return W

def DirichletApprox(fBin):
    fBinMin = 0.5 # Approximately where the Approximation hits the main-lobe
    #W_dB = -10 + mag2db(0.5) * np.log2(fBin)
    W = 10**-(1/2) * (1/fBin)
    W[fBin<fBinMin] = np.nan
    return W

def Bartlett(fBin, N):
#    theta = (2*pi) * fBin / N
#    W = (2/N) * np.exp(-1j * (N-1)/2 * theta) * (np.sin(N*theta/4) / np.sin(theta/2))**2
    W = (2/N) * np.exp(-1j * (1-1/N) * pi * fBin) * (np.sin(pi * fBin / 2) / np.sin(pi * fBin / N))**2
    return W

def BartlettApprox(fBin):
    fBinMin = 1 # Approximately where the Approximation hits the main-lobe
    #W_dB = -8 + mag2db(0.25) * np.log2(xB)
    W =  10**-(2/5) * (1/fBin)**2
    W[fBin<fBinMin] = np.nan
    return W

N = len(time_s)

fBin = np.linspace(0, 10, num = N)

D = Dirichlet(fBin, N)
Dmag = np.abs(D)
Dmax = np.nanmax(Dmag)
Dnorm_mag = Dmag / Dmax
Dnorm_dB = mag2db(Dnorm_mag)

Dapprox_mag = DirichletApprox(fBin)
Dapprox_dB = mag2db( Dapprox_mag )


B = Bartlett(fBin, N)
Bmag = np.abs(B)
Bmax = np.nanmax(Bmag)
Bnorm_mag = Bmag / Bmax
Bnorm_dB = mag2db(Bnorm_mag)

Bapprox_mag = BartlettApprox(fBin)
Bapprox_dB = mag2db( Bapprox_mag )


fig = 1
fig = FreqTrans.PlotGainType(fBin, Dnorm_mag, None, None, None, fig = fig, dB = True, linestyle='-', color='b', label = 'Dirichlet Function')
fig = FreqTrans.PlotGainType(fBin, Dapprox_mag, None, None, None, fig = fig, dB = True, linestyle='--', color='b', label = 'Dirichlet Approximation')
fig = FreqTrans.PlotGainType(fBin, Bnorm_mag, None, None, None, fig = fig, dB = True, linestyle='-', color='r', label = 'Bartlett Function')
fig = FreqTrans.PlotGainType(fBin, Bapprox_mag, None, None, None, fig = fig, dB = True, linestyle='--', color='r', label = 'Bartlett Approximation')

ax = fig.get_axes()
ax[0].set_xscale('linear')
ax[0].set_xlim(left = 0.0, right = 10)
ax[0].set_xlabel('Normalized Bin')
ax[0].set_ylim(bottom = -60, top = 10)
ax[0].set_ylabel('Normalized Power Magnitude [dB]')
ax[0].grid(True)
ax[0].legend()

fig.set_size_inches([6.4,3.6])
if False:
    FreqTrans.PrintPrettyFig(fig, 'WindowFunc.pgf')


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
PwwList = np.zeros((numSeg, numExc, lenFreq))
PwwNList = np.zeros((numSeg, numExc, lenFreqN))
for iSeg in range(0, numSeg):
    iEnd = iSeg * stride
    print ( 100 * iSeg / numSeg )
    _, _, Pww = FreqTrans.Spectrum(exc[:, 0:iEnd+1], optTemp)
    _, _, PwwN = FreqTrans.Spectrum(exc[:, 0:iEnd+1], optTempN)
    
    t_s[iSeg] = time_s[iEnd+1]
    PwwList[iSeg, ] = Pww
    PwwNList[iSeg, ] = PwwN


#%% Plot
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
#tBinLen = 2 * 2 * pi / freqStep_rps
freqMinDes_rps = freqMinDes_hz * hz2rps
#tBinLen = fbinSel * 2 * freqMinDes_hz / (numCyclesDes * freqStepDes_hz**2)
tBinLen = fbinSel * 2 * 2* pi * freqMinDes_rps / (numCyclesDes * freqStepDes_rps**2)
#tBinLen = 2 * numCyclesDes / (fbinSel * freqMinDes_hz) # If freqStep was selected from the bin, then this is true

numCycles / freqMinDes_hz
tBinD = np.linspace(0, time_s[-1] / tBinLen, 80)
timeD_s = tBinLen * tBinD


D = Dirichlet(tBinD, N+1)
Dmag = np.abs(D)
Dmax = np.nanmax(Dmag)
Dnorm_mag = Dmag / Dmax


tBinDapprox = np.linspace(0.0, time_s[-1] / tBinLen, 80)
timeDapprox_s = tBinLen * tBinDapprox
Dapprox_mag = 10**-0.5 * 1/tBinDapprox


tBinB = np.linspace(0, numCycles, 80)
timeB_s = tBinLen * tBinB

B = Bartlett(tBinB, N+1)
Bmag = np.abs(B)
Bmax = np.nanmax(Bmag)
Bnorm_mag = Bmag / Bmax

tBinBapprox = np.linspace(0.0, time_s[-1] / tBinLen, 80)
timeBapprox_s = tBinLen * tBinBapprox
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
fig = FreqTrans.PlotGainTemporal(t_s, uENRMean, None, None, uENRMax, fig = fig, dB = False, UncSide = 'Max', linestyle='-', color='k', label = 'Estimate')
fig = FreqTrans.PlotGainTemporal(timeD_s, dENR, None, None, None, fig = fig, dB = False, UncSide = 'Max', linestyle='-', color='r', label = 'Dirichlet Function')
fig = FreqTrans.PlotGainTemporal(timeDapprox_s, dENRapprox, None, None, None, fig = fig, dB = False, UncSide = 'Max', linestyle=':', color='r', label = 'Dirichlet Approximation')

ax = fig.get_axes()
ax[0].set_xlim(0, time_s[-1])
ax[0].set_ylim(0, 1.2)
ax[0].set_ylabel(str.replace(ax[0].get_ylabel(), 'Gain', 'Null / Excitation [mag]'))

ax[0].legend()
FreqTrans.FixLegend(ax[0])
fig.set_size_inches([6.4,2.4])
if False:
    FreqTrans.PrintPrettyFig(fig, 'PowerLeakage.pgf')


#%%

fBin = np.linspace(0, 10, num = 100)

freqExc_rps / freqExc_rps[0]
freqExc_rps / freqRate_rps

freqStep_rps / freqExc_rps[0]
freqStep_rps / freqRate_rps

Pww = PwwListMean.sum()

Pexc = Pww * t / T

Pexc = uPeak**2 * 
PwwListMag = np.abs(PwwList[:,0,:])
PwwNListMag = np.abs(PwwNList[:,0,:])

PwwListMean = np.sum(PwwListMag, axis=-1)


Dapprox_mag = DirichletApprox(fBin)
Dapprox_dB = mag2db( Dapprox_mag )

Bapprox_mag = BartlettApprox(fBin)
Bapprox_dB = mag2db( Bapprox_mag )


plt.semilogx(fBin, Dapprox_dB, label = 'Dirichlet Approximation')
plt.semilogx(fBin, Bapprox_dB, label = 'Bartlett Approximation')
plt.grid(True)
plt.xlabel('Normalized Bin')
plt.ylabel('Null / Excitation [mag]')
plt.legend()


