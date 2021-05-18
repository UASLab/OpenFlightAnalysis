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
pfGuess = 2.0
freqMinDes_hz = 0.10
freqMaxDes_hz = 10.0
numCyclesDes = 3


binType = 'step'
if binType == 'step':
    # Specify Length (number of cycles and lowest frequency), solve for step size
    freqMinDes_hz = 0.10
elif binType == 'length':
    # Specify step size, solve for (number of cycles and lowest frequency)
    freqStepDes_hz = 0.10


#mDesire_dB = -20
#P_Desire_mag = db2mag(mDesire_dB)
#PexcPsd = 0.5 * (ampExcit_nd**2).sum() * numCyclesDes / freqMinDes_hz

PleakageDesire_dB = -34
PleakageDesire_mag = db2mag(PleakageDesire_dB)
PexcPsdGuess = (1/pfGuess) * numCyclesDes / freqMinDes_hz
P_Desire_mag = PleakageDesire_mag * PexcPsdGuess

winName = 'Dirichlet'
if winName == 'Dirichlet':
    fbinSel = 2 * DirichletApproxInv(P_Desire_mag)  # Factor of 2 is because the Null is 1/2 spaced
    DirichletApprox(fbinSel)
if winName == 'Bartlett':
    fbinSel = 2 * BartlettApproxInv(P_Desire_mag)
    BartlettApprox(fbinSel)


if binType == 'step':
    # Specify Length (number of cycles and lowest frequency), solve for step size
    freqStepDes_hz = fbinSel * 2 * freqMinDes_hz / numCyclesDes

elif binType == 'length':
    # Specify step size, solve for (number of cycles and lowest frequency)
    freqMinDes_hz = numCyclesDes * freqStepDes_hz / (2 * fbinSel)

freqStepDes_hz = 0.1

freqRate_rps = freqRate_hz * hz2rps
freqStepDes_rps = freqStepDes_hz * hz2rps
freqMinDes_rps = freqMinDes_hz * hz2rps

dt = 1/freqRate_hz
T = numCyclesDes / freqMinDes_hz

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
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_rps, numCycles, freqStepDes_rps, methodSW)
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

# Generate Schroeder MultiSine Signal
ampExcit_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps))
exc, phaseElem_rad, sigExcit = GenExcite.MultiSine(freqExc_rps, ampExcit_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = None, costType = 'Schroeder')
#exc = exc / np.std(exc)
excStd = np.std(exc, axis = -1)
peakFactor = GenExcite.PeakFactor(exc)
excPeak = np.mean(peakFactor * excStd)**2

peakFactorRel = peakFactor / np.sqrt(2)

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]
freqChan_hz = freqChan_rps * rps2hz


#%%
N = 2**8

fBin = np.linspace(0, 10, num = N)

D = FreqTrans.Dirichlet(fBin, N)
Dmag = np.abs(D)
Dmax = np.nanmax(Dmag)

Dapprox_mag = FreqTrans.DirichletApprox(fBin)
Dapprox_dB = mag2db( Dapprox_mag )


B = FreqTrans.Bartlett(fBin, N)
Bmag = np.abs(B)
Bmax = np.nanmax(Bmag)

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
optTemp = FreqTrans.OptSpect(dftType = 'dftMat', scaleType = 'density', freqRate_rps = freqRate_rps, smooth = ('box', 1))

if winName == 'Dirichlet':
    optTemp.winType = ('tukey', 0.0)
if winName == 'Bartlett':
    optTemp.winType = 'bartlett'

optTemp.freq_rps = freqChan_rps
optTemp.freqInterp = freqExc_rps


optTempN = FreqTrans.OptSpect(dftType = 'dftMat', scaleType = 'density', freqRate_rps = freqRate_rps, smooth = ('box', 1))

if winName == 'Dirichlet':
    optTempN.winType = ('tukey', 0.0)
if winName == 'Bartlett':
    optTempN.winType = 'bartlett'

optTempN.freq_rps = np.atleast_2d(freqGap_rps)

# FRF Estimate
stride = 10
lenX = len(time_s) / stride
lenFreq = optTemp.freq_rps.shape[-1]
lenFreqN = optTempN.freq_rps.shape[-1]

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

axisSel = -1
PwwListMag = np.abs(PwwList)
PwwListSum = PwwListMag.sum(axis = -1).sum(axis = -1)
PwwListStd = PwwListMag.std(axis = -1).sum(axis = -1)

PwwNListMag = np.abs(PwwNList)
PwwNListSum = PwwNListMag.sum(axis = -1)


freqMin_hz = freqExc_rps[0] * rps2hz
T = numCycles / freqMin_hz
N = T / dt

freqStep_rps = freqExc_rps[1] - freqExc_rps[0]
freqStep_hz = freqStep_rps * rps2hz
freqMinDes_rps = freqMinDes_hz * hz2rps
tBinWidth = FreqTrans.FreqStep2TimeBin(freqStepDes_rps)

numCycles / freqMinDes_hz
tBinD = np.linspace(0, time_s[-1] / tBinWidth, 80)
timeD_s = tBinWidth * tBinD

tBin = np.linspace(0, time_s[-1] / tBinLen, 80)
timeBin_s = tBinLen * tBin

D = FreqTrans.Dirichlet(tBinD, N+1)
Dmag = np.abs(D)
Dmax = np.nanmax(Dmag)
Dnorm_mag = Dmag / Dmax

#%% Excitation Power
PexcSpec = 0.5 * (ampExcit_nd**2).sum()
PexcPsd = PexcSpec * N / freqRate_hz # Convert Total Spectrum Power to Density
PparsevalSpec = (1/N) * ((np.abs(exc)**2).sum(axis = -1)).sum()
PparsevalPsd = PparsevalSpec * N / freqRate_hz # Convert Total Spectrum Power to Density

if winName == 'Dirichlet':
    Pexc = PexcPsd * (timeBin_s / timeBin_s[-1])
#    Pexc = PparsevalPsd * (timeBin_s / timeBin_s[-1])
if winName == 'Bartlett':
    Pexc = np.sqrt(0.5) * PexcPsd * (timeBin_s / timeBin_s[-1])
#    Pexc = np.sqrt(0.5) * PparsevalPsd * (timeBin_s / timeBin_s[-1])

if True:
    fig = 2
    fig = FreqTrans.PlotGainTemporal(t_s, PwwListSum, None, None, PwwListStd, fig = fig, dB = False, linestyle='-', color='k', label = 'Excitation Estimate at Input')
    fig = FreqTrans.PlotGainTemporal(timeBin_s, Pexc, None, None, None, fig = fig, dB = False, linestyle=':', color='r', marker = '', label = 'Excitation Estimate at Input')
#    fig = FreqTrans.PlotGainTemporal(timeBin_s[-1], PparsevalPsd, None, None, None, fig = fig, dB = False, linestyle='-', color='b', marker = '*', label = 'Excitation Estimate at Input')

    ax = fig.get_axes()
    ax[0].set_ylabel(str.replace(ax[0].get_ylabel(), 'Gain', 'Power Spectral Density'))


tBinDapprox = np.linspace(0.0, time_s[-1] / tBinWidth, 80)
timeDapprox_s = tBinWidth * tBinDapprox
Dapprox_mag = 10**-0.5 * 1/tBinDapprox

# Dirichlet
D = Dirichlet(tBin, N+1)
Dmag = np.abs(D)
Pnorm_Dirichlet_mag = 2**1 * Dmag / N * P2Leak # Normalized, 2**1 is correct, want 2x spacing

tBinB = np.linspace(0, numCycles, 80)
timeB_s = tBinWidth * tBinB

B = FreqTrans.Bartlett(tBinB, N+1)
Bmag = np.abs(B)
Pnorm_Bartlett_mag = 2**2 * Bmag / N * P2Leak # Normalized, 2**2 is correct, want 2x spacing

tBinBapprox = np.linspace(0.0, time_s[-1] / tBinWidth, 80)
timeBapprox_s = tBinWidth * tBinBapprox
Bapprox_mag = 10**-(2/5) * (1/tBinBapprox)**2


if True:
    fig = 3
    fig = FreqTrans.PlotGainTemporal(t_s, PwwNListSum, None, None, None, fig = fig, dB = False, linestyle='-', color='b', label = 'Null Estimate at Input')

    if winName == 'Dirichlet':
        fig = FreqTrans.PlotGainTemporal(timeBin_s, Pnorm_Dirichlet_mag, None, None, None, fig = fig, dB = False, UncSide = 'Max', linestyle='-', color='r', label = 'Dirichlet Function')
        fig = FreqTrans.PlotGainTemporal(timeBin_s, Pnorm_DirichletApprox_mag, None, None, None, fig = fig, dB = False, UncSide = 'Max', linestyle=':', color='r', label = 'Dirichlet Approximation')

    if winName == 'Bartlett':
        fig = FreqTrans.PlotGainTemporal(timeBin_s, Pnorm_Bartlett_mag, None, None, None, fig = fig, dB = False, UncSide = 'Max', linestyle='-', color='r', label = 'Bartlett Function')
        fig = FreqTrans.PlotGainTemporal(timeBin_s, Pnorm_BartlettApprox_mag, None, None, None, fig = fig, dB = False, UncSide = 'Max', linestyle=':', color='r', label = 'Bartlett Approximation')

    ax = fig.get_axes()
    #ax[0].set_xlim(0, time_s[-1])
    #ax[0].set_ylim(-40, 0)
    #ax[0].set_ylim(0, 1.2 )
    ax[0].set_ylabel(str.replace(ax[0].get_ylabel(), 'Gain', 'Normalized Power Magnitude'))


#%% Null-to-Excitation Ratio
uENR = PwwNListSum / PwwListSum
uENRStd = PwwListStd / PwwListSum
uENRMax = uENR - 0.5 * uENRStd
#uENRMax[uENRMax>uENR] = uENR[uENRMax>uENR]

dENR = Pnorm_Dirichlet_mag / Pexc
dENRapprox = Pnorm_DirichletApprox_mag / Pexc

bENR = Pnorm_Bartlett_mag / Pexc
bENRapprox = Pnorm_BartlettApprox_mag / Pexc

fig = 4
fig = FreqTrans.PlotGainTemporal(t_s, uENR, None, None, uENRMax, fig = fig, dB = True, UncSide = 'Max', linestyle='-', color='k', label = 'Estimate')

if winName == 'Dirichlet':
    fig = FreqTrans.PlotGainTemporal(timeBin_s, dENR, None, None, None, fig = fig, dB = True, UncSide = 'Max', linestyle='-', color='r', label = 'Dirichlet Function')
    fig = FreqTrans.PlotGainTemporal(timeBin_s, dENRapprox, None, None, None, fig = fig, dB = True, UncSide = 'Max', linestyle=':', color='r', label = 'Dirichlet Approximation')
if winName == 'Bartlett':
    fig = FreqTrans.PlotGainTemporal(timeBin_s, bENR, None, None, None, fig = fig, dB = True, UncSide = 'Max', linestyle='-', color='r', label = 'Bartlett Function')
    fig = FreqTrans.PlotGainTemporal(timeBin_s, bENRapprox, None, None, None, fig = fig, dB = True, UncSide = 'Max', linestyle=':', color='r', label = 'Bartlett Approximation')

# Excitation Power and Null Target to Target ENR
t_nd = (timeBin_s / timeBin_s[-1])
#t_nd[timeBin_s < tBinLen] = np.nan

#PleakagePredict_mag = PleakageDesire_mag / t_nd**(2) # Factor of 0.5 because null spacing is 1/2 that of excitation
#PleakagePredict_mag = (peakFactor.mean() / pfGuess) * PleakageDesire_mag / t_nd**2 # Factor of 0.5 because null spacing is 1/2 that of excitation
PleakagePredict_mag = PleakageDesire_mag * np.ones_like(timeBin_s)

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
