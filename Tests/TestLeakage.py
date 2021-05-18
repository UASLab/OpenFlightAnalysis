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
def Dirichlet(fBin, N):
    theta = (2*pi) * fBin / N
    W = np.exp(-1j * (N-1)/2 * theta) * np.sin(N*theta/2) / np.sin(theta/2)
#    W = np.exp(-1j * (1-1/N) * pi * fBin) * np.sin(pi * fBin) / np.sin(pi * fBin / N)
    W = np.atleast_1d(W)
    return W

def DirichletApprox(fBin):
    fBinMin = 0.5 # Approximately where the Approximation hits the main-lobe
    #W_dB = -10 + mag2db(0.5) * np.log2(fBin)
    W = 10**-(1/2) * fBin**-1
    W = np.atleast_1d(W)
    W[fBin<fBinMin] = np.nan
    return W

def DirichletApproxInv(W):
    fBinMin = 0.5 # Approximately where the Approximation hits the main-lobe
    fBin = 10**-(1/2) * W**-1
    fBin = np.atleast_1d(fBin)
    fBin[fBin<fBinMin] = fBinMin
    return fBin

def Bartlett(fBin, N):
    theta = (2*pi) * fBin / N
    W = (2/N) * np.exp(-1j * (N-1)/2 * theta) * (np.sin(N*theta/4) / np.sin(theta/2))**2
#    W = (2/N) * np.exp(-1j * (1-1/N) * pi * fBin) * (np.sin(pi * fBin / 2) / np.sin(pi * fBin / N))**2
    W = np.atleast_1d(W)
    return W

def BartlettApprox(fBin):
    fBinMin = 1 # Approximately where the Approximation hits the main-lobe
    W =  10**-(7/10) * fBin**-2
    W = np.atleast_1d(W)
    W[fBin<fBinMin] = np.nan
    return W

def BartlettApproxInv(W):
    fBinMin = 1 # Approximately where the Approximation hits the main-lobe
    fBin =  10**-(7/20)  * W**(-1/2)
    fBin = np.atleast_1d(fBin)
    fBin[fBin<fBinMin] = fBinMin
    return fBin


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


freqRate_rps = freqRate_hz * hz2rps
freqStepDes_rps = freqStepDes_hz * hz2rps
freqMinDes_rps = freqMinDes_hz * hz2rps
    
dt = 1/freqRate_hz
T = numCyclesDes / freqMinDes_hz

#%% Excitation
numCycles = numCyclesDes
numExc = 3
ampInit = 1
ampFinal = ampInit
freqMinDes_rps = freqMinDes_hz * hz2rps * np.ones(numExc)
freqMaxDes_rps = freqMaxDes_hz * hz2rps *  np.ones(numExc)
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
N = len(time_s)

fBin = np.linspace(0, 10, num = N)

D = Dirichlet(fBin, N)
Dmag = np.abs(D)
Dmax = np.nanmax(Dmag)

Dapprox_mag = DirichletApprox(fBin) * N


B = Bartlett(fBin, N)
Bmag = np.abs(B)
Bmax = np.nanmax(Bmag)

Bapprox_mag = BartlettApprox(fBin) * N


fig = 1
fig = FreqTrans.PlotGainType(fBin, Dmag / Dmax, None, None, None, fig = fig, dB = True, linestyle='-', color='b', label = 'Dirichlet Function')
fig = FreqTrans.PlotGainType(fBin, Dapprox_mag / Dmax, None, None, None, fig = fig, dB = True, linestyle='--', color='b', label = 'Dirichlet Approximation')
fig = FreqTrans.PlotGainType(fBin, Bmag / Bmax, None, None, None, fig = fig, dB = True, linestyle='-', color='r', label = 'Bartlett Function')
fig = FreqTrans.PlotGainType(fBin, Bapprox_mag / Bmax, None, None, None, fig = fig, dB = True, linestyle='--', color='r', label = 'Bartlett Approximation')

ax = fig.get_axes()
ax[0].set_xscale('linear')
ax[0].set_xlim(left = 0.0, right = 10)
ax[0].set_xlabel('Frequency Bin')
ax[0].set_ylim(bottom = -60, top = 10)
ax[0].set_ylabel('Normalized Power Magnitude [dB]')
ax[0].grid(True)
ax[0].legend()

fig.set_size_inches([6.4,3.6])
if False:
    FreqTrans.PrintPrettyFig(fig, 'WindowFunc.pgf')


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
PwwList = np.zeros((numSeg, numExc, lenFreq))
PwwNList = np.zeros((numSeg, lenFreqN))
for iSeg in range(0, numSeg):
    iEnd = iSeg * stride
    print ( 100 * iSeg / numSeg )
    _, _, Pww = FreqTrans.Spectrum(exc[:, 0:iEnd+1], optTemp)
    _, _, PwwN = FreqTrans.Spectrum(exc[0, 0:iEnd+1], optTempN)
    
    t_s[iSeg] = time_s[iEnd+1]
    PwwList[iSeg, ] = Pww
    PwwNList[iSeg, ] = PwwN


#%% Plot
#PwwList[:,] = np.nan
#PwwNList[:,] = np.nan

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

#tBinLen = 4 * pi / freqStep_rps
tBinLen = 2 / freqStep_hz

tBin = np.linspace(0, time_s[-1] / tBinLen, 80)
timeBin_s = tBinLen * tBin


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
    

#%% Null Power (from Leakage)
P2Leak = 2 * PexcSpec ## XXX Why 2??

# Dirichlet
D = Dirichlet(tBin, N+1)
Dmag = np.abs(D)
Pnorm_Dirichlet_mag = 2**1 * Dmag / N * P2Leak # Normalized, 2**1 is correct, want 2x spacing

Dapprox_mag = DirichletApprox(tBin)
Pnorm_DirichletApprox_mag = 2**1 * Dapprox_mag * P2Leak # 2**1 is correct, want 2x spacing

# Bartlett
B = Bartlett(tBin, N+1)
Bmag = np.abs(B)
Pnorm_Bartlett_mag = 2**2 * Bmag / N * P2Leak # Normalized, 2**2 is correct, want 2x spacing

Bapprox_mag = BartlettApprox(tBin)
Pnorm_BartlettApprox_mag = Bapprox_mag * 2**2 * P2Leak # 2**2 is correct, want 2x spacing


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

fig = FreqTrans.PlotGainTemporal(timeBin_s, PleakagePredict_mag, None, None, None, fig = fig, dB = True, UncSide = 'Max', linestyle=':', color='g', label = 'Target Theshold')
#fig = FreqTrans.PlotGainTemporal(timeBin_s[-1], PleakageDesire_mag, None, None, None, fig = fig, dB = False, UncSide = 'Max', linestyle=':', color='g', marker = 'o', label = 'Target Theshold')

ax = fig.get_axes()
#ax[0].set_xscale('log')
#ax[0].set_yscale('log')
#ax[0].set_xlim(left = 1)
#ax[0].set_xlim(left = 0)
ax[0].set_ylim(bottom = -60)
#ax[0].set_ylim(0, 2)
ax[0].set_ylabel(str.replace(ax[0].get_ylabel(), 'Gain', 'Null / Excitation [mag]'))

ax[0].legend()
FreqTrans.FixLegend(ax[0])
fig.set_size_inches([6.4,2.4])
if False:
    FreqTrans.PrintPrettyFig(fig, 'PowerLeakage.pgf')


#%%
if False:
    #%%
    bD = np.linspace(0, 5, num = 80)
    Da_mag = DirichletApprox(bD)
    Da_dB = mag2db( Da_mag )
    
    bB = np.linspace(0, 10, num = 80)
    Ba_mag = BartlettApprox(bB)
    Ba_dB = mag2db( Ba_mag )
    
    plt.figure()
    plt.semilogx(bD, Da_dB, label = 'Dirichlet Approximation')
    plt.semilogx(bB, Ba_dB, label = 'Bartlett Approximation')
    plt.semilogx(bB, mag2db(PleakagePredict_mag), label = 'Target')
    
    
    plt.semilogx(fbinSel, mag2db(PleakageDesire_mag), '*')
    plt.grid(True)
    plt.xlabel('Normalized Bin')
    plt.ylabel('Null / Excitation [mag]')
    plt.legend()

