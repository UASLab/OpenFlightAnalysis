"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Example script for generating Schroeder type Multisine Excitations.
"""

import numpy as np
import matplotlib.pyplot as plt

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
hz2rps = 2*np.pi
rps2hz = 1/hz2rps


#%% Signal length - Window analytic
numSampList = np.arange(100, 1001, 100)
numSampMin = np.min(numSampList)
numSampMax = np.max(numSampList)

theta = np.linspace(0, 0.05*np.pi, numSampMin)
theta = theta[1:]
bins = theta * numSampMin / (2*np.pi)

plt.figure(1)
binList = []
for i, N in enumerate(numSampList):
    W_mag = np.exp(-1j * theta * (N-1)/2) * np.sin(N * theta/2) / np.sin(theta/2)
    W_mag = np.abs(W_mag) / np.max(np.abs(W_mag))
    
    W_dB = 20 * np.log10(W_mag)
    plt.plot(bins, W_dB)
    
    binList.append( 2 * np.interp(12, -W_dB, bins))

plt.grid('on')
plt.xlabel('Bin (-)')
plt.ylabel('Power Spectrum (dB)')
plt.ylim(bottom = -60, top = 10)
plt.show

#
plt.figure(2)
plt.title('12dB Bandwidth')
overSamp = numSampList / numSampMin
plt.loglog(overSamp, np.asarray(binList), '-*')
plt.xlabel('Oversample (nSample/nBins)')
plt.ylabel('Number of bins')
plt.grid('on')
plt.show

#%% Signal length - Window only
numSampList = np.arange(100, 1001, 100)
numSampMax = int(np.max(numSampList))
numSampMin = int(np.min(numSampList))
overSamp = numSampList / numSampMin

optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = 1 * hz2rps, smooth = ('box', 1))
#optSpec.scaleType = 'density'
optSpec.winType = ('tukey', 0.0)
optSpec.freq = np.linspace(0, 0.05*np.pi, numSampMin-1)

sig = np.ones(numSampMax)
noiseSamples = np.random.randn(numSampMax)
noiseLev = 0.0
sig += noiseLev * noiseSamples

plt.figure(1)
binList = []
for numSamp in numSampList:
    numSamp = int(numSamp)
    
    theta, sigDft, P_mag  = FreqTrans.Spectrum(sig[0:numSamp], optSpec)
    P_dB = 20*np.log10(P_mag[0]) / 2 - 3
    
    bins = theta[0] * len(optSpec.freq[0]) / (2*np.pi)
    plt.plot(bins, P_dB, '-', label = 'Window: ' + str(optSpec.winType) + ' Over Sample: ' + str(numSamp/numSampMin))
    
    binList.append( 2 * np.interp(12, -P_dB, bins))

plt.xlabel('Bins (-)')
plt.ylabel('Power Spectrum (dB)')
plt.ylim(bottom = -60, top = 10)
plt.xlim(left = 0.0)
plt.grid(); plt.legend()
plt.show

#
plt.figure(2)
plt.title('12dB Bandwidth')
plt.loglog(overSamp, np.asarray(binList), '-*')
plt.xlabel('Oversample (nSample/nBins)')
plt.ylabel('Number of bins')
plt.grid('on')
plt.show


#%% Window effects - Noise and Windows
numSamp = 500
sig = np.ones(numSamp)
noiseSamples = np.random.randn(numSamp)

numBin = 500
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = 1 * hz2rps, smooth = ('box', 1))
optSpec.freq = np.linspace(0, 0.05*np.pi, numBin-1)

noiseList = np.linspace(0.0, 1.0, 5)
winList = [('tukey', 0.0), ('tukey', 0.1), ('tukey', 0.5), 'hann', 'hamming', 'blackman', 'blackmanharris']
#winList = [('tukey', 0.0), 'blackmanharris']

if False:
    for win in winList:
        plt.figure()
        for noiseLev in noiseList:
            noise = sig + noiseLev * noiseSamples
            
            optSpec.winType = win
            theta, _, P_mag  = FreqTrans.Spectrum(noise, optSpec)
            P_dB = 20*np.log10(P_mag[0]) / 2 - 3
            
            bins = theta[0] * len(optSpec.freq[0]) / (2*np.pi)
            plt.plot(bins, P_dB, '-', label = 'Window: ' + str(optSpec.winType) + ' - Noise Mag: ' + str(noiseLev))
            
        plt.xlabel('Bins (-)')
        plt.ylabel('Power Spectrum (dB)')
        plt.grid(); plt.legend()
        plt.show

if True:
    for noiseLev in noiseList:
        noise = sig + noiseLev * noiseSamples
        
        plt.figure()
        for win in winList:
            optSpec.winType = win
            theta, _, P_mag  = FreqTrans.Spectrum(noise, optSpec)
            P_dB = 20*np.log10(P_mag[0]) / 2 - 3
        
            bins = theta[0] * len(optSpec.freq[0]) / (2*np.pi)
            plt.plot(bins, P_dB, '-', label = 'Window: ' + str(optSpec.winType) + ' - Noise Mag: ' + str(noiseLev))

        plt.xlabel('Bins (-)')
        plt.ylabel('Power Spectrum (dB)')
        plt.grid(); plt.legend()
        plt.show


#%% Signal Length - Oscillating Signals
freqRate_hz = 100
freq_rps = np.array([1, 2]) * hz2rps
numCycles = np.array([1, 2, 3, 4, 5, 10])
numCycleMin = np.min(numCycles)
numCycleMax = np.max(numCycles)

timeDur_s = numCycleMax/(np.min(freq_rps) * rps2hz)
time_s = np.linspace(0, timeDur_s, int(timeDur_s*freqRate_hz) + 1)
sig = 1 * np.sin(freq_rps[0] * time_s) + 1 * np.sin(freq_rps[1] * time_s)

optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_hz * hz2rps, smooth = ('box', 1))

#winList = [('tukey', 0.0), ('tukey', 0.1), 'hann', 'hamming', 'blackman']
optSpec.winType = ('tukey', 0.0)
numFreq = int((len(sig) - 1) * (numCycleMin/numCycleMax))
optSpec.freq = np.linspace(np.min(freq_rps)-0.5*hz2rps, np.max(freq_rps)+0.5*hz2rps, numFreq)

plt.figure(1)
for iCycles in numCycles:
    iMax = int((len(sig) - 1) * (iCycles/numCycleMax) + 1)
    
    freq_rps, _, P_mag  = FreqTrans.Spectrum(sig[0:iMax], optSpec)
    freq_hz = freq_rps * rps2hz
    P_dB = 20*np.log10(P_mag)
    
    plt.plot(freq_hz[0], P_mag[0], '-', label = 'Cycles: ' + str(iCycles))
    
#plt.xlim(1, 2)
plt.xlabel('Frequency (Hz)')
plt.grid('on')
#plt.ylim(bottom = -60, top = 10)
plt.legend()
plt.show


#%% Define the frequency selection and distribution of the frequencies into the signals
numChan = 1
freqRate_hz = 50;
timeDur_s = 10.0
numCycles = 1

freqMinDes_rps = (numCycles/timeDur_s) * hz2rps * np.ones(numChan)
#freqMaxDes_rps = (freqRate_hz/2) * hz2rps *  np.ones(numChan)
freqMaxDes_rps = 10 * hz2rps *  np.ones(numChan)
freqStepDes_rps = (40 / freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

## Generate MultiSine Frequencies
freqElem_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
timeDur_s = time_s[-1] - time_s[0]

## Generate Schroeder MultiSine Signal
ampElem_nd = np.ones_like(freqElem_rps) ## Approximate relative signal amplitude, create flat
sigList, phaseElem_rad, sigElem = GenExcite.MultiSine(freqElem_rps, ampElem_nd, sigIndx, time_s, costType = 'Schroeder', phaseInit_rad = 0, boundPhase = True, initZero = True, normalize = 'peak');


## Results
peakFactor = GenExcite.PeakFactor(sigList)
peakFactorRel = peakFactor / np.sqrt(2)
print(peakFactorRel)

# Signal Power
sigPowerRel = (ampElem_nd / max(ampElem_nd))**2 / len(ampElem_nd)

if False:
    fig, ax = plt.subplots(ncols=1, nrows=numChan, sharex=True)
    for iChan in range(0, numChan):
        ax[iChan].plot(time_s, sigList[iChan])
        ax[iChan].set_ylabel('Amplitude (nd)')
        ax[iChan].grid(True)
    ax[iChan].set_xlabel('Time (s)')


#%% Plot the Excitation Spectrum
sig = sigList[0]

round(len(sig) / len(freqElem_rps))

dFreq_rps = np.mean(np.diff(freqElem_rps))
freqNull_rps = freqElem_rps[0:-1] + 0.5 * dFreq_rps
freqDense_rps = np.arange(freqElem_rps[0], freqElem_rps[-1], dFreq_rps/36)

optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_hz * hz2rps, freq = freqElem_rps, smooth = ('box', 1), winType=('tukey', 0.0))
optSpecN = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_hz * hz2rps, freq = freqNull_rps, smooth = ('box', 1), winType=('tukey', 0.0))
optSpecD = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_hz * hz2rps, freq = freqDense_rps, smooth = ('box', 1), winType=('tukey', 0.0))


freq_rps, _, P_mag  = FreqTrans.Spectrum(sig, optSpec)
freq_hz = freq_rps * rps2hz
P_dB = 20*np.log10(P_mag)

freqN_rps, _, Pnull_mag  = FreqTrans.Spectrum(sig, optSpecN)
freqN_hz = freqN_rps * rps2hz
Pnull_dB = 20*np.log10(Pnull_mag)

freqD_rps, _, Pdens_mag  = FreqTrans.Spectrum(sig, optSpecD)
freqD_hz = freqD_rps * rps2hz
Pdens_dB = 20*np.log10(Pdens_mag)

    
plt.figure(1)
plt.plot(freqD_hz[0], Pdens_dB[0], '-', label = 'Dense Fill Set')
plt.plot(freq_hz[0], P_dB[0], '*', label = 'Excited Set')
plt.plot(freqN_hz[0], Pnull_dB[0], '*', label = 'Null Set')
plt.legend()
plt.grid()

