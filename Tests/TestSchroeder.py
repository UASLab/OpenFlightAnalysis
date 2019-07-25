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

#%% Define the frequency selection and distribution of the frequencies into the signals
numChan = 3
freqRate_hz = 50;
timeDur_s = 10.0
numCycles = 5

freqMinDes_rps = (numCycles/timeDur_s) * hz2rps * np.ones(numChan)
#freqMaxDes_rps = (freqRate_hz/2) * hz2rps *  np.ones(numChan)
freqMaxDes_rps = 10 * hz2rps *  np.ones(numChan)
freqStepDes_rps = (5 / freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

## Generate MultiSine Frequencies
freqElem_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
timeDur_s = time_s[-1] - time_s[0]

## Generate Schroeder MultiSine Signal
ampElem_nd = np.ones_like(freqElem_rps) ## Approximate relative signal amplitude, create flat
sigList, phaseElem_rad, sigElem = GenExcite.MultiSine(freqElem_rps, ampElem_nd, sigIndx, time_s, costType = 'Schroeder', phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak');


## Results
peakFactor = GenExcite.PeakFactor(sigList)
peakFactorRel = peakFactor / np.sqrt(2)
print(peakFactorRel)

# Signal Power
sigPowerRel = (ampElem_nd / max(ampElem_nd))**2 / len(ampElem_nd)



if True:
    fig, ax = plt.subplots(ncols=1, nrows=numChan, sharex=True)
    for iChan in range(0, numChan):
        ax[iChan].plot(time_s, sigList[iChan])
        ax[iChan].set_ylabel('Amplitude (nd)')
        ax[iChan].grid(True)
    ax[iChan].set_xlabel('Time (s)')

if True:
    fig, ax = plt.subplots(ncols=1, nrows=numChan, sharex=True)
    for iChan in range(0, numChan):
        for iElem in sigIndx[iChan]:
            ax[iChan].plot(time_s, sigElem[iElem])
        ax[iChan].set_ylabel('Amplitude (nd)')
        ax[iChan].grid(True)
    ax[iChan].set_xlabel('Time (s)')


#%% Plot the Excitation Spectrum

## Compute Spectrum of each channel
optFFT = FreqTrans.OptSpect(freqRate = freqRate_hz * hz2rps)
optCZT = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_hz * hz2rps)

freq_fft = []
P_dB_fft = []
freq_czt = []
P_dB_czt = []

for iChan, sig in enumerate(sigList):
    freq_rps_fft, _, P_fft  = FreqTrans.Spectrum(sig, optFFT)
    freq_fft.append(freq_rps_fft * rps2hz)
    P_dB_fft.append(20*np.log10(P_fft))
    
    optCZT.freq = freqElem_rps[sigIndx[iChan]]
    freq_rps_czt, _, P_czt  = FreqTrans.Spectrum(sig, optCZT)
    freq_czt.append(freq_rps_czt * rps2hz)
    P_dB_czt.append(20*np.log10(P_czt))
    
nChan = len(P_dB_fft)

plt.figure()
for iChan in range(0, nChan):
    plt.subplot(nChan, 1, iChan+1)
    plt.plot(freq_fft[iChan].T, P_dB_fft[iChan].T, '-k', label='FFT Pxx')
    plt.plot(freq_czt[iChan].T, P_dB_czt[iChan].T, '.r-', label='CZT Pxx')
    plt.grid()
    plt.ylabel('Spectrum (dB)');

plt.xlabel('frequency (Hz)');
plt.legend()
plt.show()


#%% Create the output for JSON config

timeFinal_s = time_s[-1]
timeStart_s = time_s[0]

jsonMulti = {}
for iChan in range(0, numChan):
    iElem = sigIndx[iChan]
        
    dictChan = {}
    dictChan['Type'] = 'MultiSine'
    dictChan['Duration'] = timeDur_s
    dictChan['Frequency'] = list(freqElem_rps[iElem])
    dictChan['Phase'] = list(phaseElem_rad[iElem])
    dictChan['Amplitude'] = list(ampElem_nd[iElem])

    nameChan = 'OMS_' + str(iChan+1)
    
    jsonMulti[nameChan] = dictChan

import json
print(json.dumps(jsonMulti, separators=(', ', ': ')))
