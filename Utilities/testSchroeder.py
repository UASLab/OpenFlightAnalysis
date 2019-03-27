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
import scipy.signal as signal
import json

import GenExcite
import FreqTrans

# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps

#%% Define the frequency selection and distribution of the frequencies into the signals
numChan = 3
freqRate_hz = 50;
timeDur_s = 10.0
numCycles = 1

freqMinDes_rps = (numCycles/timeDur_s) * hz2rps * np.ones(numChan)
#freqMaxDes_rps = (freqRate_hz/2) * hz2rps *  np.ones(numChan)
freqMaxDes_rps = 10 * hz2rps *  np.ones(numChan)
freqStepDes_rps = (5/freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

## Generate MultiSine Frequencies
freqElem_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
timeDur_s = time_s[-1] - time_s[0]

## Generate Schroeder MultiSine Signal
ampElem_nd = np.ones_like(freqElem_rps) / np.sqrt(len(freqElem_rps)) ## Approximate relative signal amplitude, create flat
sigList, phaseElem_rad, sigElem = GenExcite.Schroeder(freqElem_rps, ampElem_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak');

## Results
peakFactor = GenExcite.PeakFactor(sigList)
peakFactorRel = peakFactor / np.sqrt(2)
print(peakFactorRel)

plt.figure()
for iChan in range(0, numChan):
    plt.plot(time_s, sigList[iChan])
plt.xlabel('Time (s)');
plt.ylabel('Amplitude (nd)');
plt.grid()
plt.show()


#%% Plot the Excitation Spectrum

## Compute Spectrum of each channel
scaleType = 'spectrum'
winType = ('tukey', 0.0)
detrendType = 'constant'

freq_fft = []
P_dB_fft = []
freq_czt = []
P_dB_czt = []

for iChan, sig in enumerate(sigList):
    freq_rps_fft, _, P_fft  = FreqTrans.Spectrum(sig, fs = freqRate_hz * hz2rps, dftType = 'fft', winType = winType, detrendType = detrendType, scaleType = scaleType)
    freq_fft.append(freq_rps_fft * rps2hz)
    P_dB_fft.append(20*np.log10(P_fft))
    
    freqChan_rps = freqElem_rps[sigIndx[iChan]]
    freq_rps_czt, _, P_czt  = FreqTrans.Spectrum(sig, fs = freqRate_hz * hz2rps, freq = freqChan_rps, dftType = 'czt', winType = winType, detrendType = detrendType, scaleType = scaleType)
    freq_czt.append(freq_rps_czt * rps2hz)
    P_dB_czt.append(20*np.log10(P_czt))
    
nChan = len(P_dB_fft)

plt.figure()
for iChan in range(0, nChan):
    plt.subplot(nChan, 1, iChan+1)
    plt.plot(freq_fft[iChan], P_dB_fft[iChan])
    plt.plot(freq_czt[iChan], P_dB_czt[iChan])
    plt.grid()
    plt.ylabel('Spectrum (dB)');

plt.xlabel('frequency (Hz)');
plt.show()


#%% Create the output for JSON config

timeFinal_s = time_s[-1]
timeStart_s = time_s[0]

jsonMulti = []
for iChan in range(0, numChan):
    iElem = sigIndx[iChan]

    dictChan = {}
    dictChan['Type'] = 'MultiSine'
    dictChan['Duration'] = timeDur_s
    dictChan['Frequency'] = list(freqElem_rps[iElem])
    dictChan['Phase'] = list(phaseElem_rad[iElem])
    dictChan['Amplitude'] = list(ampElem_nd[iElem])

    jsonMulti.append(dictChan)

# print(json.dumps(jsonMulti))
