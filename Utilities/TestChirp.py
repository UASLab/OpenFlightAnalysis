"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Exampe script for generating sin sweep type excitations.
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
freqRate_hz = 50;
freqInit_rps = 0.1 * hz2rps
freqFinal_rps = 10 * hz2rps
timeDur_s = 10.0
ampInit = 1.0
ampFinal = 1.0

time_s = np.linspace(0, timeDur_s, (timeDur_s * freqRate_hz) + 1)
sig, ampChirp, freqChirp_rps = GenExcite.Chirp(freqInit_rps, freqFinal_rps, time_s, ampInit, ampFinal, freqType = 'linear', ampType = 'linear', initZero = 1)

## Results
plt.figure()
plt.subplot(3,1,1)
plt.plot(time_s, sig); plt.grid()
plt.ylabel('Signal (nd)');
plt.subplot(3,1,2)
plt.plot(time_s, ampChirp); plt.grid()
plt.ylabel('ampitude (nd)');
plt.subplot(3,1,3)
plt.plot(time_s, freqChirp_rps * rps2hz); plt.grid()
plt.xlabel('Time (s)'); plt.ylabel('Frequency (Hz)')
plt.show()


#%% Plot the Excitation Spectrum
## Compute Spectrum of each channel
scaleType = 'spectrum'
winType = ('tukey', 0.0)
detrendType = 'constant'
smooth = ('box', 1)

freq_hz, sigDft, Psd_mag = FreqTrans.Spectrum(sig, freqRate_hz, None, 'fft', winType, detrendType, smooth, scaleType)
Psd_dB = 20*np.log10(Psd_mag)

## Plot Spectrum
plt.figure()
plt.plot(freq_hz, Psd_dB)
plt.xlabel('frequency (Hz)');
plt.ylabel('Spectrum (dB)');
plt.grid()
plt.show()


#%% Create the output for JSON config
timeFinal_s = time_s[-1]
timeStart_s = time_s[0]

jsonChirp = {}
jsonChirp['Type'] = 'LinearChirp'
jsonChirp['Duration'] = timeDur_s
jsonChirp['ampitude'] = [ampInit, ampFinal]
jsonChirp['Frequency'] = [freqInit_rps, freqFinal_rps]

print(json.dumps(jsonChirp))
