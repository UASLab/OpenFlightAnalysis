"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal


import FreqTrans
import GenExcite

# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps


#%% Define a linear system
freqRate_hz = 50.0
freqRate_rps = freqRate_hz * hz2rps

wn = 2 * hz2rps
d = 0.2

sys = signal.TransferFunction([wn**2], [1, 2*d*wn, wn**2])
freqSys_rps = np.fft.rfftfreq(500, 1/freqRate_rps)

freqSys_rps, Txy = signal.freqresp(sys, w=freqSys_rps)
freqSys_rps, gainSys_dB, phaseSys_deg = signal.bode(sys, w=freqSys_rps)
freqSys_hz = freqSys_rps * rps2hz


#%% Chirp signal with FFT
freqInit_rps = 0.1 * hz2rps
freqFinal_rps = 10 * hz2rps
timeDur_s = 10.0
ampInit = 1.0
ampFinal = ampInit

time_s = np.linspace(0.0, timeDur_s, int(timeDur_s * freqRate_hz) + 1)

# Generate the chirp
x, ampChirpX, freqChirp_rps = GenExcite.Chirp(freqInit_rps, freqFinal_rps, time_s, ampInit, ampFinal, freqType = 'linear', ampType = 'linear', initZero = 1)

# Simulate the excitation through the system
time_s, y, _ = signal.lsim(sys, x, time_s)

# Estimate the transfer function
optSpec = FreqTrans.OptSpect(freqRate = freqRate_rps, smooth = ('box', 5))
freq_rps, Txy, Cxy, Pxx, Pyy, Pxy = FreqTrans.FreqRespFuncEst(x, y, optSpec)
gain_dB, phase_deg = FreqTrans.GainPhase(Txy)
freq_hz = freq_rps * rps2hz

plt.figure(1)
ax1 = plt.subplot(3,1,1); plt.grid
ax1.semilogx(freqSys_hz, gainSys_dB) 
ax1.semilogx(freq_hz, gain_dB, '--')
ax2 = plt.subplot(3,1,2); plt.grid
ax2.semilogx(freqSys_hz, phaseSys_deg)
ax2.semilogx(freq_hz, phase_deg, '--'); plt.ylim([-180, 180]);
ax3 = plt.subplot(3,1,3); plt.grid
ax3.semilogx(freqSys_hz, np.ones_like(freqSys_hz))
ax3.semilogx(freq_hz, Cxy, '--'); plt.ylim([0, 1.2])


#%% Multisine signal with CZT
numChan = 1
numCycles = 1

freqMinDes_rps = 0.1 * hz2rps * np.ones(numChan)
freqMaxDes_rps = 10 * hz2rps *  np.ones(numChan)
freqStepDes_rps = (5/freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqElem_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)

# Generate Schroeder MultiSine Signal
ampElem_nd = np.linspace(ampInit, ampInit, len(freqElem_rps)) / np.sqrt(len(freqElem_rps))
x, _, sigElem = GenExcite.Schroeder(freqElem_rps, ampElem_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak');


# Simulate the excitation through the system
time_s, y, _ = signal.lsim(sys, x.squeeze(), time_s)


# Estimate the transfer function
optSpectCzt = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, freq = freqElem_rps)
freq_rps, Txy, Cxy, Pxx, Pyy, Pxy = FreqTrans.FreqRespFuncEst(x, y, optSpectCzt)
gain_dB, phase_deg = FreqTrans.GainPhase(Txy)
freq_hz = freq_rps * rps2hz


ax1.semilogx(freq_hz, gain_dB, '.')
ax2.semilogx(freq_hz, phase_deg, '.')
ax3.semilogx(freq_hz, Cxy, '.')

