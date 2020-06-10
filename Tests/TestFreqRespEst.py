"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Example script for testing Frequency Response Estimation - SISO.
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal
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
hz2rps = 2*np.pi
rps2hz = 1/hz2rps

rad2deg = 180/np.pi
deg2rad = 1/rad2deg

mag2db = control.mag2db
db2mag = control.db2mag


#%% Define a linear system
freqRate_hz = 50.0
freqRate_rps = freqRate_hz * hz2rps

wn = 3 * hz2rps
d = 0.2

sys = signal.TransferFunction([wn**2], [1, 2*d*wn, wn**2])
freqLin_rps = np.fft.rfftfreq(1000, 1/freqRate_rps)

freqLin_rps, Txy = signal.freqresp(sys, w=freqLin_rps)
freqLin_rps, gainLin_dB, phaseLin_deg = signal.bode(sys, w=freqLin_rps)
freqLin_hz = freqLin_rps * rps2hz


#%% Chirp signal with FFT
freqInit_rps = 0.1 * hz2rps
freqFinal_rps = 20 * hz2rps
timeDur_s = 10.0
ampInit = 1.0
ampFinal = ampInit

time_s = np.linspace(0.0, timeDur_s, int(timeDur_s * freqRate_hz) + 1)

# Generate the chirp
x, ampChirpX, freqChirp_rps = GenExcite.Chirp(freqInit_rps, freqFinal_rps, time_s, ampInit, ampFinal, freqType = 'linear', ampType = 'linear', initZero = 1)

# Simulate the excitation through the system
time_s, y, _ = signal.lsim(sys, x, time_s)
y = np.atleast_2d(y)

# Estimate the transfer function
optSpec = FreqTrans.OptSpect(freqRate = freqRate_rps, smooth = ('box', 1), winType=('tukey', 0.0))
freq_rps, Txy, Cxy, Pxx, Pyy, Pxy = FreqTrans.FreqRespFuncEst(x, y, optSpec)
gain_dB, phase_deg = FreqTrans.GainPhase(Txy)
freq_hz = freq_rps * rps2hz

freq_hz = np.squeeze(freq_hz)
gain_dB = np.squeeze(gain_dB)
phase_deg = np.unwrap(np.squeeze(phase_deg) * deg2rad) * rad2deg
Cxy = np.squeeze(Cxy)


#%% Multisine signal with CZT
numChan = 1
numCycles = 1

freqMinDes_rps = freqInit_rps * np.ones(numChan)
freqMaxDes_rps = freqFinal_rps *  np.ones(numChan)
freqStepDes_rps = (10/freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqElem_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)

# Generate Schroeder MultiSine Signal
ampElem_nd = np.linspace(ampInit, ampInit, len(freqElem_rps)) / np.sqrt(len(freqElem_rps))
x, _, sigElem = GenExcite.MultiSine(freqElem_rps, ampElem_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder');


# Simulate the excitation through the system
time_s, y, _ = signal.lsim(sys, x.squeeze(), time_s)
y = np.atleast_2d(y)

# Estimate the transfer function
optSpectCzt = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, freq = freqElem_rps, smooth = ('box', 1), winType=('tukey', 0.0))
freq_czt_rps, Txy_czt, Cxy_czt, Pxx_czt, Pyy_czt, Pxy_czt = FreqTrans.FreqRespFuncEst(x, y, optSpectCzt)
gain_czt_dB, phase_czt_deg = FreqTrans.GainPhase(Txy_czt)
freq_czt_hz = freq_czt_rps * rps2hz

# Plot
freq_czt_hz = np.squeeze(freq_czt_hz)
gain_czt_dB = np.squeeze(gain_czt_dB)
phase_czt_deg = np.squeeze(phase_czt_deg)
Cxy_czt = np.squeeze(Cxy_czt)


#%%
plt.figure(1)
ax1 = plt.subplot(3,1,1); plt.grid(True)
ax1.semilogx(freqLin_hz, gainLin_dB, 'b-') 
ax1.semilogx(freq_hz, gain_dB, '.g--')
ax1.semilogx(freq_czt_hz, gain_czt_dB, '*r')
ax2 = plt.subplot(3,1,2); plt.grid(True)
ax2.semilogx(freqLin_hz, phaseLin_deg, 'b-'); plt.ylim([-180, 180]);
ax2.semilogx(freq_hz, phase_deg, '.g--')
ax2.semilogx(freq_czt_hz, phase_czt_deg, '*r')
ax3 = plt.subplot(3,1,3); plt.grid(True)
ax3.semilogx(freqLin_hz, np.ones_like(freqLin_hz), 'b-', label = 'Linear Model'); plt.ylim([0, 1.2])
ax3.semilogx(freq_hz, Cxy, '.g--', label = 'FFT Estimate')
ax3.semilogx(freq_czt_hz, Cxy_czt, '*r', label = 'CZT Estimate')

ax3.legend()


