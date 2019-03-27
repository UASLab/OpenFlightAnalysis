# -*- coding: utf-8 -*-
"""
Created on Wed Mar 20 13:52:13 2019

@author: rega0051
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
dftType = 'fft'
scaleType = 'spectrum'
detrendType = None
winType = ('tukey', 0.0)
smooth = ('box', 5)
freq_rps, gain_dB, phase_deg, Cxy, Txy, Pxx, Pyy, Pxy = FreqTrans.TransFuncEst(x, y, freqRate_rps, dftType = dftType, winType = winType, detrendType = detrendType, smooth = smooth, scaleType = scaleType)
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
dftType = 'czt'

freq_rps, gain_dB, phase_deg, Cxy, Txy, Pxx, Pyy, Pxy = FreqTrans.TransFuncEst(x, y, freqRate_rps, freq = freqElem_rps,  dftType = dftType, winType = winType, detrendType = detrendType, smooth = smooth, scaleType = scaleType)
freq_hz = freq_rps * rps2hz


ax1.semilogx(freq_hz, gain_dB, '.')
ax2.semilogx(freq_hz, phase_deg, '.')
ax3.semilogx(freq_hz, Cxy, '.')


#%% Multi-Channel Multisine signal with CZT
wn = 2 * hz2rps
d = 0.2
sys11 = signal.TransferFunction([1.0 * wn**2], [1, 2.0*d*wn, wn**2])
_, gainSys11_dB, phaseSys11_deg = signal.bode(sys11, w=freqSys_rps)
_, TSys11 = signal.freqresp(sys11, w=freqSys_rps)

wn = 4 * hz2rps
d = 0.4
sys21 = signal.TransferFunction([0.1 * wn**2], [1,  2.0*d*wn, wn**2])
_, gainSys21_dB, phaseSys21_deg = signal.bode(sys21, w=freqSys_rps)

wn = 6 * hz2rps
d = 0.6
sys12 = signal.TransferFunction([0.1 * wn**2], [1,  2.0*d*wn, wn**2])
_, gainSys12_dB, phaseSys12_deg = signal.bode(sys12, w=freqSys_rps)

wn = 8 * hz2rps
d = 0.8
sys22 = signal.TransferFunction([1.0 * wn**2], [1,  2.0*d*wn, wn**2])
_, gainSys22_dB, phaseSys22_deg = signal.bode(sys22, w=freqSys_rps)



##
numChan = 2
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
_, out11, _ = signal.lsim2(sys11, x[0], time_s)
_, out21, _ = signal.lsim2(sys21, x[0], time_s)
_, out12, _ = signal.lsim2(sys12, x[1], time_s)
_, out22, _ = signal.lsim2(sys22, x[1], time_s)

y = np.zeros_like(x)
y[0] = out11 + out12
y[1] = out21 + out22

# Estimate the transfer function
dftType = 'czt'
scaleType = 'spectrum'
detrendType = 'constant'
winType = ('tukey', 0.0)
smooth = ('box', 5)

freqMat_hz = []
gainMat_dB = []
phaseMat_deg = []
TxyMat = []
CxyMat = []
PxxMat = []
PyyMat = []
for iChan in range(0, numChan):
    freqChan_rps = freqElem_rps[sigIndx[iChan]]
    freqChan_rps, gain_dB, phase_deg, Cxy, Txy, Pxx, Pyy, Pxy = FreqTrans.TransFuncEst(x[np.newaxis, iChan], y, freqRate_rps, freqChan_rps,  dftType, winType, detrendType, smooth, scaleType)
    freqMat_hz.append(freqChan_rps * rps2hz)
    gainMat_dB.append(gain_dB)
    phaseMat_deg.append(phase_deg)
    CxyMat.append(Cxy)
    PxxMat.append(Pxx)
    PyyMat.append(Pyy)
    TxyMat.append(Txy)
   


#%% Plot
plt.figure(2)

freqChan_hz = freqElem_rps[sigIndx[0]] * rps2hz

ax1 = plt.subplot(4,2,1)
ax1.semilogx(freqSys_hz, gainSys11_dB)
ax1.semilogx(freqChan_hz, gainMat_dB[0][0], '.')
ax3 = plt.subplot(4,2,3, sharex = ax1)
ax3.semilogx(freqSys_hz, phaseSys11_deg)
ax3.semilogx(freqChan_hz, phaseMat_deg[0][0], '.')

ax2 = plt.subplot(4,2,2)
ax2.semilogx(freqSys_hz, gainSys21_dB)
ax2.semilogx(freqChan_hz, gainMat_dB[0][1], '.')
ax4 = plt.subplot(4,2,4, sharex = ax2)
ax4.semilogx(freqSys_hz, phaseSys21_deg)
ax4.semilogx(freqChan_hz, phaseMat_deg[0][1], '.')

freqChan_hz = freqElem_rps[sigIndx[1]] * rps2hz

ax5 = plt.subplot(4,2,5);
ax5.semilogx(freqSys_hz, gainSys12_dB)
ax5.semilogx(freqChan_hz, gainMat_dB[1][0], '.')
ax7 = plt.subplot(4,2,7, sharex = ax5);
ax7.semilogx(freqSys_hz, phaseSys12_deg)
ax7.semilogx(freqChan_hz, phaseMat_deg[1][0], '.')

ax6 = plt.subplot(4,2,6);
ax6.semilogx(freqSys_hz, gainSys22_dB)
ax6.semilogx(freqChan_hz, gainMat_dB[1][1], '.')
ax8 = plt.subplot(4,2,8, sharex = ax6);
ax8.semilogx(freqSys_hz, phaseSys22_deg)
ax8.semilogx(freqChan_hz, phaseMat_deg[1][1], '.')


    
#
plt.figure(3)
plt.plot(TSys11.imag, TSys11.real)
plt.plot(TxyMat[0][0].imag, TxyMat[0][0].real, '.')
