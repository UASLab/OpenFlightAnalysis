"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Example script for testing Frequency Response Estimation - MIMO with Noise.
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal

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


#%% Define a linear systems
freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps

freqSys_rps = np.fft.rfftfreq(1500, 1/freqRate_rps)
freqSys_hz = freqSys_rps * rps2hz

wn = 2 * hz2rps
d = 0.2
sys11 = signal.TransferFunction([1.0 * wn**2], [1, 2.0*d*wn, wn**2])
_, gainSys11_dB, phaseSys11_deg = signal.bode(sys11, w=freqSys_rps)
_, TSys11 = signal.freqresp(sys11, w=freqSys_rps)

wn = 4 * hz2rps
d = 0.4
sys21 = signal.TransferFunction([1.0 * wn**2], [1,  2.0*d*wn, wn**2])
_, gainSys21_dB, phaseSys21_deg = signal.bode(sys21, w=freqSys_rps)
_, TSys21 = signal.freqresp(sys21, w=freqSys_rps)

wn = 6 * hz2rps
d = 0.6
sys12 = signal.TransferFunction([1.0 * wn**2], [1,  2.0*d*wn, wn**2])
_, gainSys12_dB, phaseSys12_deg = signal.bode(sys12, w=freqSys_rps)
_, TSys12 = signal.freqresp(sys12, w=freqSys_rps)

wn = 8 * hz2rps
d = 0.8
sys22 = signal.TransferFunction([1.0 * wn**2], [1,  2.0*d*wn, wn**2])
_, gainSys22_dB, phaseSys22_deg = signal.bode(sys22, w=freqSys_rps)
_, TSys22 = signal.freqresp(sys22, w=freqSys_rps)



#%%
numExc = 2
numCycles = 1
ampInit = 1
ampFinal = 1
freqMinDes_rps = 0.1 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 10 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (10/freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

# Generate Schroeder MultiSine Signal
ampExcit_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
exc, _, sigExcit = GenExcite.MultiSine(freqExc_rps, ampExcit_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')

# Simulate the excitation through the system
_, out11, _ = signal.lsim2(sys11, exc[0], time_s)
_, out21, _ = signal.lsim2(sys21, exc[0], time_s)
_, out12, _ = signal.lsim2(sys12, exc[1], time_s)
_, out22, _ = signal.lsim2(sys22, exc[1], time_s)

y = np.zeros_like(exc)
y[0] = out11 + out12
y[1] = out21 + out22

# Generate Noise
sigmaNoise = 0.25
sigmaNoise_dB = 20.0 * np.log10(sigmaNoise)
np.random.seed(seed=0)
inNoise = np.random.normal(0, sigmaNoise, size = y.shape)

wn = 6 * hz2rps
d = 0.1
sysNoise = signal.TransferFunction([1.0 * wn**2], [1, 2.0*d*wn, wn**2])
yNoise = np.zeros_like(inNoise)
for iOut in range(0, len(inNoise)):
#    _, yNoise[iOut], _ = signal.lsim2(sysNoise, inNoise[iOut], time_s)
    yNoise[iOut] += 1.0 * inNoise[iOut]


# Add Noise
y += yNoise


#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps)
optSpecN = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps)

# Excited Frequencies per input channel
optSpec.freq = []
for iChan in range(0, numExc):
    optSpec.freq.append(freqExc_rps[sigIndx[iChan]])
optSpec.freq = np.asarray(optSpec.freq)

# Null Frequencies
optSpecN.freq = freqGap_rps

# FRF Estimate
freq_rps, Txy, Cxy, Pxx, Pyy, Pxy, TxyUnc = FreqTrans.FreqRespFuncEstNoise(exc, y, optSpec, optSpecN)
gain_dB, phase_deg = FreqTrans.GainPhase(Txy)


freq_hz = freq_rps * rps2hz
phase_deg = np.unwrap(phase_deg * deg2rad) * rad2deg

gainUnc_dB, phaseUnc_deg = FreqTrans.GainPhase(TxyUnc)
phaseUnc_deg = np.unwrap(phaseUnc_deg * deg2rad) * rad2deg


#%% Plot
plt.figure(1)

ax1 = plt.subplot(4,2,1); ax1.grid()
ax1.semilogx(freqSys_hz, gainSys11_dB, label='Sys')
ax1.semilogx(freq_hz[0, 0], gain_dB[0, 0], '.', label='Sys Estimate')
ax1.semilogx(freqSys_hz, sigmaNoise_dB * np.ones_like(freqSys_hz), label='Noise')
ax1.semilogx(freq_hz[0, 0], gainUnc_dB[0, 0], '.', label='Noise Estimate')
ax1.legend()
ax3 = plt.subplot(4,2,3, sharex = ax1); ax3.grid()
ax3.semilogx(freqSys_hz, phaseSys11_deg)
ax3.semilogx(freq_hz[0, 0], phase_deg[0, 0], '.')
ax3.set_ylim(-270, 90); ax3.set_yticks([-270,-180,-90,0,90])


ax2 = plt.subplot(4,2,2); ax2.grid()
ax2.semilogx(freqSys_hz, gainSys21_dB)
ax2.semilogx(freq_hz[1, 0], gain_dB[1, 0], '.')
ax2.semilogx(freqSys_hz, sigmaNoise_dB * np.ones_like(freqSys_hz))
ax2.semilogx(freq_hz[1, 0], gainUnc_dB[1, 0], '.')
ax4 = plt.subplot(4,2,4, sharex = ax2); ax4.grid()
ax4.semilogx(freqSys_hz, phaseSys21_deg)
ax4.semilogx(freq_hz[1, 0], phase_deg[1, 0], '.')
ax4.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])

ax5 = plt.subplot(4,2,5); ax5.grid()
ax5.semilogx(freqSys_hz, gainSys12_dB)
ax5.semilogx(freq_hz[0, 0], gain_dB[0, 1], '.')
ax5.semilogx(freqSys_hz, sigmaNoise_dB * np.ones_like(freqSys_hz))
ax5.semilogx(freq_hz[0, 0], gainUnc_dB[0, 1], '.')
ax7 = plt.subplot(4,2,7, sharex = ax5); ax7.grid()
ax7.semilogx(freqSys_hz, phaseSys12_deg)
ax7.semilogx(freq_hz[0, 0], phase_deg[0, 1], '.')
ax7.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])

ax6 = plt.subplot(4,2,6); ax6.grid()
ax6.semilogx(freqSys_hz, gainSys22_dB)
ax6.semilogx(freq_hz[1, 0], gain_dB[1, 1], '.')
ax6.semilogx(freqSys_hz, sigmaNoise_dB * np.ones_like(freqSys_hz))
ax6.semilogx(freq_hz[1, 0], gainUnc_dB[1, 1], '.')
ax8 = plt.subplot(4,2,8, sharex = ax6); ax8.grid()
ax8.semilogx(freqSys_hz, phaseSys22_deg)
ax8.semilogx(freq_hz[1, 0], phase_deg[1, 1], '.')
ax8.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])


    
#%%
plt.figure(2)
ax1 = plt.subplot(2,2,1); ax1.grid()
ax1.plot(TSys11.imag, TSys11.real)

iIn = 0; iOut = 0
ax1.plot(Txy[iOut, iIn].imag, Txy[iOut, iIn].real, '.')
for iNom, nom in enumerate(Txy[iOut, iIn]):
    unc = np.abs(TxyUnc[iOut, iIn][iNom])
    uncCirc = plt.Circle((nom.imag, nom.real), unc, color='b', alpha=0.5)
    ax1.add_artist(uncCirc)


ax2 = plt.subplot(2,2,2, sharex = ax1, sharey = ax1); ax2.grid()
ax2.plot(TSys21.imag, TSys21.real)

iIn = 0; iOut = 1
ax2.plot(Txy[iOut, iIn].imag, Txy[iOut, iIn].real, '.')
for iNom, nom in enumerate(Txy[iOut, iIn]):
    unc = np.abs(TxyUnc[iOut, iIn][iNom])
    uncCirc = plt.Circle((nom.imag, nom.real), unc, color='b', alpha=0.5)
    ax2.add_artist(uncCirc)


ax3 = plt.subplot(2,2,3, sharex = ax1, sharey = ax1); ax3.grid()
ax3.plot(TSys12.imag, TSys12.real)

iIn = 1; iOut = 0
ax3.plot(Txy[iOut, iIn].imag, Txy[iOut, iIn].real, '.')
for iNom, nom in enumerate(Txy[iOut, iIn]):
    unc = np.abs(TxyUnc[iOut, iIn][iNom])
    uncCirc = plt.Circle((nom.imag, nom.real), unc, color='b', alpha=0.5)
    ax3.add_artist(uncCirc)


ax4 = plt.subplot(2,2,4, sharex = ax1, sharey = ax1); ax4.grid()
ax4.plot(TSys22.imag, TSys22.real)

iIn = 1; iOut = 1
ax4.plot(Txy[iOut, iIn].imag, Txy[iOut, iIn].real, '.')
for iNom, nom in enumerate(Txy[iOut, iIn]):
    unc = np.abs(TxyUnc[iOut, iIn][iNom])
    uncCirc = plt.Circle((nom.imag, nom.real), unc, color='b', alpha=0.5)
    ax4.add_artist(uncCirc)

