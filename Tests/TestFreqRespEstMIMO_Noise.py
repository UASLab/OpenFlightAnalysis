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

sys = [[0] * 2 for i in range(2)]

K = 1.0
wn = 2 * hz2rps
d = 0.1
sys[0][0] = signal.TransferFunction([K * wn**2], [1, 2.0*d*wn, wn**2])

K = 1.0
wn = 4 * hz2rps
d = 0.4
sys[1][0] = signal.TransferFunction([K * wn**2], [1,  2.0*d*wn, wn**2])

K = 0.25
wn = 6 * hz2rps
d = 0.6
sys[0][1] = signal.TransferFunction([K * wn**2], [1,  2.0*d*wn, wn**2])

K = 1.0
wn = 8 * hz2rps
d = 0.8
sys[1][1] = signal.TransferFunction([K * wn**2], [1,  2.0*d*wn, wn**2])


gainSys_dB = [[0] * 2 for i in range(2)]
phaseSys_deg = [[0] * 2 for i in range(2)]
TSys = [[0] * 2 for i in range(2)]
for iOut in range(2):
    for iIn in range(2):
        _, gainSys_dB[iIn][iOut], phaseSys_deg[iIn][iOut] = signal.bode(sys[iIn][iOut], w=freqSys_rps)
        _, TSys[iIn][iOut] = signal.freqresp(sys[iIn][iOut], w=freqSys_rps)


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


# Generate input Noise
sigmaD = 0.0
np.random.seed(seed=0)

wn = 10 * hz2rps
d = 1.0
sysD = signal.TransferFunction([sigmaD * wn**2], [1, 2.0*d*wn, wn**2])

xD = np.zeros_like(exc)
for iExc in range(2):
    inD = np.random.normal(0, ampInit, size = exc[iExc].shape)
    _, xD[iExc], _ = signal.lsim2(sysD, inD, time_s)


# Simulate the excitation through the system
out = [[0] * 2 for i in range(2)]
_, out[0][0], _ = signal.lsim2(sys[0][0], exc[0]+xD[0], time_s)
_, out[0][1], _ = signal.lsim2(sys[0][1], exc[0]+xD[0], time_s)
_, out[1][0], _ = signal.lsim2(sys[1][0], exc[1]+xD[1], time_s)
_, out[1][1], _ = signal.lsim2(sys[1][1], exc[1]+xD[1], time_s)


# Generate Noise
sigmaN = 0.125

wn = 3 * hz2rps
d = 0.2
sysN = signal.TransferFunction([sigmaN * wn**2], [1, 2.0*d*wn, wn**2])

yN = np.zeros_like(exc)
if True:
    for iOut in range(2):
        inNoise = np.random.normal(0, ampInit, size = exc[iOut].shape)
        _, yN[iOut], _ = signal.lsim2(sysN, inNoise, time_s)


# Combine output signals
y = [0] * 2
iOut = 0
y[iOut] = out[0][iOut] + out[1][iOut] + yN[iOut]
iOut = 1
y[iOut] = out[0][iOut] + out[1][iOut] + yN[iOut]



#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear')
optSpecN = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear')

# Excited Frequencies per input channel
optSpec.freq = []
for iChan in range(0, numExc):
    optSpec.freq.append(freqExc_rps[sigIndx[iChan]])
optSpec.freq = np.asarray(optSpec.freq)
#optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpecN.freq = freqGap_rps
#optSpecN.freqInterp = freqExc_rps

# FRF Estimate
freq_rps, Txy, Cxy, Pxx, Pyy, Pxy, TxyUnc, Pyy_N = FreqTrans.FreqRespFuncEstNoise(exc, y, optSpec, optSpecN)
freqE_rps, _, Pee  = FreqTrans.Spectrum(exc, optSpecN)
freqN_rps, _, Pnn  = FreqTrans.Spectrum(y, optSpecN)

gain_dB, phase_deg = FreqTrans.GainPhase(Txy)


freq_hz = freq_rps * rps2hz
phase_deg = np.unwrap(phase_deg * deg2rad) * rad2deg

TxyUnc = np.abs(TxyUnc) # Uncertainty is Magnitude only, uncorrelated

gainUnc_dB, phaseUnc_deg = FreqTrans.GainPhase(TxyUnc)
phaseUnc_deg = np.unwrap(phaseUnc_deg * deg2rad) * rad2deg


# Ideal Noise Mode
freqNoise_rps, gainNoise_dB, phaseNoise_deg = signal.bode(sysN, w = freqExc_rps)
freqNoise_hz = freqNoise_rps * rps2hz

#% Interpolation Scheme

freqE_hz = freqE_rps * rps2hz
freqN_hz = freqN_rps * rps2hz

plt.figure(0)
iIn = 0; iOut = 0
plt.semilogx(freq_hz[iIn,0], 20*np.log10(Pxx[iIn,0]), '*b', label='P Excitation')
plt.semilogx(freqE_hz[0], 20*np.log10(Pee[iIn]), '*m', label='P Excitation @ Null')
plt.semilogx(freq_hz[iIn,0], 20*np.log10(Pyy[iIn,iOut]), '*r-', label='P Output')
plt.semilogx(freqN_hz[0], 20*np.log10(Pnn[iOut]), '.g--', label='P Disturbance')
#plt.semilogx(freq_hz[iIn,0], 20*np.log10(Pyy_N[iIn,iOut]), '*k', label='P Disturbance @ Null')
plt.grid(True)
plt.legend()


#%% Plot
plt.figure(1)
plt.tight_layout()

iIn = 0; iOut = 0
ax1 = plt.subplot(4,2,1); ax1.grid()
ax1.semilogx(freqSys_hz, gainSys_dB[iIn][iOut], label='Sys')
ax1.semilogx(freq_hz[iIn,0], gain_dB[iIn,iOut], '.', label='Sys Estimate')
ax1.semilogx(freqNoise_hz, gainNoise_dB, label='Noise')
ax1.semilogx(freq_hz[iIn,0], gainUnc_dB[iIn,iOut], '.', label='Noise Estimate')
ax1.set_ylabel('Gain (dB)')
ax1.set_title('K = 1, wn = 2 hz, d = 0.1')
ax1.legend()
ax3 = plt.subplot(4,2,3, sharex = ax1); ax3.grid()
ax3.semilogx(freqSys_hz, phaseSys_deg[iIn][iOut])
ax3.semilogx(freq_hz[iIn,0], phase_deg[iIn,iOut], '.')
ax3.set_ylim(-270, 90); ax3.set_yticks([-270,-180,-90,0,90])
#ax3.set_xlabel('Frequency (Hz)')
ax3.set_ylabel('Phase (deg)')

iIn = 0; iOut = 1
ax2 = plt.subplot(4,2,2); ax2.grid()
ax2.semilogx(freqSys_hz, gainSys_dB[iIn][iOut])
ax2.semilogx(freq_hz[iIn,0], gain_dB[iIn,iOut], '.')
ax2.semilogx(freqNoise_hz, gainNoise_dB)
ax2.semilogx(freq_hz[iIn,0], gainUnc_dB[iIn,iOut], '.')
#ax2.set_ylabel('Gain (dB)')
ax2.set_title('K = 0.25, wn = 6 hz, d = 0.6')
ax4 = plt.subplot(4,2,4, sharex = ax2); ax4.grid()
ax4.semilogx(freqSys_hz, phaseSys_deg[iIn][iOut])
ax4.semilogx(freq_hz[iIn,0], phase_deg[iIn,iOut], '.')
#ax4.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])
#ax4.set_xlabel('Frequency (Hz)')
#ax4.set_ylabel('Phase (deg)')

iIn = 1; iOut = 0
ax5 = plt.subplot(4,2,5); ax5.grid()
ax5.semilogx(freqSys_hz, gainSys_dB[iIn][iOut])
ax5.semilogx(freq_hz[iIn,0], gain_dB[iIn,iOut], '.')
ax5.semilogx(freqNoise_hz, gainNoise_dB)
ax5.semilogx(freq_hz[iIn,0], gainUnc_dB[iIn,iOut], '.')
ax5.set_ylabel('Gain (dB)')
ax5.set_title('K = 1, wn = 4 hz, d = 0.4')
ax7 = plt.subplot(4,2,7, sharex = ax5); ax7.grid()
ax7.semilogx(freqSys_hz, phaseSys_deg[iIn][iOut])
ax7.semilogx(freq_hz[iIn,0], phase_deg[iIn,iOut], '.')
ax7.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])
ax7.set_xlabel('Frequency (Hz)')
ax7.set_ylabel('Phase (deg)')

iIn = 1; iOut = 1
ax6 = plt.subplot(4,2,6); ax6.grid()
ax6.semilogx(freqSys_hz, gainSys_dB[iIn][iOut])
ax6.semilogx(freq_hz[iIn,0], gain_dB[iIn,iOut], '.')
ax6.semilogx(freqNoise_hz, gainNoise_dB)
ax6.semilogx(freq_hz[iIn,0], gainUnc_dB[iIn,iOut], '.')
#ax6.set_ylabel('Gain (dB)')
ax6.set_title('K = 1, wn = 8 hz, d = 0.8')
ax8 = plt.subplot(4,2,8, sharex = ax6); ax8.grid()
ax8.semilogx(freqSys_hz, phaseSys_deg[iIn][iOut])
ax8.semilogx(freq_hz[iIn,0], phase_deg[iIn,iOut], '.')
ax8.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])
ax8.set_xlabel('Frequency (Hz)')
#ax8.set_ylabel('Phase (deg)')


    
#%%
import matplotlib.patches as patch
plt.figure(2)
plt.tight_layout()

iIn = 0; iOut = 0
ax1 = plt.subplot(2,2,1); ax1.grid()
ax1.plot(TSys[0][0].real, TSys[0][0].imag)
ax1.set_title('K = 1, wn = 2 hz, d = 0.1')

ax1.plot(Txy[iIn,iOut].real, Txy[iIn,iOut].imag, '.')
for iNom, nom in enumerate(Txy[iIn,iOut]):
    unc =TxyUnc[iIn,iOut][iNom]
    uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color='b', alpha=0.25)
    ax1.add_artist(uncPatch)

ax1.set_xlabel('Real')
ax1.set_ylabel('Imag')

iIn = 0; iOut = 1
ax2 = plt.subplot(2,2,2, sharex = ax1, sharey = ax1); ax2.grid()
ax2.plot(TSys[iIn][iOut].real, TSys[iIn][iOut].imag)
ax2.set_title('K = 0.25, wn = 6 hz, d = 0.6')

ax2.plot(Txy[iIn,iOut].real, Txy[iIn,iOut].imag, '.')
for iNom, nom in enumerate(Txy[iIn,iOut]):
    unc =TxyUnc[iIn,iOut][iNom]
    uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color='b', alpha=0.25)
    ax2.add_artist(uncPatch)

ax2.set_xlabel('Real')
ax2.set_ylabel('Imag')

iIn = 1; iOut = 0
ax3 = plt.subplot(2,2,3, sharex = ax1, sharey = ax1); ax3.grid()
ax3.plot(TSys[iIn][iOut].real, TSys[iIn][iOut].imag)
ax3.set_title('K = 1, wn = 4 hz, d = 0.4')

ax3.plot(Txy[iIn,iOut].real, Txy[iIn,iOut].imag, '.')
for iNom, nom in enumerate(Txy[iIn,iOut]):
    unc =TxyUnc[iIn,iOut][iNom]
    uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color='b', alpha=0.25)
    ax3.add_artist(uncPatch)

ax3.set_xlabel('Real')
ax3.set_ylabel('Imag')

iIn = 1; iOut = 1
ax4 = plt.subplot(2,2,4, sharex = ax1, sharey = ax1); ax4.grid(True)
ax4.plot(TSys[iIn][iOut].real, TSys[iIn][iOut].imag)
ax4.set_title('K = 1, wn = 8 hz, d = 0.8')

ax4.plot(Txy[iIn,iOut].real, Txy[iIn,iOut].imag, '.')
for iNom, nom in enumerate(Txy[iIn,iOut]):
    unc =TxyUnc[iIn,iOut][iNom]
    uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color='b', alpha=0.25)
    ax4.add_artist(uncPatch)

ax4.set_xlabel('Real')
ax4.set_ylabel('Imag')
ax4.legend(['Sys', 'Sys Estimate', 'Sys Estimate Uncertainty'])
