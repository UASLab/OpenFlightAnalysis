"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Example script for testing Frequency Response Estimation - MIMO.
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


#%% Define a linear plant systems
freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps

freqLin_rps = np.fft.rfftfreq(1500, 1/freqRate_rps)
freqLin_hz = freqLin_rps * rps2hz

plantK11 = 1.0 ; plantWn11 = 1 * hz2rps   ; plantD11 = 0.2;
plantK21 = 0.25; plantWn21 = 2 * plantWn11; plantD21 = 0.1;
plantK12 = 0.5 ; plantWn12 = 3 * plantWn11; plantD12 = 0.3;
plantK22 = 1.0 ; plantWn22 = 4 * plantWn11; plantD22 = 0.4;

sysPlant = control.tf([[[0, 0, plantK11 * plantWn11**2], [0, 0, plantK21 * plantWn21**2]],
                       [[0, 0, plantK12 * plantWn12**2], [0, 0, plantK22 * plantWn22**2]]], 
                      [[[1, 2.0*plantD11*plantWn11, plantWn11**2], [1, 2.0*plantD21*plantWn21, plantWn21**2]], 
                       [[1, 2.0*plantD12*plantWn12, plantWn12**2], [1, 2.0*plantD22*plantWn22, plantWn22**2]]])

# Plant Response
gainLin_mag, phaseLin_rad, _ = control.freqresp(sysPlant, omega = freqLin_rps)
gainLin_dB = mag2db(gainLin_mag)
phaseLin_deg = np.unwrap(phaseLin_rad) * rad2deg

TLin = gainLin_mag * np.exp(1j*phaseLin_rad)

#%%
numExc = 2
numCycles = 3
ampInit = 1
ampFinal = 1
freqMinDes_rps = 0.1 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 10 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (20/freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

# Generate Schroeder MultiSine Signal
ampExcit_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
uExc, _, sigExcit = GenExcite.MultiSine(freqExc_rps, ampExcit_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]

# Simulate the excitation through the system
_, y, _ = control.forced_response(sysPlant, T = time_s, U = uExc)

# Plant-Output Noise
noiseK11 = 0.25 * plantK11; noiseWn11 = 6 * hz2rps; noiseD11 = 0.1;
noiseK21 = 0.25 * plantK11; noiseWn21 = 6 * hz2rps; noiseD21 = 0.1;
noiseK12 = 0.25 * plantK11; noiseWn12 = 4 * hz2rps; noiseD12 = 0.7;
noiseK22 = 0.25 * plantK11; noiseWn22 = 2 * hz2rps; noiseD22 = 0.1;

sysNoise = control.tf([[[-noiseK11, 0, noiseK11 * noiseWn11**2], [-noiseK21, 0, noiseK21 * noiseWn21**2]],
                       [[-noiseK12, 0, noiseK12 * noiseWn12**2], [-noiseK22, 0, noiseK22 * noiseWn22**2]]],
                      [[[1, 2.0*noiseD11*noiseWn11, noiseWn11**2], [1, 2.0*noiseD21*noiseWn21, noiseWn21**2]], 
                       [[1, 2.0*noiseD12*noiseWn12, noiseWn12**2], [1, 2.0*noiseD22*noiseWn22, noiseWn22**2]]])


np.random.seed(0)
m = np.random.normal(0.0, 1.0, size = uExc.shape)
_, n, _ = control.forced_response(sysNoise, T = time_s, U = m)

# Output with Noise
z = y + n

# Linear Noise response
gainNoise_mag, _, _ = control.freqresp(sysNoise, omega = freqLin_rps)
gainNoise_dB = mag2db(gainNoise_mag)

#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear')

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps
optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
#freq_rps, Txy, Cxy, Pxx, Pyy, Pxy = FreqTrans.FreqRespFuncEst(uExc, z, optSpec)
freq_rps, Txy, Cxy, Pxx, Pyy, Pxy, TxyUnc, Pxx_N, Pyy_N = FreqTrans.FreqRespFuncEstNoise(uExc, z, optSpec)
freq_hz = freq_rps * rps2hz

T = Txy
TUnc = TxyUnc

# Nominal Response
gain_dB, phase_deg = FreqTrans.GainPhase(T)
phase_deg = np.unwrap(phase_deg * deg2rad) * rad2deg

# Uncertain Response
gainUnc_dB = FreqTrans.Gain(TUnc)


#%% Plot
plt.figure(1)
plt.tight_layout()

iIn = 0; iOut = 0
ax1 = plt.subplot(4,2,1); ax1.grid()
ax1.semilogx(freqLin_hz, gainLin_dB[iOut, iIn], label='Sys')
ax1.semilogx(freq_hz[iIn], gain_dB[iOut, iIn], '.', label='Sys Estimate')
ax1.semilogx(freqLin_hz, gainNoise_dB[iOut, iIn] - 6, label='Noise') # Power from noise is split between channels
ax1.semilogx(freq_hz[0], gainUnc_dB[iOut,iIn], '.', label='Noise Estimate')
ax1.set_ylabel('Gain (dB)')
ax1.legend()

ax3 = plt.subplot(4,2,3, sharex = ax1); ax3.grid()
ax3.semilogx(freqLin_hz, phaseLin_deg[iOut, iIn])
ax3.semilogx(freq_hz[iIn], phase_deg[iOut, iIn], '.')
#ax3.set_ylim(-270, 90); ax3.set_yticks([-270,-180,-90,0,90])
#ax3.set_xlabel('Frequency (Hz)')
ax3.set_ylabel('Phase (deg)')

iIn = 0; iOut = 1
ax2 = plt.subplot(4,2,2); ax2.grid()
ax2.semilogx(freqLin_hz, gainLin_dB[iOut, iIn])
ax2.semilogx(freq_hz[iIn], gain_dB[iOut, iIn], '.')
ax2.semilogx(freqLin_hz, gainNoise_dB[iOut, iIn] - 6, label='Noise')
ax2.semilogx(freq_hz[0], gainUnc_dB[iOut,iIn], '.')
#ax2.set_ylabel('Gain (dB)')

ax4 = plt.subplot(4,2,4, sharex = ax2); ax4.grid()
ax4.semilogx(freqLin_hz, phaseLin_deg[iOut, iIn])
ax4.semilogx(freq_hz[iIn], phase_deg[iOut, iIn], '.')
#ax4.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])
#ax4.set_xlabel('Frequency (Hz)')
#ax4.set_ylabel('Phase (deg)')

iIn = 1; iOut = 0
ax5 = plt.subplot(4,2,5); ax5.grid()
ax5.semilogx(freqLin_hz, gainLin_dB[iOut, iIn])
ax5.semilogx(freq_hz[iIn], gain_dB[iOut, iIn], '.')
ax5.semilogx(freqLin_hz, gainNoise_dB[iOut, iIn] - 6, label='Noise')
ax5.semilogx(freq_hz[0], gainUnc_dB[iOut,iIn], '.')
ax5.set_ylabel('Gain (dB)')

ax7 = plt.subplot(4,2,7, sharex = ax5); ax7.grid()
ax7.semilogx(freqLin_hz, phaseLin_deg[iOut, iIn])
ax7.semilogx(freq_hz[iIn], phase_deg[iOut, iIn], '.')
#ax7.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])
ax7.set_xlabel('Frequency (Hz)')
ax7.set_ylabel('Phase (deg)')

iIn = 1; iOut = 1
ax6 = plt.subplot(4,2,6); ax6.grid()
ax6.semilogx(freqLin_hz, gainLin_dB[iOut, iIn])
ax6.semilogx(freq_hz[iIn], gain_dB[iOut, iIn], '.')
ax6.semilogx(freqLin_hz, gainNoise_dB[iOut, iIn] - 6, label='Noise')
ax6.semilogx(freq_hz[0], gainUnc_dB[iOut,iIn], '.')
#ax6.set_ylabel('Gain (dB)')

ax8 = plt.subplot(4,2,8, sharex = ax6); ax8.grid()
ax8.semilogx(freqLin_hz, phaseLin_deg[iOut, iIn])
ax8.semilogx(freq_hz[iIn], phase_deg[iOut, iIn], '.')
#ax8.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])
ax8.set_xlabel('Frequency (Hz)')
#ax8.set_ylabel('Phase (deg)')


#%%
import matplotlib.patches as patch
plt.figure(2)
plt.tight_layout()

iIn = 0; iOut = 0
ax1 = plt.subplot(2,2,1); ax1.grid()
ax1.plot(TLin[iOut, iIn].real, TLin[iOut, iIn].imag)
ax1.plot(T[iOut, iIn].real, T[iOut, iIn].imag, '.')
for iNom, nom in enumerate(T[iOut,iIn]):
    unc = TUnc[iOut,iIn][iNom]
    uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color='b', alpha=0.25)
    ax1.add_artist(uncPatch)
ax1.plot(-1, 0, '+r')
ax1.set_xlabel('Real')
ax1.set_ylabel('Imag')

iIn = 0; iOut = 1
ax2 = plt.subplot(2,2,2, sharex = ax1, sharey = ax1); ax2.grid()
ax2.plot(TLin[iOut, iIn].real, TLin[iOut, iIn].imag)
ax2.plot(T[iOut, iIn].real, T[iOut, iIn].imag, '.')
for iNom, nom in enumerate(T[iOut,iIn]):
    unc = TUnc[iOut,iIn][iNom]
    uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color='b', alpha=0.25)
    ax2.add_artist(uncPatch)
ax2.plot(-1, 0, '+r')
ax2.set_xlabel('Real')
ax2.set_ylabel('Imag')

iIn = 1; iOut = 0
ax3 = plt.subplot(2,2,3, sharex = ax1, sharey = ax1); ax3.grid()
ax3.plot(TLin[iOut, iIn].real, TLin[iOut, iIn].imag)
ax3.plot(T[iOut, iIn].real, T[iOut, iIn].imag, '.')
for iNom, nom in enumerate(T[iOut,iIn]):
    unc = TUnc[iOut,iIn][iNom]
    uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color='b', alpha=0.25)
    ax3.add_artist(uncPatch)
ax3.plot(-1, 0, '+r')
ax3.set_xlabel('Real')
ax3.set_ylabel('Imag')

iIn = 1; iOut = 1
ax4 = plt.subplot(2,2,4, sharex = ax1, sharey = ax1); ax4.grid(True)
ax4.plot(TLin[iOut, iIn].real, TLin[iOut, iIn].imag)
ax4.plot(T[iOut, iIn].real, T[iOut, iIn].imag, '.')
for iNom, nom in enumerate(T[iOut,iIn]):
    unc = TUnc[iOut,iIn][iNom]
    uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color='b', alpha=0.25)
    ax4.add_artist(uncPatch)
ax4.plot(-1, 0, '+r')
ax4.set_xlabel('Real')
ax4.set_ylabel('Imag')
ax4.legend(['Sys', 'Sys Estimate'])
