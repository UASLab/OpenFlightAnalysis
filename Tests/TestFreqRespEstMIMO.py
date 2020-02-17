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


iIn = 0; iOut = 0;
K = 1.0; wn = 2 * hz2rps; d = 0.1;
sys[iOut][iIn] = signal.TransferFunction([K * wn**2], [1, 2.0*d*wn, wn**2])

iIn = 0; iOut = 1;
K = 1.0; wn = 4 * hz2rps; d = 0.4;
sys[iOut][iIn] = signal.TransferFunction([K * wn**2], [1,  2.0*d*wn, wn**2])

iIn = 1; iOut = 0;
K = 0.25; wn = 6 * hz2rps; d = 0.6;
sys[iOut][iIn] = signal.TransferFunction([K * wn**2], [1,  2.0*d*wn, wn**2])

iIn = 1; iOut = 1;
K = 1.0; wn = 8 * hz2rps; d = 0.8;
sys[iOut][iIn] = signal.TransferFunction([K * wn**2], [1,  2.0*d*wn, wn**2])


gainSys_dB = [[0] * 2 for i in range(2)]
phaseSys_deg = [[0] * 2 for i in range(2)]
TSys = [[0] * 2 for i in range(2)]
for iOut in range(2):
    for iIn in range(2):
        _, gainSys_dB[iOut][iIn], phaseSys_deg[iOut][iIn] = signal.bode(sys[iOut][iIn], w=freqSys_rps)
        _, TSys[iOut][iIn] = signal.freqresp(sys[iOut][iIn], w=freqSys_rps)


#%%
numExc = 2
numCycles = 3
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
out = [[0] * 2 for i in range(2)]
_, out[0][0], _ = signal.lsim2(sys[0][0], exc[0], time_s)
_, out[1][0], _ = signal.lsim2(sys[1][0], exc[0], time_s)
_, out[0][1], _ = signal.lsim2(sys[0][1], exc[1], time_s)
_, out[1][1], _ = signal.lsim2(sys[1][1], exc[1], time_s)

# Combine output signals
y = [0] * 2
iOut = 0
y[iOut] = out[iOut][0] + out[iOut][1]
iOut = 1
y[iOut] = out[iOut][0] + out[iOut][1]


#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps)

# Excited Frequencies per input channel
optSpec.freq = []
for iChan in range(0, numExc):
    optSpec.freq.append(freqExc_rps[sigIndx[iChan]])
optSpec.freq = np.asarray(optSpec.freq)
optSpec.freqInterp = freqExc_rps

# FRF Estimate
freq_rps, Txy, Cxy, Pxx, Pyy, Pxy = FreqTrans.FreqRespFuncEst(exc, y, optSpec)
gain_dB, phase_deg = FreqTrans.GainPhase(Txy)


freq_hz = freq_rps * rps2hz
phase_deg = np.unwrap(phase_deg * deg2rad) * rad2deg

#%% Plot
plt.figure(1)
plt.tight_layout()

iIn = 0; iOut = 0
ax1 = plt.subplot(4,2,1); ax1.grid()
ax1.semilogx(freqSys_hz, gainSys_dB[iOut][iIn], label='Sys')
ax1.semilogx(freq_hz[0], gain_dB[iOut, iIn], '.', label='Sys Estimate')
ax1.set_ylabel('Gain (dB)')
ax1.set_title('K = 1, wn = 2 hz, d = 0.1')
ax1.legend()
ax3 = plt.subplot(4,2,3, sharex = ax1); ax3.grid()
ax3.semilogx(freqSys_hz, phaseSys_deg[iOut][iIn])
ax3.semilogx(freq_hz[0], phase_deg[iOut, iIn], '.')
ax3.set_ylim(-270, 90); ax3.set_yticks([-270,-180,-90,0,90])
#ax3.set_xlabel('Frequency (Hz)')
ax3.set_ylabel('Phase (deg)')

iIn = 0; iOut = 1
ax2 = plt.subplot(4,2,2); ax2.grid()
ax2.semilogx(freqSys_hz, gainSys_dB[iOut][iIn])
ax2.semilogx(freq_hz[0], gain_dB[iOut, iIn], '.')
#ax2.set_ylabel('Gain (dB)')
ax2.set_title('K = 1, wn = 4 hz, d = 0.4')
ax4 = plt.subplot(4,2,4, sharex = ax2); ax4.grid()
ax4.semilogx(freqSys_hz, phaseSys_deg[iOut][iIn])
ax4.semilogx(freq_hz[0], phase_deg[iOut, iIn], '.')
#ax4.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])
#ax4.set_xlabel('Frequency (Hz)')
#ax4.set_ylabel('Phase (deg)')

iIn = 1; iOut = 0
ax5 = plt.subplot(4,2,5); ax5.grid()
ax5.semilogx(freqSys_hz, gainSys_dB[iOut][iIn])
ax5.semilogx(freq_hz[0], gain_dB[iOut, iIn], '.')
ax5.set_ylabel('Gain (dB)')
ax5.set_title('K = 0.25, wn = 6 hz, d = 0.6')
ax7 = plt.subplot(4,2,7, sharex = ax5); ax7.grid()
ax7.semilogx(freqSys_hz, phaseSys_deg[iOut][iIn])
ax7.semilogx(freq_hz[0], phase_deg[iOut, iIn], '.')
ax7.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])
ax7.set_xlabel('Frequency (Hz)')
ax7.set_ylabel('Phase (deg)')

iIn = 1; iOut = 1
ax6 = plt.subplot(4,2,6); ax6.grid()
ax6.semilogx(freqSys_hz, gainSys_dB[iOut][iIn])
ax6.semilogx(freq_hz[0], gain_dB[iOut, iIn], '.')
#ax6.set_ylabel('Gain (dB)')
ax6.set_title('K = 1, wn = 8 hz, d = 0.8')
ax8 = plt.subplot(4,2,8, sharex = ax6); ax8.grid()
ax8.semilogx(freqSys_hz, phaseSys_deg[iOut][iIn])
ax8.semilogx(freq_hz[0], phase_deg[iOut, iIn], '.')
ax8.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])
ax8.set_xlabel('Frequency (Hz)')
#ax8.set_ylabel('Phase (deg)')


#%%
plt.figure(2)

iIn = 0; iOut = 0
ax1 = plt.subplot(2,2,1); ax1.grid()
ax1.plot(TSys[iOut][iIn].real, TSys[iOut][iIn].imag)
ax1.set_title('K = 1, wn = 2 hz, d = 0.1')
ax1.plot(Txy[iOut, iIn].real, Txy[iOut, iIn].imag, '.')
ax1.set_xlabel('Real')
ax1.set_ylabel('Imag')

iIn = 0; iOut = 1
ax2 = plt.subplot(2,2,2, sharex = ax1, sharey = ax1); ax2.grid()
ax2.plot(TSys[iOut][iIn].real, TSys[iOut][iIn].imag)
ax3.set_title('K = 1, wn = 4 hz, d = 0.4')
ax2.plot(Txy[iOut, iIn].real, Txy[iOut, iIn].imag, '.')
ax2.set_xlabel('Real')
ax2.set_ylabel('Imag')

iIn = 1; iOut = 0
ax3 = plt.subplot(2,2,3, sharex = ax1, sharey = ax1); ax3.grid()
ax3.plot(TSys[iOut][iIn].real, TSys[iOut][iIn].imag)
ax2.set_title('K = 0.25, wn = 6 hz, d = 0.6')
ax3.plot(Txy[iOut, iIn].real, Txy[iOut, iIn].imag, '.')
ax3.set_xlabel('Real')
ax3.set_ylabel('Imag')

iIn = 1; iOut = 1
ax4 = plt.subplot(2,2,4, sharex = ax1, sharey = ax1); ax4.grid(True)
ax4.plot(TSys[iOut][iIn].real, TSys[iOut][iIn].imag)
ax4.set_title('K = 1, wn = 8 hz, d = 0.8')
ax4.plot(Txy[iOut, iIn].real, Txy[iOut, iIn].imag, '.')
ax4.set_xlabel('Real')
ax4.set_ylabel('Imag')
ax4.legend(['Sys', 'Sys Estimate'])