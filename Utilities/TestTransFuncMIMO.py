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


#%% Define a linear systems
freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps

freqSys_rps = np.fft.rfftfreq(500, 1/freqRate_rps)
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



##
numChan = 2
numCycles = 1
ampInit = 1
ampFinal = 1
freqMinDes_rps = 0.1 * hz2rps * np.ones(numChan)
freqMaxDes_rps = 10 * hz2rps *  np.ones(numChan)
freqStepDes_rps = (5/freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExcit_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)

# Generate Schroeder MultiSine Signal
ampElem_nd = np.linspace(ampInit, ampFinal, len(freqExcit_rps)) / np.sqrt(len(freqExcit_rps))
exc, _, sigElem = GenExcite.Schroeder(freqExcit_rps, ampElem_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak');

# Simulate the excitation through the system
_, out11, _ = signal.lsim2(sys11, exc[0], time_s)
_, out21, _ = signal.lsim2(sys21, exc[0], time_s)
_, out12, _ = signal.lsim2(sys12, exc[1], time_s)
_, out22, _ = signal.lsim2(sys22, exc[1], time_s)

y = np.zeros_like(exc)
y[0] = out11 + out12
y[1] = out21 + out22

# Estimate the transfer function
dftType = 'czt'
scaleType = 'spectrum'
detrendType = 'constant'
winType = ('tukey', 0.0)
smooth = ('box', 1)

freqMat_hz = []
gainMat_dB = []
phaseMat_deg = []
TxyMat = []
CxyMat = []
PxxMat = []
PyyMat = []
for iChan in range(0, numChan):
    freqChan_rps = freqExcit_rps[sigIndx[iChan]]
    freqChan_rps, Txy, Cxy, Pxx, Pyy, Pxy = FreqTrans.FreqRespFuncEst(exc[np.newaxis, iChan], y, freqRate_rps, freqChan_rps,  dftType, winType, detrendType, smooth, scaleType)
    gain_dB, phase_deg = FreqTrans.GainPhase(Txy)
    
    freqMat_hz.append(freqChan_rps * rps2hz)
    gainMat_dB.append(gain_dB)
    phaseMat_deg.append(phase_deg)
    CxyMat.append(Cxy)
    PxxMat.append(Pxx)
    PyyMat.append(Pyy)
    TxyMat.append(Txy)
   

#%% Plot
plt.figure(1)

freqChan_hz = freqExcit_rps[sigIndx[0]] * rps2hz

ax1 = plt.subplot(4,2,1); ax1.grid()
ax1.semilogx(freqSys_hz, gainSys11_dB)
ax1.semilogx(freqChan_hz, gainMat_dB[0][0], '.')
ax3 = plt.subplot(4,2,3, sharex = ax1); ax3.grid()
ax3.semilogx(freqSys_hz, phaseSys11_deg)
ax3.semilogx(freqChan_hz, phaseMat_deg[0][0], '.')
ax3.set_ylim(-180, 180); ax3.set_yticks([-180,-90,0,90,180])


ax2 = plt.subplot(4,2,2); ax2.grid()
ax2.semilogx(freqSys_hz, gainSys21_dB)
ax2.semilogx(freqChan_hz, gainMat_dB[0][1], '.')
ax4 = plt.subplot(4,2,4, sharex = ax2); ax4.grid()
ax4.semilogx(freqSys_hz, phaseSys21_deg)
ax4.semilogx(freqChan_hz, phaseMat_deg[0][1], '.')
ax4.set_ylim(-180, 180); ax4.set_yticks([-180,-90,0,90,180])

freqChan_hz = freqExcit_rps[sigIndx[1]] * rps2hz

ax5 = plt.subplot(4,2,5); ax5.grid()
ax5.semilogx(freqSys_hz, gainSys12_dB)
ax5.semilogx(freqChan_hz, gainMat_dB[1][0], '.')
ax7 = plt.subplot(4,2,7, sharex = ax5); ax7.grid()
ax7.semilogx(freqSys_hz, phaseSys12_deg)
ax7.semilogx(freqChan_hz, phaseMat_deg[1][0], '.')
ax7.set_ylim(-180, 180); ax4.set_yticks([-180,-90,0,90,180])

ax6 = plt.subplot(4,2,6); ax6.grid()
ax6.semilogx(freqSys_hz, gainSys22_dB)
ax6.semilogx(freqChan_hz, gainMat_dB[1][1], '.')
ax8 = plt.subplot(4,2,8, sharex = ax6); ax8.grid()
ax8.semilogx(freqSys_hz, phaseSys22_deg)
ax8.semilogx(freqChan_hz, phaseMat_deg[1][1], '.')
ax8.set_ylim(-180, 180); ax4.set_yticks([-180,-90,0,90,180])


#%%
plt.figure(2)
ax1 = plt.subplot(2,2,1); ax1.grid()
ax1.plot(TSys11.imag, TSys11.real)
ax1.plot(TxyMat[0][0].imag, TxyMat[0][0].real, '.')

ax2 = plt.subplot(2,2,2, sharex = ax1, sharey = ax1); ax2.grid()
ax2.plot(TSys21.imag, TSys21.real)
ax2.plot(TxyMat[0][1].imag, TxyMat[0][1].real, '.')

ax3 = plt.subplot(2,2,3, sharex = ax1, sharey = ax1); ax3.grid()
ax3.plot(TSys12.imag, TSys12.real)
ax3.plot(TxyMat[1][0].imag, TxyMat[1][0].real, '.')

ax4 = plt.subplot(2,2,4, sharex = ax1, sharey = ax1); ax4.grid()
ax4.plot(TSys22.imag, TSys22.real)
ax4.plot(TxyMat[1][1].imag, TxyMat[1][1].real, '.')