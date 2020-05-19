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
#import scipy.signal as signal
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
from Core.Systems import ConnectName


# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps

rad2deg = 180/np.pi
deg2rad = 1/rad2deg

def weighting(wb, m, a):
    """weighting(wb,m,a) -> wf
    wb - design frequency (where |wf| is approximately 1)
    m - high frequency gain of 1/wf; should be > 1
    a - low frequency gain of 1/wf; should be < 1
    wf - SISO LTI object
    """
    s = control.tf([1, 0], [1])
    return (s/m + wb) / (s + wb*a)

#%% Define a linear plant systems
freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps

freqLin_rps = np.fft.rfftfreq(1500, 1/freqRate_rps)
freqLin_rps = np.linspace(1e-2, 1e2, 400)
freqLin_hz = freqLin_rps * rps2hz

if False:
    K11 = 1.0       ; wn11 = 2 * hz2rps; d11 = 0.1;
    K21 = 0.25 * K11; wn21 = 2 * wn11  ; d21 = 0.2;
    K12 = 0.5  * K11; wn12 = 3 * wn11  ; d12 = 0.3;
    K22 = 1.0  * K11; wn22 = 4 * wn11  ; d22 = 0.4;
    
    sysPlant = control.tf([[[K11 * wn11**2], [K21 * wn21**2]],
                       [[K12 * wn12**2], [K22 * wn22**2]]], 
                        [[[1, 2.0*d11*wn11, wn11**2], [1, 2.0*d21*wn21, wn21**2]], 
                         [[1, 2.0*d12*wn12, wn12**2], [1, 2.0*d22*wn22, wn22**2]]])
elif False: # SP #3.8
    wn = np.sqrt(5)
    d = 6 / (2 * wn)
    den = [1, 2.0*d*wn, wn**2]
    sysPlant = control.tf([[[wn**2], [wn**2]],
                  [[2 * wn**2, wn**2], [2 * wn**2]]],
                 [[den, den],
                  [den, den]])
    
    # Based on Example 3.8 from Multivariable Feedback Control, Skogestad and Postlethwaite, 2nd Edition.
    wu = control.ss([], [], [], np.eye(2))
    wp1 = control.ss(weighting(wb = 0.25, m = 1.5, a = 1e-4))
    wp2 = control.ss(weighting(wb = 0.25, m = 1.5, a = 1e-4))
    wp = wp1.append(wp2)
    
    sysK, sysCL, (gam, rcond) = control.mixsyn(sysPlant, wp, wu)

else: # SP Section #3.7.1
    a = 10
    den = [1, 0.0, a**2]
    sysPlant = control.tf([[[1, -a**2], [a, a]],
                  [[-a, -a], [1, -a**2]]],
                 [[den, den],
                  [den, den]])
    
    sysK = control.ss([], [], [], np.eye(2))

sysPlant.InputNames = ['u1', 'u2']
sysPlant.OutputNames = ['y1', 'y2']

if False:
    for iIn in range(sysPlant.inputs):
        tStep, yStep = control.step_response(sysPlant, T = np.arange(0, 1, 0.01), input = iIn)
        
        for iOut in range(sysPlant.outputs):
            print(iOut, iIn)
            plt.figure()
            plt.title(['Out: ', sysPlant.OutputNames[iOut], ' In: ', sysPlant.InputNames[iIn]])
            plt.plot(tStep, yStep[iOut])


#%% Define a linear control system
sysK.InputNames = ['e1', 'e2']
sysK.OutputNames = ['uCtrl1', 'uCtrl2']

# Reference Input: e = r - z
sysR = control.ss([], [], [], [[1, 0, -1, 0], [0, 1, 0, -1]])
sysR.InputNames = ['r1', 'r2', 'z1', 'z2']
sysR.OutputNames = ['e1', 'e2']

connectNames = sysK.InputNames
inKeep = sysR.InputNames
outKeep = sysK.OutputNames
sysCtrl = ConnectName([sysR, sysK], connectNames, inKeep, outKeep)

#%%
numExc = 2
numCycles = 3
ampInit = 1
ampFinal = 1
freqMinDes_rps = 0.01 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 5 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (10/freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

# Generate Schroeder MultiSine Signal
ampExcit_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
uExc, _, sigEx = GenExcite.MultiSine(freqExc_rps, ampExcit_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')


#%%
sysD = control.ss([], [], [], [[1, 0, 1, 0], [0, 1, 0, 1]])
sysD.InputNames = ['uCtrl1', 'uCtrl2', 'd1', 'd2']
sysD.OutputNames = ['u1', 'u2']

sysN = control.ss([], [], [], [[1, 0, 1, 0], [0, 1, 0, 1]])
sysN.InputNames = ['y1', 'y2', 'n1', 'n2']
sysN.OutputNames = ['z1', 'z2']

# Add the disturbances to the Plant model
# z = n + P * (u + d)
connectNames = sysPlant.InputNames + sysPlant.OutputNames
inKeep = sysD.InputNames + sysN.InputNames[2:]
outKeep = sysN.OutputNames
sysPlantDist = ConnectName([sysD, sysPlant, sysN], connectNames, inKeep, outKeep)


#%%
# 
inNames = sysCtrl.InputNames + sysPlantDist.InputNames
outNames = sysCtrl.OutputNames + sysPlantDist.OutputNames
connectNames = sysCtrl.OutputNames + sysPlantDist.OutputNames
inKeep = sysCtrl.InputNames[:2] + sysPlantDist.InputNames[-4:]
outKeep = connectNames
sysCL = ConnectName([sysCtrl, sysPlantDist], connectNames, inKeep, outKeep)

# Inputs
r = np.zeros_like(uExc)
d = 0.0 * np.random.randn(uExc.shape[0], uExc.shape[1])
n = 0.0 * np.random.randn(uExc.shape[0], uExc.shape[1])

d += uExc # disturbance + excitation

u = np.concatenate((r, d, n))


#%%
# CL Response
shapeCL = (sysCL.outputs, sysCL.inputs, len(time_s))

gainLin_mag, phaseLin_rad, _ = control.freqresp(sysCL[0:2, 2:4], omega = freqLin_rps)

TxyLin = gainLin_mag * np.exp(1j*phaseLin_rad)
gainLin_dB = 20 * np.log10(gainLin_mag)
phaseLin_deg = np.unwrap(phaseLin_rad) * rad2deg
rCritLin_mag = np.abs(TxyLin - (-1 + 0j))

sigmaLin_mag, _ = FreqTrans.Sigma(TxyLin, typeSigma = 0)
sigmaLinMax_mag = np.max(sigmaLin_mag, axis=0)
muLin_mag = 1 / sigmaLin_mag

#%% Step
if False:
    for iOut in [2, 3]:
        for iIn in [0, 1]:
            tStep, yStep = control.step_response(sysCL, T = np.arange(0, 10, 0.01), input = iIn, output = iOut)
            plt.figure()
            plt.title(['Out: ', sysCL.OutputNames[iOut], ' In: ', sysCL.InputNames[iIn]])
            plt.plot(tStep, yStep)


#%% Simulate the excitation through the system
_, y, _ = control.forced_response(sysCL, T = time_s, U = u)


#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps)

# Excited Frequencies per input channel
optSpec.freq = []
for iChan in range(0, numExc):
    optSpec.freq.append(freqExc_rps[sigIndx[iChan]])
optSpec.freq = np.asarray(optSpec.freq)
optSpec.freqInterp = freqExc_rps

# FRF Estimate
freq_rps, Txy, Cxy, Pxx, Pyy, Pxy = FreqTrans.FreqRespFuncEst(u[2:4, :], y[0:2, :], optSpec)
gain_dB, phase_deg = FreqTrans.GainPhase(Txy)

freq_hz = freq_rps * rps2hz
phase_deg = np.unwrap(phase_deg * deg2rad) * rad2deg

sigmaNom_mag, _ = FreqTrans.Sigma(Txy, typeSigma = 0) # Singular Value Decomp


#%% Sigma Plot
Cmin = np.min(np.min(Cxy, axis = 0), axis = 0)
sigmaNomMax_mag = np.max(sigmaNom_mag, axis=0)
muNom_mag = 1 / sigmaNom_mag

fig = 20
fig = FreqTrans.PlotSigma(freqLin_hz.transpose(), muLin_mag.transpose(), coher_nd = np.ones_like(freqLin_hz), fig = fig, fmt = 'k', label='Linear')
fig = FreqTrans.PlotSigma(freq_hz.transpose(), muNom_mag.transpose(), coher_nd = Cmin, fmt = 'bo', fig = fig, label = 'Excitation Nominal')
fig = FreqTrans.PlotSigma(freqLin_hz, (0.4) * np.ones_like(freqLin_hz), fmt = '--r', fig = fig, label = 'Critical Limit')
ax = fig.get_axes()
#ax[0].set_xscale('log')
ax[0].set_xlim(0, 5)
ax[0].set_ylabel('1/Sigma (nd)')
ax[0].set_yscale('log')
ax[0].set_ylim(bottom = 1e-1)
#ax[0].set_ylim(0, 5)
#ax[0].set_xscale('log')
#ax[0].set_yscale('log')

# Figure 8.12
#plt.figure('8.12')
#plt.loglog(freqLin_rps.transpose(), sigmaLin_mag.transpose())
#plt.xlim(1e-2, 1e2)
#plt.ylim(1e-1, 2e1)
#plt.grid(axis = 'y')


#%% Plot
plt.figure(1)
plt.tight_layout()

iIn = 0; iOut = 0
ax1 = plt.subplot(4,2,1); ax1.grid()
plt.semilogx(freqLin_hz, gainLin_dB[iOut, iIn], label='Sys')
plt.semilogx(freq_hz[0], gain_dB[iOut, iIn], '.', label='Sys Estimate')
ax1.set_ylabel('Gain (dB)')
ax1.legend()

ax3 = plt.subplot(4,2,3, sharex = ax1); ax3.grid()
ax3.semilogx(freqLin_hz, phaseLin_deg[iOut, iIn])
ax3.semilogx(freq_hz[0], phase_deg[iOut, iIn], '.')
#ax3.set_ylim(-270, 90); ax3.set_yticks([-270,-180,-90,0,90])
#ax3.set_xlabel('Frequency (Hz)')
ax3.set_ylabel('Phase (deg)')

iIn = 0; iOut = 1
ax2 = plt.subplot(4,2,2); ax2.grid()
ax2.semilogx(freqLin_hz, gainLin_dB[iOut, iIn])
ax2.semilogx(freq_hz[0], gain_dB[iOut, iIn], '.')
#ax2.set_ylabel('Gain (dB)')

ax4 = plt.subplot(4,2,4, sharex = ax2); ax4.grid()
ax4.semilogx(freqLin_hz, phaseLin_deg[iOut, iIn])
ax4.semilogx(freq_hz[0], phase_deg[iOut, iIn], '.')
#ax4.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])
#ax4.set_xlabel('Frequency (Hz)')
#ax4.set_ylabel('Phase (deg)')

iIn = 1; iOut = 0
ax5 = plt.subplot(4,2,5); ax5.grid()
ax5.semilogx(freqLin_hz, gainLin_dB[iOut, iIn])
ax5.semilogx(freq_hz[0], gain_dB[iOut, iIn], '.')
ax5.set_ylabel('Gain (dB)')

ax7 = plt.subplot(4,2,7, sharex = ax5); ax7.grid()
ax7.semilogx(freqLin_hz, phaseLin_deg[iOut, iIn])
ax7.semilogx(freq_hz[0], phase_deg[iOut, iIn], '.')
#ax7.set_ylim(-270, 90); ax7.set_yticks([-270,-180,-90,0,90])
ax7.set_xlabel('Frequency (Hz)')
ax7.set_ylabel('Phase (deg)')

iIn = 1; iOut = 1
ax6 = plt.subplot(4,2,6); ax6.grid()
ax6.semilogx(freqLin_hz, gainLin_dB[iOut, iIn])
ax6.semilogx(freq_hz[0], gain_dB[iOut, iIn], '.')
#ax6.set_ylabel('Gain (dB)')

ax8 = plt.subplot(4,2,8, sharex = ax6); ax8.grid()
ax8.semilogx(freqLin_hz, phaseLin_deg[iOut, iIn])
ax8.semilogx(freq_hz[0], phase_deg[iOut, iIn], '.')
#ax8.set_ylim(-270, 90); ax8.set_yticks([-270,-180,-90,0,90])
ax8.set_xlabel('Frequency (Hz)')
#ax8.set_ylabel('Phase (deg)')


#%%
plt.figure(2)

iIn = 0; iOut = 0
ax1 = plt.subplot(2,2,1); ax1.grid()
ax1.plot(TxyLin[iOut, iIn].real, TxyLin[iOut, iIn].imag)
ax1.plot(Txy[iOut, iIn].real, Txy[iOut, iIn].imag, '.')
#ax1.set_title('K = 1, wn = 2 hz, d = 0.1')
ax1.set_xlabel('Real')
ax1.set_ylabel('Imag')

iIn = 0; iOut = 1
ax2 = plt.subplot(2,2,2, sharex = ax1, sharey = ax1); ax2.grid()
ax2.plot(TxyLin[iOut, iIn].real, TxyLin[iOut, iIn].imag)
ax2.plot(Txy[iOut, iIn].real, Txy[iOut, iIn].imag, '.')
#ax2.set_title('K = 1, wn = 4 hz, d = 0.4')
ax2.set_xlabel('Real')
ax2.set_ylabel('Imag')

iIn = 1; iOut = 0
ax3 = plt.subplot(2,2,3, sharex = ax1, sharey = ax1); ax3.grid()
ax3.plot(TxyLin[iOut, iIn].real, TxyLin[iOut, iIn].imag)
ax3.plot(Txy[iOut, iIn].real, Txy[iOut, iIn].imag, '.')
#ax3.set_title('K = 0.25, wn = 6 hz, d = 0.6')
ax3.set_xlabel('Real')
ax3.set_ylabel('Imag')

iIn = 1; iOut = 1
ax4 = plt.subplot(2,2,4, sharex = ax1, sharey = ax1); ax4.grid(True)
ax4.plot(TxyLin[iOut, iIn].real, TxyLin[iOut, iIn].imag)
ax4.plot(Txy[iOut, iIn].real, Txy[iOut, iIn].imag, '.')
#ax4.set_title('K = 1, wn = 8 hz, d = 0.8')
ax4.set_xlabel('Real')
ax4.set_ylabel('Imag')
ax4.legend(['Sys', 'Sys Estimate'])