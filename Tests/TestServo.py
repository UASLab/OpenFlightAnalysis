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

# Hack to allow loading the Core package
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), "..")))
    path.insert(0, abspath(join(dirname(argv[0]), "..", 'Core')))

    del path, argv, dirname, abspath, join

from Core import GenExcite
from Core import FreqTrans
from Core import Servo


# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps

rad2deg = 180.0/np.pi
deg2rad = 1/rad2deg

#%% Define the frequency selection and distribution of the frequencies into the signals
numChan = 1
freqRate_hz = 500
freqRate_rps = freqRate_hz * hz2rps
timeDur_s = 2.0
numCycles = 3

freqMinDes_rps = (numCycles/timeDur_s) * hz2rps * np.ones(numChan)
#freqMaxDes_rps = (freqRate_hz/2) * hz2rps *  np.ones(numChan)
freqMaxDes_rps = 30 * hz2rps *  np.ones(numChan)
freqStepDes_rps = (50 / 50) * hz2rps
methodSW = 'zip' # "zippered" component distribution

## Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_rps, numCycles, freqStepDes_rps, methodSW)
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)
dt = time_s[1] - time_s[0]
timeDur_s = time_s[-1] - time_s[0]
N = time_s.shape[-1]
M = freqExc_rps.shape[-1]

## Generate Schroeder MultiSine Signal
ampElem_nd = (1/M) * np.ones_like(freqExc_rps) ## Approximate relative signal amplitude, create flat
sigList, phaseElem_rad, sigElem = GenExcite.MultiSine(freqExc_rps, ampElem_nd, sigIndx, time_s, costType = 'Schroeder', phaseInit_rad = 0, boundPhase = False, initZero = True);
uPeakFactor = GenExcite.PeakFactor(sigList)
print(sigList.max() - sigList.min())

if False:
    sigList[0], _, _ = GenExcite.Chirp(freqMinDes_rps, freqMaxDes_rps, time_s)

if False:
    zPts = ampElem_nd * np.exp(1j*phaseElem_rad)
    plt.plot(zPts.real, zPts.imag, '.')

# np.diff(np.diff(phaseElem_rad * 180/np.pi))
# plt.plot(np.diff(phaseElem_rad * 180/np.pi), '.')


#%%
freeplay = 3.0
ampLim = 10.0

# ampList = np.array([2, 3, 4])
ampList = np.arange(2, 30, 0.5)

# Create Servo Object
freqNat_rps = 10 * hz2rps
objServo = Servo.Servo(1/freqRate_hz, freqNat_rps, damp = 0.8)
objServo.freeplay = freeplay
# objServo.pLim = 0.2

pCmdList = []
pOutList = []
lim = 0.0
for amp in ampList:
    pCmd = amp * sigList[0]
    
    pOut = np.zeros_like(pCmd)
    p = 0; a = 0; v = 0; av = 0
    for i, s in enumerate(pCmd):
        pOut[i] = objServo.Update(s)
        p = max(p, abs(objServo.p))
        v = max(v,  abs(objServo.v))
        a = max(a,  abs(objServo.a))
        av = max(av,  abs(objServo.a * objServo.v))

    # Apply non-linearities
    if (amp >= ampLim) & (lim == 0.0):
        # lim = p; objServo.pLim = lim
        lim = v; objServo.vLim = lim
        # lim = a; objServo.aLim = lim
        # lim = av; objServo.pwrLim = lim
        
    # power = uPeakFactor[0] * amp * freqNat_rps**2
    power = uPeakFactor[0] * amp * np.sum(freqExc_rps**2) / N
    print(str(amp * rad2deg) + '\t' + str(v * rad2deg) + '\t' + str(a * rad2deg) + '\t' + str(av) + '\t' + str(power))

    print(av / power)
    
    pCmdList.append(pCmd)
    pOutList.append(pOut)
    if False:
        plt.figure()
        plt.plot(time_s, pCmd, time_s, pOut)


#%% Plot the Excitation Spectrum
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate_rps = freqRate_hz * hz2rps, smooth = ('box', 3), winType='bartlett')
optSpec.freq_rps = freqExc_rps
optSpec.freqNull = freqGap_rps

fig1 = 1
fig2 = 2
CxyMin = []
for i, pOut in enumerate(pOutList):
    pCmd = pCmdList[i]
    freq_rps, Txy, Cxy, Sxx, Syy, Sxy, Txn, SxxNull, Snn = FreqTrans.FreqRespFuncEstNoise(pCmd, pOut, optSpec)
    # print(np.sum(SxxNull))
    gain_mag, phase_deg = FreqTrans.GainPhase(Txy, magUnit='mag')
    freq_hz = freq_rps * rps2hz
    
    # Cxy[Cxy > 1.0] = 1.0
    
    Sxy_smooth = FreqTrans.SmoothPolar(Sxy, optSpec) # Smooth
    
    Sxx_smooth = FreqTrans.Smooth(Sxx, optSpec.smooth) # Smooth
    Syy_smooth = FreqTrans.Smooth(Syy, optSpec.smooth) # Smooth
    
    Syy_cohered = Cxy * Syy
    
    Syy_cohered[Syy_cohered>Syy] = Syy[Syy_cohered>Syy]
    
    SNR = Syy / Snn
    SNR_cohered = Syy_cohered / Snn
    
    gainUnc_mag = FreqTrans.Gain(Txn, magUnit='mag')
    
    CxyMin.append(np.min(Cxy[freqNat_rps>freq_rps]))
    
    fig1 = FreqTrans.PlotBode(freq_hz[0], gain_mag[0], phase_deg[0], coher_nd = Cxy[0], gainUnc_mag = gainUnc_mag[0], fig = fig1, dB = True, label = 'Amplitude: ' + str(ampList[i]))
    fig2 = FreqTrans.PlotBode(freq_hz[0], gainUnc_mag[0], None, fig = fig2, dB = True, label = 'Amplitude: ' + str(ampList[i]))
    
#%%
print(Sxx.sum() * N)
print(np.sum(np.abs(pCmd)**2) * dt * N)
print((freqExc_rps**2 * amp**1).sum() / N/16)

#%%
cohFreeplay = 1 - (freeplay/2 / ampList)**2
cohPosLim = 1 - 0.022 * (ampList - ampLim) / ampLim
cohPosLim[cohPosLim > 1] = 1.0

plt.figure(3)
plt.plot(ampList * 1, CxyMin)
plt.plot(ampList * 1, cohFreeplay)
plt.plot(ampList * 1, cohPosLim)
plt.grid(True)

# plt.subplot(3,1,1);
# plt.legend()