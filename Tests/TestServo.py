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

#%% Define the frequency selection and distribution of the frequencies into the signals

numChan = 1
freqRate_hz = 200;
timeDur_s = 60.0
numCycles = 1
amp = 4

freqMinDes_rps = (numCycles/timeDur_s) * hz2rps * np.ones(numChan)
#freqMaxDes_rps = (freqRate_hz/2) * hz2rps *  np.ones(numChan)
freqMaxDes_rps = 10 * hz2rps *  np.ones(numChan)
freqStepDes_rps = (10 / 50) * hz2rps
methodSW = 'zip' # "zippered" component distribution

## Generate MultiSine Frequencies
freqElem_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
timeDur_s = time_s[-1] - time_s[0]

## Generate Schroeder MultiSine Signal
ampElem_nd = np.ones_like(freqElem_rps) ## Approximate relative signal amplitude, create flat
sigList, phaseElem_rad, sigElem = GenExcite.MultiSine(freqElem_rps, ampElem_nd, sigIndx, time_s, costType = 'Schroeder', phaseInit_rad = 0, boundPhase = True, initZero = True, normalize = 'peak');

if 0:
    sigList[0], _, _ = GenExcite.Chirp(freqMinDes_rps, freqMaxDes_rps, time_s)


#%%
freeplay = 1.0
#ampList = np.array([3, 5, 10, 20])
ampList = np.arange(2, 10)

# Create Servo Object
objServo = Servo.Servo(1/freqRate_hz, freqNat_rps = 18 * hz2rps, damp = 0.8)
objServo.pwrLim = 5e6
objServo.aLim = 1e5 / (500 * np.pi/180)

pCmdList = []
pOutList = []
for amp in ampList:
    pCmd = amp * sigList[0]
    objServo.freeplay = freeplay
    pOut = np.zeros_like(pCmd)
    av = 0
    for i, s in enumerate(pCmd):
        pOut[i] = objServo.Update(s)
        av = max(av, objServo.a*objServo.v)

    print(av)

    pCmdList.append(pCmd)
    pOutList.append(pOut)

    if False:
        plt.figure()
        plt.plot(time_s, pCmd, time_s, pOut)


#%% Plot the Excitation Spectrum

optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_hz * hz2rps, freq = freqElem_rps, smooth = ('box', 3), winType=('tukey', 0.0))

plt.figure()
for i, pOut in enumerate(pOutList):
    pCmd = pCmdList[i]
    freq_rps, Txy, Cxy, Pxx, Pyy, Pxy = FreqTrans.FreqRespFuncEst(pCmd, pOut, optSpec)
    gain_dB, phase_deg = FreqTrans.GainPhase(Txy)
    freq_hz = freq_rps * rps2hz

    freq_hz = np.squeeze(freq_hz)
    gain_dB = np.squeeze(gain_dB)
    phase_deg = np.squeeze(phase_deg)
    Cxy = np.squeeze(Cxy)

    ax1 = plt.subplot(3,1,1); plt.grid()
    ax1.semilogx(freq_hz, gain_dB, '-', label = 'Amplitude: ' + str(ampList[i]))
    ax2 = plt.subplot(3,1,2); plt.grid()
    ax2.semilogx(freq_hz, phase_deg, '-'); plt.ylim([-180, 180]);
    ax3 = plt.subplot(3,1,3); plt.grid()
    ax3.semilogx(freq_hz, Cxy, '-'); #plt.ylim([0, 1.2])

plt.subplot(3,1,1);
plt.legend()

#%%
import numpy as np
import matplotlib.pyplot as plt

sat = 1
cmd = np.linspace(0, 10, 101)

def SatFunc(delta, A):
    # A is a vector of amplitudes
    # delta is a threshold
    gam = delta / A
    gam = np.clip(gam, -1, 1)

    f = 2/np.pi * (np.arcsin(gam) + gam * np.sqrt(1 - gam**2))

    return f

A = sat
delta = cmd
f = SatFunc(A, delta)

plt.figure()
plt.plot(delta/A, f)
plt.grid(True)

#%%
m = 1
freeplay = 0.5
defLim = 5
cmdLim = 10

cmd = np.linspace(0, 12, 101)

# Saturation (#7)
delta = cmd
A = defLim
nSat_p = m * SatFunc(A, delta)
nSat_q = 0.0

nSat = nSat_p + 1j * nSat_q

# Time Delay (#33)
omega = 1
tDelay_s = 1/1000
nDelay_p = np.cos(omega * tDelay_s)
nDelay_q = -np.sin(omega * tDelay_s)

nDelay = (nDelay_p + 1j * nDelay_q) * np.ones_like(cmd)

# Hard-limit with Freeplay (#42)
delta = freeplay
D = defLim
A = cmd

# A[A < 2.5] = np.nan

deltaPos = (D/m + delta)
deltaNeg = (D/m - delta)
nLim_p = m/2 * (SatFunc(A, deltaPos) + SatFunc(A, deltaNeg))
nLim_q = -4*D*delta / (np.pi * A**2)

nLim = nLim_p + 1j * nLim_q
nLim[np.abs(nLim) > 1] = 1.0

# Cmd-limit (#46)
D = m * freeplay
A = cmd
nCmdLim_p = m
nCmdLim_q = 4 * D / (np.pi * A)

nCmdLim = nCmdLim_p + 1j * nCmdLim_q

plt.figure()
plt.subplot(2,1,1)
plt.plot(cmd, np.abs(nSat), label = 'Saturation')
plt.plot(cmd, np.abs(nDelay), label = 'Time Delay')
plt.plot(cmd, np.abs(nLim), label = 'Hard Limit')
# plt.plot(cmd, np.abs(nCmdLim), label = 'Command Limit')
plt.grid(True)
plt.ylim([0,1.1])
plt.legend()
plt.subplot(2,1,2)
plt.plot(cmd, np.angle(nSat, deg=True), label = 'Saturation')
plt.plot(cmd, np.angle(nDelay, deg=True), label = 'Time Delay')
plt.plot(cmd, np.angle(nLim, deg=True), label = 'Hard Limit')
# plt.plot(cmd, np.angle(nCmdLim, deg=True), label = 'Command Limit')
plt.grid(True)
# plt.ylim([0,1.1])