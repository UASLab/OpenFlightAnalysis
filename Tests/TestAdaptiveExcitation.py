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
from Core import Servo


# Constants
pi = np.pi

hz2rps = 2*pi
rps2hz = 1/hz2rps

rad2deg = 180/pi
deg2rad = 1/rad2deg

mag2db = control.mag2db
db2mag = control.db2mag


#%% Define the frequency selection and distribution of the frequencies into the signals
# W_dB = -20.0
# W = db2mag(W_dB)
# binSel = FreqTrans.LeakageGoal(W, winType = 'rect')[0]

binSel = 1
W = FreqTrans.DirichletApprox(np.asarray([binSel]))

numChan = 1
freqRate_hz = 50
numCycles = 1

freqMaxDes_rps = 15 * hz2rps *  np.ones(numChan)
freqStepDes_rps = (20 / 50) * hz2rps
tBinWidth = FreqTrans.FreqStep2TimeBin(freqStepDes_rps)
timeDur_s = binSel * tBinWidth
freqMinDes_rps = (1/timeDur_s) * hz2rps * np.ones(numChan)

methodSW = 'zip' # "zippered" component distribution

## Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

## Generate Schroeder MultiSine Signal
ampExc_nd = np.ones_like(freqExc_rps) ## Approximate relative signal amplitude, create flat
sigList, phaseElem_rad, sigElem = GenExcite.MultiSine(freqExc_rps, ampExc_nd, sigIndx, time_s, costType = 'Schroeder', phaseInit_rad = 0, boundPhase = True, initZero = True, normalize = 'rms');
sigPeakFactor = GenExcite.PeakFactor(sigList)

ampExc_nd *= 0.5
sigList *= 0.5

# Excited Amplitude per input channel
ampChan_nd = ampExc_nd[sigIndx]

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]
freqChan_hz = freqChan_rps * rps2hz

#%% Setup Servo Model
freqNat_hz = 6.0
freqNat_rps = freqNat_hz * hz2rps
objServo = Servo.Servo(1/freqRate_hz, freqNat_rps = freqNat_rps, damp = 0.8)
objServo.freeplay = 1.0
objServo.timeDelay_s = 50 / 1000 # this ends up rounded to an integer (timeDelay_s * freqRate_hz)
objServo.vLim = 560

# np.random.seed(584)
rStd = [0.5]
r = np.random.normal([0.0], rStd, size = (len(time_s), 1)).T

nStd = [0.5]
n = np.random.normal([0.0], nStd, size = (len(time_s), 1)).T


#%% Setup FRE
optSpec = FreqTrans.OptSpect(dftType = 'czt', scaleType = 'density', freqRate = freqRate_hz * hz2rps, smooth = ('box', 3), winType = 'rect', detrendType = 'linear')

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps
optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True


#%% Simulate
timeRefine_s = []
ampList = []
pCmdRefine = []
pMeasRefine = []

TuzRefine = []
CuzRefine = []
SuuRefine = []
SzzRefine = []
SuzRefine = []
TunRefine = []
SuuNullRefine = []
SnnRefine = []

maxIter = 10
objServo.Start() # Initialize the Servo model
ampInc = ampChan_nd.mean()
ampDec = ampInc/2
for iIter in range(maxIter):
    timeStart_s = iIter * time_s[-1]
    timeRefine_s.append(timeStart_s + time_s)

    # Simulate Servo Response
    # Resample RNG
    r = np.random.normal([0.0], rStd, size = (len(time_s), 1)).T
    n = np.random.normal([0.0], nStd, size = (len(time_s), 1)).T

    pCmd = sigList + r

    pOut = np.zeros_like(pCmd[0])
    for i, s in enumerate(pCmd[0]):
        pOut[i] = objServo.Update(s)

    # Add outputs
    pMeas = pOut + n # pMeas = G (x + r) + n = Gx + (Gr + n); SNR = Gx / (Gr + n)

    ampList.append(np.mean(ampExc_nd))
    pCmdRefine.append(pCmd)
    pMeasRefine.append(pMeas)

    # Estimate the FRF
    # freq_rps, Tuz, Cuz, Suu, Szz, Suz = FreqTrans.FreqRespFuncEst(pCmd, pMeas, optSpec)
    freq_rps, Tuz, Cuz, Suu, Szz, Suz, Tun, SuuNull, Snn = FreqTrans.FreqRespFuncEstNoise(pCmd, pMeas, optSpec)
    freq_hz = freq_rps * rps2hz

    Cuz = np.abs(Cuz)**2 # Mean Square Coherence
    Cuz[Cuz>1] = 1.0

    TuzRefine.append(Tuz.squeeze())
    CuzRefine.append(Cuz.squeeze())
    SuuRefine.append(Suu.squeeze())
    SzzRefine.append(Szz.squeeze())
    SuzRefine.append(Suz.squeeze())
    TunRefine.append(Tun.squeeze())
    SuuNullRefine.append(SuuNull.squeeze())
    SnnRefine.append(Snn.squeeze())

    # Increase Amplitude
    iAmpIncr = [True] * np.ones_like(ampChan_nd, dtype=bool)
    iAmpDecr = [False] * np.ones_like(ampChan_nd, dtype=bool)
    iAmpGoal = [False] * np.ones_like(ampChan_nd, dtype=bool)

    if iIter > 0:
        CuzCurr = CuzRefine[-1]
        CuzPrev = CuzRefine[-2]
        dCdA = (CuzCurr - CuzPrev) / ampInc

        zSNRCurr = SzzRefine[-1] / SnnRefine[-1]
        zSNRPrev = SzzRefine[-2] / SnnRefine[-2]
        dSNRdA = (zSNRCurr - zSNRPrev) / ampInc

        uE2NCurr = SuuNullRefine[-1] / SuuRefine[-1]
        uE2NPrev = SuuNullRefine[-2] / SuuRefine[-2]
        dE2NdA = (uE2NCurr - uE2NPrev) / ampInc

        # Stopping Criteria metrics
        iCohere = (CuzCurr > 0.75) # Coherence achieved goal
        iCohereSlow = ((CuzCurr - CuzPrev) < 0.05) # Coherence is changing slowly (invariant to amplitude)
        iSNRHuge = (zSNRCurr > 1e3) # SNR is really large
        iTooSmall = (np.abs(TuzRefine[-1]) + np.abs(TunRefine[-1])) < 0.1 # T is nowhere near (-1+0j)

        iAmpGoal = iCohere | iCohereSlow | iAmpGoal
        iAmpIncr = ((dCdA > 0) & (dSNRdA < 0)) & ~iAmpGoal
        iAmpDecr = ((dCdA < 0) & (dSNRdA > 0)) & ~iAmpGoal

        if all(iAmpGoal.all(axis=0) == True):
            print('!!!!!!!!!!!!!!!')
            break

    # Adjust amplitude
    ampChan_nd = ampChan_nd + ampInc # All components
    # ampChan_nd = ampChan_nd - ampDec

    # Each components individually
    # ampChan_nd[iAmpIncr] = ampChan_nd[iAmpIncr] + ampInc
    # ampChan_nd[iAmpDecr] = ampChan_nd[iAmpDecr] - ampInc

    ampExc_nd = ampChan_nd.flatten(order='F')

    sigList, phaseElem_rad, sigElem = GenExcite.MultiSine(freqExc_rps, ampExc_nd, sigIndx, time_s, costType = 'Schroeder', phaseInit_rad = 0, boundPhase = True, initZero = True);
    sigPeakFactor = GenExcite.PeakFactor(sigList)
    ampChan_nd = ampExc_nd[sigIndx]


#%%
# Leakage Goal
WGoal_dB = -20.0
WGoal = db2mag(WGoal_dB)
binGoal = FreqTrans.LeakageGoal(WGoal, winType = 'boxcar')[0]
# binGoal = FreqTrans.LeakageGoal(WGoal, winType = 'hann')[0]

freqStepDes_rps = (10 / 50) * hz2rps
tBinWidth = FreqTrans.FreqStep2TimeBin(freqStepDes_rps)

# XXX - SNR Goal - zSNRProjectMean = numCycles * np.mean(zSNRCurr)
zSNRGoal = 100
numCyclesSNR = zSNRGoal / np.mean(zSNRCurr)

tLeakageGoal = binGoal * tBinWidth

# Estimate how many cycles are required to achieve objective
tCycle = 1/(freqMinDes_rps * rps2hz)[0]
numCycles = np.ceil(tLeakageGoal / tCycle)

# Predict the final SNR estimate
zSNRCurr = SzzRefine[-1] / SnnRefine[-1]
zSNRProjectMean = numCycles * np.mean(zSNRCurr)
# zSNRMergeMeanErgotic
# zSNRMean

## Generate MultiSine Frequencies
freqExcFinal_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
freqGapFinal_rps = freqExcFinal_rps[0:-1] + 0.5 * np.diff(freqExcFinal_rps)

## Generate Schroeder MultiSine Signal
ampExcFinal_nd = np.mean(ampExc_nd) * np.ones_like(freqExcFinal_rps) # Need full amplitude vector
sigList, phaseElem_rad, sigElem = GenExcite.MultiSine(freqExcFinal_rps, ampExcFinal_nd, sigIndx, time_s, costType = 'Schroeder', phaseInit_rad = 0, boundPhase = True, initZero = True)
sigPeakFactor = GenExcite.PeakFactor(sigList)

# Resample RNG
r = np.random.normal([0.0], rStd, size = (len(time_s), 1)).T
n = np.random.normal([0.0], nStd, size = (len(time_s), 1)).T

# Simulate Servo Response
pCmd = sigList + r

pOut = np.zeros_like(pCmd[0])
for i, s in enumerate(pCmd[0]):
    pOut[i] = objServo.Update(s)

# Add outputs
pMeas = pOut + n

# Estimate the Final FRF
freqChanFinal_rps = freqExcFinal_rps[sigIndx]
freqGapFinal_rps = freqExcFinal_rps[0:-1] + 0.5 * np.diff(freqExcFinal_rps)

optSpec.freq = freqChanFinal_rps
optSpec.freqInterp = freqExcFinal_rps

# Null Frequencies
optSpec.freqNull = freqGapFinal_rps
optSpec.freqNullInterp = True

optSpec.winType = 'boxcar'

## Split the signals into segments, process each, merge
cyclepseg = int(tBinWidth / tCycle) # # of cycles per segment
numSeg = numCycles / cyclepseg
nperseg = int(pCmd.shape[-1] / numSeg)
noverlap = 0
numFreq = freqExcFinal_rps.shape[-1]

timeFinal_s = []
pCmdFinal = []
pMeasFinal = []

TuzFinal = []
CuzFinal = []
SuuFinal = []
SzzFinal = []
SuzFinal = []
TunFinal = []
SuuNullFinal = []
SnnFinal = []

for iSeg in range(int(numSeg)):
  iSeg = iSeg*nperseg + np.arange(0, nperseg)
  timeFinal_s.append(time_s[iSeg])
  pCmdFinal.append(pCmd[0][iSeg])
  pMeasFinal.append(pMeas[0][iSeg])

  # freq_rps, Tuz, Cuz, Suu, Szz, Suz = FreqTrans.FreqRespFuncEst(pCmd, pMeas, optSpec)
  freq_rps, Tuz, Cuz, Suu, Szz, Suz, Tun, SuuNull, Snn = FreqTrans.FreqRespFuncEstNoise(pCmdFinal[-1], pMeasFinal[-1], optSpec)

  Cuz = np.abs(Cuz)**2 # Mean Square Coherence
  Cuz[Cuz>1] = 1.0

  TuzFinal.append(Tuz.squeeze())
  CuzFinal.append(Cuz.squeeze())
  SuuFinal.append(Suu.squeeze())
  SzzFinal.append(Szz.squeeze())
  SuzFinal.append(Suz.squeeze())
  TunFinal.append(Tun.squeeze())
  SuuNullFinal.append(SuuNull.squeeze())
  SnnFinal.append(Snn.squeeze())


# Run the whole 'Final' sequence together for comparison
freq_rps, Tuz, Cuz, Suu, Szz, Suz, Tun, SuuNull, Snn = FreqTrans.FreqRespFuncEstNoise(pCmd, pMeas, optSpec)

Cuz = np.abs(Cuz)**2 # Mean Square Coherence
Cuz[Cuz>1] = 1.0

#%%
fig = plt.figure()
ax1 = plt.subplot(2,1,1)
ax2 = plt.subplot(2,1,2)
for iRefine, t in enumerate(timeRefine_s):
    ax1.plot(timeRefine_s[iRefine], pCmdRefine[iRefine][0])
    ax2.plot(timeRefine_s[iRefine], pMeasRefine[iRefine][0])

for iFinal, t in enumerate(timeFinal_s):
    ax1.plot(timeFinal_s[iFinal] + timeRefine_s[iRefine].max(), pCmdFinal[iFinal])
    ax2.plot(timeFinal_s[iFinal] + timeRefine_s[iRefine].max(), pMeasFinal[iFinal])

ax1.grid(True)
ax1.set_ylabel('Command')
ax2.grid(True)
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Measurement')


#%%
# Refinement Stage Estimates
iIterRefine = np.arange(1, len(timeRefine_s)+1)

# Null - to - Excitation Ratio Statistics
uN2ERefine = np.abs(SuuRefine) / np.abs(SuuNullRefine)

uN2ERefineMean = np.mean(uN2ERefine, axis=-1)
uN2ERefineStd = np.std(uN2ERefine, axis=-1)
uN2ERefineMin = np.min(uN2ERefine, axis=-1)
uN2ERefineMax = np.max(uN2ERefine, axis=-1)

# SNR Statistics
zSNRRefine = np.abs(SzzRefine) / np.abs(SnnRefine)

zSNRRefineMean = np.mean(zSNRRefine, axis=-1)
zSNRRefineStd = np.std(zSNRRefine, axis=-1)
zSNRRefineMin = np.min(zSNRRefine, axis=-1)
zSNRRefineMax = np.max(zSNRRefine, axis=-1)

# Coherence Statistics
CuzRefineMean = np.mean(CuzRefine, axis=-1)
CuzRefineStd = np.std(CuzRefine, axis=-1)
CuzRefineMin = np.min(CuzRefine, axis=-1)
CuzRefineMax = np.max(CuzRefine, axis=-1)

# Final Stage Estimates
iIterFinal = iIterRefine[-1] + np.arange(1, len(timeFinal_s)+1)
iIterFinalRange = [iIterFinal[0], iIterFinal[-1]]

# Excitation - to - Null Statistics
uN2EFinal = np.abs(SuuFinal) / np.abs(SuuNullFinal)

uN2EFinalMean = np.mean(uN2EFinal, axis=-1)
uN2EFinalStd = np.std(uN2EFinal, axis=-1)
uN2EFinalMin = np.min(uN2EFinal, axis=-1)
uN2EFinalMax = np.max(uN2EFinal, axis=-1)

uN2EMergeMean = np.mean(uN2EFinalMean, axis=-1)
uN2EMergeStd = np.std(uN2EFinalMean, axis=-1) + np.mean(uN2EFinalStd, axis=-1)
uN2EMergeStd / uN2EMergeMean

uN2E = (np.abs(Suu) / np.abs(SuuNull)).squeeze()
uN2EMean = np.mean(uN2E, axis=-1)
uN2EStd = np.std(uN2E, axis=-1)

# SNR Statistics
zSNRFinal = np.abs(SzzFinal) / np.abs(SnnFinal)

zSNRFinalSum = np.sum(zSNRFinal, axis=-1)
zSNRFinalMean = np.mean(zSNRFinal, axis=-1)
zSNRFinalStd = np.std(zSNRFinal, axis=-1)
zSNRFinalMin = np.min(zSNRFinal, axis=-1)
zSNRFinalMax = np.max(zSNRFinal, axis=-1)

zSNRMergeMean = np.mean(zSNRFinalMean, axis=-1)
zSNRMergeStd = np.std(zSNRFinalMean, axis=-1) + np.mean(zSNRFinalStd, axis=-1)

zSNRMergeMeanErgotic = (numSeg) * zSNRMergeMean
zSNRMergeStdErgotic = (numSeg) * zSNRMergeStd

zSNR = np.abs(Szz) / np.abs(Snn)
zSNRMean = np.mean(zSNR, axis=-1)
zSNRStd = np.std(zSNR, axis=-1)


# Coherence Statistics
CuzFinalMean = np.mean(CuzFinal, axis=-1)
CuzFinalStd = np.std(CuzFinal, axis=-1)
CuzFinalMin = np.min(CuzFinal, axis=-1)
CuzFinalMax = np.max(CuzFinal, axis=-1)

CuzMergeMean = np.mean(CuzFinalMean, axis=-1)
CuzMergeStd = np.std(CuzFinalMean, axis=-1) + np.mean(CuzFinalStd, axis=-1)

CuzMean = np.mean(Cuz, axis=-1)
CuzStd = np.std(Cuz, axis=-1)


#%% Linear System
sysLin = control.tf([objServo.freqNat_rps**2], [1, 2*objServo.damp*objServo.freqNat_rps, objServo.freqNat_rps**2])

TLinRefine = FreqTrans.FreqResp(sysLin, freqExc_rps)
TLinFinal = FreqTrans.FreqResp(sysLin, freqExcFinal_rps)

# Error
TErrRefine = (TLinRefine - TuzRefine).squeeze()
TErrRefineMean = np.mean(np.abs(TErrRefine), axis = -1)
TErrRefineStd = np.std(np.abs(TErrRefine), axis = -1)

TErrRefineMSE = np.mean(np.abs(TErrRefine)**2, axis = -1)
TErrRefineMSD = np.std(np.abs(TErrRefine)**2, axis = -1)


TErrFinal = (TLinFinal - TuzFinal).squeeze()
TErrFinalMean = np.mean(np.abs(TErrFinal), axis = -1)
TErrFinalStd = np.std(np.abs(TErrFinal), axis = -1)

TErrFinalMSE = np.mean(np.abs(TErrFinal)**2, axis = -1)
TErrFinalMSD = np.std(np.abs(TErrFinal)**2, axis = -1)

TErrMergeMean = np.mean(TErrFinalMean, axis=-1)
TErrMergeStd = np.std(TErrFinalMean, axis=-1) + np.mean(TErrFinalStd, axis=-1)

TErrMergeMSE = np.mean(TErrFinalMSE)
TErrMergeMSD = np.mean(TErrFinalMSD)

TErr = (TLinFinal - Tuz).squeeze()
TErrMean = np.mean(np.abs(TErr))
TErrStd = np.std(np.abs(TErr))

TErrMSE = np.mean(np.abs(TErr)**2)
TErrMSD = np.std(np.abs(TErr)**2, axis = -1)


#%% Plot the Error Distribution
fig = plt.figure()
plt.plot(freqExcFinal_rps * rps2hz, np.abs(TErrFinal).T, ':b', label = 'Final Estimates')
plt.plot(freqExcFinal_rps * rps2hz, np.mean(np.abs(TErrFinal), axis=0).T, '-b', label = 'Final Merge Estimate')
plt.plot(freqExcFinal_rps * rps2hz, [TErrMergeMean] * np.ones_like(freqExcFinal_rps), '-b', label = 'Final Merge Estimate')

plt.plot(freqExcFinal_rps * rps2hz, np.abs(TErr).T, '-k', label = 'Continuous Estimate')
plt.plot(freqExcFinal_rps * rps2hz, [TErrMean] * np.ones_like(freqExcFinal_rps), '-k', label = 'Final Merge Estimate')

plt.xlabel('Frequency [Hz]')
plt.ylabel('Error')
plt.grid(True)
plt.legend()

#%% Plot the Error Distribution
fig = plt.figure()
# plt.plot(freqExcFinal_rps * rps2hz, np.abs(TErrFinal**2).T, ':b', label = 'Final Estimates')
plt.plot(freqExcFinal_rps * rps2hz, np.mean(np.abs(TErrFinal**2), axis=0).T, '-b', label = 'Final Merge Estimate')
plt.plot(freqExcFinal_rps * rps2hz, [TErrMergeMSE] * np.ones_like(freqExcFinal_rps), '-b', label = 'Final Merge Estimate')

plt.plot(freqExcFinal_rps * rps2hz, np.abs(TErr**2).T, '-k', label = 'Continuous Estimate')
plt.plot(freqExcFinal_rps * rps2hz, [TErrMSE] * np.ones_like(freqExcFinal_rps), '-k', label = 'Final Merge Estimate')

plt.xlabel('Frequency [Hz]')
plt.ylabel('Mean Squared Error')
plt.grid(True)
plt.legend()

#%% Plot the SNR Distribution
fig = plt.figure()
# plt.plot(freqExcFinal_rps * rps2hz, 1/zSNRFinal.T, ':b', label = 'Final Estimates')
plt.plot(freqExcFinal_rps * rps2hz, np.mean(1/zSNRFinal, axis=0).T, '-b', label = 'Final Merge Estimate')
plt.plot(freqExcFinal_rps * rps2hz, np.mean(1/zSNRFinal) * np.ones_like(freqExcFinal_rps), '-b', label = 'Final Merge Estimate')

plt.plot(freqExcFinal_rps * rps2hz, (1/zSNRMergeMeanErgotic) * np.ones_like(freqExcFinal_rps), '-r', label = 'Final Merge Estimate (Ergodic)')

plt.plot(freqExcFinal_rps * rps2hz, 1/zSNR.T, '-k', label = 'Continuous Estimate')
plt.plot(freqExcFinal_rps * rps2hz, np.mean(1/zSNR) * np.ones_like(freqExcFinal_rps), '-k', label = 'Final Merge Estimate')

plt.xlabel('Frequency [Hz]')
plt.ylabel('Noise/Signal Ratio')
plt.grid(True)
plt.legend()

#%% Plot the iteration history
fig = plt.figure()

plt.subplot(4,1,1)
plt.plot(iIterRefine, (TErrRefineMSE), '--.k', label = 'Refine Estimates')
# plt.fill_between(iIterRefine, (TErrRefineMSE - TErrRefineMSD), (TErrRefineMSE + TErrRefineMSD), alpha = 0.25, color = 'k', label = 'Refine Estimates')

plt.plot(iIterFinal, (TErrFinalMSE), '--.b', label = 'Final Estimate')
plt.fill_between(iIterFinal, (TErrFinalMSE - TErrFinalMSD), (TErrFinalMSE + TErrFinalMSD), alpha = 0.25, color = 'b', label = 'Final Estimates')

plt.plot(iIterFinalRange, [TErrMergeMSE]*2, '-r', label = 'Final Merge')
plt.fill_between(iIterFinalRange, [TErrMergeMSE - TErrMergeMSD]*2, [TErrMergeMSE + TErrMergeMSD]*2, alpha = 0.25, color = 'r', label = 'Final Merge')

plt.plot(iIterFinalRange, [TErrMSE]*2, '-k', label = 'Final Continuous')
plt.fill_between(iIterFinalRange, [TErrMSE - TErrMSD]*2, [TErrMSE + TErrMSD]*2, alpha = 0.25, color = 'k', label = 'Final Continuous')
plt.yscale('log')
plt.ylabel('Mean Square Error')
plt.grid(True)
plt.legend()

# Fix Legend
ax = fig.get_axes()
handles, labels = ax[0].get_legend_handles_labels()
handles = [handles[0], (handles[1], handles[4]), (handles[2], handles[5]), (handles[3], handles[6])]
labels = [labels[0], labels[1], labels[2], labels[3]]
ax[0].legend(handles, labels)


plt.subplot(4,1,2)
plt.plot(iIterRefine, 1/(uN2ERefineMean), '--.k', label = 'Refine Estimates')
# plt.fill_between(iIterRefine, (1/uN2ERefineMean - 1/uN2ERefineStd), (1/uN2ERefineMean + 1/uN2ERefineStd), alpha = 0.25, color = 'k', label = 'Refine Estimates')

plt.plot(iIterFinal, 1/(uN2EFinalMean), '--.b', label = 'Final Estimate')
plt.fill_between(iIterFinal, (1/uN2EFinalMean - 1/uN2EFinalStd), (1/uN2EFinalMean + 1/uN2EFinalStd), alpha = 0.25, color = 'b', label = 'Final Estimates')

plt.plot(iIterFinalRange, [1/uN2EMergeMean]*2, '-r', label = 'Final Merge')
plt.fill_between(iIterFinalRange, (1/uN2EMergeMean - 1/uN2EMergeStd), (1/uN2EMergeMean + 1/uN2EMergeStd), alpha = 0.25, color = 'r', label = 'Final Merge')

plt.plot(iIterFinalRange, [1/uN2EMean]*2, '-k', label = 'Final Continuous')
plt.fill_between(iIterFinalRange, (1/uN2EMean - 1/uN2EStd), (1/uN2EMean + 1/uN2EStd), alpha = 0.25, color = 'k', label = 'Final Continuous')
plt.yscale('log')
plt.ylabel('Input Excitation/Null')
plt.grid(True)


plt.subplot(4,1,3)
plt.plot(iIterRefine, (1/zSNRRefineMean), '--.k', label = 'Refine Estimates')
# plt.fill_between(iIterRefine, (1/zSNRRefineMean - 1/zSNRRefineStd), (1/zSNRRefineMean + 1/zSNRRefineStd), alpha = 0.25, color = 'k', label = 'Refine Estimates')

plt.plot(iIterFinal, (1/zSNRFinalMean), '--.b', label = 'Final Estimate')
plt.fill_between(iIterFinal, (1/zSNRFinalMean - 1/zSNRFinalStd), (1/zSNRFinalMean + 1/zSNRFinalStd), alpha = 0.25, color = 'b', label = 'Final Estimates')

plt.plot(iIterFinalRange, [1/zSNRMergeMean]*2, '-r', label = 'Final Merge')
plt.fill_between(iIterFinalRange, (1/zSNRMergeMean - 1/zSNRMergeStd), (1/zSNRMergeMean + 1/zSNRMergeStd), alpha = 0.25, color = 'r', label = 'Final Merge')

plt.plot(iIterFinalRange, [1/zSNRMergeMeanErgotic] * 2, '-m', label = 'Final Merge Estimate (Ergodic)')
plt.fill_between(iIterFinalRange, (1/zSNRMergeMeanErgotic - 1/zSNRMergeStdErgotic), (1/zSNRMergeMeanErgotic + 1/zSNRMergeStdErgotic), alpha = 0.25, color = 'm', label = 'Final Merge Estimate (Ergodic)')

plt.plot(iIterFinalRange, [1/zSNRMean]*2, '-k', label = 'Final Continuous')
plt.fill_between(iIterFinalRange, (1/zSNRMean - 1/zSNRStd), (1/zSNRMean + 1/zSNRStd), alpha = 0.25, color = 'k', label = 'Final Continuous')
plt.yscale('log')
plt.ylabel('Output Noise/Signal')
plt.grid(True)


plt.subplot(4,1,4)
plt.plot(iIterRefine, (CuzRefineMean), '--.k', label = 'Refine Estimates')
# plt.fill_between(iIterRefine, (CuzRefineMean - CuzRefineStd), (CuzRefineMean + CuzRefineStd), alpha = 0.25, color = 'k', label = 'Refine Estimates')

plt.plot(iIterFinal, (CuzFinalMean), '--.b', label = 'Final Estimate')
plt.fill_between(iIterFinal, (CuzFinalMean - CuzFinalStd), (CuzFinalMean + CuzFinalStd), alpha = 0.25, color = 'b', label = 'Final Estimates')

plt.plot(iIterFinalRange, [CuzMergeMean]*2, '-r', label = 'Final Merge')
plt.fill_between(iIterFinalRange, (CuzMergeMean - CuzMergeStd), (CuzMergeMean + CuzMergeStd), alpha = 0.25, color = 'r', label = 'Final Merge')

plt.plot(iIterFinalRange, [CuzMean]*2, '-k', label = 'Final Continuous')
plt.fill_between(iIterFinalRange, (CuzMean - CuzStd), (CuzMean + CuzStd), alpha = 0.25, color = 'k', label = 'Final Continuous')

plt.xlabel('Iteration')
plt.ylabel('Coherence')
plt.grid(True)

#%%
pCmdRefineMean = np.mean(np.abs(pCmdRefine), axis=-1).squeeze()
pMeasRefineMean = np.mean(np.abs(pMeasRefine), axis=-1).squeeze()

plt.figure()
plt.plot(iIterRefine, ampList, '-*k', label = 'Nominal Amplitude')
plt.plot(iIterRefine, pCmdRefineMean, '-*b', label = 'Command Amplitude')
plt.plot(iIterRefine, pMeasRefineMean, '-*r', label = 'Measurement Amplitude')
plt.xlabel('Iteration')
plt.ylabel('Amplitude')
plt.legend()
plt.grid(True)


if False:
  plt.figure()
  plt.plot(pCmdRefineMean, pMeasRefineMean, '.k', label = 'Refinement Estimates')

  plt.xlabel('Command')
  plt.ylabel('Output')
  plt.legend()
  plt.grid(True)


#%% Bode Progression
if False:
  gainLin_mag, phaseLin_deg = FreqTrans.GainPhase(TLinFinal, magUnit = 'mag', unwrap = False)

  gainRefine_mag, phaseRefine_deg = FreqTrans.GainPhase(TuzRefine, magUnit = 'mag', unwrap = False)
  gainFinal_mag, phaseFinal_deg = FreqTrans.GainPhase(TuzFinal, magUnit = 'mag', unwrap = False)
  gainFinalMean_mag = np.mean(gainFinal_mag, axis=0)
  phaseFinalMean_deg = np.mean(phaseFinal_deg, axis=0)
  phaseFinalMean_deg = np.mean(phaseFinal_deg, axis=0)
  CuzFinalMean = np.mean(CuzFinal, axis=0)

  fig = None
  fig = FreqTrans.PlotBode(freqExcFinal_rps * rps2hz, gainLin_mag.squeeze(), phaseLin_deg.squeeze(), coher_nd = np.ones_like(gainLin_mag).squeeze(), gainUnc_mag = None, fig = fig, dB = True, linestyle = '-', color='k', label='Linear Model')

  fig = FreqTrans.PlotBode(freqExc_rps * rps2hz, gainRefine_mag[0], phaseRefine_deg[0], coher_nd = CuzRefine[0], gainUnc_mag = None, fig = fig, dB = True, linestyle = ':', color='b', label='First iteration')
  fig = FreqTrans.PlotBode(freqExc_rps * rps2hz, gainRefine_mag[-1], phaseRefine_deg[-1], coher_nd = CuzRefine[-1], gainUnc_mag = None, fig = fig, dB = True, linestyle = '-', color='b', label='Last Refinement iteration')

  for iFinal, t in enumerate(timeFinal_s):
    fig = FreqTrans.PlotBode(freqExcFinal_rps * rps2hz, gainFinal_mag[iFinal], phaseFinal_deg[iFinal], coher_nd = CuzFinal[iFinal], gainUnc_mag = None, fig = fig, dB = True, linestyle = ':', color='r', label='Final Estimates')

  fig = FreqTrans.PlotBode(freqExcFinal_rps * rps2hz, gainFinalMean_mag, phaseFinalMean_deg, coher_nd = CuzFinalMean, gainUnc_mag = None, fig = fig, dB = True, linestyle = '-', color='r', label='Final Merge')

  ax = fig.get_axes()
  ax[-1].set_ylim(0, 1)

