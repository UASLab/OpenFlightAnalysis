"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan


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

K = 1.0 ; wn = 3 * hz2rps; d = 0.2;
sysPlant = control.tf([K * wn**2], [1, 2.0*d*wn, wn**2])


#%%
numExc = 1
numCycles = 3
ampInit = 1
ampFinal = ampInit
freqMinDes_rps = 0.1 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 10 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (30 / freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)

# Generate Schroeder MultiSine Signal
ampExcit_nd = np.sqrt(2/len(freqExc_rps)) * np.linspace(ampInit, ampFinal, len(freqExc_rps))
uExc, _, sigExcit = GenExcite.MultiSine(freqExc_rps, ampExcit_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, costType = 'Schroeder')

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]

# Null Frequencies
freqGap_rps = freqExc_rps[:-1] + 0.5 * np.diff(freqExc_rps)

# Plant-Input Noise
d = np.zeros_like(uExc)
u = uExc + d

# Simulate the excitation through the system
_, y, _ = control.forced_response(sysPlant, T = time_s, U = u)

# Plant-Output Noise
wn = 6.0 * hz2rps; d = 0.2;
#sigma = 1.0 * d # This puts the peak at the constant specified

sigma = 1.0
#sysNoise = sigma * control.tf([1], [1])
sysNoise = sigma * control.tf([-1, 0, wn**2], [1, 2*d*wn, wn**2])

np.random.seed(0)
m = np.random.normal(0.0, 1.0, size = uExc.shape) # Input to the Noise Filter
_, n, _ = control.forced_response(sysNoise, T = time_s, U = m)

# Output with Noise
z = y + n

#control.bode(sysNoise)

#plt.figure()
#plt.subplot(3,1,1)
#plt.plot(time_s, u.T)
#plt.subplot(3,1,2)
#plt.plot(time_s, y.T)
#plt.subplot(3,1,3)
#plt.plot(time_s, n.T, time_s, z.T)

#%%
# Index
N = u.shape[-1]
iN = np.arange(N, dtype = float) # time indices

# Z axis coordinates
zPts = np.exp(1j * 2*pi * freqChan_rps / freqRate_rps)
zPtsNull = np.exp(1j * 2*pi * np.atleast_2d(freqGap_rps) / freqRate_rps)

#plt.figure(); plt.plot(zPts[0].real, zPts[0].imag, '*', zPtsNull[0].real, zPtsNull[0].imag, '*')

# Compute the Chirp-Z Transform (Generalized DFT) via a Matrix
#dftMat = np.power(np.atleast_2d(zPts).T, -np.arange(N, dtype = float))
uDft, uDftHist = FreqTrans.CZTMat(uExc, zPts)
zDft, zDftHist = FreqTrans.CZTMat(z, zPts)

yDft, _ = FreqTrans.CZTMat(y, zPts)
mDft, _ = FreqTrans.CZTMat(m, zPts)
nDft, _ = FreqTrans.CZTMat(n, zPts)

# Compute Power, factor of 2 because DFT is one-sided
scale = 2 * FreqTrans.PowerScale('spectrum', np.ones_like(uExc), freqRate_hz)

Puu = (uDft.conj() * uDft).real * scale
Puz = (uDft.conj() * zDft) * scale
Pzz = (zDft.conj() * zDft).real * scale

Pyy = (yDft.conj() * yDft).real * scale
Pmm = (mDft.conj() * mDft).real * scale
Pnn = (nDft.conj() * nDft).real * scale


#%%

# Nominal
Tuz = Puz / Puu

gainTuz_mag, phaseTuz_deg = FreqTrans.GainPhase(Tuz, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTuz_dB = mag2db(gainTuz_mag)

## Variance Technique

uDftAccum = uDftHist.cumsum(axis = -1)
zDftAccum = zDftHist.cumsum(axis = -1)

scaleHist = scale * (N/np.arange(1, N+1))**2
PuuHist = (uDftAccum.conj() * uDftAccum).real * scaleHist
PuzHist = (uDftAccum.conj() * zDftAccum) * scaleHist
PzzHist = (zDftAccum.conj() * zDftAccum).real * scaleHist

#plt.figure
#plt.subplot(2,1,1); plt.grid(True)
#plt.plot(mag2db(PuuHist)[0].T)
#plt.subplot(2,1,2); plt.grid(True)
#plt.plot(mag2db(PzzHist)[0].T)


PuuVar = PuuHist.var(axis = -1)
PzzVar = PzzHist.var(axis = -1)

#plt.figure
#plt.subplot(2,1,1); plt.grid(True)
#plt.plot(freqChan_rps[0], mag2db(PuuVar).T)
#plt.subplot(2,1,2); plt.grid(True)
#plt.plot(freqChan_rps[0], mag2db(PzzVar).T)


## Null Technique
freq = freqChan_rps[0]
freq_N = freqGap_rps

uDft_N, uDftHist_N = FreqTrans.CZTMat(uExc, zPtsNull)
zDft_N, zDftHist_N = FreqTrans.CZTMat(z, zPtsNull)

#
PuuNull_N = (uDft_N.conj() * uDft_N).real * scale
PuuNull_E = FreqTrans.InterpPolar(PuuNull_N, freq_N, freq, 'linear')

uDftNull_E = FreqTrans.InterpPolar(uDft_N, freq_N, freq, 'linear')
PuuNull_E1 = (uDftNull_E.conj() * uDftNull_E).real * scale

# 
PzzNull_N = (zDft_N.conj() * zDft_N).real * scale
PzzNull_E = FreqTrans.InterpVal(PzzNull_N, freq_N, freq, 'linear')


zDftNull_E = FreqTrans.InterpPolar(zDft_N, freq_N, freq, 'linear')
PzzNull_E1 = (zDftNull_E.conj() * zDftNull_E).real * scale


# Null Cross Spectrum
numFreqNull = len(freq_N)
uDftOnes_N = np.ones_like(zDft_N) / (numFreqNull * scale)**0.5
PuuOnes_N = (uDftOnes_N.conj() * uDftOnes_N).real * scale

PuzNull_N = uDft_N.conj() * zDft_N * scale
Puu_N = uDft_N.conj() * uDft_N * scale
TuzNull_N = PuzNull_N / Puu_N

Puz_N = uDft.conj() * FreqTrans.InterpPolar(zDft_N, freq_N, freq, 'linear') * scale


TuzUnc = FreqTrans.InterpPolar(uDft_N, freq_N, freq, 'linear').conj() / zDft
TuzUnc_N = uDft_N.conj() / FreqTrans.InterpPolar(zDft, freq, freq_N, 'linear')


#Pxx = xDft.conj() * xDft * scale = (xR - xI) * (xR + xI) * scale
#Pyy = yDft.conj() * yDft * scale = (yR - yI) * (yR + yI) * scale
#
#Pxy = xDft.conj() * yDft * scale = (xR - xI) * (yR + yI) * scale
#Txy = Pxy / Pxx = [xDft.conj() * yDft] / [xDft.conj() * xDft]
#    = [(xR - xI) * (yR + yI)] / [(xR - xI) * (xR + xI)] = (yR + yI) / (xR + xI)
#    = yDft / xDft

#Cxy = |Pxy|^2 / [Pxx * Pyy]

#Cxy = 1, then
#Pyy = |Pxy|^2 / Pxx
#|Pxy|^2 = [Pxx * Pyy]
# Txy = Pxy / Pxx
# |Txy| = |Pxy| / |Pxx|

gainTuzNull_mag, phaseTuzNull_deg = FreqTrans.GainPhase(TuzNull_N, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTuzNull_dB = mag2db(gainTuzNull_mag)


#%%
#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, scaleType = 'spectrum', smooth = ('box', 3), winType = ('tukey', 0.1), detrendType = 'Linear')

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps
optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
#freq_rps, Txy, Cxy, Pxx, Pyy, Pxy = FreqTrans.FreqRespFuncEst(uExc, z, optSpec)
freq_Func_rps, Tuz_Func, Cuz_Func, Puu_Func, Pzz_Func, Puz_Func, TuzUnc_Func, PuuNull_Func, PzzNull_Func = FreqTrans.FreqRespFuncEstNoise(uExc, z, optSpec)
freq_hz = freq_Func_rps * rps2hz

T_Func = Tuz_Func
TUnc_Func = np.abs(TuzUnc_Func)

# Nominal Response
gain_Func_mag, phase_Func_deg = FreqTrans.GainPhase(T_Func, magUnit = 'mag')
gain_Func_db = mag2db(gain_Func_mag)
phase_Func_deg = np.unwrap(phase_Func_deg * deg2rad) * rad2deg

# Uncertain Response
gainUnc_Func_mag = FreqTrans.Gain(TUnc_Func, magUnit = 'mag')
gainUnc_Func_dB = mag2db(gainUnc_Func_mag)


#%%
freqLin_rps = freqExc_rps
freqLin_hz = freqLin_rps * rps2hz

# OL Response
TuyLinReal, TuyLinImag, _ = control.nyquist(sysPlant, omega = freqLin_rps, Plot = False)
TuyLin = TuyLinReal + 1j * TuyLinImag

TmzLinReal, TmzLinImag, _ = control.nyquist(sysNoise, omega = freqLin_rps, Plot = False)
TmzLin = TmzLinReal + 1j * TmzLinImag

# Just the plant
numFreqLin = len(freqLin_rps)
PuuLin = np.ones_like(TuyLin, dtype = float) * Puu.sum() / len(freqLin_hz)
PyyLin = np.abs(TuyLin)**2 * PuuLin

# Just the noise model
PmmLin = np.ones_like(TmzLin, dtype = float) * Pmm.sum() / len(freqLin_hz)
PnnLin = np.abs(TmzLin)**2 * PmmLin

PzzLin = PyyLin + PnnLin

gainTmzLin_mag, phaseTmzLin_deg = FreqTrans.GainPhase(TmzLin, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTmzLin_dB = mag2db(gainTmzLin_mag)
gainTuyLin_mag, phaseTuyLin_deg = FreqTrans.GainPhase(TuyLin, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTuyLin_dB = mag2db(gainTuyLin_mag)


#%%
plt.figure('Power via Null')
plt.subplot(2,1,1); plt.grid(True)
plt.semilogx(freqLin_hz, mag2db(PuuLin), '.k-', label = 'Linear Plant')
plt.semilogx(freq * rps2hz, mag2db(Puu_Func[0]), '*b-', label = 'Estimate at Excitation')
plt.semilogx(freq * rps2hz, mag2db(PuuNull_Func[0]), '.m--', label = 'Estimate of Null')
plt.legend()
plt.ylabel('Input Power (dB)')

plt.subplot(2,1,2); plt.grid(True)
plt.semilogx(freqLin_hz, mag2db(PyyLin), '.k-', label = 'Linear Plant')
plt.semilogx(freqLin_hz, mag2db(PnnLin), '*k--', label = 'Linear Noise ')
plt.semilogx(freqLin_hz, mag2db(PzzLin), '*k-.', label = 'Linear System')
plt.semilogx(freq * rps2hz, mag2db(Pzz_Func[0]), '*b-', label = 'Estimate at Excitation')
plt.semilogx(freq * rps2hz, mag2db(PzzNull_Func[0]), '.m--', label = 'Estimate of Null')
ylim = plt.ylim()
plt.legend()
plt.xlabel('Frequency (Hz)')
plt.ylabel('Output Power (dB)')

#plt.subplot(3,1,3); plt.grid(True)
#plt.semilogx(freqLin_hz, mag2db(PyyLin), '.k-', label = 'Output (Linear Plant Model)')
#plt.semilogx(freqLin_hz, mag2db(PzzLin), '*k-', label = 'Output (Linear Model)')
#
#PzzErrRatio = PzzNull_Func[0] / Pzz_Func[0]
#PzzErrMin = 1 - PzzErrRatio
#PzzErrMin_db = -mag2db(PzzErrMin)
#PzzErrMin_db[np.isnan(PzzErrMin_db)] = 1e6
#
#PzzErrMax = 1 + PzzErrRatio
#PzzErrMax_db = mag2db(PzzErrMax)
#
#PzzErr_db = [PzzErrMin_db, PzzErrMax_db]
#
#plt.errorbar(freq * rps2hz, mag2db(Pzz_Func[0].T), yerr = PzzErr_db, fmt = '*b-', label = 'Output (Estimate with Uncertainty)')
#plt.ylim(ylim)
#plt.legend();

#%%
plt.figure('Gain')
plt.subplot(3,1,1)
plt.semilogx(freqLin_rps     * rps2hz, gainTuyLin_dB, '.k-', label = 'Linear Plant')
#plt.semilogx(freqLin_rps     * rps2hz, gainTuzLin_dB, '*k--', label = 'Nominal (Linear System )')
#plt.semilogx(freqLin_rps * rps2hz, gainTmzLin_dB, '*k-.', label = 'Noise (Linear System)')
#plt.semilogx(freqChan_rps[0] * rps2hz, gain_Func_db[0], '*b-', label = 'Nominal (Estimate)')
#plt.semilogx(freq * rps2hz, gainUnc_Func_dB[0], '.g-', label = 'Noise (Noise Estimate)')
ylim = plt.ylim()

gainErrRatio = gainUnc_Func_mag[0] / gain_Func_mag[0]
gainErrMin = 1 - gainErrRatio
gainErrMin[gainErrMin < 0] = np.finfo(float).tiny
gainErrMin_db = -mag2db(gainErrMin)

gainErrMax = 1 + gainErrRatio
gainErrMax_db = mag2db(gainErrMax)

gainErr_db = [gainErrMin_db, gainErrMax_db]

plt.semilogx(freq * rps2hz, mag2db(gainUnc_Func_mag[0]), '.m-', label = 'Noise Estimate')
plt.errorbar(freq * rps2hz, gain_Func_db[0].T, yerr = gainErr_db, fmt = '*b-', label = 'Estimate with Uncertainty')


plt.ylim(ylim)
plt.legend(); plt.grid(True);

plt.subplot(3,1,2)

plt.semilogx(freqLin_rps     * rps2hz, phaseTuyLin_deg, '.k-', label = 'Nominal (Linear Plant )')
plt.semilogx(freq  * rps2hz, phase_Func_deg[0], '*b-', label = 'Nominal (Linear Plant )')

plt.grid(True);

plt.subplot(3,1,3)
plt.semilogx(freqLin_rps     * rps2hz, np.ones_like(gainTuyLin_mag), '.k-', label = 'Nominal (Linear Plant )')
plt.semilogx(freq  * rps2hz, Cuz_Func[0], '*b-', label = 'Nominal (Linear Plant )')
plt.ylim(0, 1.2)
plt.grid(True);

#%%
fig = 'Bode'
fig = FreqTrans.PlotBode(freqLin_rps * rps2hz, gainTuyLin_mag, phaseTuyLin_deg, coher_nd = np.ones_like(gainTuyLin_mag), gainUnc_mag = None, fig = fig, fmt = 'k', label='Linear Plant', dB = True)
fig = FreqTrans.PlotBode(freq * rps2hz, gain_Func_mag[0], phase_Func_deg[0], coher_nd = Cuz_Func[0], gainUnc_mag = gainUnc_Func_mag[0], fig = fig, fmt = '*b-', label='Estimate', dB = True)

#fig = FreqTrans.PlotBode(freq * rps2hz, gainUnc_Func_mag[0], phase_Func_deg[0], coher_nd = Cuz_Func[0], gainUnc_mag = gainUnc_Func_mag[0], fig = fig, fmt = '*b-', label='Estimate', dB = True)

ax = fig.get_axes()
ax[-1].set_ylim(0, 1.2)

