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
freqStepDes_rps = (10 / freqRate_hz) * hz2rps
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
scale = FreqTrans.PowerScale('spectrum', np.ones_like(uExc), freqRate_hz)

Suu = (uDft.conj() * uDft).real * scale
Suz = (uDft.conj() * zDft) * scale
Szz = (zDft.conj() * zDft).real * scale

Syy = (yDft.conj() * yDft).real * scale
Smm = (mDft.conj() * mDft).real * scale
Snn = (nDft.conj() * nDft).real * scale


#%%

# Nominal
#Tuz = Suz / Suu
Tuz = zDft / uDft

gainTuz_mag, phaseTuz_deg = FreqTrans.GainPhase(Tuz, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTuz_dB = mag2db(gainTuz_mag)

## Variance Technique

uDftAccum = uDftHist.cumsum(axis = -1)
zDftAccum = zDftHist.cumsum(axis = -1)

scaleHist = scale * (N/np.arange(1, N+1))**2
SuuHist = (uDftAccum.conj() * uDftAccum).real * scaleHist
SuzHist = (uDftAccum.conj() * zDftAccum) * scaleHist
SzzHist = (zDftAccum.conj() * zDftAccum).real * scaleHist

#plt.figure
#plt.subplot(2,1,1); plt.grid(True)
#plt.plot(mag2db(SuuHist)[0].T)
#plt.subplot(2,1,2); plt.grid(True)
#plt.plot(mag2db(SzzHist)[0].T)


SuuVar = SuuHist.var(axis = -1)
SzzVar = SzzHist.var(axis = -1)

#plt.figure
#plt.subplot(2,1,1); plt.grid(True)
#plt.plot(freqChan_rps[0], mag2db(SuuVar).T)
#plt.subplot(2,1,2); plt.grid(True)
#plt.plot(freqChan_rps[0], mag2db(SzzVar).T)


## Null Technique
freq = freqChan_rps[0]
freq_N = freqGap_rps

uDftNull_N, uDftNullHist_N = FreqTrans.CZTMat(uExc, zPtsNull)
nDft_N, nDftHist_N = FreqTrans.CZTMat(z, zPtsNull) # z at Null is n

#
SuuNull_N = (uDftNull_N.conj() * uDftNull_N).real * scale
SuuNull_E = FreqTrans.InterpPolar(SuuNull_N, freq_N, freq, 'linear')

uDftNull_E = FreqTrans.InterpPolar(uDftNull_N, freq_N, freq, 'linear')
SuuNull_E1 = (uDftNull_E.conj() * uDftNull_E).real * scale

# 
Snn_N = (nDft_N.conj() * nDft_N).real * scale
Snn_E = FreqTrans.InterpVal(Snn_N, freq_N, freq, 'linear')

nDft = FreqTrans.InterpPolar(nDft_N, freq_N, freq, 'linear')
Snn_E1 = (nDft.conj() * nDft).real * scale


# Null Cross Spectrum
numFreqNull = len(freq_N)
#uDftOnes_N = np.ones_like(nDft_N) / (numFreqNull * scale)**0.5
#SuuOnes_N = (uDftOnes_N.conj() * uDftOnes_N).real * scale

#SuzNull_N = uDftNull_N.conj() * nDft_N * scale
#SuuNull_N = uDftNull_N.conj() * uDftNull_N * scale
#TuzNull_N = SuzNull_N / SuuNull_N

Sun = uDft.conj() * nDft * scale


TuzUnc = nDft / uDft
TuzUnc_N = nDft_N / FreqTrans.InterpPolar(uDft, freq, freq_N, 'linear')

#Sxx = xDft.conj() * xDft * scale = (xR - xI) * (xR + xI) * scale
#Syy = yDft.conj() * yDft * scale = (yR - yI) * (yR + yI) * scale
#
#Sxy = xDft.conj() * yDft * scale = (xR - xI) * (yR + yI) * scale
#Txy = Sxy / Sxx = [xDft.conj() * yDft] / [xDft.conj() * xDft]
#    = [(xR - xI) * (yR + yI)] / [(xR - xI) * (xR + xI)] = (yR + yI) / (xR + xI)
#    = yDft / xDft

#Cxy = |Sxy|^2 / [Sxx * Syy]
#    = [Sxy.conj() * Sxy] / [Sxx * Syy]
#    = [(xR + xI) * (yR - yI) * (xR - xI) * (yR + yI)] / [(xR - xI) * (xR + xI) * (yR - yI) * (yR + yI)]
#    = 1 # Always =1 by definition, some alteration of Sxy, Sxx, or Szz required!!

#Cxy = 1, then
#Syy = |Sxy|^2 / Sxx = |Txy * Sxx|^2 / Sxx
# Sxx is real and >= 0, so: Syy = |Txy|^2 * Sxx

# Perturbed
# Syy_p = Syy + Snn
#       = |Sxy|^2 / Sxx + |Sxn|^2 / Sxx # Additive Output
#       = |Txy|^2 * Sxx + |Txn|^2 * Sxx # Additive Output
# Then: Txn = Sxn / Sxx = nDft / xDft

# Syy_p = Syy + Snn
#       = |Sxy|^2 / Sxx + |Syn|^2 / Syy # Multiplicative Output
#       = |Txy|^2 * Sxx + |Tyn|^2 * Syy # Multiplicative Output
# Then: Tyn = Syn / Syy = nDft / yDft

#%%
#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, scaleType = 'spectrum', smooth = ('box', 5), winType = ('tukey', 0.0), detrendType = 'Linear')

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps
optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
#freq_rps, Txy, Cxy, Sxx, Syy, Sxy = FreqTrans.FreqRespFuncEst(uExc, z, optSpec)
freq_Func_rps, Tuz_Func, Cuz_Func, Suu_Func, Szz_Func, Suz_Func, TuzUnc_Func, SuuNull_Func, Snn_Func = FreqTrans.FreqRespFuncEstNoise(uExc, z, optSpec)
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

TmnLinReal, TmnLinImag, _ = control.nyquist(sysNoise, omega = freqLin_rps, Plot = False)
TmnLin = TmnLinReal + 1j * TmnLinImag

# Just the plant
numFreqLin = len(freqLin_rps)
SuuLin = np.ones_like(TuyLin, dtype = float) * Suu.sum() / len(freqLin_hz)
SyyLin = np.abs(TuyLin)**2 * SuuLin
SuyLin = TuyLin * SuuLin

# Just the noise model, this is an exogenous addition!!
SmmLin = np.ones_like(TmnLin, dtype = float) * Smm.sum() / len(freqLin_hz)
SnnLin = np.abs(TmnLin)**2 * SmmLin
SmnLin = TmnLin * SmmLin


TunLin = (TmnLin * SmmLin) / SuuLin # Additive Output Equivalance
TynLin = (TmnLin * SmmLin) / SyyLin # Multiplicative Output Equivalance

SzzLin = SyyLin + SnnLin

gainTuyLin_mag, phaseTuyLin_deg = FreqTrans.GainPhase(TuyLin, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTuyLin_dB = mag2db(gainTuyLin_mag)
gainTmnLin_mag, phaseTmnLin_deg = FreqTrans.GainPhase(TmnLin, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTmnLin_dB = mag2db(gainTmnLin_mag)
gainTunLin_mag, phaseTunLin_deg = FreqTrans.GainPhase(TunLin, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTunLin_dB = mag2db(gainTunLin_mag)
gainTynLin_mag, phaseTynLin_deg = FreqTrans.GainPhase(TynLin, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)
gainTynLin_dB = mag2db(gainTynLin_mag)


#%%
plt.figure('Power via Null')
plt.subplot(2,2,1); plt.grid(True)
plt.semilogx(freqLin_hz, mag2db(SuuLin), '.k-', label = 'Linear Plant')
#plt.semilogx(freq * rps2hz, mag2db(Suu[0]), '*b-', label = 'Estimate at Excitation XX')
plt.semilogx(freq * rps2hz, mag2db(Suu_Func[0]), '*b-', label = 'Estimate')
plt.legend()
plt.ylabel('Spectra (dB)')
plt.title('Input at Excitation Set')

plt.subplot(2,2,2); plt.grid(True)
plt.semilogx(freqLin_hz, mag2db(SyyLin), '.k-', label = 'Linear Plant')
plt.semilogx(freqLin_hz, mag2db(SzzLin), '*k-.', label = 'Linear System')
#plt.semilogx(freq * rps2hz, mag2db(Szz[0]), '*g-.', label = 'Estimate at Excitation XX')
plt.semilogx(freq * rps2hz, mag2db(Szz_Func[0]), '*b-', label = 'Estimate')
ylim = plt.ylim()
plt.legend()
#plt.ylabel('Spectra (dB)')
plt.title('Output at Excitation Set')

plt.subplot(2,2,3); plt.grid(True)
#plt.semilogx(freq_N * rps2hz, mag2db(SuuNull_N[0]), '.m--', label = 'Estimate of Null XX')
plt.semilogx(freq * rps2hz, mag2db(SuuNull_Func[0]), '.m--', label = 'Estimate')
plt.semilogx(freqLin_hz, mag2db(np.ones_like(freqLin_hz)/N**2), '-k', label = 'DFT Accuracy')
plt.legend()
plt.xlabel('Frequency (Hz)')
plt.ylabel('Spectra (dB)')
plt.title('Input at Null Set')

plt.subplot(2,2,4); plt.grid(True)
plt.semilogx(freqLin_hz, mag2db(SnnLin), '*k--', label = 'Linear Noise')
#plt.semilogx(freq_N * rps2hz, mag2db(Snn_N[0]), '*g-.', label = 'Estimate of Null XX')
#plt.semilogx(freq * rps2hz, mag2db(Snn[0]), '*m-.', label = 'Estimate of Null XX')
plt.semilogx(freq * rps2hz, mag2db(Snn_Func[0]), '.m--', label = 'Estimatel')
plt.legend()
plt.xlabel('Frequency (Hz)')
#plt.ylabel('Spectra (dB)')
plt.title('Output at Null Set')


#%%
plt.figure('Gain')
plt.subplot(3,1,1)
plt.semilogx(freqLin_rps     * rps2hz, gainTuyLin_dB, '.k-', label = 'Linear Plant')
#plt.semilogx(freqLin_rps     * rps2hz, gainTuzLin_dB, '*k--', label = 'Linear System ')
#plt.semilogx(freqLin_rps * rps2hz, gainTmnLin_dB, '*k-.', label = 'Linear Noise (Exogenous)')
plt.semilogx(freqLin_rps * rps2hz, gainTunLin_dB, '*k-.', label = 'Linear Noise (Additive)')
#plt.semilogx(freqLin_rps * rps2hz, gainTynLin_dB, '*k-.', label = 'Linear Noise (Multiplicitive)')
plt.semilogx(freq * rps2hz, gain_Func_db[0], '*b-', label = 'Estimate')
plt.semilogx(freq * rps2hz, gainUnc_Func_dB[0], '.g-', label = 'Noise Estimate (Additive)')
ylim = plt.ylim()

gainErrRatio = gainUnc_Func_mag[0] / gain_Func_mag[0]
gainErrMin = 1 - gainErrRatio
gainErrMin[gainErrMin < 0] = np.finfo(float).tiny
gainErrMin_db = -mag2db(gainErrMin)

gainErrMax = 1 + gainErrRatio
gainErrMax_db = mag2db(gainErrMax)

gainErr_db = [gainErrMin_db, gainErrMax_db]

#plt.semilogx(freq * rps2hz, mag2db(gainUnc_Func_mag[0]), '.m-', label = 'Noise Estimate')
#plt.errorbar(freq * rps2hz, gain_Func_db[0].T, yerr = gainErr_db, fmt = '*b-', label = 'Estimate with Uncertainty')


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
#fig = 'Bode'
#fig = FreqTrans.PlotBode(freqLin_rps * rps2hz, gainTuyLin_mag, phaseTuyLin_deg, coher_nd = np.ones_like(gainTuyLin_mag), gainUnc_mag = None, fig = fig, fmt = 'k', label='Linear Plant', dB = True)
#fig = FreqTrans.PlotBode(freq * rps2hz, gain_Func_mag[0], phase_Func_deg[0], coher_nd = Cuz_Func[0], gainUnc_mag = gainUnc_Func_mag[0], fig = fig, fmt = '*b-', label='Estimate', dB = True)
#
##fig = FreqTrans.PlotBode(freq * rps2hz, gainUnc_Func_mag[0], phase_Func_deg[0], coher_nd = Cuz_Func[0], gainUnc_mag = gainUnc_Func_mag[0], fig = fig, fmt = '*b-', label='Estimate', dB = True)
#
#ax = fig.get_axes()
#ax[-1].set_ylim(0, 1.2)

