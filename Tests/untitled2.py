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


# plt.rcParams.update({
#     "text.usetex": True,
#     "font.family": "serif",
#     "font.serif": ["Palatino"],
#     "font.size": 10
# })

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

freqLin_hz = np.linspace(1e-1, 1e1, 400)
freqLin_rps = freqLin_hz * hz2rps

plantK11 = 1.0 ; plantWn11 = 3 * hz2rps; plantD11 = 0.2;
plantK21 = 0.25; plantWn21 = 4 * hz2rps; plantD21 = 0.1;
plantK12 = 0.5 ; plantWn12 = 5 * hz2rps; plantD12 = 0.3;
plantK22 = 1.0 ; plantWn22 = 6 * hz2rps; plantD22 = 0.4;


sysPlant = control.tf([[[0, 0, plantK11 * plantWn11**2], [0, 0, plantK21 * plantWn21**2]],
                       [[0, 0, plantK12 * plantWn12**2], [0, 0, plantK22 * plantWn22**2]]],
                      [[[1, 2.0*plantD11*plantWn11, plantWn11**2], [1, 2.0*plantD21*plantWn21, plantWn21**2]],
                       [[1, 2.0*plantD12*plantWn12, plantWn12**2], [1, 2.0*plantD22*plantWn22, plantWn22**2]]])

# Plant Response
gainTLinNom_mag, phaseTLinNom_rad, _ = control.freqresp(sysPlant, omega = freqLin_rps)
TLinNom = gainTLinNom_mag * np.exp(1j * phaseTLinNom_rad)


#%% Excitation
numExc = 2
numCycles = 3
ampInit = 1
ampFinal = 1
freqMinDes_rps = 0.1 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 10.1 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (10/freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

time_s = time_s[0:-5]

# Generate Schroeder MultiSine Signal
ampExcit_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
uExc, phaseElem_rad, sigExcit = GenExcite.MultiSine(freqExc_rps, ampExcit_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')
uExc = uExc / np.std(uExc)
uPeak = np.mean(GenExcite.PeakFactor(uExc) * np.std(uExc))**2

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]
freqChan_hz = freqChan_rps * rps2hz

# Simulate the excitation through the system
_, y, _ = control.forced_response(sysPlant, T = time_s, U = uExc)


# Plant-Output Noise
noiseK11 = (1/8) * np.abs(sysPlant.dcgain())[0, 0]; noiseWn11 = 6 * hz2rps; noiseD11 = 0.1;
noiseK12 = (1/8) * np.abs(sysPlant.dcgain())[0, 1]; noiseWn12 = 6 * hz2rps; noiseD12 = 0.1;
noiseK21 = (4/8) * np.abs(sysPlant.dcgain())[1, 0]; noiseWn21 = 4 * hz2rps; noiseD21 = 1.0;
noiseK22 = (4/8) * np.abs(sysPlant.dcgain())[1, 1]; noiseWn22 = 4 * hz2rps; noiseD22 = 1.0;

sysN = control.tf([[[-noiseK11, 0, noiseK11 * noiseWn11**2], [-noiseK12, 0, noiseK12 * noiseWn12**2]],
                   [[-noiseK21, 0, noiseK21 * noiseWn21**2], [-noiseK22, 0, noiseK22 * noiseWn22**2]]],
                  [[[1, 2.0*noiseD11*noiseWn11, noiseWn11**2], [1, 2.0*noiseD12*noiseWn12, noiseWn12**2]],
                   [[1, 2.0*noiseD21*noiseWn21, noiseWn21**2], [1, 2.0*noiseD22*noiseWn22, noiseWn22**2]]])


np.random.seed(0)
mSigma = 1.0
m = np.random.normal([0.0], [mSigma], size = uExc.shape)
_, n, _ = control.forced_response(sysN, T = time_s, U = m)


# Output with Noise
z = y + n

# Linear Noise response
gainNoise_mag, phaseNoise_deg, _ = control.freqresp(sysN * (mSigma / uPeak), omega = freqLin_rps)
gainNoise_dB = 20 * np.log10(gainNoise_mag)
NLin = gainNoise_mag * np.exp(1j * phaseNoise_deg)

TLinUnc = NLin



#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', scaleType = 'density', freqRate = freqRate_rps, smooth = ('box', 1), winType = ('tukey', 0.0), detrendType = 'Linear')

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps


optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
#freq_rps, Tuz, Cuz, Suu, Szz, Suz = FreqTrans.FreqRespFuncEst(uExc, z, optSpec)
freq_rps, Tuz, Cuz, Suu, Szz, Suz, Tun, SuuNull, Snn = FreqTrans.FreqRespFuncEstNoise(uExc, z, optSpec)
freq_hz = freq_rps * rps2hz
# [4.51287801e-26 8.57989266e-22]
#print(np.sum(SuuNull, axis = -1)) # [1.06898819e-25 9.65232964e-23]
#print(np.sum(Suu, axis = -1))
#print(np.sum(Szz, axis = -1))
#print(np.sum(Snn, axis = -1))

TEstNom = Tuz
TEstUnc = Tun
TEstCoh = Cuz

# Nominal Response
gainTEstNom_dB, phaseTEstNom_deg = FreqTrans.GainPhase(TEstNom)
phaseTEstNom_deg = np.unwrap(phaseTEstNom_deg * deg2rad) * rad2deg

# Uncertain Response
gainTEstUnc_mag = np.abs(TEstUnc)
gainTEstUnc_dB, _ = FreqTrans.GainPhase(TEstUnc)

ampU = (1.0 / 1.0) * np.ones_like(freqLin_hz) / len(freqLin_rps)
SuuLin = np.sqrt(ampU)

ampM = mSigma * np.ones_like(freqLin_hz) / len(freqLin_rps)
SmmLin = np.sqrt(ampM)

SnnLin = np.abs(TLinUnc)**2 * SmmLin
SyyLin = np.abs(TLinNom)**2 * SuuLin

SzzLin = SyyLin + SnnLin

