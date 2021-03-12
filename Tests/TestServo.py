"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019-2021 Regents of the University of Minnesota
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
numChan = 1
freqRate_hz = 50;
timeDur_s = 10.0
numCycles = 3

freqMinDes_rps = (1/timeDur_s) * hz2rps * np.ones(numChan)
freqMaxDes_rps = 15 * hz2rps *  np.ones(numChan)
freqStepDes_rps = (10 / 50) * hz2rps
methodSW = 'zip' # "zippered" component distribution

## Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
timeDur_s = time_s[-1] - time_s[0]

## Generate Schroeder MultiSine Signal
ampElem_nd = np.ones_like(freqExc_rps) ## Approximate relative signal amplitude, create flat
sigList, phaseElem_rad, sigElem = GenExcite.MultiSine(freqExc_rps, ampElem_nd, sigIndx, time_s, costType = 'Schroeder', phaseInit_rad = 0, boundPhase = True, initZero = True, normalize = 'rms');
sigPeakFactor = GenExcite.PeakFactor(sigList)

if 0:
    sigList[0], _, _ = GenExcite.Chirp(freqMinDes_rps, freqMaxDes_rps, time_s)


#%%
# ampList = np.array([3, 5, 10, 20])
ampList = np.arange(1.0, 40.1, 0.5)
# ampList = np.array([1, 2, 3, 4, 5, 10, 15])

# Create Servo Object (HiTec HS-225BB)
freqNat_hz = 6.0
freqNat_rps = freqNat_hz * hz2rps
objServo = Servo.Servo(1/freqRate_hz, freqNat_rps = freqNat_rps, damp = 0.8)
objServo.freeplay = 1.0 # @ 2.0
objServo.timeDelay_s = 50 / 1000 # this ends up rounded to an integer (timeDelay_s * freqRate_hz)
# objServo.cmdLim = 20
objServo.pLim = 20
objServo.vLim = 560 # (28 / sigPeakFactor) * (freqNat_rps)
# objServo.aLim = 46993 # (28 / sigPeakFactor) * (freqNat_rps**2)
# objServo.pwrLim = 2e6 # pwr = 0.5 * J * (amp**2 * freqNat_rps**3)

pCmdList = []
pOutList = []
for amp in ampList:
    pCmd = amp * sigList[0]
    pOut = np.zeros_like(pCmd)
    p = 0; v = 0; a = 0; av = 0
    objServo.Start() # Resets the servo states
    for i, s in enumerate(pCmd):
        pOut[i] = objServo.Update(s)
        p = max(p, np.abs(objServo.pOut))
        v = max(v, np.abs(objServo.v))
        a = max(a, np.abs(objServo.a))
        av = max(av, a*v)

    print(amp, p, v, a, av)

    pCmdList.append(pCmd)
    pOutList.append(pOut)

    if True:
        plt.figure(1)
        plt.plot(time_s, pCmd, time_s, pOut)


#%% Plot the Excitation Spectrum
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_hz * hz2rps, freq = freqExc_rps, smooth = ('box', 3), winType = 'hann')

plt.figure(2)
TxyList = []
CxyList = []
PxxList = []
PyyList = []
PxyList = []
for i, pOut in enumerate(pOutList):
    pCmd = pCmdList[i]
    freq_rps, Txy, Cxy, Pxx, Pyy, Pxy = FreqTrans.FreqRespFuncEst(pCmd, pOut, optSpec)
    gain_dB, phase_deg = FreqTrans.GainPhase(Txy)
    freq_hz = freq_rps * rps2hz

    freq_hz = np.squeeze(freq_hz)
    gain_dB = np.squeeze(gain_dB)
    phase_deg = np.squeeze(phase_deg)
    Cxy = np.squeeze(Cxy)
    # Cxy = np.squeeze(np.abs(Cxy)**2)

    TxyList.append(Txy)
    CxyList.append(Cxy)
    PxxList.append(Pxx)
    PyyList.append(Pyy)
    PxyList.append(Pxy)

    ax1 = plt.subplot(3,1,1); plt.grid(True)
    ax1.semilogx(freq_hz, gain_dB, '-', label = 'Amplitude: ' + str(ampList[i]))
    ax2 = plt.subplot(3,1,2); plt.grid(True)
    ax2.semilogx(freq_hz, phase_deg, '-'); plt.ylim([-180, 180]);
    ax3 = plt.subplot(3,1,3); plt.grid(True)
    ax3.semilogx(freq_hz, Cxy, '-'); #plt.ylim([0, 1.2])

plt.subplot(3,1,1);
plt.legend()


#%%
TxyArray = np.array(TxyList)
CxyArray = np.array(CxyList)
PxxArray = np.array(PxxList)
PyyArray = np.array(PyyList)
PxyArray = np.array(PxyList)

# plt.figure()
# plt.plot(ampList, CxyArray)
# plt.plot(ampList, np.mean(CxyArray, axis=-1))
# plt.plot(ampList, np.min(CxyArray, axis=-1))
# plt.plot(ampList, np.max(CxyArray, axis=-1))
# plt.grid(True)


#%%
import numpy as np
import matplotlib.pyplot as plt

sat = 10
cmd = np.linspace(0, 11 * sat, 201)

def SatFunc(gam):
    # A is a vector of amplitudes
    # delta is a threshold
    gam = np.clip(gam, -1, 1)

    f = 2/np.pi * (np.arcsin(gam) + gam * np.sqrt(1 - gam**2))

    return f

delta = sat
A = cmd
f = SatFunc(delta/A)

plt.figure()
plt.plot(A/delta, f, '-', label = 'Saturation Function')
# plt.plot(A/delta, 1 - f, '-', label = 'Limiter Function')
plt.grid(True)

plt.xlim([0, 10])
plt.ylim([0, 1.1])
plt.xlabel('Input Amplitude [$A / \delta$]')
plt.ylabel('Output Amplitude [$|N(A)| / m$]')
plt.legend()

if False:
  fig.set_size_inches([6.4, 2.4])
  FreqTrans.PrintPrettyFig(fig, 'SaturationFunction.pgf')


#%%

def DF_Saturation(A, delta, m = 1):
    # Saturation (Gelb #7)
    n_r = m * SatFunc(delta/A)
    n_i = 0.0

    n = n_r + 1j * n_i

    return n

def DF_TimeDelay(A, omega, tDelay_s):
    # Time Delay (Gelb #33)
    n_r = np.cos(omega * tDelay_s)
    n_i = -np.sin(omega * tDelay_s)

    n = (n_r + 1j * n_i) * np.ones_like(A)

    return n

def DF_HardLimFreeplay(A, D, delta, m = 1):
    # Hard-limit with Freeplay (Gelb #42)
    # D is the position limit
    # delta is half the freeplay
    deltaPos = D/m + delta
    deltaNeg = D/m - delta

    A[A <= deltaPos] = np.nan

    n_r = m/2 * (SatFunc(deltaPos/A) + SatFunc(deltaNeg/A))
    n_i = -4*D*delta / (np.pi * A**2)

    n = n_r + 1j * n_i

    return n

def DF_CmdLimFreeplay(A, D, m = 1):
    # Cmd-limit with Freeplay (Gelb #46)

    n_r = m
    n_i = -4 * D / (np.pi * A)

    n = n_r + 1j * n_i

    return n

def DF_BacklashFrict(A, b):
    # Backlash (friction controlled) (Gelb #48)
    A[np.abs(A) <= b/2] = np.nan

    n_r = 1/2 * (1 + SatFunc(1 - b/A))
    n_i = -1/np.pi * (2*b/A - (b/A)**2)

    n = n_r + 1j * n_i

    return n

def DF_RateLimit(A, rateLimit, omega):
    # Rate Limit (Duda)
    omegaOnset = rateLimit / A
    omegaRatio = omegaOnset / omega

    n = (4/np.pi) * omegaRatio * np.exp(-1j * np.arccos(np.pi/2 * omegaRatio))

    return n

m = 1
freeplay = objServo.freeplay
cmdLim = objServo.cmdLim
defLim = objServo.pLim
vLim = objServo.vLim / sigPeakFactor

cmd = np.linspace(0, ampList.max(), 401)

# Saturation (Gelb #7)
A = np.copy(cmd)
nSat = DF_Saturation(A, defLim)

# Time Delay (Gelb #33)
A = np.copy(cmd)
omega = objServo.freqNat_rps * rps2hz
tDelay_s = objServo.timeDelay_s

nDelay = DF_TimeDelay(A, omega, tDelay_s)

# Hard-limit with Freeplay (#42)
delta = freeplay/2
D = defLim
A = np.copy(cmd)

nLim = DF_HardLimFreeplay(A, D, delta, m)
# nLim[np.abs(nLim) > 1] = 1.0

# Cmd-limit with Freeplay (Gelb #46)
D = m * freeplay/2
A = np.copy(cmd)

nCmdLim = DF_CmdLimFreeplay(A, D, m)

# Backlash (friction controlled) (Gelb #48)
b = freeplay

nBack = DF_BacklashFrict(A, b)

# Rate Limit
A = np.copy(cmd)

nRL = DF_RateLimit(A, vLim, freqNat_rps)


# Combine into a single DF response
nSat_temp = np.copy(nSat)
nSat_temp[np.isnan(nSat_temp)] = 1.0

nBack_temp = np.copy(nBack)
# nBack_temp[np.isnan(nBack_temp)] = 0.0

nRL_temp = np.copy(nRL)
nRL_temp[np.isnan(nRL_temp)] = 1.0

nDF = nSat_temp * nDelay * nBack_temp * nRL_temp

# Linear System
sysLin = control.tf([objServo.freqNat_rps**2], [1, 2*objServo.damp*objServo.freqNat_rps, objServo.freqNat_rps**2])
nLin = FreqTrans.FreqResp(sysLin, freqExc_rps)


#% Plot
fig = None
fig = FreqTrans.PlotGainType(cmd, np.abs(nSat), np.angle(nSat, deg=True), fig=fig, dB = False, label = 'Saturation Limit')
fig = FreqTrans.PlotGainType(cmd, np.abs(nDelay), np.angle(nDelay, deg=True), fig=fig, dB = False, label = 'Time Delay')
# fig = FreqTrans.PlotGainType(cmd, np.abs(nLim), np.angle(nLim, deg=True), fig=fig, dB = False, label = 'Hard Limit with Freeplay')
# fig = FreqTrans.PlotGainType(cmd, np.abs(nCmdLim), np.angle(nCmdLim, deg=True), fig=fig, dB = False, label = 'Command Limit')
fig = FreqTrans.PlotGainType(cmd, np.abs(nBack), np.angle(nBack, deg=True), fig=fig, dB = False, label = 'Backlash Limit')
fig = FreqTrans.PlotGainType(cmd, np.abs(nRL), np.angle(nRL, deg=True), fig=fig, dB = False, label = 'Rate Limit')
fig = FreqTrans.PlotGainType(cmd, np.abs(nDF), np.angle(nDF, deg=True), fig=fig, dB = False, label = 'Describing Function')

ax = fig.get_axes()
ax[0].set_xscale("linear")
# ax[0].set_ylabel("|NL/L|")
ax[1].set_xscale("linear")
ax[1].set_xlim(left = 0, right = 20)
ax[1].set_ylim(bottom = -90, top = 0)
ax[1].set_xlabel('Normalize Input Amplitude')

if False:
  fig.set_size_inches([6.4, 3.6])
  FreqTrans.PrintPrettyFig(fig, 'DescribingFunction.pgf')


#%% Plot
if False:
  plt.figure()
  plt.plot(np.angle(nSat, deg=True), mag2db(np.abs(nSat)), label = 'Saturation')
  plt.plot(np.angle(nDelay, deg=True), np.abs(nDelay), label = 'Time Delay')
  # plt.plot(np.angle(nLim, deg=True), np.abs(nLim), label = 'Hard Limit with Freeplay')
  # plt.plot(np.angle(nCmdLim, deg=True), np.abs(nCmdLim), label = 'Command Limit')
  plt.plot(np.angle(nBack, deg=True), mag2db(np.abs(nBack)), label = 'Backlash Limit')
  plt.plot(np.angle(nRL, deg=True), mag2db(np.abs(nRL)), label = 'Rate Limit')
  plt.plot(np.angle(nDF, deg=True), mag2db(np.abs(nDF)), label = 'Describing Function')
  for Txy in TxyList:
      plt.plot(np.angle(Txy[0], deg=True), mag2db(np.abs(Txy[0])), '.', label = 'Simulation')

  plt.grid(True)
  plt.legend()


#%%
ampPeakFactor = ampList / sigPeakFactor

gainDF_mag, phaseDF_deg = FreqTrans.GainPhase(nDF, magUnit = 'mag', unwrap = True)



# TxyArrayMax = np.nanmax(TxyArray.squeeze(), axis=-1)
# TxyArrayNorm = np.nanmean(TxyArray.squeeze().T, axis=0)
# TxyArrayNorm = np.nanmean(TxyArray.squeeze().T / TxyArrayMax, axis=0)

# gainTxyNorm_mag = FreqTrans.Gain(TxyArrayNorm.T, magUnit = 'mag')
# phaseTxyNorm_deg = FreqTrans.Phase(np.nanmax(TxyArray.squeeze(), axis=-1), phaseUnit = 'deg', unwrap = True)
# phaseTxy_deg = np.unwrap(np.angle(-TxyArray.squeeze())) * rad2deg

CxyArrayMax = np.nanmax(CxyArray.squeeze(), axis=-1)
# CxyArrayNorm = np.nanmean(CxyArray.squeeze().T, axis=0)
CxyArrayNorm = np.nanmean(CxyArray.squeeze().T / CxyArrayMax, axis=0)

gainTxyNorm_mag, phaseTxyNorm_deg = FreqTrans.GainPhase(CxyArrayNorm, magUnit = 'mag', phaseUnit = 'deg', unwrap = True)


# gainTxyMax_mag = np.nanmax(gainTxy_mag, axis=-1)
# gainTxyNorm_mag = np.nanmean(gainTxy_mag.T / gainTxyMax_mag, axis=-1)

# phaseTxyMax_deg = np.nanmax(phaseTxy_deg, axis=-1)
# phaseTxyNorm_deg = np.nanmean(phaseTxy_deg.T - phaseTxyMax_deg, axis=-1)

CxyArrayMin = np.nanmin(np.abs(CxyArray), axis=-1)
CxyArrayNorm = CxyArrayMin / np.nanmax(CxyArrayMin)


fig = None
fig = FreqTrans.PlotGainType(ampPeakFactor, gainTxyNorm_mag, phaseTxyNorm_deg, coher_nd = CxyArrayNorm**2, gainUnc_mag = None, fig = fig, dB = False, color='r', label='Normalized Estimates')
# fig = FreqTrans.PlotGainType(cmd, gainDF_mag, phaseDF_deg, coher_nd = None, gainUnc_mag = None, fig = fig, dB = False, color='k', label='Describing Function')

ax = fig.get_axes()
ax[0].set_xscale("linear")
ax[0].set_xlim([0,20])
# ax[1].set_ylim([-90,0])
# ax[-1].set_ylim(bottom = 0.0)
ax[-1].set_xlabel("Command Amplitude")

if False:
  fig.set_size_inches([6.4,4.8])
  FreqTrans.PrintPrettyFig(fig, 'ServoResponseBode.pgf')


#%%
if False:
  freqNat_hz = 10
  freqNat_rps = freqNat_hz * hz2rps

  t = np.arange(0, 2.2*pi, 1/freqRate_hz)
  x = np.sin(1 * t)

  objServo2 = Servo.Servo(1/freqRate_hz, freqNat_rps = freqNat_rps, damp = 0.8)
  objServo2.freeplay = 0.05
  # objServo2.cmdLim = 0.7
  objServo2.pLim = 0.5
  objServo2.vLim = 0.7

  y = np.zeros_like(x)
  objServo2.Start()
  for iElem, xElem in enumerate(x):
    y[iElem] = objServo2.Update(xElem)

  cmd = np.linspace(-2, 2, 41)

  A = np.copy(cmd)
  nSat = DF_Saturation(A, objServo2.pLim)
  nBack = DF_BacklashFrict(A, objServo2.freeplay)


  plt.figure()
  plt.subplot(2,1,1)
  plt.plot(t, x, ':k')
  plt.plot(t, y, 'b')
  plt.grid(True)
  plt.subplot(2,1,2)
  plt.plot(x, y, '-b')
  plt.plot(cmd, np.abs(nSat)*cmd, ':r')
  # plt.plot(cmd, np.abs(nBack)*cmd, ':m')

  iArrow = [80, 650, 1250]
  u = np.diff(x)
  v = np.diff(y)
  pos_x = (x[:-1] + u/2)[iArrow]
  pos_y = (y[:-1] + v/2)[iArrow]
  norm = np.sqrt(u**2+v**2)[iArrow]
  uNorm = u[iArrow] / norm
  vNorm = v[iArrow] / norm
  plt.quiver(pos_x, pos_y, uNorm, vNorm, angles="xy", color = 'b', zorder=5, pivot="mid")

  plt.grid(True)
