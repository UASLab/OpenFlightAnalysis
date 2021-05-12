# -*- coding: utf-8 -*-
"""
Created on Sun Mar  7 12:32:14 2021

@author: Chris
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


# Constants
pi = np.pi

hz2rps = 2*pi
rps2hz = 1/hz2rps

rad2deg = 180/pi
deg2rad = 1/rad2deg


#%%
numChan = 1
freqRate_hz = 1000;
timeDur_s = 1.0
numCycles = 1

freqMinDes_rps = (1/timeDur_s) * hz2rps * np.ones(numChan)
freqMaxDes_rps = 10 * hz2rps *  np.ones(numChan)
freqStepDes_rps = 1 * hz2rps
methodSW = 'zip' # "zippered" component distribution

## Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)
timeDur_s = time_s[-1] - time_s[0]

## Generate Schroeder MultiSine Signal
ampElem_nd = np.ones_like(freqExc_rps) ## Approximate relative signal amplitude, create flat
# ampElem_nd = np.sqrt(2 / freqExc_rps**3) # Pwr/J = 0.5 * A**2 * freq_rps**3
sigList, phaseElem_rad, sigElem = GenExcite.MultiSine(freqExc_rps, ampElem_nd, sigIndx, time_s, costType = 'Schroeder', phaseInit_rad = 0, boundPhase = True, initZero = True, normalize = 'rms');
sigPeakFactor = GenExcite.PeakFactor(sigList)

sigElem *= 0.3

numPts = time_s.shape[-1]
angle = np.linspace(0, freqMinDes_rps[0], numPts)

xDir = np.cos(angle)
yDir = np.sin(angle)

xCent = 0.0
yCent = 0.0
R = 1

xBase = xCent + R * xDir
yBase = yCent + R * yDir

# Plot
plt.figure()
plt.plot(time_s, sigElem.T)

xWaveList = []
yWaveList = []
for i, d in enumerate(sigElem):
  xWave = xBase + d * xDir
  yWave = yBase + d * yDir

  xWaveList.append(xWave)
  yWaveList.append(yWave)

plt.figure()
plt.plot(xBase, yBase, color = 'k', linestyle = ':')
plt.plot(np.array(xWaveList).T, np.array(yWaveList).T, color = 'gray')
