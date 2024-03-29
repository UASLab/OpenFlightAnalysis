#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#%%
import numpy as np
from scipy import signal
from scipy import fft
import matplotlib.pyplot as plt
import control

# Hack to allow loading the Core package
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), "..")))
    path.insert(0, abspath(join(dirname(argv[0]), "..", 'Core')))

    del path, argv, dirname, abspath, join

from Core import FreqTrans

# Constants
pi = np.pi

mag2db = control.mag2db
db2mag = control.db2mag

#%%
from scipy.optimize import curve_fit

def func(b, a, s):
    W = 10**a * b**s
    return W

lenX = 20
N = 2**12
Nhalf = int(N/2**1)

winTypeList = ['Boxcar', 'Bartlett', 'Hann']
winLabelList = ['Dirichlet', 'Bartlett', 'Hann']
colorList = ['b', 'g', 'r']
# winTypeList = ['Boxcar']

# fig1 = plt.figure()
# ax1 = fig1.subplots(1,1)

fig2 = plt.figure()
ax2 = fig2.subplots(1,1)

for i, winType in enumerate(winTypeList):
    winLabel = winLabelList[i]
    win = signal.get_window(winType.lower(), lenX)

    # ax1.plot(win, label = str(winType))

    freq = np.linspace(0, 1, N)
    bins = freq * lenX

    A = fft.fft(win, N)
    M_mag = np.abs(A) / np.nanmax(np.abs(A))
    M_dB = mag2db(M_mag)

    ax2.plot(bins[:Nhalf], M_dB[:Nhalf], linestyle = '-', color = colorList[i], label = str(winLabel))

    iPeak, _ = signal.find_peaks(M_dB)
    iPeak = iPeak[bins[iPeak] < lenX/2]
    binPeak = bins[iPeak]
    MPeak_mag = M_mag[iPeak]
    MPeak_dB = M_dB[iPeak]

    color = ax2.get_lines()[-1].get_color()
    # ax2.plot(binPeak, MPeak_dB, '.', color = color, linestyle = 'None', label = str(winType) + ' Peaks')

    popt, pcov = curve_fit(func, binPeak, MPeak_mag)

    Mfit_mag = func(bins, *popt)
    Mfit_dB = mag2db(Mfit_mag)

    iSide1 = np.argwhere(bins > binPeak[0])[0][0]-20
    iStart = np.argwhere(Mfit_dB[:iSide1] < M_dB[:iSide1])[-1][0]
    ax2.plot(bins[iStart:], Mfit_dB[iStart:], color = color, linestyle = ':', label = str(winLabel) + ' Approx')

    print(str(winLabel) + 'binMin: ' + str(bins[iStart]) + ' a: ' + str(popt[0]) + ' s: ' + str(popt[1]))


# ax1.set_ylabel("Amplitude")
# ax1.set_xlabel("Sample")
# ax1.grid(True)
# ax1.set_ylim([0, 1.1])
# ax1.legend()

ax2.set_xlim([0, 10])
ax2.set_ylim([-80, 10])
ax2.grid(True)
ax2.set_ylabel("Normalized Power Magnitude [dB]")
ax2.set_xlabel("Normalized Bin")
ax2.legend(loc = 'upper right', framealpha  = 1)

fig2.set_size_inches([6.4,3.6])
fig2.tight_layout()

if False:
    FreqTrans.PrintPrettyFig(fig2, 'WindowFunc.pgf')
