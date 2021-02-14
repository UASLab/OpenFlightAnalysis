#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#%%
import numpy as np
from scipy import signal
from scipy import fft
import matplotlib.pyplot as plt

pi = np.pi

#%%

fBin = np.linspace(0, 20, 80)

Dapprox_mag =  10**-0.5 * 1/fBin
Bapprox_mag =  10**-(2/5) * (1/fBin)**2


#%%
from scipy.optimize import curve_fit

def func(b, a, s):
    W = 10**a * b**s     
    return W

lenX = 200
N = 2**12
Nhalf = int(N/2**2)

winTypeList = ['Boxcar', 'Bartlett', 'Hann']
#winTypeList = ['Boxcar']

#fig1 = plt.figure()
#ax1 = fig1.subplots(1,1)

fig2 = plt.figure()
ax2 = fig2.subplots(1,1)

for i, winType in enumerate(winTypeList):
    win = signal.get_window(winType.lower(), lenX)
    
#    ax1.plot(win, label = str(winType))
    
    freq = np.linspace(0, 1, N)
    bins = freq * lenX

    A = fft(win, N)
    M_mag = np.abs(A / A.max())
    M_dB = 20 * np.log10(M_mag)

    ax2.plot(bins[:Nhalf], M_dB[:Nhalf], linestyle = '-', label = str(winType))
    
    iPeak, _ = signal.find_peaks(M_dB)
    iPeak = iPeak[bins[iPeak] < 10]
    binPeak = bins[iPeak]
    MPeak_mag = M_mag[iPeak]
    MPeak_dB = M_dB[iPeak]
    
    color = ax2.get_lines()[-1].get_color()
#    ax2.plot(binPeak, MPeak_dB, '.', color = color, linestyle = 'None', label = str(winType) + ' Peaks')
    
    popt, pcov = curve_fit(func, binPeak, MPeak_mag)
        
    Mfit_mag = func(bins, *popt)
    Mfit_dB = 20 * np.log10(Mfit_mag)
    
    iSide1 = np.argwhere(bins > binPeak[0])[0][0]-5
    iStart = np.argwhere(Mfit_dB[:iSide1] < M_dB[:iSide1])[-1][0]
    ax2.plot(bins[iStart:], Mfit_dB[iStart:], color = color, linestyle = ':', label = str(winType) + ' Approx')
    
    print(str(winType) + 'binMin: ' + str(bins[iStart]) + ' a: ' + str(popt[0]) + ' s: ' + str(popt[1]))
    
        
#ax1.set_ylabel("Amplitude")
#ax1.set_xlabel("Sample")
#ax1.grid(True)
#ax1.set_ylim([0, 1.1])
#ax1.legend()

ax2.set_xlim([0, 10])
ax2.set_ylim([-70, 0])
ax2.grid(True)
ax2.set_ylabel("Normalized Power Magnitude [dB]")
ax2.set_xlabel("Normalized Frequency [bin]")
ax2.legend(loc = 'upper right')

fig2.set_size_inches([6.4,3.6])
fig2.tight_layout()

if True:
    FreqTrans.PrintPrettyFig(fig2, 'WindowFunc.pgf')
