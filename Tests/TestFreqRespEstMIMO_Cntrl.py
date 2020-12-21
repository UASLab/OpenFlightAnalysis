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
import copy

# Hack to allow loading the Core package
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), "..")))
    path.insert(0, abspath(join(dirname(argv[0]), "..", 'Core')))

    del path, argv, dirname, abspath, join

from Core import GenExcite
from Core import FreqTrans
from Core.Systems import ConnectName


# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps

rad2deg = 180/np.pi
deg2rad = 1/rad2deg


def weighting(wb, m, a):
    """weighting(wb,m,a) -> wf
    wb - design frequency (where |wf| is approximately 1)
    m - high frequency gain of 1/wf; should be > 1
    a - low frequency gain of 1/wf; should be < 1
    wf - SISO LTI object
    """
    s = control.tf([1, 0], [1])
    return (s/m + wb) / (s + wb*a)


#%% Define a linear plant systems
freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps

#freqLin_rps = np.fft.rfftfreq(1500, 1/freqRate_rps)
freqLin_hz = np.linspace(1e-1, 1e1, 800)
freqLin_rps = freqLin_hz * hz2rps


if True:
    plantK11 = 1.0 ; plantWn11 = 1 * hz2rps;    plantD11 = 0.2;
    plantK21 = 0.25; plantWn21 = 2 * plantWn11; plantD21 = 0.1;
    plantK12 = 0.5 ; plantWn12 = 3 * plantWn11; plantD12 = 0.3;
    plantK22 = 1.0 ; plantWn22 = 4 * plantWn11; plantD22 = 0.4;
    
    sysPlant = control.tf([[[0, 0, plantK11 * plantWn11**2], [0, 0, plantK21 * plantWn21**2]],
                           [[0, 0, plantK12 * plantWn12**2], [0, 0, plantK22 * plantWn22**2]]], 
                          [[[1, 2.0*plantD11*plantWn11, plantWn11**2], [1, 2.0*plantD21*plantWn21, plantWn21**2]], 
                           [[1, 2.0*plantD12*plantWn12, plantWn12**2], [1, 2.0*plantD22*plantWn22, plantWn22**2]]])
    
    sysK = control.ss([], [], [], 0.5 * np.eye(2))
    
elif False: # SP Example #3.8
    plantK11 = 1.0; plantWn11 = np.sqrt(5);      plantD11 = 6 / (2 * plantWn11);
    plantK21 = 1.0; plantWn21 = 1.0 * plantWn11; plantD21 = 1.0 * plantD11;
    plantK12 = 1.0; plantWn12 = 1.0 * plantWn11; plantD12 = 1.0 * plantD11;
    plantK22 = 1.0; plantWn22 = 1.0 * plantWn11; plantD22 = 1.0 * plantD11;
    
    sysPlant = control.tf([[[0, 0, plantK11 * plantWn11**2], [0, 0, plantK21 * plantWn21**2]],
                           [[0, 0, plantK12 * plantWn12**2], [0, 0, plantK22 * plantWn22**2]]], 
                          [[[1, 2.0*plantD11*plantWn11, plantWn11**2], [1, 2.0*plantD21*plantWn21, plantWn21**2]], 
                           [[1, 2.0*plantD12*plantWn12, plantWn12**2], [1, 2.0*plantD22*plantWn22, plantWn22**2]]])
    
    # Based on Example 3.8 from Multivariable Feedback Control, Skogestad and Postlethwaite, 2nd Edition.
    wu = control.ss([], [], [], np.eye(2))
    wp1 = control.ss(weighting(wb = 0.25, m = 1.5, a = 1e-4))
    wp2 = control.ss(weighting(wb = 0.25, m = 1.5, a = 1e-4))
    wp = wp1.append(wp2)
    
    sysK, sysCL, (gam, rcond) = control.mixsyn(sysPlant, wp, wu)

else: # SP Section #3.7.1
    plantWn = 10
    den = [1, 0.0, plantWn**2]
    sysPlant = control.tf([[[1, -1.0 * plantWn**2], [1.0 * plantWn, 1.0 * plantWn]],
                           [[-1.0 * plantWn, -1.0 * plantWn], [1, -1.0 * plantWn**2]]],
                          [[den, den], [den, den]])
    
    sysK = control.ss([], [], [], 1.0 * np.eye(2))

sysPlant.InputNames = ['u1', 'u2']
sysPlant.OutputNames = ['y1', 'y2']


#%% Plant Disturbances
# Add the disturbances to the Plant model
sysD = control.ss([], [], [], [[1, 0, 1, 0], [0, 1, 0, 1]])
sysD.InputNames = ['uCtrl1', 'uCtrl2', 'd1', 'd2']
sysD.OutputNames = ['u1', 'u2']

sysN = control.ss([], [], [], [[1, 0, 1, 0], [0, 1, 0, 1]])
sysN.InputNames = ['y1', 'y2', 'n1', 'n2']
sysN.OutputNames = ['z1', 'z2']

# z = n + P * (u + d)
connectNames = sysPlant.InputNames + sysPlant.OutputNames
inKeep = sysD.InputNames + sysN.InputNames[2:]
outKeep = sysN.OutputNames
sysPlantDist = ConnectName([sysD, sysPlant, sysN], connectNames, inKeep, outKeep)


#%% Define a linear control system
sysK.InputNames = ['e1', 'e2']
sysK.OutputNames = ['uCtrl1', 'uCtrl2']

# Reference Input: e = r - z
sysR = control.ss([], [], [], [[1, 0, -1, 0], [0, 1, 0, -1]])
sysR.InputNames = ['r1', 'r2', 'z1', 'z2']
sysR.OutputNames = ['e1', 'e2']

connectNames = sysK.InputNames
inKeep = sysR.InputNames
outKeep = sysK.OutputNames
sysCtrl = ConnectName([sysR, sysK], connectNames, inKeep, outKeep)


#%% Closed-Loop
connectNames = sysPlantDist.OutputNames
inKeep = sysPlantDist.InputNames + sysCtrl.InputNames[:2]
outKeep = sysPlantDist.OutputNames + sysCtrl.OutputNames
sysOL = ConnectName([sysPlantDist, sysCtrl], connectNames, inKeep, outKeep)

connectNames = sysOL.OutputNames[2:]
inKeep = sysOL.InputNames[2:]
outKeep = sysOL.OutputNames
sysCL = ConnectName([sysOL], connectNames, inKeep, outKeep)

# OL Response 'd to uCtrl' (uExc will be input as 'd')
gainLin_mag, phaseLin_rad, _ = control.freqresp(sysOL[2:4, 2:4], omega = freqLin_rps)

TLin = gainLin_mag * np.exp(1j*phaseLin_rad)
gainLin_dB = 20 * np.log10(gainLin_mag)
phaseLin_deg = np.unwrap(phaseLin_rad) * rad2deg
rCritLin_mag = np.abs(TLin - (-1 + 0j))

sigmaLin_mag, _ = FreqTrans.Sigma(TLin)
sigmaLinMax_mag = np.max(sigmaLin_mag, axis = 0)
muLin_mag = 1 / sigmaLin_mag

# OL Noise 'n to z'
#gainLinNoise_mag, phaseLinNoise_rad, _ = control.freqresp(sysOL[0:2, 6:8], omega = freqLin_rps)

#TLinNoise = gainLinNoise_mag * np.exp(1j*phaseLinNoise_rad)
#gainLinNoise_dB = 20 * np.log10(gainLinNoise_mag)
#phaseLinNoise_deg = np.unwrap(phaseLinNoise_rad) * rad2deg
#rCritLinNoise_mag = np.abs(TLinNoise - (-1 + 0j))


#%% Excitation
numExc = 2
numCycles = 3
ampInit = 1
ampFinal = 1
freqMinDes_rps = 0.1 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 10.0 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (20/freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles, freqStepDes_rps, methodSW)

# Generate Schroeder MultiSine Signal
ampExcit_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
uExc, _, sigEx = GenExcite.MultiSine(freqExc_rps, ampExcit_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]
freqChan_hz = freqChan_rps * rps2hz

# Null Frequencies
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)
freqGap_hz = freqGap_rps * rps2hz


#%% Simulate the excitation through the system
# Reference Inputs
mean = 0.0; sigma = 0.0
wn = 3 * hz2rps; d = 0.2; den = [1, 2*d*wn, wn**2]
sysR = control.tf([[[sigma * wn**2], [sigma * wn**2]], [[sigma * wn**2], [sigma * wn**2]]], [[den, den], [den, den]])

np.random.seed(0)
rIn = np.random.normal(0.0, 1.0, size = uExc.shape)
_, r, _ = control.forced_response(sysR, T = time_s, U = rIn)

# Plant-Input Noise
mean = 0.0; sigma = 0.0
wn = 3 * hz2rps; d = 0.2; den = [1, 2*d*wn, wn**2]
sysD = control.tf([[[sigma * wn**2], [sigma * wn**2]], [[sigma * wn**2], [sigma * wn**2]]], [[den, den], [den, den]])

np.random.seed(12)
dIn = np.random.normal(0.0, 1.0, size = uExc.shape)
_, d, _ = control.forced_response(sysD, T = time_s, U = dIn)

# Plant-Output Noise
noiseK11 = 0.0     * plantK11; noiseWn11 = 6 * hz2rps; noiseD11 = 0.1;
noiseK21 = noiseK11 * plantK21; noiseWn21 = 5 * hz2rps; noiseD21 = 0.1;
noiseK12 = noiseK11 * plantK12; noiseWn12 = 4 * hz2rps; noiseD12 = 0.7;
noiseK22 = noiseK11 * plantK22; noiseWn22 = 3 * hz2rps; noiseD22 = 0.1;

sysNoise = control.tf([[[-noiseK11, 0, noiseK11 * noiseWn11**2], [-noiseK21, 0, noiseK21 * noiseWn21**2]],
                       [[-noiseK12, 0, noiseK12 * noiseWn12**2], [-noiseK22, 0, noiseK22 * noiseWn22**2]]],
                      [[[1, 2.0*noiseD11*noiseWn11, noiseWn11**2], [1, 2.0*noiseD21*noiseWn21, noiseWn21**2]], 
                       [[1, 2.0*noiseD12*noiseWn12, noiseWn12**2], [1, 2.0*noiseD22*noiseWn22, noiseWn22**2]]])

np.random.seed(23)
nIn = np.random.normal(0.0, 1.0, size = uExc.shape)
_, n, _ = control.forced_response(sysNoise, T = time_s, U = nIn)


# Time Response
inCL = np.concatenate((d + uExc, n, r))
_, outCL, _ = control.forced_response(sysCL, T = time_s, U = inCL)
z = outCL[0:2]
uRet = outCL[2:4]

# Ideal Noise Model Response
gainNoise_mag, phaseNoise_deg, freqNoise_rps = control.freqresp(-0.75 * sysNoise, omega = freqExc_rps)
freqNoise_hz = freqNoise_rps * rps2hz
gainNoise_dB = 20 * np.log10(gainNoise_mag)


#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear')
optSpecN = copy.deepcopy(optSpec)

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps
optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
#freq_rps, Txy, Cez, Pxx, Pyy, Pxy = FreqTrans.FreqRespFuncEst(uExc, uRet+uExc, optSpec)
freq_rps, Txy, Cxy, Pxx, Pyy, Pxy, TxyUnc, PxxNull, PyyNull = FreqTrans.FreqRespFuncEstNoise(uExc, uRet+uExc, optSpec)
freq_hz = freq_rps * rps2hz

optSpecN.freq = freqGap_rps
_, _, PxxNull_N  = FreqTrans.Spectrum(uExc, optSpecN)
freqN_rps, _, PuuNull_N  = FreqTrans.Spectrum(uRet+uExc, optSpecN)

# CP = inv((uRet+uExc) / uExc) - I = inv(Txy) - I

#  = Txy * inv(Txy + I)
CP = np.zeros_like(Txy, dtype = complex)
Eo = np.zeros_like(Txy, dtype = complex)
Mo = np.zeros_like(Txy, dtype = complex)
T = np.zeros_like(Txy, dtype = complex)
TUnc = np.zeros_like(Txy, dtype = float)
To = np.zeros_like(Txy, dtype = complex)
for i in range(T.shape[-1]):
    CP[...,i] = np.linalg.inv(Txy[...,i]) - np.eye(2)
    Eo[...,i] = TUnc[...,i] * np.linalg.inv(CP[...,i]) # Uncertainty as Output Multiplicative
    Mo[...,i] = CP[...,i] @ np.linalg.inv(np.eye(2) + CP[...,i])

    To[...,i] = TUnc[...,i] @ T[...,i]
    
    T[...,i] = Txy[...,i] @ np.linalg.inv(Txy[...,i] + np.eye(2))
    TUnc[...,i] = np.abs(TxyUnc[...,i] @ np.linalg.inv(Txy[...,i] + np.eye(2)))
    

# Nominal Response
gain_dB, phase_deg = FreqTrans.GainPhase(T)
phase_deg = np.unwrap(phase_deg * deg2rad) * rad2deg

# Singular Value
#sigmaNom_mag = FreqTrans.Sigma(T) # Singular Value Decomp
sigmaNom_mag, sigmaUnc_mag = FreqTrans.Sigma(T, TUnc) # Singular Value Decomp
sigmaCrit_mag = sigmaNom_mag + sigmaUnc_mag

gainUnc_dB = FreqTrans.Gain(TUnc)



from scipy import optimize


def specrad(M):
    return max(np.abs(np.linalg.eigvals(M)))

def mu_ubound(M):
    """ We use equation 8.87 and minimise directly
    """
    def objFunc(d, M):
        scale = 1 / d[0] # scale d such that d1 == 1 as in SP note 10 of 8.8.3
        D = np.diag(scale * d)
        Dinv = np.diag(1/(scale * d))
        M_scaled = D @ M @ Dinv
        
        s = np.linalg.svd(M_scaled, full_matrices=True, compute_uv = False)
        sMax = np.max(s, axis = 0)
        return sMax
    
    d = np.ones_like(M[0,...])
    sMax = np.zeros(M.shape[-1])
    for i in range(M.shape[-1]):
        res = optimize.minimize(objFunc, d[...,i], args = M[...,i])
        d[...,i] = res.x
        sMax[i] = res.fun
    
    return sMax

Mo = TUnc @ T.I
s = mu_ubound(M)




#%%
if False:
    plt.figure(0)
    iIn = 0; iOut = 0
    indxE = np.where(np.isin(freq_rps[iIn], freqChan_rps[0]))[0]
    plt.subplot(2,1,1)
    plt.semilogx(freq_hz[iIn, indxE], 20*np.log10(Pxx[iIn, indxE]), '*b-', label='P Excitation @ Excited')
    plt.semilogx(freq_hz[iIn], 20*np.log10(Pxx[iIn]), '*r--', label='P Excitation @ Excited Interp to Full')
    plt.grid(True); plt.legend()
    plt.subplot(2,1,2)
    plt.semilogx(freq_hz[iIn, indxE], 20*np.log10(Pyy[iOut, iIn, indxE]), '*b-', label='P Output @ Excited')
    plt.semilogx(freq_hz[iIn], 20*np.log10(Pyy[iOut, iIn]), '*r--', label='P Output @ Excited Interp to Full')
    #plt.semilogx(freqGap_hz, 20*np.log10(PxxNull_N[iIn]), '*g-', label='P Excitation @ Null')
    plt.semilogx(freqGap_hz, 20*np.log10(PuuNull_N[iOut]), '*g-', label='P Disturbance @ Null')
    plt.semilogx(freq_hz[iIn], 20*np.log10(PyyNull[iOut, iIn]), '*m--', label='P Output @ Null Interp to Full')
    plt.grid(True); plt.legend()


#%% Sigma Plot
Cmin = np.min(np.min(Cxy, axis = 0), axis = 0)
sigmaNomMax_mag = np.max(sigmaNom_mag, axis = 0)
sigmaUncMax_mag = np.max(sigmaNom_mag + sigmaUnc_mag, axis = 0)

sigmaUncMag_mag = sigmaUncMax_mag - sigmaNomMax_mag

fig = 20
fig = FreqTrans.PlotSigma(freqLin_hz, sigmaLin_mag, coher_nd = np.ones_like(freqLin_hz), fig = fig, fmt = 'k', label='Linear')
#fig = FreqTrans.PlotSigma(freq_hz, sigmaNom_mag, coher_nd = Cmin, fmt = 'bo', fig = fig, label = 'Estimation')
fig = FreqTrans.PlotSigma(freq_hz, sigmaNom_mag, err = sigmaUnc_mag, coher_nd = Cmin, fmt = 'bo', fig = fig, label = 'Estimation')
fig = FreqTrans.PlotSigma(freqLin_hz, (1/0.4) * np.ones_like(freqLin_hz), fmt = '--r', fig = fig, label = 'Critical Limit')
ax = fig.get_axes()
#ax[0].set_xscale('log')
ax[0].set_xlim(0, 10)
ax[0].set_yscale('log')
ax[0].set_ylim(1e-2, 1e1)


#%% Mu Plot
muNom_mag = 1 / sigmaNomMax_mag
muUncMin_mag = 1 / sigmaUncMax_mag

muUnc_mag = np.abs(muUncMin_mag - muNom_mag)

fig = 21
fig = FreqTrans.PlotSigma(freqLin_hz, muLin_mag, coher_nd = np.ones_like(freqLin_hz), fig = fig, fmt = 'k', label='Linear')
#fig = FreqTrans.PlotSigma(freq_hz[0], muNom_mag, coher_nd = Cmin, fmt = 'bo', fig = fig, label = 'Estimation')
fig = FreqTrans.PlotSigma(freq_hz[0], muNom_mag, err = muUnc_mag, coher_nd = Cmin, fmt = 'bo', fig = fig, label = 'Estimation')
fig = FreqTrans.PlotSigma(freqLin_hz, (0.4) * np.ones_like(freqLin_hz), fmt = '--r', fig = fig, label = 'Critical Limit')
ax = fig.get_axes()
ax[0].set_ylabel('Mu (1/SigmaMax(T))')
#ax[0].set_xscale('log')
ax[0].set_xlim(0, 10)
ax[0].set_yscale('log')
ax[0].set_ylim(1e-1, 1e2)


#%% Plot
plt.figure(1)
plt.tight_layout()

iIn = 0; iOut = 0
ax1 = plt.subplot(4,2,1); ax1.grid()
ax1.semilogx(freqLin_hz, gainLin_dB[iOut, iIn], label='Sys')
ax1.semilogx(freq_hz[0], gain_dB[iOut, iIn], '.', label='Sys Estimate')
#ax1.semilogx(freqNoise_hz, gainNoise_dB[iOut, iIn] - 6, label='Noise')
ax1.semilogx(freq_hz[0], gainUnc_dB[iOut, iIn], '.', label='Noise Estimate')
ax1.set_ylabel('Gain (dB)')
ax1.legend()

ax3 = plt.subplot(4,2,3, sharex = ax1); ax3.grid()
ax3.semilogx(freqLin_hz, phaseLin_deg[iOut, iIn])
ax3.semilogx(freq_hz[0], phase_deg[iOut, iIn], '.')
#ax3.set_ylim(-270, 90); ax3.set_yticks([-270,-180,-90,0,90])
#ax3.set_xlabel('Frequency (Hz)')
ax3.set_ylabel('Phase (deg)')

iIn = 0; iOut = 1
ax2 = plt.subplot(4,2,2); ax2.grid()
ax2.semilogx(freqLin_hz, gainLin_dB[iOut, iIn])
ax2.semilogx(freq_hz[0], gain_dB[iOut, iIn], '.')
#ax2.semilogx(freqNoise_hz, gainNoise_dB[iOut, iIn] - 6)
ax2.semilogx(freq_hz[0], gainUnc_dB[iOut, iIn], '.')
#ax2.set_ylabel('Gain (dB)')

ax4 = plt.subplot(4,2,4, sharex = ax2); ax4.grid()
ax4.semilogx(freqLin_hz, phaseLin_deg[iOut, iIn])
ax4.semilogx(freq_hz[0], phase_deg[iOut, iIn], '.')
#ax4.set_ylim(-270, 90); ax4.set_yticks([-270,-180,-90,0,90])
#ax4.set_xlabel('Frequency (Hz)')
#ax4.set_ylabel('Phase (deg)')

iIn = 1; iOut = 0
ax5 = plt.subplot(4,2,5); ax5.grid()
ax5.semilogx(freqLin_hz, gainLin_dB[iOut, iIn])
ax5.semilogx(freq_hz[0], gain_dB[iOut, iIn], '.')
#ax5.semilogx(freqNoise_hz, gainNoise_dB[iOut, iIn] - 6)
ax5.semilogx(freq_hz[0], gainUnc_dB[iOut, iIn], '.')
ax5.set_ylabel('Gain (dB)')

ax7 = plt.subplot(4,2,7, sharex = ax5); ax7.grid()
ax7.semilogx(freqLin_hz, phaseLin_deg[iOut, iIn])
ax7.semilogx(freq_hz[0], phase_deg[iOut, iIn], '.')
#ax7.set_ylim(-270, 90); ax7.set_yticks([-270,-180,-90,0,90])
ax7.set_xlabel('Frequency (Hz)')
ax7.set_ylabel('Phase (deg)')

iIn = 1; iOut = 1
ax6 = plt.subplot(4,2,6); ax6.grid()
ax6.semilogx(freqLin_hz, gainLin_dB[iOut, iIn])
ax6.semilogx(freq_hz[0], gain_dB[iOut, iIn], '.')
#ax6.semilogx(freqNoise_hz, gainNoise_dB[iOut, iIn] - 6)
ax6.semilogx(freq_hz[0], gainUnc_dB[iOut, iIn], '.')
#ax6.set_ylabel('Gain (dB)')

ax8 = plt.subplot(4,2,8, sharex = ax6); ax8.grid()
ax8.semilogx(freqLin_hz, phaseLin_deg[iOut, iIn])
ax8.semilogx(freq_hz[0], phase_deg[iOut, iIn], '.')
#ax8.set_ylim(-270, 90); ax8.set_yticks([-270,-180,-90,0,90])
ax8.set_xlabel('Frequency (Hz)')
#ax8.set_ylabel('Phase (deg)')


#%%
import matplotlib.patches as patch
plt.figure(2)
plt.tight_layout()

iIn = 0; iOut = 0
ax1 = plt.subplot(2,2,1); ax1.grid(True)
ax1.plot(TLin[iOut, iIn].real, TLin[iOut, iIn].imag)
ax1.plot(T[iOut, iIn].real, T[iOut, iIn].imag, '.')
for iNom, nom in enumerate(T[iOut,iIn]):
    unc = TUnc[iOut,iIn][iNom]
    uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color='b', alpha=0.25)
    ax1.add_artist(uncPatch)
ax1.plot(-1, 0, '+r')
critPatch = patch.Ellipse((-1, 0), 2*0.4, 2*0.4, color='r', alpha=0.25)
ax1.add_artist(critPatch)
ax1.set_xlabel('Real')
ax1.set_ylabel('Imag')

iIn = 0; iOut = 1
ax2 = plt.subplot(2,2,2, sharex = ax1, sharey = ax1); ax2.grid(True)
ax2.plot(TLin[iOut, iIn].real, TLin[iOut, iIn].imag)
ax2.plot(T[iOut, iIn].real, T[iOut, iIn].imag, '.')
for iNom, nom in enumerate(T[iOut,iIn]):
    unc = TUnc[iOut,iIn][iNom]
    uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color='b', alpha=0.25)
    ax2.add_artist(uncPatch)
ax2.plot(-1, 0, '+r')
critPatch = patch.Ellipse((-1, 0), 2*0.4, 2*0.4, color='r', alpha=0.25)
ax2.add_artist(critPatch)
ax2.set_xlabel('Real')
ax2.set_ylabel('Imag')

iIn = 1; iOut = 0
ax3 = plt.subplot(2,2,3, sharex = ax1, sharey = ax1); ax3.grid(True)
ax3.plot(TLin[iOut, iIn].real, TLin[iOut, iIn].imag)
ax3.plot(T[iOut, iIn].real, T[iOut, iIn].imag, '.')
for iNom, nom in enumerate(T[iOut,iIn]):
    unc = TUnc[iOut,iIn][iNom]
    uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color='b', alpha=0.25)
    ax3.add_artist(uncPatch)
ax3.plot(-1, 0, '+r')
critPatch = patch.Ellipse((-1, 0), 2*0.4, 2*0.4, color='r', alpha=0.25)
ax3.add_artist(critPatch)
ax3.set_xlabel('Real')
ax3.set_ylabel('Imag')

iIn = 1; iOut = 1
ax4 = plt.subplot(2,2,4, sharex = ax1, sharey = ax1); ax4.grid(True)
ax4.plot(TLin[iOut, iIn].real, TLin[iOut, iIn].imag)
ax4.plot(T[iOut, iIn].real, T[iOut, iIn].imag, '.')
for iNom, nom in enumerate(T[iOut,iIn]):
    unc = TUnc[iOut,iIn][iNom]
    uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color='b', alpha=0.25)
    ax4.add_artist(uncPatch)
ax4.plot(-1, 0, '+r')
critPatch = patch.Ellipse((-1, 0), 2*0.4, 2*0.4, color='r', alpha=0.25)
ax4.add_artist(critPatch)
ax4.set_xlabel('Real')
ax4.set_ylabel('Imag')
ax4.legend(['Linear', 'Sys Estimate'])