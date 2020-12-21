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

TLinUnc = gainNoise_mag * np.exp(1j * phaseNoise_deg)


#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', scaleType = 'spectrum', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.0), detrendType = 'Linear')

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

print(1/np.sum(SuuNull, axis = -1))

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


#%% Sigma Plot
# Linear Model SVD magnitude
svTLinNom_mag = FreqTrans.Sigma(TLinNom)
svTLinUnc_mag = FreqTrans.Sigma(TLinUnc)

# Estimate SVD magnitude
I2 = np.repeat([np.eye(2)], TEstNom.shape[-1], axis=0).T
svTEstNom_mag = FreqTrans.Sigma(I2 + TEstNom) # sigma(I + Li) = 1 / sigma(Si)
# svTEstNom_mag = 1 / FreqTrans.Sigma(SiEstNom)
svTEstUnc_mag = FreqTrans.Sigma(TEstUnc) # Uncertain SVD magnitude

# Estimation Coherence
cohEstMin = np.mean(TEstCoh, axis = (0, 1))

svTLinNomMin_mag = np.min(svTLinNom_mag, axis=0)
svTEstNomMin_mag = np.min(svTEstNom_mag, axis=0)

svTLinUncMax_mag = np.max(svTLinUnc_mag, axis=0) # Overly Conservative
svTEstUncMax_mag = np.max(svTEstUnc_mag, axis=0) # Overly Conservative

svTLinLower_mag = svTLinNomMin_mag - svTLinUncMax_mag
svTEstLower_mag = svTEstNomMin_mag - svTEstUncMax_mag

if False:
    fig = 10
    fig = FreqTrans.PlotSigma(freqLin_hz, svTLinNomMin_mag, lower = svTLinLower_mag, coher_nd = np.ones_like(freqLin_hz), fig = fig, color = 'k', label = 'Linear')
    # fig = FreqTrans.PlotSigma(freq_hz, svTEstNomMin_mag, coher_nd = cohTEstMin, color = 'b', fig = fig, label = 'Estimate')
    fig = FreqTrans.PlotSigma(freq_hz[0], svTEstNomMin_mag, lower = svTEstLower_mag, coher_nd = cohTEstMin, color = 'b', fig = fig, label = 'Estimate')
    fig = FreqTrans.PlotSigma(freqLin_hz, (0.4) * np.ones_like(freqLin_hz), linestyle = '--', color = 'r', fig = fig, label = 'Critical Limit')
    ax = fig.get_axes()
    # ax[0].set_xlim(0, 10)
    # ax[0].set_yscale('log')

    # ax[0].set_ylim(0, 2)
    # ax[0].set_title(r' ')
    ax[0].legend()
    # ax[0].set_ylabel(r'$\mathbf{\underbar\!\!\!\sigma } (I + L_i ( \omega ))$')

    ax[1].legend('')
    # ax[1].set_ylabel(r'$\gamma^2 (\omega) $')
    # ax[1].set_xlabel(r'$\omega [Hz] $')


#%% Vector Margin Plots
rCritTLinNom_mag, rCritTLinUnc_mag, rCritTLinMin_mag = FreqTrans.DistCrit(TLinNom, TLinUnc, typeUnc = 'circle')
rCritTEstNom_mag, rCritTEstUnc_mag, rCritTEstMin_mag = FreqTrans.DistCrit(TEstNom, TEstUnc, typeUnc = 'circle')

if True:
    numOut, numIn = TLinNom.shape[0:-1]
    fig, ax = plt.subplots(num=11, ncols=numOut, nrows=numIn, sharex=True, sharey=True)
    fig.tight_layout()

    ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

    for io in ioArray:
        [iOut, iIn] = io

        ax[iOut, iIn].plot(freqLin_hz, rCritTLinNom_mag[iOut, iIn].T, color = 'k', label = 'Linear Nominal')
        ax[iOut, iIn].fill_between(freqLin_hz, rCritTLinMin_mag[iOut, iIn].T, rCritTLinNom_mag[iOut, iIn].T, color = 'k', alpha = 0.25, label = 'Linear Uncertainty Lower')

        ax[iOut, iIn].plot(freq_hz[iIn], rCritTEstNom_mag[iOut, iIn].T, color = 'b', label = 'Estimate Nominal (MIMO) Lower')
        ax[iOut, iIn].fill_between(freq_hz[iIn], rCritTEstMin_mag[iOut, iIn].T, rCritTEstNom_mag[iOut, iIn].T, color = 'b', alpha = 0.25, label = 'Estimate Uncertainty (MIMO) Lower ')

        ax[iOut, iIn].plot(freq_hz[iIn, sigIndx[iIn]], rCritTEstNom_mag[iOut, iIn, sigIndx[iIn]].T, color = 'g', label = 'Estimate Nominal (SIMO)')
        ax[iOut, iIn].fill_between(freq_hz[iIn, sigIndx[iIn]], rCritTEstMin_mag[iOut, iIn, sigIndx[iIn]].T, rCritTEstNom_mag[iOut, iIn, sigIndx[iIn]].T, color = 'g', alpha = 0.25, label = 'Estimate Uncertainty (SIMO) Lower')

        ax[iOut, iIn].grid(True)
        ax[iOut, iIn].tick_params(axis = 'both')
        ax[iOut, iIn].set_xlabel('Freq [Hz]')
        ax[iOut, iIn].set_ylabel('Vector Margin [mag]')

    ax[0, 0].legend()


#%% Bode Plot
# Linear Model Gain and Phase
gainTLinNom_dB = mag2db(gainTLinNom_mag)
phaseTLinNom_deg = np.unwrap(phaseTLinNom_rad) * rad2deg
gainTLinUnc_dB, phaseTLinUnc_deg = FreqTrans.GainPhase(TLinUnc)

gainTLinNom_dB = mag2db(gainTLinNom_mag)
phaseTLinUnc_deg = np.unwrap(phaseTLinUnc_deg * deg2rad) * rad2deg


# Estimated Nominal Response
gainTEstNom_dB, phaseTEstNom_deg = FreqTrans.GainPhase(TEstNom)
phaseTEstNom_deg = np.unwrap(phaseTEstNom_deg * deg2rad) * rad2deg

# Estimation Uncertain Response
gainTEstUnc_mag = np.abs(TEstUnc)
gainTEstUnc_dB = FreqTrans.Gain(TEstUnc)


if True:
    numOut, numIn = TLinNom.shape[0:-1]
    fig, ax = plt.subplots(num=12, ncols=numOut, nrows=2*numIn)
    fig.tight_layout()

    ioArray = np.array(np.meshgrid([0,1], [0,1])).T.reshape(-1, 2)
    gainPlotArray = np.array(np.meshgrid([0,2], [0,1])).T.reshape(-1, 2)
    phasePlotArray = np.array(np.meshgrid([1,3], [0,1])).T.reshape(-1, 2)

    for iPlot, io in enumerate(ioArray):
        [iOut, iIn] = io
        gainPlot = gainPlotArray[iPlot]
        phasePlot = phasePlotArray[iPlot]

        [gOut, gIn] = gainPlot
        [pOut, pIn] = phasePlot

        ax[gOut, gIn].semilogx(freqLin_hz, gainTLinNom_dB[iOut, iIn], '-k', label='Linear Nominal')
        ax[gOut, gIn].semilogx(freqLin_hz, gainTLinUnc_dB[iOut, iIn], '--r', label='Linear Uncertainty')

        ax[gOut, gIn].semilogx(freq_hz[iIn], gainTEstNom_dB[iOut, iIn], '.b', label='Estimate Nominal [MIMO]')
        ax[gOut, gIn].semilogx(freq_hz[iIn], gainTEstUnc_dB[iOut, iIn], '.b', label='Estimate Uncertainty [MIMO]')

        ax[gOut, gIn].semilogx(freq_hz[iIn, sigIndx[iIn]], gainTEstNom_dB[iOut, iIn, sigIndx[iIn]], '.g', label='Estimate Nominal [SIMO]')
        ax[gOut, gIn].semilogx(freq_hz[iIn, sigIndx[iIn]], gainTEstUnc_dB[iOut, iIn, sigIndx[iIn]], '.g', label='Estimate Uncertainty [SIMO]')

        ax[pOut, pIn].semilogx(freqLin_hz, phaseTLinNom_deg[iOut, iIn], '-k', label='Linear Nominal')
        ax[pOut, pIn].semilogx(freq_hz[iIn], phaseTEstNom_deg[iOut, iIn], '.b', label='Estimate Nominal [MIMO]')
        ax[pOut, pIn].semilogx(freq_hz[iIn, sigIndx[iIn]], phaseTEstNom_deg[iOut, iIn, sigIndx[iIn]], '.g', label='Estimate Nominal [SIMO]')

        ax[gOut, gIn].grid(True); ax[pOut, pIn].grid(True)
        ax[gOut, gIn].tick_params(axis ='both'); ax[pOut, pIn].tick_params(axis ='both')

        # ax[pOut, pIn].set_ylim(-270, 90); ax.set_yticks([-270,-180,-90,0,90])
        ax[gOut, gIn].set_ylabel('Gain [dB]')
        ax[pOut, pIn].set_xlabel('Frequency [Hz]')
        ax[pOut, pIn].set_ylabel('Phase [deg]')

        ax[0, 0].legend()


#%% Nyquist Plot
TLinUnc = np.abs(TLinUnc)

# fig = 13
# FreqTrans.PlotNyquistMimo(LiLinNom, TUnc = LiLinUnc, fig = fig, linestyle = '-', color = 'k', fillType = 'fill', label = '')

if True:
    numOut, numIn = TLinNom.shape[0:-1]
    fig, ax = plt.subplots(num=13, ncols=numOut, nrows=numIn, sharex=True, sharey=True)
    fig.tight_layout()

    ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

    for io in ioArray:
        [iOut, iIn] = io

        ax[iOut, iIn].plot(TLinNom[iOut, iIn].real, TLinNom[iOut, iIn].imag, '-k', label = 'Linear')
        # FreqTrans.PlotUncPts(TLinNom[iOut, iIn], TLinUnc[iOut, iIn], ax[iOut, iIn], color='k')
        FreqTrans.PlotUncFill(TLinNom[iOut, iIn], TLinUnc[iOut, iIn], ax[iOut, iIn], color = 'k')

        ax[iOut, iIn].plot(TEstNom[iOut, iIn].real, TEstNom[iOut, iIn].imag, 'ob', label = 'Estimation Nominal (MIMO)')
        FreqTrans.PlotUncPts(TEstNom[iOut, iIn], TEstUnc[iOut, iIn], ax[iOut, iIn], color='b')
        # PlotFill(TEstNom[iOut, iIn], TEstUnc[iOut, iIn], ax[iOut, iIn], color = 'b')

        ax[iOut, iIn].plot(TEstNom[iOut, iIn, sigIndx[iIn]].real, TEstNom[iOut, iIn, sigIndx[iIn]].imag, 'og', label = 'Estimation Nominal (SIMO)')
        FreqTrans.PlotUncPts(TEstNom[iOut, iIn, sigIndx[iIn]], TEstUnc[iOut, iIn, sigIndx[iIn]], ax[iOut, iIn], color='g')
        # PlotFill(TEstNom[iOut, iIn, sigIndx[iIn]], TEstUnc[iOut, iIn, sigIndx[iIn]], ax[iOut, iIn], color = 'g')

        ax[iOut, iIn].grid(True)
        ax[iOut, iIn].tick_params(axis = 'both')
        # critPatch = patch.Ellipse((-1, 0), 2*0.4, 2*0.4, color='r', alpha=0.25)
        # ax[iOut, iIn].add_artist(critPatch)
        ax[iOut, iIn].plot(-1, 0, '+r')
        ax[iOut, iIn].set_xlabel('Real')
        ax[iOut, iIn].set_ylabel('Imag')

    ax[0,0].legend()

