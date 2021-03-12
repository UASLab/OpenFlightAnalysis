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
plantK12 = 0.5 ; plantWn12 = 5 * hz2rps; plantD12 = 0.3;
plantK21 = 0.25; plantWn21 = 4 * hz2rps; plantD21 = 0.1;
plantK22 = 1.0 ; plantWn22 = 6 * hz2rps; plantD22 = 0.4;

sysPlant = control.tf([[[0, 0, plantK11 * plantWn11**2], [0, 0, plantK21 * plantWn21**2]],
                       [[0, 0, plantK12 * plantWn12**2], [0, 0, plantK22 * plantWn22**2]]],
                      [[[1, 2.0*plantD11*plantWn11, plantWn11**2], [1, 2.0*plantD21*plantWn21, plantWn21**2]],
                       [[1, 2.0*plantD12*plantWn12, plantWn12**2], [1, 2.0*plantD22*plantWn22, plantWn22**2]]])

# Plant Response
TLinNom = FreqTrans.FreqResp(sysPlant, freqLin_rps)


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

#time_s = time_s[0:-5]

# Generate Schroeder MultiSine Signal
ampExcit_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
uExc, phaseElem_rad, sigExcit = GenExcite.MultiSine(freqExc_rps, ampExcit_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'rms', costType = 'Schroeder')
uExc = (uExc.T * (1 / np.std(uExc, axis = -1))).T
uStd = np.std(uExc, axis = -1)
uPeak = np.mean(GenExcite.PeakFactor(uExc) * uStd)**2

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]
freqChan_hz = freqChan_rps * rps2hz

# Simulate the excitation through the system
_, y, _ = control.forced_response(sysPlant, T = time_s, U = uExc)


# Plant-Output Noise
noiseK11 = (2/8) * np.abs(sysPlant.dcgain())[0, 0]; noiseWn11 = 6 * hz2rps; noiseD11 = 0.1;
noiseK22 = (8/8) * np.abs(sysPlant.dcgain())[1, 1]; noiseWn22 = 4 * hz2rps; noiseD22 = 1.0;

sysN = control.tf([[[-noiseK11, 0, noiseK11 * noiseWn11**2], [0]],
                   [[0], [-noiseK22, 0, noiseK22 * noiseWn22**2]]],
                  [[[1, 2.0*noiseD11*noiseWn11, noiseWn11**2], [1]],
                   [[1], [1, 2.0*noiseD22*noiseWn22, noiseWn22**2]]])


np.random.seed(324)
mStd = [1.0, 1.0]
m = np.random.normal([0.0, 0.0], mStd, size = (len(time_s), 2)).T
_, n, _ = control.forced_response(sysN, T = time_s, U = m)

# Output with Noise
z = y + n

# Linear Noise response
sysTUnc = -sysN * control.ss([],[],[],np.outer(mStd, 1/uStd)) # sysTUnc is uExc -> n for comparison with the estimate, scale by mStd/uStd.

numFreq = len(freqExc_rps)
numTime = len(time_s)
scaleLinUnc = np.sqrt(numFreq / numTime)
TLinUnc = scaleLinUnc * FreqTrans.FreqResp(sysTUnc, freqLin_rps) # Noise STD scaled to Uncertainty by sqrt(numFreq)


#%% Estimate the frequency response function
optSpec = FreqTrans.OptSpect(dftType = 'czt', scaleType = 'density', freqRate = freqRate_rps, smooth = ('box', 3), winType = 'bartlett', detrendType = 'linear')

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

TEstNom = Tuz
TEstUnc = Tun
TEstCoh = Cuz

#%%
if False:
#%%
    optTemp = FreqTrans.OptSpect(dftType = 'czt', scaleType = 'density', freqRate = freqRate_rps, smooth = ('box', 3), winType = 'bartlett', detrendType = 'linear')
    optTemp.freq = freqGap_rps


    _, _, SnnN = FreqTrans.Spectrum(z, optTemp)


    [iOut, iIn] = [0,0]
    fig = plt.figure(); plt.grid(True)
    plt.plot(freq_hz[iIn], mag2db(Szz[iOut, iIn]), '.b', label = 'Estimate Nominal [MIMO]')
    plt.plot(freq_hz[iIn, sigIndx[iIn]], mag2db(Szz[iOut, iIn, sigIndx[iIn]]), '.r:', label = 'Estimate Nominal [SIMO]')
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Power Spectral Density [dB]')
    plt.legend()
    plt.xlim([1,5])
    plt.ylim([0,20])
    fig.set_tight_layout(True)
    fig.set_size_inches([6.4,2.4])

    if False:
        FreqTrans.PrintPrettyFig(fig, 'OpenMimoExcitationInterp.pgf')


    fig = plt.figure(); plt.grid(True)
    fig.tight_layout()
    plt.plot(freq_hz[iIn], mag2db(Snn[iOut, iIn]), '.b', label = 'Estimate Null [MIMO]');
    plt.plot(freqGap_rps*rps2hz, mag2db(SnnN[iOut]), '.g:', label = 'Estimate from Null')
    plt.ylabel('Power Spectral Density [dB]')
    plt.xlabel('Frequency [Hz]')
    plt.legend()
    plt.xlim([4,8])
    fig.set_tight_layout(True)
    fig.set_size_inches([6.4,2.4])

    if False:
        FreqTrans.PrintPrettyFig(fig, 'OpenMimoNullInterp.pgf')


#%% Sigma Plot
# Linear Model SVD magnitude
svTLinNom_mag = FreqTrans.Sigma(TLinNom)
svTLinUnc_mag = FreqTrans.Sigma(TLinUnc)

svTLinNomMin_mag = np.min(svTLinNom_mag, axis=0)
svTLinUncMax_mag = np.max(svTLinUnc_mag, axis=0) # Overly Conservative

cohTLin_mag = np.ones_like(TLinNom)
cohTLinMin = np.min(cohTLin_mag, axis = (0, 1))

# Estimate SVD magnitude
svTEstNom_mag = FreqTrans.Sigma(TEstNom)
svTEstUnc_mag = FreqTrans.Sigma(TEstUnc) # Uncertain SVD magnitude

svTEstNomMin_mag = np.min(svTEstNom_mag, axis=0)
svTEstUncMax_mag = np.max(svTEstUnc_mag, axis=0) # Overly Conservative

cohTEst_mag = TEstCoh # Estimation Coherence
cohTEstMin = np.min(cohTEst_mag, axis = (0, 1))

if False:
    fig = 10
    fig = FreqTrans.PlotSigma(freqLin_hz, svTLinNomMin_mag, svUnc_mag = svTLinUncMax_mag, coher_nd = cohTLinMin, fig = fig, color = 'k', label = 'Linear')
    fig = FreqTrans.PlotSigma(freq_hz[0], svTEstNomMin_mag, svUnc_mag = svTEstUncMax_mag, coher_nd = cohTEstMin, marker='.', color = 'r', fig = fig, label = 'Estimate (MIMO)')

    ax = fig.get_axes()
    handles, labels = ax[0].get_legend_handles_labels()
    handles = [handles[0], handles[2], handles[1], handles[3]]
    labels = [labels[0], labels[2], labels[1], labels[3]]
    ax[0].legend(handles, labels)


#%% Vector Margin Plots
vmTLinNom_mag, vmTLinUnc_mag, vmTLinMin_mag = FreqTrans.VectorMargin(TLinNom, TLinUnc, typeUnc = 'circle')
vmTEstNom_mag, vmTEstUnc_mag, vmTEstMin_mag = FreqTrans.VectorMargin(TEstNom, TEstUnc, typeUnc = 'circle')

numOut, numIn = TLinNom.shape[0:-1]
ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

if False:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = None
        fig = FreqTrans.PlotVectorMargin(freqLin_hz, vmTLinNom_mag[iOut, iIn], cohTLin_mag[iOut, iIn], vmTLinUnc_mag[iOut, iIn], fig = fig, linestyle='-', color='k', label='Linear Model')
        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn], vmTEstNom_mag[iOut, iIn], cohTEst_mag[iOut, iIn], vmTEstUnc_mag[iOut, iIn], fig = fig, linestyle='-', marker='.', color='r', label='Estimate [MIMO]')
        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn, sigIndx[iIn]], vmTEstNom_mag[iOut, iIn, sigIndx[iIn]], cohTEst_mag[iOut, iIn, sigIndx[iIn]], vmTEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, linestyle='-', marker='.', color='b', label='Estimate [SIMO]')

        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
        labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
        ax[0].legend(handles, labels)

        fig.suptitle('$u_' + str(iIn) + '$ to ' + '$z_' + str(iOut) + '$')
        fig.set_tight_layout(True)


#%% Bode Plot
# Linear Model Gain and Phase
gainTLinNom_mag, phaseTLinNom_deg = FreqTrans.GainPhase(TLinNom, magUnit = 'mag', unwrap = True)
gainTLinUnc_mag = FreqTrans.Gain(TLinUnc, magUnit = 'mag')

# Estimated Gain and Phase
gainTEstNom_mag, phaseTEstNom_deg = FreqTrans.GainPhase(TEstNom, magUnit = 'mag', unwrap = True)
gainTEstUnc_mag = FreqTrans.Gain(TEstUnc, magUnit = 'mag')

np.mean(gainTEstUnc_mag, axis = -1) / np.mean(gainTLinUnc_mag, axis = -1) # should be 1 if scaled correctly

if False:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = None
        fig = FreqTrans.PlotBode(freqLin_hz, gainTLinNom_mag[iOut, iIn], phaseTLinNom_deg[iOut, iIn], coher_nd = cohTLin_mag[iOut, iIn], gainUnc_mag = None, fig = fig, dB = True, color='k', label='Linear Nominal' + ' [$u_' + str(iIn+1) + '$ to ' + '$z_' + str(iOut+1) + '$]')
        fig = FreqTrans.PlotBode(freqLin_hz, gainTLinUnc_mag[iOut, iIn], None, coher_nd = None, gainUnc_mag = None, fig = fig, dB = True, color='k', linestyle='--', label='Linear Uncertainty' + ' [$u_' + str(iIn+1) + '$ to ' + '$z_' + str(iOut+1) + '$]')

        fig = FreqTrans.PlotBode(freq_hz[iIn], gainTEstNom_mag[iOut, iIn], phaseTEstNom_deg[iOut, iIn], coher_nd = cohTEst_mag[iOut, iIn], gainUnc_mag = None, fig = fig, dB = True, color='b', linestyle='None', marker='.', label='Estimate Nominal' + ' [$u_' + str(iIn+1) + '$ to ' + '$z_' + str(iOut+1) + '$]')
        fig = FreqTrans.PlotBode(freq_hz[iIn], gainTEstUnc_mag[iOut, iIn], None, coher_nd = None, gainUnc_mag = None, fig = fig, dB = True, color='r', linestyle='None', marker='.', label='Estimate Uncertainty' + ' [$u_' + str(iIn+1) + '$ to ' + '$z_' + str(iOut+1) + '$]')

#        fig = FreqTrans.PlotBode(freq_hz[iIn, sigIndx[iIn]], gainTEstNom_mag[iOut, iIn, sigIndx[iIn]], phaseTEstNom_deg[iOut, iIn, sigIndx[iIn]], coher_nd = cohTEst_mag[iOut, iIn, sigIndx[iIn]], gainUnc_mag = gainTEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, dB = True, color='b', linestyle='None', marker='.', label='Estimate [SIMO]')
#        fig = FreqTrans.PlotBode(freq_hz[iIn, sigIndx[iIn]], gainTEstUnc_mag[iOut, iIn, sigIndx[iIn]], None, coher_nd = None, gainUnc_mag = None, fig = fig, dB = True, color='b', linestyle='None', marker='.', label='Estimate [SIMO]')

        ax = fig.get_axes()
#        handles, labels = ax[0].get_legend_handles_labels()
#        handles = [(handles[0], handles[2]), (handles[1], handles[3])]
#        labels = [labels[0], labels[1]]
#        ax[0].legend(handles, labels)
#        ax[0].set_xlim([1,10])

#        fig.set_size_inches([6.4,4.8])
        if False:
            FreqTrans.PrintPrettyFig(fig, 'OpenMimoBode' + str(iOut+1) + str(iIn+1) + '.pgf')

#%% Nyquist Plot
TLinUnc = np.abs(TLinUnc)

if True:
    numOut, numIn = TLinNom.shape[0:-1]

    ioArray = np.array(np.meshgrid(np.arange(numOut), np.arange(numIn))).T.reshape(-1, 2)

    for io in ioArray:
        [iOut, iIn] = io

        fig = None
        fig = FreqTrans.PlotNyquist(TLinNom[iOut, iIn], TLinUnc[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear' + ' [$u_' + str(iIn+1) + '$ to ' + '$z_' + str(iOut+1) + '$]')
        fig = FreqTrans.PlotNyquist(TEstNom[iOut, iIn], TEstUnc[iOut, iIn], fig = fig, fillType = 'circle', marker='.', color = 'r', linestyle='None', label = 'Estimate' + ' [$u_' + str(iIn+1) + '$ to ' + '$z_' + str(iOut+1) + '$]')
#        fig = FreqTrans.PlotNyquist(TEstNom[iOut, iIn, sigIndx[iIn]], TEstUnc[iOut, iIn, sigIndx[iIn]], fig = fig, fillType = 'circle', marker='.', color = 'b', linestyle='None', label = 'Estimate (SIMO)')
        fig = FreqTrans.PlotNyquist(np.array([-1+0j]), fig = fig, fillType = 'circle', marker='+', color = 'r', linestyle='None')
        # fig = FreqTrans.PlotNyquist(np.array([-1+0j]), np.array([0.4]), fig = fig, fillType = 'circle', marker='+', color = 'r', linestyle='None')

        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [(handles[0], handles[2]), handles[1]]
        labels = [labels[0], labels[1]]
        ax[0].legend(handles, labels)

#        fig.set_size_inches([6.4,4.8])
        if False:
            FreqTrans.PrintPrettyFig(fig, 'OpenMimoNyquist' + str(iOut+1) + str(iIn+1) + '.pgf')


#%% Estimate the frequency response function time history
optSpec = FreqTrans.OptSpect(dftType = 'czt', scaleType = 'density', freqRate = freqRate_rps, smooth = ('box', 3), winType = 'bartlett', detrendType = 'linear')

# Excited Frequencies per input channel
optSpec.freq = freqChan_rps
optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
stride = 10
lenX = len(time_s) / stride
lenFreq = optSpec.freqInterp.shape[-1]

numSeg = int(lenX)

t_s = np.zeros(numSeg)
TuzList = np.zeros((numSeg, numOut, numIn, lenFreq), dtype=complex)
CuzList = np.zeros((numSeg, numOut, numIn, lenFreq))
SuuList = np.zeros((numSeg, numIn, lenFreq))
SzzList = np.zeros((numSeg, numOut, numIn, lenFreq))
SuzList = np.zeros((numSeg, numOut, numIn, lenFreq), dtype=complex)
TunList = np.zeros((numSeg, numOut, numIn, lenFreq), dtype=complex)
SuuNullList = np.zeros((numSeg, numIn, lenFreq))
SnnList = np.zeros((numSeg, numOut, numIn, lenFreq))


iStart = 0
lenCyc = int(len(time_s) / numCycles)
for iSeg in range(0, numSeg):
    iEnd = iSeg * stride
    print ( 100 * iSeg / numSeg )

    if iEnd > lenFreq-1:

        # Move the Start index once the End index has reached 2* cycle
#        if iEnd > (2 * lenCyc):
#            iStart = iEnd - lenCyc

        freq_rps, Tuz, Cuz, Suu, Szz, Suz, Tun, SuuNull, Snn = FreqTrans.FreqRespFuncEstNoise(uExc[:, iStart:iEnd+1], z[:, iStart:iEnd+1], optSpec)

        t_s[iSeg] = time_s[iEnd+1]
        TuzList[iSeg, ] = Tuz
        CuzList[iSeg, ] = Cuz
        SuuList[iSeg, ] = Suu
        SzzList[iSeg, ] = Szz
        SuzList[iSeg, ] = Suz
        TunList[iSeg, ] = Tun
        SuuNullList[iSeg, ] = SuuNull
        SnnList[iSeg, ] = Snn


    freq_hz = freq_rps * rps2hz


#%%
numFreqLin = len(freqLin_rps)
TLinNomList = np.zeros((numSeg, numOut, numIn, numFreqLin), dtype='complex')
TLinUncList = np.zeros((numSeg, numOut, numIn, numFreqLin), dtype='complex')
for iSeg in range(0, numSeg):
    TLinNomList[iSeg, ] = TLinNom
    TLinUncList[iSeg, ] = np.sqrt(numFreq / (iSeg*stride+1)) * TLinUnc


#%%
ones = np.ones_like(t_s)

gainThLinNom_mag = FreqTrans.Gain(TLinNomList, magUnit = 'mag')
gainThLinUnc_mag = FreqTrans.Gain(TLinUncList, magUnit = 'mag')
gainThEstNom_mag = FreqTrans.Gain(TuzList, magUnit = 'mag')
gainThEstUnc_mag = FreqTrans.Gain(TunList, magUnit = 'mag')

vmThLinNom_mag, vmThLinUnc_mag, vmThLinMin_mag = FreqTrans.VectorMargin(TLinNomList, TLinUncList, typeUnc = 'circle')
vmThEstNom_mag, vmThEstUnc_mag, vmThEstMin_mag = FreqTrans.VectorMargin(TuzList, TunList, typeUnc = 'circle')

if False:
    for iPlot, [iOut, iIn] in enumerate(ioArray):

        gainThLinNomMean = np.mean(np.abs(gainThLinNom_mag[:,iOut,iIn,:]), axis=-1)
        gainThLinUncMean = np.mean(np.abs(gainThLinUnc_mag[:,iOut,iIn,:]), axis=-1)
        gainThLinUncMin = np.mean(np.abs(gainThLinNom_mag[:,iOut,iIn,:]) - np.abs(gainThLinUnc_mag[:,iOut,iIn,:]), axis=-1) * ones

        gainThEstNomMean = np.mean(np.abs(gainThEstNom_mag[:,iOut,iIn,:]), axis=-1)
        gainThEstUncMean = np.mean(np.abs(gainThEstUnc_mag[:,iOut,iIn,:]), axis=-1)
        gainThEstUncMin = np.mean(np.abs(gainThEstNom_mag[:,iOut,iIn,:]) - np.abs(gainThEstUnc_mag[:,iOut,iIn,:]), axis=-1)

        cohEst = np.abs(CuzList[:,iOut,iIn,:])
        cohEst[cohEst < 0] = 0
        cohEst[cohEst > 1] = 1
        cohEstMean = np.mean(cohEst, axis=-1)
        cohEstStd = np.std(cohEst, axis=-1)
        cohEstMin = np.min(cohEst, axis=-1)


        fig = None
        fig = FreqTrans.PlotGainTemporal(t_s, gainThLinNomMean, None, coher_nd = ones, gainUnc_mag = gainThLinUncMean, fig = fig, dB = False, linestyle='-', color='k', label = 'Linear' + ' [$u_' + str(iIn+1) + '$ to ' + '$z_' + str(iOut+1) + '$]')
#        fig = FreqTrans.PlotGainTemporal(t_s, gainThLinUncMin, None, fig = fig, dB = False, linestyle=':', color='k', label = 'Linear - Lower')

        fig = FreqTrans.PlotGainTemporal(t_s, gainThEstNomMean, None, coher_nd = cohEstMean, gainUnc_mag = gainThEstUncMean, fig = fig, dB = False, linestyle='-', color='b', label = 'Estimate' + ' [$u_' + str(iIn+1) + '$ to ' + '$z_' + str(iOut+1) + '$]')
#        fig = FreqTrans.PlotGainTemporal(t_s, gainThEstUncMin, None, coher_nd = cohEstMin, fig = fig, dB = False, linestyle=':', color='r', label = 'Estimate - Lower')

        ax = fig.get_axes()
        ax[0].set_ylabel("Gain [mag]")

        handles, labels = ax[0].get_legend_handles_labels()
        handles = [(handles[0], handles[2]), (handles[1], handles[3])]
        labels = [labels[0], labels[1]]
        ax[0].legend(handles, labels)

#        fig.set_size_inches([6.4,4.8])
        if False:
            FreqTrans.PrintPrettyFig(fig, 'OpenMimoGainTemporal' + str(iOut+1) + str(iIn+1) + '.pgf')

#%%
if False:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        # Best Case SNR can be estimated as the Null input to Excited input
        uNER = np.abs(SuuNullList[:,iIn,:]) / np.abs(SuuList[:,iIn,:])
        uNERMean = np.mean(uNER, axis=-1)
        uNERMin = np.min(uNER, axis=-1)

        zSNR = np.abs(SnnList[:,iOut, iIn,:]) / np.abs(SzzList[:,iOut, iIn,:])
        zSNRMean = np.mean(zSNR, axis=-1)
        zSNRMin = np.min(zSNR, axis=-1)
        zSNRMin[zSNRMin < 0] = 0

#        zSNR2 = np.abs(TunList[:,iOut, iIn,:])
#        zSNR2Mean = np.mean(zSNR2, axis=-1)
#        zSNR2Min = np.min(zSNR2, axis=-1)
#        zSNR2Min[zSNR2Min < 0] = 0

        cohEst = np.abs(CuzList[:,iOut,iIn,:])
        cohEst[cohEst < 0] = 0
        cohEst[cohEst > 1] = 1
        cohEstMean = np.mean(cohEst, axis=-1)
        cohEstMin = np.min(cohEst, axis=-1)

        fig = None
        fig = FreqTrans.PlotGainTemporal(t_s, uNERMean, None, None, uNERMin, fig = fig, dB = False, UncSide = 'Max', linestyle='-', color='g', label = 'Estimate at Input' + ' [$u_' + str(iIn+1) + '$]')
        fig = FreqTrans.PlotGainTemporal(t_s, zSNRMean, None, None, zSNRMin, fig = fig, dB = False, UncSide = 'Max', linestyle='-', color='b', label = 'Estimate at Output' + ' [$z_' + str(iOut+1) + '$]')
#        fig = FreqTrans.PlotGainTemporal(t_s, zSNR2Mean, None, None, zSNR2Min, fig = fig, dB = False, UncSide = 'Max', linestyle='-', color='r', label = 'Estimate at Output' + ' [$z_' + str(iOut+1) + '$]')
#        fig = FreqTrans.PlotGainTemporal(t_s, uNERMean, None, cohEstMin, uNERMin, fig = fig, dB = True, UncSide = 'Min', linestyle='-', color='k', label = 'Estimate at Input')
#        fig = FreqTrans.PlotGainTemporal(t_s, zSNRMean, None, cohEstMean, zSNRMin, fig = fig, dB = True, UncSide = 'Min', linestyle='-', color='r', label = 'Estimate at Output')

        ax = fig.get_axes()
        ax[0].set_ylim([0, 2.0])
        ax[0].set_ylabel("Noise/Signal [mag]")

        handles, labels = ax[0].get_legend_handles_labels()
        handles = [(handles[0], handles[2]), (handles[1], handles[3])]
        labels = [labels[0], labels[1]]
        ax[0].legend(handles, labels)

        fig.set_size_inches([6.4,3.0])
        if False:
            FreqTrans.PrintPrettyFig(fig, 'OpenMimoSNRTemporal' + str(iOut+1) + str(iIn+1) + '.pgf')


