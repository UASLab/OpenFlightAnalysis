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
numCycles = 4
ampInit = 1
ampFinal = 1
freqMinDes_rps = 0.1 * hz2rps * np.ones(numExc)
freqMaxDes_rps = 10.0 * hz2rps *  np.ones(numExc)
freqStepDes_rps = (10 / freqRate_hz) * hz2rps
methodSW = 'zip' # "zippered" component distribution

# Generate MultiSine Frequencies
freqExc_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_rps, numCycles, freqStepDes_rps, methodSW)
freqGap_rps = freqExc_rps[0:-1] + 0.5 * np.diff(freqExc_rps)

#time_s = time_s[0:-5]

# Generate Schroeder MultiSine Signal
ampExcit_nd = np.linspace(ampInit, ampFinal, len(freqExc_rps)) / np.sqrt(len(freqExc_rps))
uExc, phaseElem_rad, sigExcit = GenExcite.MultiSine(freqExc_rps, ampExcit_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1, normalize = 'peak', costType = 'Schroeder')
uExc = (uExc.T * (1 / np.std(uExc, axis = -1))).T
uStd = np.std(uExc, axis = -1)
uPeakFactor = GenExcite.PeakFactor(uExc)

# Excited Frequencies per input channel
freqChan_rps = freqExc_rps[sigIndx]
freqChan_hz = freqChan_rps * rps2hz

# Simulate the excitation through the system
_, y, _ = control.forced_response(sysPlant, T = time_s, U = uExc)


# Plant-Output Noise
noiseK11 = (2/8); noiseWn11 = 6 * hz2rps; noiseD11 = 0.1;
noiseK22 = (1/8); noiseWn22 = 4 * hz2rps; noiseD22 = 1.0;

sysN = control.tf([[[-noiseK11, 0, noiseK11 * noiseWn11**2], [0]],
                   [[0], [-noiseK22, 0, noiseK22 * noiseWn22**2]]],
                  [[[1, 2.0*noiseD11*noiseWn11, noiseWn11**2], [1]],
                   [[1], [1, 2.0*noiseD22*noiseWn22, noiseWn22**2]]])


#noiseK11 = (1/8) ; noiseWn11 = 6 * hz2rps; noiseD11 = 0.1;
#noiseK12 = (4/8) ; noiseWn12 = 6 * hz2rps; noiseD12 = 0.7;
#noiseK21 = (4/8) ; noiseWn21 = 4 * hz2rps; noiseD21 = 0.7;
#noiseK22 = (4/8) ; noiseWn22 = 4 * hz2rps; noiseD22 = 1.0;
#
#sysN = control.tf([[[-noiseK11, 0, noiseK11 * noiseWn11**2], [-noiseK21, 0, noiseK21 * noiseWn21**2]],
#                   [[-noiseK12, 0, noiseK12 * noiseWn12**2], [-noiseK22, 0, noiseK22 * noiseWn22**2]]],
#                  [[[1, 2.0*noiseD11*noiseWn11, noiseWn11**2], [1, 2.0*noiseD21*noiseWn21, noiseWn21**2]],
#                   [[1, 2.0*noiseD12*noiseWn12, noiseWn12**2], [1, 2.0*noiseD22*noiseWn22, noiseWn22**2]]])



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
optSpec = FreqTrans.OptSpect(dftType = 'czt', scaleType = 'density', freqRate_rps = freqRate_rps, smooth = ('box', 5), winType = 'bartlett', detrendType = 'linear')

# Excited Frequencies per input channel
optSpec.freq_rps = freqChan_rps
optSpec.freqInterp = freqExc_rps

# Null Frequencies
optSpec.freqNull = freqGap_rps
optSpec.freqNullInterp = True

# FRF Estimate
#freq_rps, Tuz, Cuz, Suu, Szz, Suz = FreqTrans.FreqRespFuncEst(uExc, z, optSpec)
freq_rps, Tuz, Cuz, Suu, Szz, Suz, Tun, SuuNull, Snn = FreqTrans.FreqRespFuncEstNoise(uExc, z, optSpec)
freq_hz = freq_rps * rps2hz
SuuNull.sum()

print(SuuNull.sum(axis=-1) / Suu.sum(axis=-1))
print(Snn.sum(axis=-1))
print(Szz.sum(axis=-1))

TEstNom = Tuz
TEstUnc = Tun
TEstCoh = Cuz


# Check
N = len(time_s)
#PexcSpec = 0.5 * (ampExcit_nd**2).sum()
#PexcPsd = PexcSpec * N / freqRate_hz # Convert Total Spectrum Power to Density

PexcParsevalSpec = (1/N) * (np.abs(uExc)**2).sum(axis = -1)
PexcParsevalPsd = PexcParsevalSpec * N / freqRate_hz # Convert Total Spectrum Power to Density

SuuMag = np.abs(Suu)
SuuSum = SuuMag.sum(axis = -1)

print(SuuSum / PexcParsevalPsd)

PnParsevalSpec = (1/N) * (np.abs(n)**2).sum(axis = -1)
PnParsevalPsd = PnParsevalSpec * N / freqRate_hz # Convert Total Spectrum Power to Density

SnnMag = np.abs(Snn)
SnnSum = SnnMag.sum(axis = -1)

print(SnnSum / PnParsevalPsd)


PzParsevalSpec = (1/N) * (np.abs(z)**2).sum(axis = -1)
PzParsevalPsd = PzParsevalSpec * N / freqRate_hz # Convert Total Spectrum Power to Density

SzzMag = np.abs(Szz)
SzzSum = SzzMag.sum(axis = -1)

print(SzzSum / PzParsevalPsd)

#%%
if False:
#%%

    [iOut, iIn] = [0,0]
    fig = plt.figure(); plt.grid(True)
    plt.plot(freq_hz[iIn], mag2db(Szz[iOut, iIn]), '.b', label = 'Estimate Nominal [MIMO]')
    plt.plot(freq_hz[iIn, sigIndx[iIn]], mag2db(Szz[iOut, iIn, sigIndx[iIn]]), '.r:', label = 'Estimate Nominal [SIMO]')
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Power Spectral Density [dB]')
    plt.legend()
    plt.xlim([1,5])
#    plt.ylim([0,20])
    fig.set_tight_layout(True)
    fig.set_size_inches([6.4,2.4])
    
    if False:
        FreqTrans.PrintPrettyFig(fig, 'OpenMimoExcitationInterp.pgf')
    
#%%
    optSpecN = FreqTrans.OptSpect(dftType = 'czt', scaleType = 'density', freqRate_rps = freqRate_rps, smooth = ('box', 5), winType = 'bartlett', detrendType = 'linear')
    optSpecN.freq_rps = freqGap_rps
    
    _, _, SnnN = FreqTrans.Spectrum(z, optSpecN)
#    _, _, _, SuuN, SnnN, _ = FreqTrans.FreqRespFuncEst(uExc, z, optSpecN)

    SnnN.sum(axis=-1)
    Snn.sum(axis=-1)

    fig = plt.figure(); plt.grid(True)
    fig.tight_layout()
    plt.plot(freq_hz[iIn], mag2db(Snn[iOut, iIn]), '.b', label = 'Estimate Null [MIMO]');
#    plt.plot(freq_hz[iIn, sigIndx[iIn]], mag2db(Snn[iOut, iIn, sigIndx[iIn]]), '.g:', label = 'Estimate from Null')
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
    fig = FreqTrans.PlotSigma(freq_hz[0], svTEstNomMin_mag, svUnc_mag = svTEstUncMax_mag, coher_nd = cohTEstMin, marker='.', color = 'r', fig = fig, label = 'Estimate')

    ax = fig.get_axes()
    handles, labels = ax[0].get_legend_handles_labels()
    handles = [handles[0], handles[2], handles[1], handles[3]]
    labels = [labels[0], labels[2], labels[1], labels[3]]
    ax[0].legend(handles, labels)
    
    fig.suptitle('$T$ : ' + '$u_{ex}$' + ' to ' + '$z$')
    
    
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
#        fig = FreqTrans.PlotVectorMargin(freq_hz[iIn, sigIndx[iIn]], vmTEstNom_mag[iOut, iIn, sigIndx[iIn]], cohTEst_mag[iOut, iIn, sigIndx[iIn]], vmTEstUnc_mag[iOut, iIn, sigIndx[iIn]], fig = fig, linestyle='-', marker='.', color='b', label='Estimate [SIMO]')
        
        ax = fig.get_axes()
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [handles[0], handles[3], handles[1], handles[4], handles[2], handles[5]]
        labels = [labels[0], labels[3], labels[1], labels[4], labels[2], labels[5]]
        ax[0].legend(handles, labels)
    
        fig.suptitle('$T$')
        fig.set_tight_layout(True)


#%% Bode Plot
# Linear Model Gain and Phase
gainTLinNom_mag, phaseTLinNom_deg = FreqTrans.GainPhase(TLinNom, magUnit = 'mag', unwrap = True)
gainTLinUnc_mag = FreqTrans.Gain(TLinUnc, magUnit = 'mag')

# Estimated Gain and Phase
gainTEstNom_mag, phaseTEstNom_deg = FreqTrans.GainPhase(TEstNom, magUnit = 'mag', unwrap = True)
gainTEstUnc_mag = FreqTrans.Gain(TEstUnc, magUnit = 'mag')

np.mean(gainTEstUnc_mag, axis = -1) / np.mean(gainTLinUnc_mag, axis = -1) # should be 1 if scaled correctly

if True:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        fig = None
        ioName = '- $T$: ' + '$u[' + str(iIn+1) + ']$ to ' + '$z[' + str(iOut+1) + ']$'
        fig = FreqTrans.PlotBode(freqLin_hz, gainTLinNom_mag[iOut, iIn], phaseTLinNom_deg[iOut, iIn], coher_nd = cohTLin_mag[iOut, iIn], gainUnc_mag = None, fig = fig, dB = True, color='k', label='Linear Nominal ' + ioName)
        fig = FreqTrans.PlotBode(freqLin_hz, gainTLinUnc_mag[iOut, iIn], None, coher_nd = None, gainUnc_mag = None, fig = fig, dB = True, color='k', linestyle='--', label='Linear Uncertainty ' + ioName)
        
        fig = FreqTrans.PlotBode(freq_hz[iIn], gainTEstNom_mag[iOut, iIn], phaseTEstNom_deg[iOut, iIn], coher_nd = cohTEst_mag[iOut, iIn], gainUnc_mag = None, fig = fig, dB = True, color='r', linestyle='None', marker='.', label='Estimate Nominal ' + ioName)
        fig = FreqTrans.PlotBode(freq_hz[iIn], gainTEstUnc_mag[iOut, iIn], None, coher_nd = None, gainUnc_mag = None, fig = fig, dB = True, color='b', linestyle='None', marker='.', label='Estimate Uncertainty ' + ioName)

        ax = fig.get_axes()
#        handles, labels = ax[0].get_legend_handles_labels()
#        handles = [(handles[0], handles[2]), (handles[1], handles[3])]
#        labels = [labels[0], labels[1]]
#        ax[0].legend(handles, labels)
#        ax[0].set_xlim([1,10])
        
        fig.set_size_inches([6.4,4.8])
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
        ioName = '- $T$: ' + '$u[' + str(iIn+1) + ']$ to ' + '$z[' + str(iOut+1) + ']$'
        fig = FreqTrans.PlotNyquist(TLinNom[iOut, iIn], TLinUnc[iOut, iIn], fig = fig, fillType = 'fill', color = 'k', label = 'Linear ' + ioName)
        fig = FreqTrans.PlotNyquist(TEstNom[iOut, iIn], TEstUnc[iOut, iIn], fig = fig, fillType = 'circle', marker='.', color = 'r', linestyle='None', label = 'Estimate ' + ioName)
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
optSpec = FreqTrans.OptSpect(dftType = 'czt', scaleType = 'density', freqRate_rps = freqRate_rps, smooth = ('box', 5), winType = 'bartlett', detrendType = 'linear')

# Excited Frequencies per input channel
optSpec.freq_rps = freqChan_rps
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
TLinNomList = np.zeros((numSeg, numOut, numIn, numFreqLin))
TLinUncList = np.zeros((numSeg, numOut, numIn, numFreqLin))
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
        ioName = '- $T$: ' + '$u[' + str(iIn+1) + ']$ to ' + '$z[' + str(iOut+1) + ']$'
        fig = FreqTrans.PlotGainTemporal(t_s, gainThLinNomMean, None, coher_nd = ones, gainUnc_mag = gainThLinUncMean, fig = fig, dB = False, linestyle='-', color='k', label = 'Linear ' + ioName)
#        fig = FreqTrans.PlotGainTemporal(t_s, gainThLinUncMin, None, fig = fig, dB = False, linestyle=':', color='k', label = 'Linear - Lower')
     
        fig = FreqTrans.PlotGainTemporal(t_s, gainThEstNomMean, None, coher_nd = cohEstMean, gainUnc_mag = gainThEstUncMean, fig = fig, dB = False, linestyle='-', color='b', label = 'Estimate ' + ioName)
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
if True:
    for iPlot, [iOut, iIn] in enumerate(ioArray):
        # Best Case E2N can be estimated as the Null input to Excited input
        uN2E = np.abs(SuuNullList[:,iIn,:]) / np.abs(SuuList[:,iIn,:])
        uN2EMean = np.mean(uN2E, axis=-1)
        uN2EMin = np.min(uN2E, axis=-1)
        
        zN2S = np.abs(SnnList[:,iOut, iIn,:]) / np.abs(SzzList[:,iOut, iIn,:])
        zN2SMean = np.mean(zN2S, axis=-1)
        zN2SMin = np.min(zN2S, axis=-1)
        zN2SMin[zN2SMin < 0] = 0
        
#        zN2S2 = np.abs(TunList[:,iOut, iIn,:])
#        zN2S2Mean = np.mean(zN2S2, axis=-1)
#        zN2S2Min = np.min(zN2S2, axis=-1)
#        zN2S2Min[zN2S2Min < 0] = 0
        
        cohEst = np.abs(CuzList[:,iOut,iIn,:])
        cohEst[cohEst < 0] = 0
        cohEst[cohEst > 1] = 1
        cohEstMean = np.mean(cohEst, axis=-1)
        cohEstMin = np.min(cohEst, axis=-1)
        
        fig = None
        ioName = '- $T$: ' + '$u[' + str(iIn+1) + ']$ to ' + '$z[' + str(iOut+1) + ']$'
        fig = FreqTrans.PlotGainTemporal(t_s, uN2EMean, None, None, uN2EMin, fig = fig, dB = False, UncSide = 'Max', linestyle='-', color='g', label = 'Estimate at Input: ' + '$u[' + str(iIn+1) + '$]')
        fig = FreqTrans.PlotGainTemporal(t_s, zN2SMean, None, None, zN2SMin, fig = fig, dB = False, UncSide = 'Max', linestyle='-', color='r', label = 'Estimate at Output: ' + ' $z[' + str(iOut+1) + '$]')
#        fig = FreqTrans.PlotGainTemporal(t_s, zN2S2Mean, None, None, zN2S2Min, fig = fig, dB = False, UncSide = 'Max', linestyle='-', color='r', label = 'Estimate at Output: ' + ' $z[' + str(iOut+1) + '$]')
#        fig = FreqTrans.PlotGainTemporal(t_s, uN2EMean, None, cohEstMin, uN2EMin, fig = fig, dB = True, UncSide = 'Min', linestyle='-', color='k', label = 'Estimate at Input')
#        fig = FreqTrans.PlotGainTemporal(t_s, zN2SMean, None, cohEstMean, zN2SMin, fig = fig, dB = True, UncSide = 'Min', linestyle='-', color='r', label = 'Estimate at Output')

        ax = fig.get_axes()
        ax[0].set_ylim([0, 2.0])
        ax[0].set_ylabel("Null/Excitation Power [mag]")
        
        handles, labels = ax[0].get_legend_handles_labels()
        handles = [(handles[0], handles[2]), (handles[1], handles[3])]
        labels = [labels[0], labels[1]]
        ax[0].legend(handles, labels)

        fig.set_size_inches([6.4,3.0])
        if False:
            FreqTrans.PrintPrettyFig(fig, 'OpenMimoN2STemporal' + str(iOut+1) + str(iIn+1) + '.pgf')
        

