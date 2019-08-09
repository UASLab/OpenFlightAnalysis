"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Analysis for Huginn (mAEWing2) FLT 03
"""

#%%
# Import Libraries
import numpy as np
import matplotlib.pyplot as plt

# Hack to allow loading the Core package
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), "..")))
    path.insert(0, abspath(join(dirname(argv[0]), "..", 'Core')))
    
    del path, argv, dirname, abspath, join

from Core import Loader
from Core import OpenData


# Constants
hz2rps = 2 * np.pi
rps2hz = 1 / hz2rps


#%% File Lists
import os.path as path

pathBase = path.join('/home', 'rega0051', 'FlightArchive', 'Huginn')
pathBase = path.join('O:', 'Shared drives', 'UAVLab', 'Flight Data', 'Huginn')

fileList = {}
flt = 'FLT03'
fileList[flt] = {}
fileList[flt]['log'] = path.join(pathBase, 'Huginn' + flt, 'Huginn' + flt + '.h5')
fileList[flt]['config'] = path.join(pathBase, 'Huginn' + flt, 'huginn.json')
fileList[flt]['def'] = path.join(pathBase, 'Huginn' + flt, 'huginn_def.json')

flt = 'FLT04'
fileList[flt] = {}
fileList[flt]['log'] = path.join(pathBase, 'Huginn' + flt, 'Huginn' + flt + '.h5')
fileList[flt]['config'] = path.join(pathBase, 'Huginn' + flt, 'huginn.json')
fileList[flt]['def'] = path.join(pathBase, 'Huginn' + flt, 'huginn_def.json')

flt = 'FLT05'
fileList[flt] = {}
fileList[flt]['log'] = path.join(pathBase, 'Huginn' + flt, 'Huginn' + flt + '.h5')
fileList[flt]['config'] = path.join(pathBase, 'Huginn' + flt, 'huginn.json')
fileList[flt]['def'] = path.join(pathBase, 'Huginn' + flt, 'huginn_def.json')

flt = 'FLT06'
fileList[flt] = {}
fileList[flt]['log'] = path.join(pathBase, 'Huginn' + flt, 'Huginn' + flt + '.h5')
fileList[flt]['config'] = path.join(pathBase, 'Huginn' + flt, 'huginn.json')
fileList[flt]['def'] = path.join(pathBase, 'Huginn' + flt, 'huginn_def.json')


#%%
from Core import FreqTrans

rtsmSegList = [
#        {'flt': 'FLT03', 'seg': ('time_us', [880217573, 893797477], 'FLT03 - Sym1 - 20 m/s')}, # 20 m/s
        {'flt': 'FLT03', 'seg': ('time_us', [710658638, 722078703], 'FLT03 - Sym1 - 23 m/s')}, # 23 m/s
        {'flt': 'FLT06', 'seg': ('time_us', [1136880543, 1146680543], 'FLT06 - Sym1 - 23 m/s')}, # 23 m/s
#        {'flt': 'FLT04', 'seg': ('time_us', [914627038, 926728286], 'FLT04 - Sym1 - 26 m/s')}, # 26 m/s
#        {'flt': 'FLT05', 'seg': ('time_us', [622279236, 634279236], 'FLT05 - Sym1 - 26 m/s')}, # 26 m/s
#        {'flt': 'FLT05', 'seg': ('time_us', [831211361, 843211361], 'FLT05 - Sym1 - 29 m/s')}, # 29 m/s
#        {'flt': 'FLT06', 'seg': ('time_us', [972425515, 984425515], 'FLT06 - Sym1 - 32 m/s')}, # 32 m/s
        
#        {'flt': 'FLT04', 'seg': ('time_us', [1054856968, 1067537708], 'FLT04 - Sym2 - 20 m/s')}, # 20 m/s
#        {'flt': 'FLT03', 'seg': ('time_us', [775518514, 788718440], 'FLT03 - Sym2 - 23 m/s')}, # 23 m/s
#        {'flt': 'FLT04', 'seg': ('time_us', [957651007, 969011852], 'FLT04 - Sym2 - 26 m/s')}, # 26 m/s
        
#        {'flt': 'FLT05', 'seg': ('time_us', [687832534, 699832534], 'FLT05 - Sym2 - 26 m/s')}, # 26 m/s
#        {'flt': 'FLT05', 'seg': ('time_us', [887575754, 899575754], 'FLT05 - Sym2 - 29 m/s')}, # 29 m/s
#        {'flt': 'FLT05', 'seg': ('time_us', [1026492809, 1036733749], 'FLT05 - Sym2 - 32 m/s')}, # 32 m/s
        
#        {'flt': 'FLT06', 'seg': ('time_us', [1122539650, 1134539650], 'FLT06 - 23 m/s')}, # 23 m/s
#        {'flt': 'FLT05', 'seg': ('time_us', [582408497, 594408497], 'FLT05 - 26 m/s')}, # 26 m/s
#        {'flt': 'FLT05', 'seg': ('time_us', [799488311, 811488311], 'FLT05 - 29 m/s')}, # 29 m/s
#        {'flt': 'FLT06', 'seg': ('time_us', [955822061, 967822061], 'FLT06 - 32 m/s')}, # 32 m/s
        ]


oDataSegs = []
for rtsmSeg in rtsmSegList:
    fltNum = rtsmSeg['flt']
    
    fileLog = fileList[fltNum]['log']
    fileConfig = fileList[fltNum]['config']
    
    # Load
    h5Data = Loader.Load_h5(fileLog) # RAPTRS log data as hdf5
    sysConfig = Loader.JsonRead(fileConfig)
    oData = Loader.OpenData_RAPTRS(h5Data, sysConfig)
    
    # Create Signal for Bending Measurement
    aZ = np.array([
            oData['aCenterFwdIMU_IMU_mps2'][2] - oData['aCenterFwdIMU_IMU_mps2'][2][0], 
            oData['aCenterAftIMU_IMU_mps2'][2] - oData['aCenterAftIMU_IMU_mps2'][2][0], 
            oData['aLeftMidIMU_IMU_mps2'][2] - oData['aLeftMidIMU_IMU_mps2'][2][0], 
            oData['aLeftFwdIMU_IMU_mps2'][2] - oData['aLeftFwdIMU_IMU_mps2'][2][0], 
            oData['aLeftAftIMU_IMU_mps2'][2] - oData['aLeftAftIMU_IMU_mps2'][2][0], 
            oData['aRightMidIMU_IMU_mps2'][2] - oData['aRightMidIMU_IMU_mps2'][2][0], 
            oData['aRightFwdIMU_IMU_mps2'][2] - oData['aRightFwdIMU_IMU_mps2'][2][0], 
            oData['aRightAftIMU_IMU_mps2'][2] - oData['aRightAftIMU_IMU_mps2'][2][0]
            ])
    
    aCoefEta1 = np.array([456.1, -565.1, -289.9, 605.4, 472.4, -292, 605.6, 472.2]) * 0.3048
    measEta1 = aCoefEta1 @ (aZ - np.mean(aZ, axis = 0)) * 1/50 * 1e-3
    
    
    aCoefEta1dt = np.array([2.956, -2.998, -1.141, 5.974, 5.349, -1.149, 5.974, 5.348]) * 0.3048
    measEta1dt = aCoefEta1dt @ (aZ - np.mean(aZ, axis = 0)) * 1e-3
    
    # Added signals
    oData['measBendDt'] = measEta1dt
    oData['measBend'] = measEta1
    oData['measPitch'] = oData['wB_I_rps'][1]
    oData['measAlt'] = oData['altBaro_m']
    
    
    oData['Control']['cmdTE1Sym_rad'] = oData['Control']['cmdTE1L_rad'] + oData['Control']['cmdTE1R_rad']
    oData['Control']['cmdTE2Sym_rad'] = oData['Control']['cmdTE2L_rad'] + oData['Control']['cmdTE2R_rad']
    oData['Control']['cmdTE3Sym_rad'] = oData['Control']['cmdTE3L_rad'] + oData['Control']['cmdTE3R_rad']
    oData['Control']['cmdTE4Sym_rad'] = oData['Control']['cmdTE4L_rad'] + oData['Control']['cmdTE4R_rad']
    oData['Control']['cmdTE5Sym_rad'] = oData['Control']['cmdTE5L_rad'] + oData['Control']['cmdTE5R_rad']
    oData['Control']['cmdLESym_rad'] = oData['Control']['cmdLEL_rad'] + oData['Control']['cmdLER_rad']
    
    oData['Excitation']['cmdTE1Sym_rad'] = oData['Excitation']['cmdTE1L_rad'] + oData['Excitation']['cmdTE1R_rad']
    oData['Excitation']['cmdTE2Sym_rad'] = oData['Excitation']['cmdTE2L_rad'] + oData['Excitation']['cmdTE2R_rad']
    oData['Excitation']['cmdTE3Sym_rad'] = oData['Excitation']['cmdTE3L_rad'] + oData['Excitation']['cmdTE3R_rad']
    oData['Excitation']['cmdTE4Sym_rad'] = oData['Excitation']['cmdTE4L_rad'] + oData['Excitation']['cmdTE4R_rad']
    oData['Excitation']['cmdTE5Sym_rad'] = oData['Excitation']['cmdTE5L_rad'] + oData['Excitation']['cmdTE5R_rad']
    oData['Excitation']['cmdLESym_rad'] = oData['Excitation']['cmdLEL_rad'] + oData['Excitation']['cmdLER_rad']
    
    oData['Control']['cmdTE1Sym_rad'] = oData['Control']['cmdTE1L_rad'] + oData['Control']['cmdTE1R_rad']
    oData['Control']['cmdTE2Sym_rad'] = oData['Control']['cmdTE2L_rad'] + oData['Control']['cmdTE2R_rad']
    oData['Control']['cmdTE3Sym_rad'] = oData['Control']['cmdTE3L_rad'] + oData['Control']['cmdTE3R_rad']
    oData['Control']['cmdTE4Sym_rad'] = oData['Control']['cmdTE4L_rad'] + oData['Control']['cmdTE4R_rad']
    oData['Control']['cmdTE5Sym_rad'] = oData['Control']['cmdTE5L_rad'] + oData['Control']['cmdTE5R_rad']
    oData['Control']['cmdLESym_rad'] = oData['Control']['cmdLEL_rad'] + oData['Control']['cmdLER_rad']
    
    oData['Surf'] = {}
    for surfName in h5Data['Sensors']['Surf'].keys():
        oData['Surf'][surfName] = h5Data['Sensors']['Surf'][surfName]['CalibratedValue']
    
    oData['Surf']['posTE1Sym'] = oData['Surf']['posTE1L'] + oData['Surf']['posTE1R']
    oData['Surf']['posTE2Sym'] = oData['Surf']['posTE2L'] + oData['Surf']['posTE2R']
    oData['Surf']['posTE3Sym'] = oData['Surf']['posTE3L'] + oData['Surf']['posTE3R']
    oData['Surf']['posTE4Sym'] = oData['Surf']['posTE4L'] + oData['Surf']['posTE4R']
    oData['Surf']['posTE5Sym'] = oData['Surf']['posTE5L'] + oData['Surf']['posTE5R']
    oData['Surf']['posLESym'] = oData['Surf']['posLEL'] + oData['Surf']['posLER']
    
    # Segments
    seg = OpenData.Segment(oData, rtsmSeg['seg'])
    oDataSegs.append(seg)
    
#    plt.plot(seg['time_s'], seg['Excitation']['cmdTE1L_rad'], seg['time_s'], seg['Excitation']['cmdTE2L_rad'], seg['time_s'], seg['measBend'], seg['time_s'], seg['measPitch'])
    plt.plot(seg['time_s'], seg['measPitch'], seg['time_s'], seg['measBend'])


#%%

sigInList = ['cmdTE1Sym_rad', 'cmdTE3Sym_rad', 'cmdTE5Sym_rad']
#sigInList = ['cmdTE2Sym_rad', 'cmdTE4Sym_rad', 'cmdLESym_rad']
#sigInList = ['posTE1Sym', 'posTE3Sym', 'posTE5Sym']
#sigInList = ['cmdRoll_rps', 'cmdPitch_rps', 'cmdBend_nd']

sigOutList = ['measPitch', 'measBend']


freqExc_rps = []
freqExc_rps.append( np.array(sysConfig['Excitation']['OMS_Surf_1']['Frequency']))
freqExc_rps.append( np.array(sysConfig['Excitation']['OMS_Surf_2']['Frequency']))
freqExc_rps.append( np.array(sysConfig['Excitation']['OMS_Surf_3']['Frequency']))

plt.figure()

eList = []
uList = []
inSigList = []
for iSeg, seg in enumerate(oDataSegs):
    eSig = np.zeros((len(sigInList), len(seg['time_s'])))
    uSig = np.zeros((len(sigInList), len(seg['time_s'])))
    
    for iSig, sigIn in enumerate(sigInList):
        eSig[iSig] = seg['Excitation'][sigIn]
        uSig[iSig] = seg['Control'][sigIn]
#        inSig[iSig] = seg['Surf'][sigIn]
        
        plt.plot(oDataSegs[iSeg]['time_s'], eSig[iSig], oDataSegs[iSeg]['time_s'], uSig[iSig])
    
    eList.append(eSig)
    uList.append(uSig)
    inSigList.append(inSig)

    
outList = []
for iSeg, seg in enumerate(oDataSegs):
    outSig = np.zeros((len(sigOutList), len(seg['time_s'])))
     
    for iSig, sigOut in enumerate(sigOutList):
        outSig[iSig] = seg[sigOut]
#        outSig[iSig] = seg['Surf'][sigOut]
        
#        plt.plot(oDataSegs[iSeg]['time_s'], outSig[iSig])
        
    outList.append(outSig)
    

#%% Estimate the frequency response function
# Define the excitation frequencies
freqRate_hz = 50
freqRate_rps = freqRate_hz * hz2rps
optSpec = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.2), detrendType = 'Linear')
optSpecN = FreqTrans.OptSpect(dftType = 'czt', freqRate = freqRate_rps, smooth = ('box', 3), winType = ('tukey', 0.2), detrendType = 'Linear')

# Excited Frequencies per input channel
optSpec.freq = np.asarray(freqExc_rps)

# Null Frequencies
freqGap_rps = optSpec.freq.flatten()[0:-1] + 0.5 * np.diff(optSpec.freq.flatten())
optSpecN.freq = freqGap_rps

# FRF Estimate
freq_rps = []
freq_hz = []
T = []
TUnc = []
C = []
for iSeg, seg in enumerate(oDataSegs):
    
    freq, Tey, Cey, Pee, Pyy, Pey, TeyUnc, Pyy_N = FreqTrans.FreqRespFuncEstNoise(eList[iSeg], outList[iSeg], optSpec, optSpecN)
    freq, Teu, Ceu, _, Puu, Peu = FreqTrans.FreqRespFuncEst(eList[iSeg], uList[iSeg], optSpec)
    
    # Form the Frequency Response
    freq_hz = freq * rps2hz

    # Form the Frequency Response
    T_seg = np.zeros_like(Tey)
    TUnc_seg = np.zeros_like(Tey)

    for i in range(T_seg.shape[-1]):
        T_seg[...,i] = (Tey[...,i].T @ np.linalg.inv(Teu[...,i].T)).T
        TUnc_seg[...,i] = (TeyUnc[...,i].T @ np.linalg.inv(Teu[...,i].T)).T


    T.append( T_seg )
    TUnc.append( TUnc_seg )
    C.append(Cey)
    

T_InputNames = sigInList
T_OutputNames = sigOutList

# Compute Gain, Phase, Crit Distance
gain_mag = []
gainUnc_mag = []
phase_deg = []
for iSeg in range(0, len(oDataSegs)):
    gain_mag.append(FreqTrans.Gain(T[iSeg], magUnit = 'mag'))
    gainUnc_mag.append(FreqTrans.Gain(TUnc[iSeg], magUnit = 'mag'))
    phase_deg.append(FreqTrans.Phase(T[iSeg], phaseUnit = 'deg', unwrap = False))


#%% Spectrograms
if False:
    
    iSgnl = 1
    
    freqRate_rps = 50 * hz2rps
#    freqExc_rps = np.linspace(0.1, 50/2, 151) * hz2rps

    optSpec = FreqTrans.OptSpect(dftType = 'czt', freq = np.asarray(freqExc_rps).flatten(), freqRate = freqRate_rps, winType = ('tukey', 0.0), smooth = ('box', 1), detrendType = 'Linear')
    
    
    for iSeg in range(0, len(oDataSegs)):
        t = oDataSegs[iSeg]['time_s']
        y = outList[iSeg][iSgnl]
        
        # Number of time segments and length of overlap, units of samples
        #lenSeg = 2**6 - 1
        lenSeg = int(1 * optSpec.freqRate * rps2hz)
        lenOverlap = 5
        
        # Compute Spectrum over time
        tSpec_s, freqSpec_rps, P_mag = FreqTrans.SpectTime(t, y, lenSeg, lenOverlap, optSpec)
        
        # Plot the Spectrogram
        fig = FreqTrans.Spectogram(tSpec_s, freqSpec_rps * rps2hz, 20 * np.log10(P_mag))
        fig.suptitle(oDataSegs[iSeg]['Desc'] + ': Spectrogram - ' + sigOutList[iSgnl])


#%% Nyquist Plots
inPlot = sigInList # Elements of sigInList
outPlot = sigOutList # Elements of sigOutList

if False:
    for iIn, inName in enumerate(inPlot):
        for iOut, outName in enumerate(outPlot):

            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotNyquist(T[iSeg][iIn, iOut], TUnc[iSeg][iIn, iOut], fig = fig, fmt = '*', label = oDataSegs[iSeg]['Desc'])

            fig.suptitle(inName + ' to ' + outName, size=20)

            ax = fig.get_axes()
            ax[0].set_xlim(-3, 1)
            ax[0].set_ylim(-2, 2)
            

#%% Bode Plots
if True:

    for iIn, inName in enumerate(inPlot):
        for iOut, outName in enumerate(outPlot):
            
            fig = None
            for iSeg in range(0, len(oDataSegs)):
                fig = FreqTrans.PlotBode(freq_hz[iIn, 0], gain_mag[iSeg][iIn, iOut], phase_deg[iSeg][iIn, iOut], C[iSeg][iIn, iOut], gainUnc_mag = gainUnc_mag[iSeg][iIn, iOut], dB = False, fig = fig, fmt = '*--', label = oDataSegs[iSeg]['Desc'])

            fig.suptitle(inName + ' to ' + outName, size = 20)
