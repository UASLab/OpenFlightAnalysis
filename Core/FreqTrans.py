"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np
import scipy.signal as signal
import scipy.interpolate as interp
import control

# Constants
pi = np.pi

hz2rps = 2*pi
rps2hz = 1/hz2rps

rad2deg = 180/pi
deg2rad = 1/ rad2deg

mag2db = control.mag2db
db2mag = control.db2mag


from dataclasses import dataclass

@dataclass
class OptSpect:
    '''Class for Spectrum Options.'''
    freqRate_rps:float = 1
    freq_rps:np.ndarray(0) = None
    freqNull:np.ndarray(0) = None
    dftType:str = 'fft'
    winType:tuple = ('tukey', 0.0)
    detrendType:str = None
    interpType:str = 'linear'
    smooth:tuple = ('box', 1)
    scaleType:str = 'density'
    freqInterp:np.ndarray(0) = None
    freqNullInterp:bool = False
    asHist:bool = False


#%% Estimate Transfer Function from Time history data
def FreqRespFuncEstSIMO(x, z, opt = OptSpect()):
    '''
    Estimates the Transfer Function Response from input/output time histories
    Single-Input Multi-Output at a Single-FreqVector

    x and z are real and must be the same length
    Assumes x and z have uniform spacing

    x has dimension (1, p) or (p,) at input and is expanded to (1, p)
    z has dimension (n, p) or (p,) at input and is expanded to (n, p)
    Sxx has dimension (1, r) and is reduced to (1, r) or (r,) depending on input form of x
    Szz has dimension (n, r) and is reduced to (n, r) or (r,) depending on input form of x
    Sxz, Cxz, and Txz has dimension (n, r) or (r,)

        m is the number of input signals
        n is the number of output signals
        p is the length of the each signal
        r is the length of the freq vector

    fs and freq must have same units. (Suu and Szz will only have correct power scale if units are rad/sec)
    '''
    x = np.atleast_2d(x)
    z = np.atleast_2d(z)

    # Compute the Power Spectrums
    _   , xDft, Sxx = Spectrum(x, opt)
    freq, zDft, Szz = Spectrum(z, opt)

    # Compute Cross Spectrum Power with scaling
    lenX = x.shape[-1]
    win = signal.get_window(opt.winType, lenX)
    scale = PowerScale(opt.scaleType, win, opt.freqRate_rps)

    Sxz = 2 * xDft.conj() * zDft * scale
    Sxz_smooth = SmoothPolar(Sxz, opt) # Smooth
    
    Sxx_smooth = Smooth(Sxx, opt.smooth) # Smooth
    Szz_smooth = Smooth(Szz, opt.smooth) # Smooth

    # Interpolate Power Spectrums to the Desired Frequency Basis
    if opt.freqInterp is not None:
        s = 1
        if opt.scaleType.lower() == 'density':
            # scale the Powers as density, mean value should be preserved
            s = freq.shape[-1] / opt.freqInterp.shape[-1]
            
        Sxx = s * InterpVal(Sxx, freq, opt.freqInterp, opt.interpType)
        Szz = s * InterpVal(Szz, freq, opt.freqInterp, opt.interpType)
        Sxz = InterpPolar(Sxz, freq, opt.freqInterp, opt.interpType)
        Sxz_smooth = InterpPolar(Sxz_smooth, freq, opt.freqInterp, opt.interpType)
        Sxx_smooth = s * InterpVal(Sxx_smooth, freq, opt.freqInterp, opt.interpType)
        Szz_smooth = s * InterpVal(Szz_smooth, freq, opt.freqInterp, opt.interpType)
        freq = opt.freqInterp
            

    # Coherence, use the Smoothed Cross Spectrum
    Cxz = np.abs(Sxz_smooth)**2 / (Sxx_smooth * Szz_smooth)
    # Cxz[Cxz > 1.0] = 1.0 # Clip, Smoothing and Interpolation can create undesireable end effects

    # Compute complex transfer function approximation
    Txz = Sxz / Sxx

    # Ensure outputs are 2D
    freq = np.atleast_2d(freq)
    Txz = np.atleast_2d(Txz)
    Cxz = np.atleast_2d(Cxz)
    Sxx = np.atleast_2d(Sxx)
    Szz = np.atleast_2d(Szz)
    Sxz = np.atleast_2d(Sxz)

    return freq, Txz, Cxz, Sxx, Szz, Sxz


def FreqRespFuncEstNoiseSIMO(x, z, opt = OptSpect()):
    '''
    _I - Input Frequency Basis
    _E - Interpolated Frequency Basis (_E is generally the set of all _I)
    _N - Null Frequency Basis (_N is generally the Gaps between _E)
    z = y + n
    '''
    import copy

    x = np.atleast_2d(x)
    z = np.atleast_2d(z)

    optN = copy.deepcopy(opt)
    optN.freq_rps = optN.freqNull

    # Compute the Power Spectrums
    _   , xDft, Sxx = Spectrum(x, opt)
    freq, zDft, Szz = Spectrum(z, opt)
    _     , xDftNull_N, SxxNull_N = Spectrum(x, optN)
    freq_N, zDftNull_N, Snn_N = Spectrum(z, optN)
    
#    zDft = zDft - zDftNull_N
    
    # Compute Cross Spectrum Power with appropriate scaling (same as in Spectrum())
    lenX = x.shape[-1]
    win = signal.get_window(opt.winType, lenX)
    scale = PowerScale(opt.scaleType, win, opt.freqRate_rps)
    
    Sxz = 2 * xDft.conj() * zDft * scale
    
    Sxz_smooth = SmoothPolar(Sxz, opt) # Smooth
    Sxx_smooth = Smooth(Sxx, opt.smooth) # Smooth
    Szz_smooth = Smooth(Szz, opt.smooth) # Smooth

    # Null Cross Spectrum at input frequencies
    nDft = InterpPolar(zDftNull_N, freq_N, freq, opt.interpType)
    Sxn = xDft.conj() * nDft * scale
    # Szn = zDft.conj() * nDft * scale

    # Null Cross Spectrum at Null frequencies
    xDft_N = InterpPolar(xDft, freq, freq_N, opt.interpType)
    Sxn_N = xDft_N.conj() * zDftNull_N * scale


    # Interpolate to the Desired Frequency Basis
    if opt.freqInterp is not None:
        s = 1
        if opt.scaleType.lower() == 'density':
            # scale the Powers as density, mean value should be preserved
            s = freq.shape[-1] / opt.freqInterp.shape[-1]
            
        Sxx = s * InterpVal(Sxx, freq, opt.freqInterp, opt.interpType)
        Szz = s * InterpVal(Szz, freq, opt.freqInterp, opt.interpType)
        Sxz = InterpPolar(Sxz, freq, opt.freqInterp, opt.interpType)
        Sxx_smooth = s * InterpVal(Sxx_smooth, freq, opt.freqInterp, opt.interpType)
        Szz_smooth = s * InterpVal(Szz_smooth, freq, opt.freqInterp, opt.interpType)
        Sxz_smooth = InterpPolar(Sxz_smooth, freq, opt.freqInterp, opt.interpType)
        Sxn = InterpPolar(Sxn, freq, opt.freqInterp, opt.interpType)

        if opt.freqNullInterp is True: # Interp _N to _E
            if opt.scaleType.lower() == 'density':
                # scale the Powers as density, mean value should be preserved
                s = freq_N.shape[-1] / opt.freqInterp.shape[-1]

            SxxNull = s * InterpVal(SxxNull_N, freq_N, opt.freqInterp, opt.interpType)
            Sxn = InterpPolar(Sxn_N, freq_N, opt.freqInterp, opt.interpType)
            Snn = s * InterpVal(Snn_N, freq_N, opt.freqInterp, opt.interpType)
        else:
            if opt.scaleType.lower() == 'density':
                # scale the Powers as density, mean value should be preserved
                s = freq_N.shape[-1] / freq.shape[-1]
                
            SxxNull = s * InterpVal(SxxNull_N, freq_N, freq, opt.interpType)
            Sxn = InterpPolar(Sxn_N, freq_N, freq, opt.interpType)
            Snn = s * InterpVal(Snn_N, freq_N, freq, opt.interpType)

        freq = opt.freqInterp

    else:
        if opt.scaleType.lower() == 'density':
            # scale the Powers as density, mean value should be preserved
            s = freq_N.shape[-1] / freq.shape[-1]
 
        SxxNull = s * InterpVal(SxxNull_N, freq_N, freq, opt.interpType)
        Sxn = InterpPolar(Sxn_N, freq_N, freq, opt.interpType)
        Snn = s * InterpVal(Snn_N, freq_N, freq, opt.interpType)


    # Compute complex transfer function approximation of the Null
    # Sxy of Null / Sxx of Input
    TUnc = np.abs(Sxn / Sxx) # Additive Uncertainty (x to n)
    # TUnc = np.abs(Szn / Szz) # Multiplicative Uncertainty (y to n)

    # Coherence, use the Smoothed Cross Spectrum
    Cxz = np.abs(Sxz_smooth)**2 / (Sxx_smooth * Szz_smooth)
    # Cxz[Cxz > 1.0] = 1.0 # Clip, Smoothing and Interpolation can create undesireable end effects

    # Compute complex transfer function approximation
    Txz = Sxz / Sxx

    # Ensure outputs are 2D
    freq = np.atleast_2d(freq)
    Txz = np.atleast_2d(Txz)
    Cxz = np.atleast_2d(Cxz)
    Sxx = np.atleast_2d(Sxx)
    Szz = np.atleast_2d(Szz)
    Sxz = np.atleast_2d(Sxz)

    TUnc = np.atleast_2d(TUnc)
    SxxNull = np.atleast_2d(SxxNull)
    Snn = np.atleast_2d(Snn)

    return freq, Txz, Cxz, Sxx, Szz, Sxz, TUnc, SxxNull, Snn


def FreqRespFuncEst(x, z, opt = OptSpect()):
    '''
    Estimates the Transfer Function Response from input/output time histories
    Single-Input Multi-Output at a Single-FreqVector

    x and z are real and must be the same length
    Assumes x and z have uniform spacing
    '''
    import copy

    x = np.atleast_2d(x)
    z = np.atleast_2d(z)

    # If the input is multidimensional, recursively call
    numIn = x.shape[0]
    numOut = z.shape[0]
    if numIn > 1: # Multi-Input

        freqOpt = np.atleast_2d(opt.freq_rps)

        # Get the shape of the frequency vectors
        numChan = freqOpt.shape[0]

        if not ((numChan == numIn) or (numChan == 1)):
            raise Exception('Number of frequency vectors should either be 1 or match the number of vectors in x; value: {}'.format(numChan))

        freqE = np.sort(freqOpt.flatten())
        numFreq = freqE.shape[-1]

        freq = np.zeros((numIn, numFreq))
        Sxx = np.zeros((numIn, numFreq))
        Szz = np.zeros((numOut, numIn, numFreq))
        Cxz = np.zeros((numOut, numIn, numFreq))
        Sxz = np.zeros((numOut, numIn, numFreq), dtype=complex)
        Txz = np.zeros((numOut, numIn, numFreq), dtype=complex)

        optIn = copy.deepcopy(opt)
        optIn.freqInterp = freqE
        if optIn.interpType is None:
            optIn.interpType = 'linear'

        for iInput in range(0, numIn):
            if numChan == 1:
                freqOptIn = np.copy(freqOpt)
            else: # Multi-Input, Multi-FreqVector
                freqOptIn = np.copy(freqOpt[iInput])

            optIn.freq_rps = np.atleast_2d(freqOptIn)

            freq[iInput, :], Txz[:, iInput, :], Cxz[:, iInput, :], Sxx[iInput, :], Szz[:, iInput, :], Sxz[:, iInput, :] = FreqRespFuncEstSIMO(x[iInput], z, optIn)

    else: # Single Input

        freq, Txz, Cxz, Sxx, Szz, Sxz = FreqRespFuncEstSIMO(x, z, opt)

    # Ensure outputs are 2D
    freq = np.atleast_2d(freq)
    Txz = np.atleast_2d(Txz)
    Cxz = np.atleast_2d(Cxz)
    Sxx = np.atleast_2d(Sxx)
    Szz = np.atleast_2d(Szz)
    Sxz = np.atleast_2d(Sxz)

    return freq, Txz, Cxz, Sxx, Szz, Sxz


def FreqRespFuncEstNoise(x, z, opt = OptSpect()):

    import copy

    x = np.atleast_2d(x)
    z = np.atleast_2d(z)

    # If the input is multidimensional, recursively call
    numIn = x.shape[0]
    numOut = z.shape[0]
    if numIn > 1: # Multi-Input

        freqOpt = np.atleast_2d(opt.freq_rps)

        # Get the shape of the frequency vectors
        numChan = freqOpt.shape[0]

        if not ((numChan == numIn) or (numChan == 1)):
            raise Exception('Number of frequency vectors should either be 1 or match the number of vectors in x; value: {}'.format(numChan))

        freqE = np.sort(freqOpt.flatten())
        numFreq = freqE.shape[-1]

        freqN = np.sort(freqOpt.flatten())
        numFreqN = freqN.shape[-1]

        freq = np.zeros((numIn, numFreq))
        Sxx = np.zeros((numIn, numFreq))
        Szz = np.zeros((numOut, numIn, numFreq))
        Cxz = np.zeros((numOut, numIn, numFreq))
        Sxz = np.zeros((numOut, numIn, numFreq), dtype = complex)
        Txz = np.zeros((numOut, numIn, numFreq), dtype = complex)

        TUnc = np.zeros((numOut, numIn, numFreq))
        SxxNull = np.zeros((numIn, numFreqN))
        Snn = np.zeros((numOut, numIn, numFreqN))

        optIn = copy.deepcopy(opt)

        for iInput in range(0, numIn):
            if numChan == 1:
                freqOptIn = np.copy(freqOpt)
            else: # Multi-Input, Multi-FreqVector
                freqOptIn = np.copy(freqOpt[iInput])

            optIn.freq_rps = np.atleast_2d(freqOptIn)

            xIn = np.expand_dims(x[iInput], 0)
            freq[iInput, :], Txz[:, iInput, :], Cxz[:, iInput, :], Sxx[iInput, :], Szz[:, iInput, :], Sxz[:, iInput, :], TUnc[:, iInput, :], SxxNull[iInput, :], Snn[:, iInput, :] = FreqRespFuncEstNoiseSIMO(xIn, z, optIn)

    else: # Single-Input, Sigle-FreqVector

        freq, Txz, Cxz, Sxx, Szz, Sxz, TUnc, SxxNull, Snn = FreqRespFuncEstNoiseSIMO(x, z, opt)

    # Ensure outputs are 2D
    freq = np.atleast_2d(freq)
    Txz = np.atleast_2d(Txz)
    Cxz = np.atleast_2d(Cxz)
    Sxx = np.atleast_2d(Sxx)
    Szz = np.atleast_2d(Szz)
    Sxz = np.atleast_2d(Sxz)

    TUnc = np.atleast_2d(TUnc)
    SxxNull = np.atleast_2d(SxxNull)
    Snn = np.atleast_2d(Snn)


    return freq, Txz, Cxz, Sxx, Szz, Sxz, TUnc, SxxNull, Snn


# Interpolate freqN into freqE, in polar coordinates
def SmoothPolar(z, opt):
    amp = np.abs(z)
    theta = np.unwrap(np.angle(z))

    smoothAmp = Smooth(amp, opt.smooth)
    smoothTheta = Smooth(theta, opt.smooth)

    zE = smoothAmp * np.exp( 1j * smoothTheta )

    return zE

# Interpolate
def InterpVal(y, x, xE, kind = 'linear', fill_value = 'extrapolate'):

    interpF = interp.interp1d(np.squeeze(x), y, kind = kind, bounds_error = False, fill_value = fill_value)

    yE = interpF(np.squeeze(xE))

    return yE

# Interpolate in polar coordinates
def InterpPolar(z, freqN, freqE, kind = 'linear', fill_value = 'extrapolate'):
    amp = np.abs(z)
    theta = np.unwrap(np.angle(z))

    interpAmp = interp.interp1d(np.squeeze(freqN), amp, kind = kind, bounds_error = False, fill_value = fill_value)
    interpTheta = interp.interp1d(np.squeeze(freqN), theta, kind = kind, bounds_error = False, fill_value = fill_value)

    ampE = interpAmp(np.squeeze(freqE))
    thetaE = interpTheta(np.squeeze(freqE))

    zE = ampE * np.exp( 1j * thetaE )

    return zE


#%%
def Spectrum(x, opt = OptSpect()):
    '''
    x is real
    returns the onesided DFT
    fs in rps (required for correct power scale)
    freq in rps (required for correct power scale)
    '''

    # Get the shape of the inputs and outputs. Expand dimensions
    x = np.atleast_2d(x)
    opt.freq_rps = np.atleast_2d(opt.freq_rps)

    # Detrend and Window
    lenX = x.shape[-1]
    win = signal.get_window(opt.winType, lenX)

    if opt.detrendType != None:
        xWin = win * signal.detrend(x, type = opt.detrendType)
    else:
        xWin = win * x

    # Compute Power Scale
    scale = PowerScale(opt.scaleType, win, opt.freqRate_rps)

    # Compute the Fourier Transforms
    if opt.dftType.lower() == 'fft':
        if opt.freq_rps[0][0] is not None:
            raise ValueError('FFT frequencies vector must be None')
        freq, xDft = FFT(xWin, opt.freqRate_rps, fftReal = False)

        s = np.ones_like(freq)

        nfft = xDft.shape[-1]

        if nfft % 2: # Odd
            iNyq = int((nfft - 1)/2) + 1 # index of the Nyquist
            s[..., 1:iNyq] = 2
        else: # Even
            iNyq = int(nfft/2) + 1 # index of the Nyquist
            s[..., 1:iNyq-1] = 2

        freq = np.abs(freq[... ,:iNyq]) # Slice to keep only positive (and zero)
        s = s[..., :iNyq]
        xDft = xDft[..., :iNyq]

        # If the signal was detrended the zero frequency component is removed
        if opt.detrendType in ['constant', 'linear']:
            freq = freq[..., 1:]
            xDft = xDft[..., 1:]
            s = s[..., 1:]

        # Compute Power
        P = s * (xDft.conj() * xDft).real * scale

    if opt.dftType.lower() == 'dftmat':
        if opt.freq_rps is None:
            raise ValueError('DFT frequency vector must be provided')

        # Compute the z coordinates for transform
        freq = opt.freq_rps
        zPts = np.exp(1j * 2*pi * freq / opt.freqRate_rps)

        # Compute the Chirp-Z Transform (Generalized DFT) via a Matrix
        xDft, xDftHist = DftMat(xWin, zPts)
        if opt.asHist == True:
            xDft = xDftHist

        # scaling factor, double everything but the DC component
        s = 2 * np.ones_like(freq)
        s[freq == 0] = 1

        # Compute Power
        P = s * (xDft.conj() * xDft).real * scale

    if opt.dftType.lower() == 'czt':
        if opt.freq_rps is None:
            raise ValueError('CZT frequency vector must be provided')
        freq, xDft  = CZT(xWin, opt.freq_rps, opt.freqRate_rps)

        # scaling factor, double everything but the DC component
        s = 2 * np.ones_like(freq)
        s[freq == 0] = 1

        # Compute Power
        P = s * (xDft.conj() * xDft).real * scale


    # Smooth the Power
    P = Smooth(P, opt.smooth)

    # Ensure the outputs are at least 2D
    freq = np.atleast_2d(freq)
    xDft = np.atleast_2d(xDft)
    P = np.atleast_2d(P)

    return freq, xDft, P


#%% Spectrogram Functions
def SpectTime(t, x, lenStep = 1, opt = OptSpect()):
    '''
    x is real
    returns the onesided DFT
    fs in rps (required for correct power scale)
    freq in rps (required for correct power scale)
    '''

    lenX = len(x)
    lenFreq = opt.freq_rps.shape[-1]

    numSeg = int((lenX) / lenStep)
    P_mag = np.zeros((numSeg, lenFreq))
    tSpec_s = np.zeros((numSeg))
    for iSeg in range(0, numSeg):

        iSel = np.arange(0, iSeg+1)

        tSpec_s[iSeg] = t[iSeg]
        if len(iSel) > lenFreq:
            freq, _, P_mag[iSeg, ] = Spectrum(x[iSel], opt)

    return tSpec_s, freq, P_mag.T

def SpectSlide(t, x, lenSeg = 50, lenOverlap = 1, opt = OptSpect()):
    '''
    x is real
    returns the onesided DFT
    fs in rps (required for correct power scale)
    freq in rps (required for correct power scale)
    '''

    lenX = len(x)

    #freqMin_rps = (lenSeg / freqRate_hz) * hz2rps
    #opt.freq_rps = opt.freq_rps[...,freqMin_rps < freqExc_rps]

    lenFreq = opt.freq_rps.shape[-1]

    numSeg = int((lenX - lenSeg) / lenOverlap)
    P_mag = np.zeros((numSeg, lenFreq))
    tSpec_s = np.zeros((numSeg))
    for iSeg in range(0, numSeg):

        iSel = iSeg * lenOverlap + np.arange(0, lenSeg)
        iCent = iSeg * lenOverlap + lenSeg//2

        tSpec_s[iSeg] = t[iCent]

        freq, _, P_mag[iSeg, ] = Spectrum(x[iSel], opt)

    return tSpec_s, freq, P_mag.T


#%%
def PowerScale(scaleType, win, fs_rps = 1 * rps2hz):
    
    fs_hz = fs_rps * rps2hz
    
    # Compute the scaling for power
    if scaleType == 'density':
        scale = 1.0 / (fs_hz * (win*win).sum()) # if win = ones, scale = dt / N
    elif scaleType == 'spectrum':
        scale = 1.0 / win.sum()**2 # if win = ones, scale = 1 / N**2
    else:
        scale = 1
        raise ValueError('Unknown scaling: %r' % scaleType)

    return scale


#%%
#
def Gain(T, magUnit = 'dB'):
    gain = np.abs(T)

    if magUnit == 'dB':
        gain = 20.0 * np.log10(gain)

    return gain

#
def Phase(T, phaseUnit = 'rad', unwrap = False):
    phase = np.angle(T)

    if unwrap:
        phase = np.unwrap(phase, axis=-1)

    if phaseUnit == 'deg':
        phase = phase * rad2deg

    return phase

#
def GainPhase(T, magUnit = 'dB', phaseUnit = 'deg', unwrap = False):
    gain = Gain(T, magUnit)
    phase = Phase(T, phaseUnit, unwrap)

    return gain, phase

# Transfer Complimentary Sensitivity to Sensitivity
def TtoS(TNom, TUnc = None, TCoh = None):

    I = np.repeat([np.eye(TNom.shape[0])], TNom.shape[-1], axis=0).T

    SNom = I - TNom

    if TUnc is not None:
        SUnc = -TUnc
    else:
        SUnc = None

    if TCoh is not None:
        SCoh = TCoh #XXX - This is not the proper transform
    else:
        SCoh = None

    return SNom, SUnc, SCoh

# Transfer Sensitivity to Complimentary Sensitivity
def StoT(SNom, SUnc = None, SCoh = None):

    I = np.repeat([np.eye(SNom.shape[0])], SNom.shape[-1], axis=0).T

    TNom = I - SNom

    if SUnc is not None:
        TUnc = -SUnc
    else:
        TUnc = None

    if SCoh is not None:
        TCoh = SCoh #XXX - This is not the proper transform
    else:
        TCoh = None

    return TNom, TUnc, TCoh

# Transfer Sensitivity to Loop Function
def StoL(SNom, SUnc = None, SCoh = None):
    # Li = inv(TNom + TUnc) - I = LNom + LUnc
    # Sherman-Woodberry Identity, good for updating the A with B, or if B is singular
    # inv(A + B) = inv(A) - inv(I + inv(A) @ B) @ inv(A) @ B @ inv(A)
    
    # Hua's Identity, good for the inverse IF B is invertable!!
    # inv(A + B) = inv(A) - inv(A + A @ inv(B) @ A)
    
    # LNom = -I + TNom^-1
    # LUnc = -(I + TNom^-1 * TUnc)^-1 * TNom^-1 * TUnc * TNom^-1
    LNom = np.zeros_like(SNom, dtype = complex)
    LUnc = np.zeros_like(SUnc, dtype = complex)
    LCoh = np.zeros_like(SCoh)

    inv = np.linalg.inv
    I = np.eye(SNom.shape[0])
    for i in range(SNom.shape[-1]):
        SNomElem = SNom[...,i]
        SUncElem = SUnc[...,i]
        SNomInvElem = inv(SNomElem)

        LNom[...,i] = -I + SNomInvElem

        if SUnc is not None:
            # Sherman-Morrison-Woodbury Identity
            LUnc[...,i] = -inv(I + SNomInvElem @ SUncElem) @ SNomInvElem @ SUncElem @ SNomInvElem

        if SCoh is not None:
            LCoh[...,i] = SCoh[...,i] #XXX - This is not the proper transform

    return LNom, LUnc, LCoh


# Return complex frequency response
def FreqResp(sys, freq_rps):
    gain_mag, phase_rad, _ = control.freqresp(sys, omega = freq_rps)
    T = gain_mag * np.exp(1j * phase_rad)

    return T

def SigmaTemporal(THist):
    numSec, numOut, numIn, numFreq = THist.shape

    sHist = np.zeros((numSec, numFreq))

    for iSec in range(numSec):
        sHist[iSec, ...] = Sigma(THist[iSec, ...])

    return sHist

#
def Sigma(T):
    numOut, numIn, numFreq = T.shape

    # Shift dimension
    shiftFlag = False
    if numFreq > numOut:
        shiftFlag = True
        T = np.moveaxis(T, -1, 0)
        numFreq, numOut, numIn = T.shape

    if numIn != numOut:
        raise ValueError('Input to Sigma must be square')

    # SVD of T
    s = np.linalg.svd(T, full_matrices=True, compute_uv = False)

    if shiftFlag:
        s = np.moveaxis(s, 0, -1)

    return s

#
def VectorMargin(T, TUnc = None, pCrit = -1 + 0j, typeUnc = 'ellipse'):

    if TUnc is None: # There is no Uncertainty estimate, just return the distance between T and pCrit
        vmNom, vmUnc, vm = VectorMarginCirc(T, TUnc, pCrit)
    else:
        if typeUnc == 'circle':
            vmNom, vmUnc, vm = VectorMarginCirc(T, TUnc, pCrit)
        elif typeUnc == 'ellipse':
            vmNom, vmUnc, vm, vContact = VectorMarginEllipse(T, TUnc, pCrit)

    return vmNom, vmUnc, vm


#
def VectorMarginCirc(T, TUnc = None, pCrit = -1 + 0j, typeNorm = 'RSS'):

    vmNom = np.abs(T - pCrit)
    vm = None

    if TUnc is not None:
        if typeNorm.lower() == 'rms':
            vmUnc = np.sqrt(0.5) * np.abs(TUnc) # RMS
        elif typeNorm.lower() == 'max':
            vmUnc = np.max([TUnc.real, TUnc.imag]) # Max
        elif typeNorm.lower() == 'mean':
            vmUnc = np.mean([TUnc.real, TUnc.imag]) # Mean
        elif typeNorm.lower() == 'rss':
            vmUnc = np.abs(TUnc) # RSS

    else:
        vmUnc = 0.0

    # Uncertain Distance is the difference between Nominal and vmUnc Distance
    # If the point is inside the circle return the distance as negative
    vm = vmNom - vmUnc

    return vmNom, vmUnc, vm

#
def VectorMarginEllipse(T, TUnc, pCrit = -1 + 0j):

    # Transform coordinates so that T is shifted to [0,0]
    pCrit_new = T - pCrit

    # Nominal Distance
    vmNom = np.abs(pCrit_new)

    # Compute the Contact location in new coordinates
    vContact_new = np.zeros_like(pCrit_new, dtype='complex')
    inside = np.zeros_like(pCrit_new, dtype='bool')

    for indx in np.ndindex(pCrit_new.shape):
        a = abs(TUnc[indx].real)
        b = abs(TUnc[indx].imag)
        p = [pCrit_new[indx].real, pCrit_new[indx].imag]
        pC, i = EllipsePoint(a, b, p)

        vContact_new[indx] = pC[0] + 1j*pC[1]
        inside[indx] = i

    # Transform back to original coordinates
    vContact = T + vContact_new

    # Compute the distance to the contact point
    vm = np.abs(vContact - pCrit)
    vm[inside] = -vm[inside] # If the point is inside the ellipse return the distance as negative

    # vm is the difference between Nominal and Uncertain Distance
    vmUnc = np.abs(vContact - T)

    return vmNom, vmUnc, vm, vContact


# Find the closes point between a give location to edge of ellipse
# If the solution is non-unique this will only return one solution
# https://github.com/0xfaded/ellipse_demo/issues/1
def EllipsePoint(a, b, p):

    px = abs(p[0])
    py = abs(p[1])

    # Check if point lies inside or on the ellipse
    inside = ((px**2) // (a**2)) + ((py**2) // (b**2)) <= 1.0

    tx = 0.707
    ty = 0.707

    for indx in range(0, 3):
        x = a * tx
        y = b * ty

        ex = (a*a - b*b) * tx**3 / a
        ey = (b*b - a*a) * ty**3 / b

        rx = x - ex
        ry = y - ey

        qx = px - ex
        qy = py - ey

        r = np.hypot(rx, ry)
        q = np.hypot(qx, qy)

        tx = min(1, max(0, (qx * r / q + ex) / a))
        ty = min(1, max(0, (qy * r / q + ey) / b))

        t = max(np.finfo(np.float32).eps, np.hypot(tx, ty))

        tx /= t
        ty /= t

    vContact = (np.copysign(a*tx, p[0]), np.copysign(b*ty, p[1]))

    return vContact, inside


# Distance to Ellipse, with rotation
def DistEllipseRot(pEllipse, a, b, a_deg, pCrit):

    # Transform coordinates so that pEllipse is at [0,0] and angle=0
    a_rad = a_deg * deg2rad
    aCos = np.cos(a_rad)
    aSin = np.sin(a_rad)

    R = np.array([[aCos, -aSin],[aSin, aCos]])

    pCrit_new = R.T @ (pCrit - pEllipse)

    # Compute the Contact location in new coordinates
    vContact_new, inside = EllipsePoint(a, b, pCrit_new)

    # Transform back to original coordinates
    vContact = R @ vContact_new + pEllipse

    # Compute the distance to the contact point
    vDist = np.linalg.norm(vContact - pCrit, 2)

    # If the point is inside the ellipse return the distance as negative
    if inside:
        vDist = -abs(vDist)

    return vContact, vDist

from scipy import optimize
def Sigma_SSV(M):
    # Use equation 8.87 and minimise directly

    def objFunc(d, M):
        scale = 1 / d[0] # scale d such that d1 == 1 as in SP note 10 of 8.8.3
        d_scaled = scale * d
        D = np.diag(d_scaled)
        Dinv = np.diag(1 / d_scaled)
        M_scaled = D @ M @ Dinv

        s = np.linalg.svd(M_scaled, full_matrices = True, compute_uv = False)
        sMax = np.max(s, axis = 0)
        return sMax

    d = np.ones_like(M[0,...], dtype = float)
    sMax = np.zeros(M.shape[-1])
    for i in range(M.shape[-1]):
        res = optimize.minimize(objFunc, d[...,i], args = M[...,i])
        d[...,i] = res.x
        sMax[i] = res.fun

    return sMax, d


#%% Compute the Fast Fourier Transform
def FFT(x, fs, fftReal = False):
    '''
    Ripped and modified for limited scope from scipy _spectral_helper
    returns the onesided DFT
    if fs is in Hz, then freq will also be in Hz
    '''

    # Ensure x is an array
    x = np.atleast_2d(np.asarray(x))

    # Compute the discrete fourier transform
    nfft = x.shape[-1]

    if fftReal == True:
        # Form the frequency vector
        freq = np.fft.rfftfreq(nfft, 1/fs)

        # Perform the fft
        xDft = np.fft.rfft(x, n = nfft)

    else: # Use the full fft, slightly more accurate but twice the computation
        # Form the frequency vector
        freq = np.fft.fftfreq(nfft, 1/fs)

        # Perform the fft
        xDft = np.fft.fft(x, n = nfft)

    # Output as 2D
    freq = np.atleast_2d(freq)
    xDft = np.atleast_2d(xDft)

    return freq, xDft


#%% Compute the DFT Transform via Matrix
def DftMat(x, zk, N = None):
    '''
    x is real
    returns the onesided DFT
    '''

    # Ensure x and freq are arrays
    x = np.atleast_2d(x)
    zk = np.atleast_2d(zk)

    # Length
    if N is None:
        N = x.shape[-1]

    # Index
    n = np.arange(N, dtype = float) # time indices

    xDftHist = np.empty((x.shape[0], zk.shape[-1], N), dtype = complex)
    for iIn in range(x.shape[0]):

        # Compute the Chirp-Z Matrix
        # DftMat = np.power(zk.T, -n).T
        DftMat = np.power(np.atleast_2d(zk[iIn]).T, -n).T

        # Compute the Chirp-Z Transform
        # xDft = x @ DftMat # This works... but doesn't provide history

        # xDftHist = (x.T * DftMat).cumsum(axis = 0) # Inner product to get the full history
        xDftHist[iIn, ...] = (np.atleast_2d(x[iIn]).T * DftMat).T

    # Summation
    xDft = xDftHist.sum(axis = -1)


    return xDft, xDftHist


#%% Compute the Chirp-Z Transform via Blustein Algorithm
def CZT(x, freq, fs, N = None):
    '''
    x is real
    returns the onesided DFT
    '''

    # Ensure x and freq are arrays
    x = np.atleast_2d(x)
    freq = np.atleast_2d(freq)

    # Length
    if N is None:
        N = x.shape[-1]

    # Algorithm is intended for fixed intervals
    # Provided frequency vector will be pulled apart and later re-formed to ensure fixed intervals
    freqMin = np.min(freq)
    freqMax = np.max(freq)

    M = freq.shape[-1]
    freqStep = (freqMax - freqMin) / (M - 1)

    # Starting point and ratio between points
    A = np.exp(1j * 2*pi * freqMin / fs)
    W = np.exp(-1j * 2*pi * freqStep / fs)

    # Indices
    k = np.arange(M) # Frequencies
    n = np.arange(N, dtype=float) # Times

    Wk2 = np.power(W, k**2 / 2.)
    AnWn2 = np.power(A, -n) * np.power(W, n**2 / 2.)

    # Helper objects
    fft = np.fft.fft
    ifft = np.fft.ifft

    nfft = int(2**np.ceil(np.log2(M+N-1)))

    # Pre-compute the Chirp Filter
    v = np.zeros(nfft, dtype=np.complex)
    v[:M] = np.power(W, -n[:M]**2/2.)
    v[nfft-N+1:] = np.power(W, -n[N-1:0:-1]**2/2.)
    vFft = fft(v, nfft)

    # Compute the Chirp-Z
    xCzt = Wk2 * ifft(vFft * fft(AnWn2 * x, nfft))[..., :M]

    # Re-form the frequency vector
    freq = k * freqStep + freqMin

    return freq, xCzt


#%% Compute the DFT Transform recursively
from collections import deque
class DftSlide:
    
    def __init__(self, freq_rps, freqRate_rps = 2*pi, N = 1):
        self.iDft = int(0)
        self.freqRate_rps = freqRate_rps
        self.freq_rps = np.array(freq_rps)

        self.M = self.freq_rps.shape[-1]
        self.N = np.max((self.M, N))
        self.freq_nd = self.freq_rps / freqRate_rps # frequencies non-dimensionalized
        self.Wk = np.exp(2j*pi * self.freq_nd )
        self.Wn = deque(np.array([]))
        
        self.UpdateCoef()
        
        self.x = deque([i for i in np.zeros(N, dtype=float)])
        self.xDft = np.zeros(self.M, dtype=complex)
        
        self.xDftMean = 0.0
        self.xDftStd = 0.0
        
    def UpdateCoef(self):
        # Update the Fourier Coefficients
    	self.Wn.append(pow(self.Wk, self.iDft));
    
    def Append(self, x):
        self.iDft += 1
        self.x.append(x) # Append new x to que
        self.UpdateCoef() # Update the Coefs
        
        self.xDft += self.x[-1] * self.Wn[-1] # Add to the end
    
    def PopFront(self):
        self.xDft -= self.x[0] * self.Wn[0] # Remove from start
        
        self.Wn.popleft()
        self.x.popleft()
        
    def Slide(self, x):
        self.Append(x)
        self.PopFront()
    
    def Update(self, x):
        
        if len(self.x) < self.N: # Append to end until length N is reached
            self.Append(x)
        else: # Slide
            self.Slide(x)


#%% Smooth Data with Convolution
def Smooth(x, kern = ('box', 1), padMode = 'edge', convMode = 'valid'):
    if type(kern) is tuple:
        if kern[0] in ['box', 'flat']:
            v = np.ones(kern[1])
        else:
            raise ValueError('Unknown kernel description: %r' % kern[0])

    elif len(kern) > 1:
        v = kern # kern is a list or ndarray
    else:
        raise ValueError('Unknown kernel value: %r' % kern)

    # Normalize
    v = np.asarray(v)
    v = v / v.sum()

    # Required pad length
    nPad = len(v) // 2 # length of the required pad

    x = np.atleast_2d(x)

    # Pad and Convolve x
    y = []
    for indxX in range(x.shape[0]):
        xPad = np.pad(x[indxX], nPad, mode = padMode)
        xSmooth = np.convolve(xPad, v, mode = convMode).tolist()
        y.append(xSmooth)

    xSmooth = np.array(y)

    return xSmooth

#%% Plotting
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import matplotlib.figure
import matplotlib.gridspec as gridspec
import matplotlib.patches as patch
from mpl_toolkits.mplot3d import Axes3D

# For print pretty to latex
mpl.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'text.usetex': True,
    'pgf.rcfonts': False,
})

# Plot the Spectrogram
def PlotSpectogram(t, freq, P, dim='2D'):
    import matplotlib.pyplot as plt

    tArray, freqArray = np.meshgrid(t, freq)

    fig = plt.figure()
    fig.tight_layout()

    if dim == '3D':
        ax = fig.gca(projection='3d', proj_type = 'ortho')
        ax.view_init(elev = 90.0, azim = -90.0)

        ax.plot_surface(tArray, freqArray, P, rstride=1, cstride=1, cmap='nipy_spectral')

        ax.set_zlabel('Power ( )')
    else:
        ax = fig.gca()

        pcm = ax.pcolormesh(tArray, freqArray, P, cmap='nipy_spectral')
        fig.colorbar(pcm, ax=ax, label = 'Power ( )')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Frequency ( )')

    return fig

# Bode/VM/Sigma uncertainty points, as Circles
def PlotGainUncPts(freq, gain_mag, gainUnc_mag, ax = None, **pltArgs):
    for iFreq, f in enumerate(freq):
        gNom = gain_mag[iFreq]
        gUnc = gainUnc_mag[iFreq]

        uncPatch = patch.Ellipse((f, gNom), 2*gUnc, 2*gUnc, alpha = 0.25, **pltArgs)
        ax.add_artist(uncPatch)

    return ax

# Bode/VM/Sigma uncertainty points, as Fill
def PlotGainUncFill(freq, gainMin, gainMax, ax = None, **pltArgs):
    ax.fill_between(freq, gainMin, gainMax, alpha = 0.25, **pltArgs)

    return ax


# SISO Bode/VM/Sigma Plot
def PlotGainType(freq, gain_mag, phase_deg = None, coher_nd = None, gainUnc_mag = None, fig = None, dB = True, UncSide = None, **pltArgs):

    plotCohere = False
    if coher_nd is not None:
        plotCohere = True

    plotPhase = False
    if phase_deg is not None:
        plotPhase = True

    if isinstance(fig, matplotlib.figure.Figure):
        ax = fig.get_axes()

    elif isinstance(fig, int) and plt.fignum_exists(fig):
        fig = plt.figure(fig) # Get the figure handle by label
        ax = fig.get_axes()
    else:
        fig = plt.figure(constrained_layout = True, num = fig)
        ax = []

        if plotCohere and plotPhase: # Gain, Phase, and Coherence
            spec = fig.add_gridspec(ncols=1, nrows=3, height_ratios=[1,1,1])
        elif plotCohere: # Gain and Coherence
            spec = fig.add_gridspec(ncols=1, nrows=2, height_ratios=[2,1])
        elif plotPhase: # Gain and Phase
            spec = fig.add_gridspec(ncols=1, nrows=2, height_ratios=[1,1])
        else:
            spec = fig.add_gridspec(ncols=1, nrows=1, height_ratios=[1])

        for i, s in enumerate(spec):
            if i == 0:
                ax.append(fig.add_subplot(s)) # Gain
            else:
                ax.append(fig.add_subplot(s, sharex=ax[0]))

        for a in ax[:-1]:
            plt.setp(a.get_xticklabels(), visible=False)

    # Plot the Gain
    if dB:
        ax[0].semilogx(freq, mag2db(gain_mag), **pltArgs)
        ax[0].set_ylabel('Gain [dB]')
    else:
        ax[0].semilogx(freq, gain_mag, **pltArgs)
        ax[0].set_ylabel('Gain [mag]')

    # Plot the Phase
    if plotPhase:
        ax[1].semilogx(freq, phase_deg, **pltArgs)
        ax[1].yaxis.set_major_locator(plticker.MultipleLocator(90))
        ax[1].grid(True)
        ax[1].set_ylabel('Phase [deg]')

    # Plot the Coherence
    if plotCohere:
        if coher_nd is None:
            coher_nd = np.ones_like(freq)

        ax[-1].semilogx(freq, coher_nd, **pltArgs)
        ax[-1].grid(True)
        ax[-1].set_ylabel('Coherence [mag]')
        ax[-1].set_ylim(0, 1)

    # Plot the Uncertainty on the Gain plot
    if gainUnc_mag is not None:

        ax[0].set_xscale("log", nonposx='clip')

        if UncSide == 'Max':
            gainUncMin_mag = gain_mag
        else:
            gainUncMin_mag = gain_mag - 0.5 * np.abs(gainUnc_mag)

        if UncSide == 'Min':
            gainUncMax_mag = gain_mag
        else:
            gainUncMax_mag = gain_mag + 0.5 * np.abs(gainUnc_mag)

        if 'marker' in pltArgs: pltArgs.pop('marker')

        if dB:
            gainUncMax_dB = mag2db(gainUncMax_mag)
            gainUncMin_dB = mag2db(gainUncMin_mag)

            ax[0] = PlotGainUncFill(freq, gainUncMin_dB, gainUncMax_dB, ax = ax[0], **pltArgs)

        else:
            ax[0] = PlotGainUncFill(freq, gainUncMin_mag, gainUncMax_mag, ax = ax[0], **pltArgs)


    # Make prettier
    ax[0].grid(True)
    ax[-1].set_xlabel('Freq [Hz]')
    ax[0].legend(loc = 'right')

    return fig

# SISO Bode Plot
def PlotBode(freq, gain_mag, phase_deg, coher_nd = None, gainUnc_mag = None, fig = None, dB = False, **pltArgs):
    fig = PlotGainType(freq, gain_mag, phase_deg, coher_nd, gainUnc_mag, fig = fig, dB = dB, **pltArgs)

    ax = fig.get_axes()
    ax[0].set_xscale("log")

    return fig

# SISO Vector Margin Plot
def PlotVectorMargin(freq, vm_mag, coher_nd = None, vmUnc_mag = None, fig = None, **pltArgs):
    fig = PlotGainType(freq, vm_mag, None, coher_nd, vmUnc_mag, fig = fig, dB = False, UncSide='Min', **pltArgs)

    ax = fig.get_axes()
    ax[0].set_xscale("linear")
    ax[0].set_ylabel(ax[0].get_ylabel().replace('Gain', 'Vector Margin'))

    return fig

# SISO Sigma Plot
def PlotSigma(freq, sv_mag, coher_nd = None, svUnc_mag = None, fig = None, **pltArgs):
    fig = PlotGainType(freq, sv_mag, None, coher_nd, svUnc_mag, fig = fig, dB = False, UncSide='Min', **pltArgs)

    ax = fig.get_axes()
    ax[0].set_xscale("linear")
    ax[0].set_ylabel(ax[0].get_ylabel().replace('Gain', 'Sigma'))

    return fig

# Bode/VM/Sigma Time History Plot
def PlotGainTemporal(time_s, gain_mag, phase_deg, coher_nd = None, gainUnc_mag = None, fig = None, dB = True, UncSide = None, **pltArgs):
    fig = PlotGainType(time_s, gain_mag, phase_deg, coher_nd, gainUnc_mag, fig = fig, dB = dB, UncSide = UncSide, **pltArgs)

    ax = fig.get_axes()
    ax[0].set_xscale("linear")
    ax[-1].set_xlabel("Time [s]")

    return fig

# SISO Vector Margin Plot - Temporal
def PlotVectorMarginTemporal(time_s, vm_mag, coher_nd = None, vmUnc_mag = None, fig = None, dB = False, **pltArgs):
    fig = PlotGainTemporal(time_s, vm_mag, None, coher_nd, vmUnc_mag, fig = fig, dB = dB, UncSide = 'Min', **pltArgs)

    ax = fig.get_axes()
    ax[0].set_ylabel(ax[0].get_ylabel().replace('Gain', 'Vector Margin'))

    return fig

# SISO Sigma Plot - Temporal
def PlotSigmaTemporal(time_s, sv_mag, coher_nd = None, svUnc_mag = None, fig = None, dB = False, **pltArgs):
    fig = PlotGainTemporal(time_s, sv_mag, None, coher_nd, svUnc_mag, fig = fig, dB = dB, UncSide = 'Min', **pltArgs)

    ax = fig.get_axes()
    ax[0].set_ylabel(ax[0].get_ylabel().replace('Gain', 'Sigma'))

    return fig


# Nyquist uncertainty points, as Circles
def PlotNyquistUncPts(T, TUnc, ax = None, **pltArgs):

    for iNom, nom in enumerate(T):
        unc = TUnc[iNom]
        if unc.imag == 0:
            uncPatch = patch.Ellipse((nom.real, nom.imag), unc, unc, alpha = 0.25, **pltArgs)
        else:
            uncPatch = patch.Ellipse((nom.real, nom.imag), unc.real, unc.imag, alpha = 0.25, **pltArgs)

        ax.add_artist(uncPatch)

    return ax

# Nyquist uncertainty points, Fill
def PlotNyquistUncFill(T, TUnc, ax = None, **pltArgs):

    diffT = np.diff(T, append=0)
    pathVec = diffT / np.abs(diffT)
    perpVec = pathVec.imag - 1j * pathVec.real
    TMin = T - perpVec * 0.5 * np.abs(TUnc)
    TMax = T + perpVec * 0.5 * np.abs(TUnc)

    re = np.hstack((TMin.real, np.flip(TMax.real)))
    im = np.hstack((TMin.imag, np.flip(TMax.imag)))

    ax.fill(re, im, alpha = 0.25, **pltArgs)

    return ax

# SISO Nyquist Plot
def PlotNyquist(T, TUnc = None, fig = None, fillType = 'circle', **pltArgs):

    if isinstance(fig, mpl.figure.Figure):
        ax = fig.get_axes()

    elif isinstance(fig, int) and plt.fignum_exists(fig):
        fig = plt.figure(fig) # Get the figure handle by label
        ax = fig.get_axes()
    else:
        fig = plt.figure(constrained_layout = True, num = fig)
        spec = fig.add_gridspec(ncols=1, nrows=1, height_ratios=[1])
        ax = []
        ax.append(fig.add_subplot(spec[0,0]))

    # Make the plots
    ax[0].plot(T.real, T.imag, **pltArgs)

    if 'marker' in pltArgs: pltArgs.pop('marker')

    if TUnc is not None:
        if fillType == 'circle':
            ax[0] = PlotNyquistUncPts(T, np.abs(TUnc), ax = ax[0], **pltArgs)
        elif fillType == 'elipse':
            ax[0] = PlotNyquistUncPts(T, TUnc, ax = ax[0], **pltArgs)
        elif fillType == 'fill':
            ax[0] = PlotNyquistUncFill(T, TUnc, ax = ax[0], **pltArgs)

    ax[0].grid(True)
    ax[0].set_xlabel('Real [mag]')
    ax[0].set_ylabel('Imag [mag]')
    ax[0].legend(loc = 'right')

    return fig


def PlotGetExist(ioArray, fig = None):

    ncols, nrows = ioArray.shape[0:-1]
    if isinstance(fig, mpl.figure.Figure):
        fig

    elif isinstance(fig, int) and plt.fignum_exists(fig):
        fig = plt.figure(fig) # Get the figure handle by label

    else:
        fig = plt.figure(constrained_layout = True, num = fig)
        spec = fig.add_gridspec(ncols = ncols, nrows = nrows)
        ioArray = np.reshape(ioArray, (ncols*nrows,-1))

        for iPlot in ioArray:
            [iOut, iIn] = iPlot
            fig.add_subplot(spec[iOut, iIn])

    # Get the fig axes
    ax = fig.get_axes()

    ax = np.asarray(ax)
    ax = ax.reshape(ncols, nrows).T

    return fig, ax

# Fix the Legends to that the Nominal and Uncertainty use the same label
def FixLegend(ax):
    handles, labels = ax.get_legend_handles_labels()
    lNew = []
    hNew = []

    for iHandle, handle in enumerate(handles):
        label = labels[iHandle]
        if label in labels[iHandle+1:]:
            iMatch = labels[iHandle+1:].index(label)+1
            hNew.append((handle, handles[iMatch]))

            labels[iMatch] = None
            handles[iMatch] = None
        else:
            hNew.append(handle)

        lNew.append(label)

        ax.legend(hNew, lNew)

    return ax


def PrintPrettyFig(fig, fileName, backend='pgf', size_inches = None):
    backend_bak = mpl.get_backend()
    if size_inches is None:
        size_inches = fig.get_size_inches()

    mpl.use(backend)
    fig.set_size_inches(size_inches)
    fig.savefig(fileName)

    mpl.use(backend_bak)
