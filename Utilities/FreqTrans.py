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

# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps

rad2deg = 180/np.pi


#%% Estimate Transfer Function from Time history data
def FreqRespFuncEstNoise(x, y, fs, freqE = None, freqN = None, dftType = 'fft', winType = ('tukey', 0.0), detrendType = 'constant', smooth = ('box', 1), scaleType = 'spectrum'):
    '''
    Estimates the Transfer Function Response from input/output time histories
    Single-Input Multi-Output at a Single-FreqVector
    
    x and y are real and must be the same length
    Assumes x and y have uniform spacing

    x has dimension (1, p) or (p,) at input and is expanded to (1, p)
    y has dimension (n, p) or (p,) at input and is expanded to (n, p)
    Pxx has dimension (1, r) and is reduced to (1, r) or (r,) depending on input form of x
    Pyy has dimension (n, r) and is reduced to (n, r) or (r,) depending on input form of x
    Pxy, Cxy, and Txy has dimension (n, r) or (r,)

        m is the number of input signals
        n is the number of output signals
        p is the length of the each signal
        r is the length of the freq vector
        
    returns the onesided DFT
    fs and freq must have same units. (Puu and Pyy will only have correct power scale if units are rad/sec)
    
    '''
    
    # Compute the Power Spectrums    
    _, xDft_E, Pxx_E = Spectrum(x, fs, freqE, dftType, winType, detrendType, smooth, scaleType)
    _, yDft_E, Pyy_E = Spectrum(y, fs, freqE, dftType, winType, detrendType, smooth, scaleType)
    _, yDft_N, Pyy_N = Spectrum(y, fs, freqN, dftType, winType, detrendType, smooth, scaleType)
        
    # Interpolate freqN into freqE, in polar coordinates
    def interpPolar(z, freqN, freqE):
        amp = np.abs(z)
        theta = np.angle(z)
                
        interpAmp = interp.interp1d(freqN, amp, fill_value="extrapolate")
        interpTheta = interp.interp1d(freqN, theta, fill_value="extrapolate")

        ampE = interpAmp(freqE)
        thetaE = interpTheta(freqE)
    
        zE = ampE * np.exp( 1j * thetaE )
            
        return zE
    
    yDft_N = interpPolar(yDft_N, freqN, freqE)
    Pyy_N = interpPolar(Pyy_N, freqN, freqE)
    
    # Compute Cross Spectrum Power with scaling
    lenX = x.shape[-1]
    win = signal.get_window(winType, lenX)
    scale = PowerScale(scaleType, fs, win)
    
    Pxy_E = np.conjugate(xDft_E) * yDft_E * 2*scale # Scale is doubled because one-sided DFT
    Pxy_N = np.conjugate(xDft_E) * yDft_N * 2*scale # Scale is doubled because one-sided DFT
    
    Pxx = Pxx_E
#    Pyy = Pyy_E - Pyy_N
    Pyy = Pyy_E
#    Pxy = Pxy_E - Pxy_N
    Pxy = Pxy_E
    
    # Smooth - 
    Pxy_smooth = Smooth(np.copy(Pxy), smooth)
    
    # Coherence, use the Smoothed Cross Spectrum
    Cxy = np.abs(Pxy_smooth)**2 / (Pxx * Pyy)
    
    # Compute complex transfer function approximation
    Txy = Pxy / Pxx
    TxyUnc = Pxy_N / Pxx

    return freqE, Txy, Cxy, Pxx, Pyy, Pxy, TxyUnc



#%% Estimate Transfer Function from Time history data
def FreqRespFuncEst(x, y, fs, freq = None, dftType = 'fft', winType = ('tukey', 0.0), detrendType = 'constant', smooth = ('box', 1), scaleType = 'spectrum'):
    '''
    Estimates the Transfer Function Response from input/output time histories
    Single-Input Multi-Output at a Single-FreqVector
    
    x and y are real and must be the same length
    Assumes x and y have uniform spacing

    x has dimension (1, p) or (p,) at input and is expanded to (1, p)
    y has dimension (n, p) or (p,) at input and is expanded to (n, p)
    Pxx has dimension (1, r) and is reduced to (1, r) or (r,) depending on input form of x
    Pyy has dimension (n, r) and is reduced to (n, r) or (r,) depending on input form of x
    Pxy, Cxy, and Txy has dimension (n, r) or (r,)

        m is the number of input signals
        n is the number of output signals
        p is the length of the each signal
        r is the length of the freq vector
        
    returns the onesided DFT
    fs and freq must have same units. (Pxx and Pyy will only have correct power scale if units are rad/sec)
    
    '''
    
    # Get the shape of the inputs and outputs. Expand dimensions
    xSingleton = False
    if len(x.shape) is 1:
        xSingleton = True
        x = x[np.newaxis, :]
    
    ySingleton = False
    if len(y.shape) is 1:
        ySingleton = True
        y = y[np.newaxis, :]

    
    # Compute the Power Spectrums
    _   , xDft, Pxx = Spectrum(x, fs, freq, dftType, winType, detrendType, smooth, scaleType)
    freq, yDft, Pyy = Spectrum(y, fs, freq, dftType, winType, detrendType, smooth, scaleType)
    
    
    # Compute Cross Spectrum Power with scaling
    lenX = x.shape[-1]
    win = signal.get_window(winType, lenX)
    scale = PowerScale(scaleType, fs, win)
    
    Pxy = np.conjugate(xDft) * yDft * 2*scale # Scale is doubled because one-sided DFT
    
    # Smooth - 
#    Pxy_smooth = Smooth(np.copy(Pxy), smooth)
    Pxy_smooth = Smooth(np.abs(Pxy), smooth) * np.exp(1j * Smooth(np.angle(Pxy), smooth))
    
    # Coherence, use the Smoothed Cross Spectrum
    Cxy = np.abs(Pxy_smooth)**2 / (Pxx * Pyy)
    
    # Compute complex transfer function approximation
    Txy = Pxy / Pxx
    
    # Collapse the singlton dimensions, if x and/or y where singleton
    if xSingleton:
        Pxx = np.squeeze(Pxx)
        
    if ySingleton:
        Pyy = np.squeeze(Pyy)
        Pxy = np.squeeze(Pxy)
        Cxy = np.squeeze(Cxy)
        Txy = np.squeeze(Txy)
        
    return freq, Txy, Cxy, Pxx, Pyy, Pxy


#%%
def Spectrum(x, fs, freq = None, dftType = 'fft', winType = ('tukey', 0.0), detrendType = 'constant', smooth = ('box', 1), scaleType = 'spectrum'):
    '''
    x is real
    returns the onesided DFT
    fs in rps (required for correct power scale)
    freq in rps (required for correct power scale)
    
    
    '''

    # Detrend and Window
    lenX = x.shape[-1]
    win = signal.get_window(winType, lenX)
    xWin = win*x
    
    # Compute Power scaling
    scale = PowerScale(scaleType, fs, win)
        
    # Compute the Fourier Transforms    
    if dftType == 'fft':
        freq, xDft  = FFT(xWin, fs)
        
        # Compute Power
        P = (np.conjugate(xDft) * xDft).real * scale
                
        if len(P) % 2:
            P[..., 1:] *= 2
        else:
            # Last point is unpaired Nyquist freq point, don't double
            P[..., 1:-1] *= 2

        # If the signal was detrended the zero frequency component is removed
        if detrendType in ['constant', 'linear']:
            freq = freq[1:]
            xDft = xDft[..., 1:]
            P = P[..., 1:]
        
    if dftType == 'czt':
        if freq is None:
            raise ValueError('CZT frequency vector must be provided')
        freq, xDft  = CZT(xWin, freq, fs)
    
        # Compute Power, factor of 2 because CZT is one-sided
        P = (np.conjugate(xDft) * xDft).real * 2*scale
    
    # Smooth the Power
    P = Smooth(P, kern = smooth)

    return freq, xDft, P


#%%
def PowerScale(scaleType, fs, win):
    
    # Compute the scaling for power
    if scaleType == 'density':
        scale = 1.0 / (fs * (win*win).sum())
    elif scaleType == 'spectrum':
        scale = 1.0 / win.sum()**2
    else:
        scale = 1
        raise ValueError('Unknown scaling: %r' % scaleType)
    
    return scale


#%%
def GainPhase(Txy):

    gain_dB = 20.0 * np.log10(np.abs(Txy))
    phase_deg = np.angle(Txy) * rad2deg

    return gain_dB, phase_deg


#%%
def Welchize(x, nperseg = 1, noverlap = 0):
    # Reference: scipy.signal._fft_helper
    
    step = nperseg - noverlap
    shape = x.shape[:-1]+((x.shape[-1] - noverlap) // step, nperseg)
    strides = x.strides[:-1]+(step*x.strides[-1], x.strides[-1])
    result = np.lib.stride_tricks.as_strided(x, shape = shape, strides = strides)
    
    return result


#%% Compute the Fast Fourrier Transform
def FFT(x, fs):
    '''
    Ripped and modified for limited scope from scipy _spectral_helper
    returns the onesided DFT
    '''
    
    # Ensure x is an array
    x = np.asarray(x)
    
    xSingleton = False
    if len(x.shape) is 1:
        xSingleton = True
        x = x[np.newaxis,:]
    
    # Compute the discrete fourier transform
    nfft = x.shape[-1]
    
    # Form the frequency vector
    freq = np.fft.rfftfreq(nfft, 1/fs)
    
    # Perform the fft
    xDft = np.fft.rfft(x, n = nfft)

    if xSingleton:
        xDft = np.squeeze(xDft)


    return freq, xDft


#%% Compute the Chirp-Z Fourrier Transform
def CZT(x, freq, fs):
    '''
    x is real
    returns the onesided DFT
    '''
    
    # Ensure x is an array
    x = np.asarray(x)
    
    # Length x should be even, remove the last data to make even
    lenX = x.shape[-1]
    if lenX % 2 is not True:
        x = x[..., :-1]
        lenX = x.shape[-1]
    
    # This Chirp-Z algorithm is intended for fixed intervals
    # The provided frequency vector will be pulled apart and later re-formed to ensure fixed intervals
    freqMin = np.min(freq)
    freqMax = np.max(freq)
    
    mChirp = len(freq)
    freqStep = (freqMax - freqMin) / (mChirp - 1)
    
    # Ratio between points
    wChirp = np.exp(-1j * (2*np.pi / fs) * freqStep);
    
    # Starting point
    aChirp = np.exp(1j * 2*np.pi * freqMin / fs)
    
    # Compute the ChirpZ
    n = -np.arange(lenX)
    k = -np.arange(mChirp)
    nk = np.outer(n, k)
    
    xDft = np.inner((wChirp ** nk).transpose(), (aChirp ** n) * x).transpose()
    
    # Re-form the frequency vector
    freq = -k * freqStep + freqMin
        

    return freq, xDft


#%% Smooth Data with Convolution
def Smooth(x, kern = ('box', 1), padMode = 'edge', convMode = 'valid'):
    '''
    
    '''
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
    
    xSingleton = False
    if len(x.shape) is 1:
        xSingleton = True
        x = x[np.newaxis,:]

    # Pad and Convolve x
    for iX in range(x.shape[0]):
        xPad = np.pad(x[iX], nPad, mode = padMode)
        x[iX] = np.convolve(xPad, v, mode = convMode)

    
    if xSingleton:
        x = np.squeeze(x)
        
    return x