"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np
import scipy.signal as signal

# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps

rad2deg = 180/np.pi


#%% Estimate Transfer Function from Time history data
def TransFuncEst(x, y, fs = 1.0, freq = None, dftType = 'fft', winType = ('tukey', 0.0), detrendType = 'constant', scaleType = 'spectrum'):
    '''
    x and y are real [x.shape[-1] equals y.shape[-1]]
    returns the onesided DFT
    fs and freq must have same units. (Pxx and Pyy will only have correct power scale if units are rps)
    
    '''
    
    

    # Compute the Power Spectrums
    _   , xDft, Pxx = Spectrum(x, fs, freq, dftType, winType, detrendType, scaleType)
    freq, yDft, Pyy = Spectrum(y, fs, freq, dftType, winType, detrendType, scaleType)
    
    
    # Compute Cross Spectrum Power with scaling
    lenX = x.shape[-1]
    win = signal.get_window(winType, lenX)
    scale = PowerScale(scaleType, fs, win)
    
    
    if len(xDft.shape) > 1:
        xDft = xDft[np.newaxis,...]
        Pxx = Pxx[np.newaxis,...]
    
    
    if len(yDft.shape) > 1:
        yDft = yDft[:,np.newaxis,:]
        Pyy = Pyy[:,np.newaxis,:]
    

    Pxy = np.conjugate(xDft) * yDft * 2*scale # Scale is doubled because one-sided FFT
    
    # Coherence
    Cxy = (abs(Pxy)**2 / (Pxx * Pyy)).real
        
    # Compute complex transfer function approximation
    Txy = Pxy / Pxx
    
    # Gain and Phase
    gain_dB, phase_deg = GainPhase(Txy)
    
    
    return freq, gain_dB, phase_deg, Cxy, Txy, Pxx, Pyy, Pxy


#%%
def Spectrum(x, fs = 1.0, freq = None, dftType = 'fft', winType = ('tukey', 0.0), detrendType = 'constant', scaleType = 'spectrum'):
    '''
    x is real
    returns the onesided DFT
    fs in rps (required for correct power scale)
    freq in rps (required for correct power scale)
    
    
    '''

    # Detrend and Window
    lenX = x.shape[-1]
    win = signal.get_window(winType, lenX)
    
    # Compute Power scaling
    scale = PowerScale(scaleType, fs, win)
        
    # Compute the Fourier Transforms    
    if dftType is 'fft':
        freq, xDft  = FFT(x, fs)
        
        # Power
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
        
    if dftType is 'czt':
        if freq is None:
            raise ValueError('CZT frequency vector must be provided')
        freq, xDft  = CZT(x, freq, fs)
    
        # Power
        P = (np.conjugate(xDft) * xDft).real * 2*scale
    

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
    
    # Compute the discrete fourier transform
    nfft = len(x)
    
    # Form the frequency vector
    freq = np.fft.rfftfreq(nfft, 1/fs)
    
    # Perform the fft
    xDft = np.fft.rfft(x, n = nfft)

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


