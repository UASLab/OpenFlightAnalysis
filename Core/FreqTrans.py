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

class OptSpect:
    def __init__(self, freqRate = 1, freq = None, dftType = 'fft', winType = ('tukey', 0.0), detrendType = 'constant', smooth = ('box', 1), scaleType = 'spectrum'):
        self.freqRate = freqRate
        self.freq = freq
        self.dftType = dftType
        self.winType = winType
        self.detrendType = detrendType
        self.smooth = smooth
        self.scaleType = scaleType


#%% Estimate Transfer Function from Time history data
def FreqRespFuncEstNoise(x, y, opt = OptSpect(), optN = OptSpect()):
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
    import copy
    
    x = np.atleast_2d(x)
    y = np.atleast_2d(y)
    
    # If the input is multidimensional, recursively call
    numIn = x.shape[0]
    if numIn > 1: # Multi-Input
        
        freqOpt = np.atleast_2d(opt.freq)
        freqOptN = np.atleast_2d(optN.freq)
        
        # Get the shape of the frequency vectors
        numFreq = freqOpt.shape[0]
        numFreqN = freqOptN.shape[0]
        
        if not ((numFreq is numIn) or (numFreq is 1)):
            raise Exception('Number of frequency vectors should either be 1 or match the number of vectors in x; value: {}'.format(numFreq))
        freq = []
        Txy = []
        Cxy = []
        Pxx = []
        Pyy = []
        Pxy = []
        TxyUnc = []
        
        optIn = copy.deepcopy(opt)
        optInN = copy.deepcopy(optN)
        
        for iInput in range(0, numIn):
            if numFreq == 1:
                freqOptIn = np.copy(freqOpt)
            else: # Multi-Input, Multi-FreqVector
                freqOptIn = np.copy(freqOpt[iInput])
            
            if numFreqN == 1:
                freqOptInN = np.copy(freqOptN)
            else: # Multi-Input, Multi-FreqNullVector 
                freqOptInN = np.copy(freqOptN[iInput])
            
            optIn.freq = np.atleast_2d(freqOptIn)
            optInN.freq = np.atleast_2d(freqOptInN)
            freqIn, TxyIn, CxyIn, PxxIn, PyyIn, PxyIn, TxyUncIn = FreqRespFuncEstNoise(np.atleast_2d(x[iInput]), y, optIn, optInN)
            
            freq.append(np.copy(freqIn))
            Txy.append(np.copy(TxyIn))
            Cxy.append(np.copy(CxyIn))
            Pxx.append(np.copy(PxxIn))
            Pyy.append(np.copy(PyyIn))
            Pxy.append(np.copy(PxyIn))
            TxyUnc.append(np.copy(TxyUncIn))

    else: # Single-Input, Sigle-FreqVector
    
        # Compute the Power Spectrums    
        _   , xDft_E, Pxx_E = Spectrum(x, opt)
        freq, yDft_E, Pyy_E = Spectrum(y, opt)
        freqN, yDft_N, Pyy_N = Spectrum(y, optN)
        
        # Interpolate freqN into freqE, in polar coordinates
        def interpPolar(z, freqN, freqE):
            amp = np.abs(z)
            theta = np.angle(z)
            
            interpAmp = interp.interp1d(np.squeeze(freqN), amp, fill_value = "extrapolate")
            interpTheta = interp.interp1d(np.squeeze(freqN), theta, fill_value = "extrapolate")
            
            ampE = interpAmp(np.squeeze(freqE))
            thetaE = interpTheta(np.squeeze(freqE))
            
            zE = ampE * np.exp( 1j * thetaE )
            
            return zE
        
        yDft_N = interpPolar(yDft_N, freqN, freq)
        Pyy_N = interpPolar(Pyy_N, freqN, freq)
        
        # Compute Cross Spectrum Power with scaling
        lenX = x.shape[-1]
        win = signal.get_window(opt.winType, lenX)
        scale = PowerScale(opt.scaleType, opt.freqRate, win)
        
        Pxy_E = np.conjugate(xDft_E) * yDft_E * 2*scale # Scale is doubled because one-sided DFT
        Pxy_N = np.conjugate(xDft_E) * yDft_N * 2*scale # Scale is doubled because one-sided DFT
        
        Pxx = Pxx_E
    #    Pyy = Pyy_E - Pyy_N
        Pyy = Pyy_E
    #    Pxy = Pxy_E - Pxy_N
        Pxy = Pxy_E
        
        # Smooth - 
        Pxy_smooth = Smooth(np.copy(Pxy), opt.smooth)
        
        # Coherence, use the Smoothed Cross Spectrum
        Cxy = np.abs(Pxy_smooth)**2 / (Pxx * Pyy)
        
        # Compute complex transfer function approximation
        Txy = Pxy / Pxx
        TxyUnc = Pxy_N / Pxx

    # Ensure outputs are 2D
    freq = np.atleast_2d(freq)
    Txy = np.atleast_2d(Txy)
    Cxy = np.atleast_2d(Cxy)
    Pxx = np.atleast_2d(Pxx)
    Pyy = np.atleast_2d(Pyy)
    Pxy = np.atleast_2d(Pxy)
    TxyUnc = np.atleast_2d(TxyUnc)

    return freq, Txy, Cxy, Pxx, Pyy, Pxy, TxyUnc



#%% Estimate Transfer Function from Time history data
def FreqRespFuncEst(x, y, opt = OptSpect()):
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
    
    x = np.atleast_2d(x)
    y = np.atleast_2d(y)
    freqOpt = np.atleast_2d(opt.freq)
    
    # Get the shape of the inputs and outputs. Expand dimensions
    numIn = x.shape[0]
    numFreq = freqOpt.shape[0]

    # If the input is multidimensional, recursively call
    if numIn > 1: # Multi-Input
        if not ((numFreq is numIn) or (numFreq is 1)):
            raise Exception('Number of frequency vectors should either be 1 or match the number of vectors in x; value: {}'.format(numFreq))
        freq = []
        Txy = []
        Cxy = []
        Pxx = []
        Pyy = []
        Pxy = []
        
        optIn = opt
        
        for iInput in range(0, numIn):
            if numFreq == 1:
                freqOptIn = freqOpt
            else: # Multi-Input, Multi-Freqvector
                freqOptIn = freqOpt[iInput]
            
            optIn.freq = np.atleast_2d(freqOptIn)
            freqIn, TxyIn, CxyIn, PxxIn, PyyIn, PxyIn = FreqRespFuncEst(np.atleast_2d(x[iInput]), y, optIn)
            
            freq.append(freqIn)
            Txy.append(TxyIn)
            Cxy.append(CxyIn)
            Pxx.append(PxxIn)
            Pyy.append(PyyIn)
            Pxy.append(PxyIn)

    else: # Single-Input, Sigle-FreqVector
        # Compute the Power Spectrums
        _   , xDft, Pxx = Spectrum(x, opt)
        freq, yDft, Pyy = Spectrum(y, opt)
        
        # Compute Cross Spectrum Power with scaling
        lenX = x.shape[-1]
        win = signal.get_window(opt.winType, lenX)
        scale = PowerScale(opt.scaleType, opt.freqRate, win)
        
        Pxy = np.conjugate(xDft) * yDft * 2*scale # Scale is doubled because one-sided DFT
        
        # Smooth - 
    #    Pxy_smooth = Smooth(np.copy(Pxy), smooth)
        Pxy_smooth = Smooth(np.abs(Pxy), opt.smooth) * np.exp(1j * Smooth(np.angle(Pxy), opt.smooth))
        
        # Coherence, use the Smoothed Cross Spectrum
        Cxy = np.abs(Pxy_smooth)**2 / (Pxx * Pyy)
        
        # Compute complex transfer function approximation
        Txy = Pxy / Pxx
        
    # Ensure outputs are 2D
    freq = np.atleast_2d(freq)
    Txy = np.atleast_2d(Txy)
    Cxy = np.atleast_2d(Cxy)
    Pxx = np.atleast_2d(Pxx)
    Pyy = np.atleast_2d(Pyy)
    Pxy = np.atleast_2d(Pxy)
        
    return freq, Txy, Cxy, Pxx, Pyy, Pxy


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
    opt.freq = np.atleast_2d(opt.freq)

    # Detrend and Window
    lenX = x.shape[-1]
    win = signal.get_window(opt.winType, lenX)
    xWin = win*x
    
    # Compute Power scaling
    scale = PowerScale(opt.scaleType, opt.freqRate, win)
    
    # Compute the Fourier Transforms
    if opt.dftType == 'fft':
        if opt.freq[0][0] is not None:
            raise ValueError('FFT frequencies vector must be None')
        freq, xDft  = FFT(xWin, opt.freqRate)
        
        # Compute Power
        P = (np.conjugate(xDft) * xDft).real * scale
                
        if P.shape[-1] % 2:
            P[..., 1:] *= 2
        else:
            # Last point is unpaired Nyquist freq point, don't double
            P[..., 1:-1] *= 2

        # If the signal was detrended the zero frequency component is removed
        if opt.detrendType in ['constant', 'linear']:
            freq = freq[..., 1:]
            xDft = xDft[..., 1:]
            P = P[..., 1:]
        
    if opt.dftType == 'czt':
        if opt.freq is None:
            raise ValueError('CZT frequency vector must be provided')
        freq, xDft  = CZT(xWin, opt.freq, opt.freqRate)
    
        # Compute Power, factor of 2 because CZT is one-sided
        P = (np.conjugate(xDft) * xDft).real * 2*scale
    
    # Smooth the Power
    P = Smooth(P, opt.smooth)


    # Ensure the outputs are at least 2D
    freq = np.atleast_2d(freq)
    xDft = np.atleast_2d(xDft)
    P = np.atleast_2d(P)

    return freq, xDft, P


#%% Spectrogram
def SpectTime(t, x, lenSeg = 50, lenOverlap = 1, opt = OptSpect()):
    '''
    x is real
    returns the onesided DFT
    fs in rps (required for correct power scale)
    freq in rps (required for correct power scale)
    '''
        
    lenX = len(x)
    
    #freqMin_rps = (lenSeg / freqRate_hz) * hz2rps
    #opt.freq = opt.freq[...,freqMin_rps < freqExc_rps]
    
    lenFreq = opt.freq.shape[-1]
    
    numSeg = int((lenX - lenSeg) / lenOverlap)
    P_mag = np.zeros((numSeg, lenFreq))
    tSpec_s = np.zeros((numSeg))
    for iSeg in range(0, numSeg):
        
        iSel = iSeg * lenOverlap + np.arange(0, lenSeg)
        iCent = iSeg * lenOverlap + lenSeg//2
    
        tSpec_s[iSeg] = t[iCent]
        
        freq, _, P_mag[iSeg, ] = Spectrum(x[iSel], opt)
        
    return tSpec_s, freq, P_mag.T


def Spectogram(t, freq, P, dim='2D'):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    tArray, freqArray = np.meshgrid(t, freq)
    
    fig = plt.figure()
    fig.tight_layout()
    
    if dim is '3D':
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
#
def Gain(T, TUnc = None, magUnit = 'dB'):
    
    gain = np.abs(T)
    
    if magUnit is 'dB':
        gain = 20.0 * np.log10(gain)
    
    return gain
    

#
def Phase(T, TUnc = None, phaseUnit = 'rad', unwrap = False):
    
    phase = np.angle(T)
    
    if unwrap:
        phase = np.unwrap(phase)
        
    if phaseUnit is 'deg':
        phase = phase * rad2deg
        
    return phase

#
def GainPhase(T, TUnc = None, magUnit = 'dB', phaseUnit = 'deg', unwrap = False):

    gain = Gain(T, TUnc, magUnit)
    phase = Phase(T, TUnc, phaseUnit, unwrap)

    return gain, phase

#
def DistCritCirc(T, TUnc = None, magUnit = 'mag'):
    
    rCritNom = Gain(T - (-1 + 0j), magUnit = magUnit)
    
    if TUnc is not None:
        rCritUnc = Gain(TUnc, magUnit = magUnit)
        
        if magUnit is 'mag':
            rCrit = rCritNom - rCritUnc
        else: # magUnit = 'dB'
            rCrit = rCritNom / rCritUnc
                
    else:
        rCritUnc = None
        rCrit = rCritNom
    
    return rCritNom, rCritUnc, rCrit

# Distance and Contact point between an elipse and point 
def DistCrit(T, TUnc, magUnit = 'mag'):
    
    p = [0, 0]
    
    px = abs(p[0])
    py = abs(p[1])

    t = np.pi / 4

    a2 = a**2
    b2 = b**2

    for x in range(0, 3):
        x = a * np.cos(t)
        y = b * np.sin(t)

        ex = (a2 - b2) * np.cos(t)**3 / a
        ey = (b2 - a2) * np.sin(t)**3 / b

        rx = x - ex
        ry = y - ey

        qx = px - ex
        qy = py - ey

        r = np.hypot(ry, rx)
        q = np.hypot(qy, qx)

        delta_c = r * np.asin((rx*qy - ry*qx)/(r*q))
        delta_t = delta_c / np.sqrt(a*a + b*b - x*x - y*y)

        t += delta_t
        t = min(np.pi/2, max(0, t))

        p = (np.copysign(x, p[0]), np.copysign(y, p[1]))
        d = np.hypot(y-p[1], x-p[0])

    return d
    
#            p = solve(semi_major, semi_minor, (x, y))
#            dist[ix, iy] = d
    
    return rCrit, rCritNom, rCritUnc


#%% Compute the Fast Fourrier Transform
def FFT(x, fs):
    '''
    Ripped and modified for limited scope from scipy _spectral_helper
    returns the onesided DFT
    '''
    
    # Ensure x is an array
    x = np.atleast_2d(np.asarray(x))
    
    # Compute the discrete fourier transform
    nfft = x.shape[-1]
    
    # Form the frequency vector
    freq = np.fft.rfftfreq(nfft, 1/fs)
    
    # Perform the fft
    xDft = np.fft.rfft(x, n = nfft)
    
    # Output as 2D
    freq = np.atleast_2d(freq)
    xDft = np.atleast_2d(xDft)

    return freq, xDft


#%% Compute the Chirp-Z Fourrier Transform
def CZT(x, freq, fs):
    '''
    x is real
    returns the onesided DFT
    '''
    
    # Ensure x is an array
    x = np.atleast_2d(x)
    freq = np.atleast_2d(freq)

    # Length x should be even, remove the last data to make even
    lenX = x.shape[-1]
    if lenX % 2 is not True:
        x = x[..., :-1]
        lenX = x.shape[-1]
    
    # This Chirp-Z algorithm is intended for fixed intervals
    # The provided frequency vector will be pulled apart and later re-formed to ensure fixed intervals
    freqMin = np.min(freq)
    freqMax = np.max(freq)
    
    mChirp = freq.shape[-1]
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
    

    x = np.atleast_2d(x)

    # Pad and Convolve x
    for iX in range(x.shape[0]):
        xPad = np.pad(x[iX], nPad, mode = padMode)
        x[iX] = np.convolve(xPad, v, mode = convMode)

    return x

#%% Plotting
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import matplotlib.figure
import matplotlib.gridspec as gridspec
import matplotlib.patches as patch

# SISO Bode Plot
def PlotBode(freq_hz, gain_dB, phase_deg, coher_nd = None, fig = None, fmt = '', label=''):
    
    if isinstance(fig, matplotlib.figure.Figure):
        ax = fig.get_axes()
        
    elif isinstance(fig, int) and plt.fignum_exists(fig):
        fig = plt.figure(fig) # Get the figure handle by label
        ax = fig.get_axes()
    else:
        fig = plt.figure(constrained_layout = True, num = fig)
        spec = fig.add_gridspec(ncols=1, nrows=3, height_ratios=[1,1,1])
        ax = []
        ax.append(fig.add_subplot(spec[0,0]))
        ax.append(fig.add_subplot(spec[1,0], sharex=ax[0]))
        ax.append(fig.add_subplot(spec[2,0], sharex=ax[0]))
        plt.setp(ax[0].get_xticklabels(), visible=False)
        plt.setp(ax[1].get_xticklabels(), visible=False)
        
    # Coherence
    if coher_nd is None:
        coher_nd = np.ones_like(freq_hz)
        
    # Make the plots
    ax[0].semilogx(freq_hz, gain_dB, fmt, label = label)
    ax[0].grid(True)
    ax[0].set_ylabel('Gain (dB)')
    
    ax[1].semilogx(freq_hz, phase_deg, fmt, label = label)
    ax[1].yaxis.set_major_locator(plticker.MultipleLocator(180))
    ax[1].grid(True)
    ax[1].set_ylabel('Phase (deg)')
    
    ax[2].semilogx(freq_hz, coher_nd, fmt, label = label)
    ax[2].grid(True)
    ax[2].set_xlabel('Freq (Hz)')
    ax[2].set_ylabel('Coherence (nd)')
    ax[2].set_ylim(0, 1)
    
    ax[2].legend()
    
    return fig


# SISO Critical Distance Plot
def PlotNyquist(T, TUnc = None, fig = None, fmt = '', label=''):
    
    if isinstance(fig, matplotlib.figure.Figure):
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
    p = ax[0].plot(T.real, T.imag, fmt, label = label)
    
    if TUnc is not None:
        for iNom, nom in enumerate(T):
            unc = TUnc[iNom]
            uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc.real, 2*unc.imag, color=p[-1].get_color(), alpha=0.25)
            ax[0].add_artist(uncPatch)
            #ax[0].set_label(label)
    
    ax[0].grid(True)
    ax[0].set_xlabel('Real (nd)')
    ax[0].set_ylabel('Imag (nd)')
    ax[0].legend()
    
    
    return fig


# SISO Critical Distance Plot
def PlotDistCrit(freq_hz, rCrit, unc = None, coher_nd = None, fig = None, fmt = '', label=''):
    
    if isinstance(fig, matplotlib.figure.Figure):
        ax = fig.get_axes()
        
    elif isinstance(fig, int) and plt.fignum_exists(fig):
        fig = plt.figure(fig) # Get the figure handle by label
        ax = fig.get_axes()
    else:
        fig = plt.figure(constrained_layout = True, num = fig)
        spec = fig.add_gridspec(ncols=1, nrows=2, height_ratios=[2,1])
        ax = []
        ax.append(fig.add_subplot(spec[0,0]))
        ax.append(fig.add_subplot(spec[1,0], sharex=ax[0]))
        plt.setp(ax[0].get_xticklabels(), visible=False)

    # Coherence
    if coher_nd is None:
        coher_nd = np.ones_like(freq_hz)
        
    # Make the plots
    if unc is None:
        ax[0].plot(freq_hz, rCrit, fmt, label = label)
    else:
        ax[0].errorbar(freq_hz, rCrit, yerr = unc, fmt = fmt, elinewidth = 0, capsize = 2, label = label)
    
    ax[0].grid(True)
    ax[0].set_xlim(left = 0.0)
    ax[0].set_ylim(bottom = 0.0)
    ax[0].set_ylabel('Dist Crit (nd)')
    ax[0].legend()
    
    ax[1].plot(freq_hz, coher_nd, fmt, label = label)
    ax[1].grid(True)
    ax[1].set_xlabel('Freq (Hz)')
    ax[1].set_ylabel('Coherence (nd)')
    ax[1].set_ylim(0, 1)
    
    
    return fig


