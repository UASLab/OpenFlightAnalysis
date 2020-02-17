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
    def __init__(self, freqRate = 1, freq = None, freqInterp = None, dftType = 'fft', winType = ('tukey', 0.0), detrendType = 'constant', smooth = ('box', 1), scaleType = 'spectrum', interpType = 'linear'):
        self.freqRate = freqRate
        self.freq = freq
        self.dftType = dftType
        self.winType = winType
        self.detrendType = detrendType
        self.interpType = interpType
        self.smooth = smooth
        self.scaleType = scaleType
        self.freqInterp = None


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
    fs and freq must have same units. (Puu and Pyy will only have correct power scale if units are rad/sec)

    '''
    import copy

    x = np.atleast_2d(x)
    y = np.atleast_2d(y)

    # If the input is multidimensional, recursively call
    numIn = x.shape[0]
    numOut = y.shape[0]
    if numIn > 1: # Multi-Input

        freqOpt = np.atleast_2d(opt.freq)

        # Get the shape of the frequency vectors
        numChan = freqOpt.shape[0]

        if not ((numChan is numIn) or (numChan is 1)):
            raise Exception('Number of frequency vectors should either be 1 or match the number of vectors in x; value: {}'.format(numChan))

        freq = np.sort(freqOpt.flatten())
        numFreq = freq.shape[-1]

        Pxx = np.zeros((numIn, numFreq))
        Pyy = np.zeros((numOut, numIn, numFreq))
        Cxy = np.zeros((numOut, numIn, numFreq))
        Pxy = np.zeros((numOut, numIn, numFreq), dtype=complex)
        Txy = np.zeros((numOut, numIn, numFreq), dtype=complex)

        optIn = copy.deepcopy(opt)

        for iInput in range(0, numIn):
            if numChan == 1:
                freqOptIn = np.copy(freqOpt)
            else: # Multi-Input, Multi-FreqVector
                freqOptIn = np.copy(freqOpt[iInput])

            # Index of frequencies associated with current iInput
            iFreq = np.where(np.isin(np.sort(freqOpt.flatten()), freqOptIn))

            optIn.freq = np.atleast_2d(freqOptIn)
            freqIn, TxyIn, CxyIn, PxxIn, PyyIn, PxyIn = FreqRespFuncEst(np.atleast_2d(x[iInput]), y, optIn)

            Pxx[iInput, :] = InterpVal(PxxIn, freqIn, freq, optIn.interpType)
            Pyy[:, iInput, :] = InterpVal(PyyIn, freqIn, freq, optIn.interpType)
            Pxy[:, iInput, :] = InterpPolar(PxyIn, freqIn, freq, optIn.interpType)
            Txy[:, iInput, :] = InterpPolar(TxyIn, freqIn, freq, optIn.interpType)
            Cxy[:, iInput, :] = InterpVal(CxyIn, freqIn, freq, optIn.interpType)


    else: # Single-Input, Sigle-FreqVector

        # Compute the Power Spectrums
        _   , xDft, Pxx = Spectrum(x, opt)
        freq, yDft, Pyy = Spectrum(y, opt)

        # Compute Cross Spectrum Power with scaling
        lenX = x.shape[-1]
        win = signal.get_window(opt.winType, lenX)
        scale = PowerScale(opt.scaleType, opt.freqRate, win)

        Pxy = np.conjugate(xDft) * yDft * 2*scale # Scale is doubled because one-sided DFT

        # Interpolate Power Spectrums to the Desired Frequency Basis
        if opt.freqInterp is not None:
            Pxx = InterpPolar(Pxx, freq, opt.freqInterp, opt.interpType)
            Pyy = InterpPolar(Pyy, freq, opt.freqInterp, opt.interpType)
            Pxy = InterpPolar(Pxy, freq, opt.freqInterp, opt.interpType)
            freq = opt.freqInterp

        # Smooth
        Pxy_smooth = SmoothPolar(Pxy, opt)

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
    numOut = y.shape[0]
    if numIn > 1: # Multi-Input

        freqOpt = np.atleast_2d(opt.freq)
        freqOptN = np.atleast_2d(optN.freq)

        # Get the shape of the frequency vectors
        numChan = freqOpt.shape[0]
        numChanN = freqOptN.shape[0]

        if not ((numChan is numIn) or (numChan is 1)):
            raise Exception('Number of frequency vectors should either be 1 or match the number of vectors in x; value: {}'.format(numChan))

        freq = np.sort(freqOpt.flatten())
        numFreq = freq.shape[-1]

        freqN = np.sort(freqOptN.flatten())
        numFreqN = freqN.shape[-1]

        Pxx = np.zeros((numIn, numFreq))
        Pyy = np.zeros((numOut, numIn, numFreq))
        Cxy = np.zeros((numOut, numIn, numFreq))
        Pxy = np.zeros((numOut, numIn, numFreq), dtype=complex)
        Txy = np.zeros((numOut, numIn, numFreq), dtype=complex)

        TxyUnc = np.zeros((numOut, numIn, numFreq))
        Pyy_N = np.zeros((numOut, numFreqN))

        optIn = copy.deepcopy(opt)
        optInN = copy.deepcopy(optN)

        for iInput in range(0, numIn):
            if numChan == 1:
                freqOptIn = np.copy(freqOpt)
            else: # Multi-Input, Multi-FreqVector
                freqOptIn = np.copy(freqOpt[iInput])

            if numChanN == 1:
                freqOptInN = np.copy(freqOptN)
            else: # Multi-Input, Multi-FreqNullVector
                freqOptInN = np.copy(freqOptN[iInput])

            # Index of frequencies associated with current iInput
            iFreq = np.where(np.isin(np.sort(freqOpt.flatten()), freqOptIn))
            iFreqN = np.where(np.isin(np.sort(freqOptN.flatten()), freqOptInN))

            optIn.freq = np.atleast_2d(freqOptIn)
            optInN.freq = np.atleast_2d(freqOptInN)
            freqIn, TxyIn, CxyIn, PxxIn, PyyIn, PxyIn, TxyUncIn, Pyy_NIn = FreqRespFuncEstNoise(np.atleast_2d(x[iInput]), y, optIn, optInN)

            Pxx[iInput, :] = InterpVal(PxxIn, freqIn, freq, optIn.interpType)
            Pyy[:, iInput, :] = InterpVal(PyyIn, freqIn, freq, optIn.interpType)
            Pxy[:, iInput, :] = InterpPolar(PxyIn, freqIn, freq, optIn.interpType)
            Txy[:, iInput, :] = InterpPolar(TxyIn, freqIn, freq, optIn.interpType)
            Cxy[:, iInput, :] = InterpVal(CxyIn, freqIn, freq, optIn.interpType)

            TxyUnc[:, iInput, :] = InterpVal(TxyUncIn, freqIn, freq, optIn.interpType)
            Pyy_N = np.expand_dims(Pyy_NIn, 1)


    else: # Single-Input, Sigle-FreqVector

        # Compute the Power Spectrums
        _   , xDft, Pxx = Spectrum(x, opt)
        freq, yDft, Pyy = Spectrum(y, opt)
        freqN, yDft_N, Pyy_N = Spectrum(y, optN) # Noise in freqN basis

        # Compute Cross Spectrum Power with scaling
        lenX = x.shape[-1]
        win = signal.get_window(opt.winType, lenX)
        scale = PowerScale(opt.scaleType, opt.freqRate, win)

        Pxy = np.conjugate(xDft) * yDft * 2*scale # Scale is doubled because one-sided DFT
        Pxy_N = np.conjugate(xDft) * InterpPolar(yDft_N, freqN, freq, opt.interpType) * 2*scale

        # Noise Power
        Pyy_N = np.conjugate(yDft_N) * yDft_N * 2*scale

        # Interpolate Power Spectrums to the Desired Frequency Basis
        if optN.freqInterp is not None:
            Pxy_N = InterpPolar(Pxy_N, freq, optN.freqInterp, optN.interpType)
            Pyy_N = InterpPolar(Pyy_N, freqN, optN.freqInterp, optN.interpType)
            freqN = optN.freqInterp

        if opt.freqInterp is not None:
            Pxx = InterpPolar(Pxx, freq, opt.freqInterp, opt.interpType)
            Pyy = InterpPolar(Pyy, freq, opt.freqInterp, opt.interpType)
            Pxy = InterpPolar(Pxy, freq, opt.freqInterp, opt.interpType)
            freq = opt.freqInterp

        # Smooth
        Pxy_smooth = SmoothPolar(Pxy, opt)
        Pxy_N_smooth = SmoothPolar(Pxy_N, optN)
        Pyy_N_smooth = SmoothPolar(Pyy_N, optN)

        # Coherence, use the Smoothed Cross Spectrum
        Cxy = np.abs(Pxy_smooth)**2 / (Pxx * Pyy)

        # Compute complex transfer function approximation
        Txy = Pxy / Pxx
        TxyUnc = np.abs(Pxy_N) / Pxx

    # Ensure outputs are 2D
    freq = np.atleast_2d(freq)
    Txy = np.atleast_2d(Txy)
    Cxy = np.atleast_2d(Cxy)
    Pxx = np.atleast_2d(Pxx)
    Pyy = np.atleast_2d(Pyy)
    Pxy = np.atleast_2d(Pxy)
    TxyUnc = np.atleast_2d(TxyUnc)
    Pyy_N =  np.atleast_2d(Pyy_N)

    return freq, Txy, Cxy, Pxx, Pyy, Pxy, TxyUnc, Pyy_N

# Interpolate freqN into freqE, in polar coordinates
def SmoothPolar(z, opt):
    amp = np.abs(z)
    theta = np.angle(z)

    smoothAmp = Smooth(amp, opt.smooth)
    smoothTheta = Smooth(theta, opt.smooth)

    zE = smoothAmp * np.exp( 1j * smoothTheta )

    return zE

# Interpolate
def InterpVal(y, x, xE, kind = 'linear', fill_value = 'extrapolate'):

    interpF = interp.interp1d(np.squeeze(x), y, kind = kind, bounds_error = False, fill_value = fill_value)

    yE = interpF(np.squeeze(xE))

    return yE

# Smooth polar coordinates
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
    opt.freq = np.atleast_2d(opt.freq)

    # Detrend and Window
    lenX = x.shape[-1]
    win = signal.get_window(opt.winType, lenX)
    xWin = win*x

    # Compute Power scaling
    scale = PowerScale(opt.scaleType, opt.freqRate, win)

    # Compute the Fourier Transforms
    if opt.dftType.lower() == 'fft':
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

    if opt.dftType.lower() == 'dftmat':
        if opt.freq is None:
            raise ValueError('DFT frequency vector must be provided')
        freq, xDft  = DFTMat(xWin, opt.freq, opt.freqRate)

        # Compute Power, factor of 2 because CZT is one-sided
        P = (np.conjugate(xDft) * xDft).real * 2*scale

    if opt.dftType.lower() == 'czt':
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

# Plot the Spectrogram
def Spectogram(t, freq, P, dim='2D'):
    import matplotlib.pyplot as plt

    tArray, freqArray = np.meshgrid(t, freq)

    fig = plt.figure()
    fig.tight_layout()

    if dim is '3D':
        from mpl_toolkits.mplot3d import Axes3D
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
        phase = np.unwrap(phase, axis=-1)

    if phaseUnit is 'deg':
        phase = phase * rad2deg

    return phase

#
def GainPhase(T, TUnc = None, magUnit = 'dB', phaseUnit = 'deg', unwrap = False):

    gain = Gain(T, TUnc, magUnit)
    phase = Phase(T, TUnc, phaseUnit, unwrap)

    return gain, phase

#
def Sigma(Tin, TinUnc = None, typeSigma = 1):

    T = np.copy(Tin)
    numOut, numIn, numFreq = T.shape

    # Shift dimension
    shiftFlag = False
    if numFreq > numOut:
        shiftFlag = True
        T = np.moveaxis(T, -1, 0)
        numFreq, numOut, numIn = T.shape

    if numIn != numOut:
        raise ValueError('Input to Sigma must be square')

    if typeSigma == 0: # T
        TNom = np.copy(T)
    if typeSigma == 1: # T^-1
        TNom = np.linalg.inv(T)
    elif typeSigma == 2: # I + T
        TNom = np.eye(numOut) + T
    elif typeSigma == 3: # I + T^-1
        TNom = np.eye(numOut) + np.linalg.inv(T)

    sNom = np.linalg.svd(TNom, full_matrices=True, compute_uv = False)


    # Uncertainty
    # TUnc is the magnitude of the uncertainty surrounding T
    sCrit = np.zeros_like(sNom)
    TUnc = np.copy(TinUnc)

    if TinUnc is not None: # There is an Uncertainty estimate

        if shiftFlag:
            TUnc = np.moveaxis(TUnc, -1, 0)

        if typeSigma == 0: # T
            TCrit = TNom - TUnc * (TNom / np.abs(TNom))
            sCrit = np.linalg.svd(TCrit, full_matrices=True, compute_uv = False)

#        elif typeSigma == 1: # T^-1

        elif typeSigma == 2: # I + T
            TCrit = np.eye(numOut) + TNom - TUnc * (TNom / np.abs(TNom))
            sCrit = -1 + np.linalg.svd(TCrit, full_matrices=True, compute_uv = False)

#        elif typeSigma == 3: # I + T^-1


    if shiftFlag:
        sNom = np.moveaxis(sNom, 0, -1)
        sCrit = np.moveaxis(sCrit, 0, -1)

    return sNom, sCrit

#
def DistCrit(T, TUnc = None, pCrit = -1 + 0j, typeUnc = 'ellipse'):

    if TUnc is None: # There is no Uncertainty estimate, just return the distance between T and pCrit
        rCritNom, rCritUnc, rCrit = DistCritCirc(T, TUnc, pCrit)
    else:
        if typeUnc is 'circle':
            rCritNom, rCritUnc, rCrit = DistCritCirc(T, TUnc, pCrit)
        elif typeUnc is 'ellipse':
            rCritNom, rCritUnc, rCrit, pCont = DistCritEllipse(T, TUnc, pCrit)

    return rCritNom, rCritUnc, rCrit


#
def DistCritCirc(T, TUnc = None, pCrit = -1 + 0j, typeNorm = 'RSS'):

    rCritNom = np.abs(T - pCrit)
    rCrit = None

    if TUnc is not None:
        if typeNorm.lower() == 'rms':
            rCritUnc = np.sqrt(0.5) * np.abs(TUnc) # RMS
        elif typeNorm.lower() == 'max':
            rCritUnc = np.max([TUnc.real, TUnc.imag]) # Max
        elif typeNorm.lower() == 'mean':
            rCritUnc = np.mean([TUnc.real, TUnc.imag]) # Mean
        elif typeNorm.lower() == 'rss':
            rCritUnc = np.abs(TUnc) # RSS

    else:
        rCritUnc = 0.0

    # Uncertain Distance is the difference between Nominal and rCritUnc Distance
    # If the point is inside the circle return the distance as negative
    rCrit = rCritNom - rCritUnc

    return rCritNom, rCritUnc, rCrit

#
def DistCritEllipse(T, TUnc, pCrit = -1 + 0j):

    # Transform coordinates so that T is shifted to [0,0]
    pCrit_new = T - pCrit

    # Nominal Distance
    rCritNom = np.abs(pCrit_new)

    # Compute the Contact location in new coordinates
    pCont_new = np.zeros_like(pCrit_new, dtype='complex')
    inside = np.zeros_like(pCrit_new, dtype='bool')

    for indx in np.ndindex(pCrit_new.shape):
        a = abs(TUnc[indx].real)
        b = abs(TUnc[indx].imag)
        p = [pCrit_new[indx].real, pCrit_new[indx].imag]
        pC, i = EllipsePoint(a, b, p)

        pCont_new[indx] = pC[0] + 1j*pC[1]
        inside[indx] = i

    # Transform back to original coordinates
    pCont = T + pCont_new

    # Compute the distance to the contact point
    rCrit = np.abs(pCont - pCrit)
    rCrit[inside] = -rCrit[inside] # If the point is inside the ellipse return the distance as negative

    # rCrit is the difference between Nominal and Uncertain Distance
    rCritUnc = np.abs(pCont - T)

    return rCritNom, rCritUnc, rCrit, pCont


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

    pCont = (np.copysign(a*tx, p[0]), np.copysign(b*ty, p[1]))

    return pCont, inside


# Distance to Ellipse, with rotation
def DistEllipseRot(pEllipse, a, b, a_deg, pCrit):

    # Transform coordinates so that pEllipse is at [0,0] and angle=0
    a_rad = a_deg * (np.pi / 180)
    aCos = np.cos(a_rad)
    aSin = np.sin(a_rad)

    R = np.array([[aCos, -aSin],[aSin, aCos]])

    pCrit_new = R.T @ (pCrit - pEllipse)

    # Compute the Contact location in new coordinates
    pCont_new, inside = EllipsePoint(a, b, pCrit_new)

    # Transform back to original coordinates
    pCont = R @ pCont_new + pEllipse

    # Compute the distance to the contact point
    dist = np.linalg.norm(pCont - pCrit, 2)

    # If the point is inside the ellipse return the distance as negative
    if inside:
        dist = -abs(dist)

    return pCont, dist


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


#%% Compute the DFT wia Matrix
def DFTMat(x, freq, fs):
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


#%% Compute the Chirp-Z Fourrier Transform
def CZT(x, freq, fs, N = None):
    '''
    x is real
    '''

    # Ensure x is an array
    x = np.atleast_2d(x)
    freq = np.atleast_2d(freq)

    # Length x should be even, remove the last data to make even
    if N is None:
        N = x.shape[-1]

    if N % 2 is not True:
        x = x[..., :-1]
        N = x.shape[-1]

    # This Chirp-Z algorithm is intended for fixed intervals
    # The provided frequency vector will be pulled apart and later re-formed to ensure fixed intervals
    freqMin = np.min(freq)
    freqMax = np.max(freq)

    M = freq.shape[-1]
    freqStep = (freqMax - freqMin) / (M - 1)

    # Ratio between points
    W = np.exp(-1j * (2*np.pi / fs) * freqStep);

    # Starting point
    A = np.exp(1j * 2*np.pi * freqMin / fs)


    # Indices
    k = np.arange(M)
    n = np.arange(N, dtype=float)

    Wk2 = np.power(W, k**2 / 2.)
    AWn2 = np.power(A,-n) * np.power(W, n**2 / 2.)

    # Helper objects
    fft = np.fft.fft
    ifft = np.fft.ifft

    nfft = int(2**np.ceil(np.log2(M+N-1)))

    # Pre-compute the Chirp Filter
    v = np.zeros(nfft, dtype=np.complex)
    v[:M] = np.power(W, -n[:M]**2/2.)
    v[nfft-N+1:] = np.power(W, -n[N-1:0:-1]**2/2.)
    V = fft(v, nfft)

    # Compute the Chirp-Z
    Y = fft(AWn2 * x, nfft)

    xCzt = Wk2 * ifft(V*Y)[...,:M]


    # Re-form the frequency vector
    freq = k * freqStep + freqMin


    return freq, xCzt

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
def PlotBode(freq, gain_mag, phase_deg, coher_nd = None, gainUnc_mag = None, fig = None, fmt = '', label='', dB = True):

    plotCohere = True
    if coher_nd is None:
        plotCohere = False

    if isinstance(fig, matplotlib.figure.Figure):
        ax = fig.get_axes()

    elif isinstance(fig, int) and plt.fignum_exists(fig):
        fig = plt.figure(fig) # Get the figure handle by label
        ax = fig.get_axes()
    else:
        fig = plt.figure(constrained_layout = True, num = fig)

        if plotCohere:
            spec = fig.add_gridspec(ncols=1, nrows=3, height_ratios=[1,1,1])
        else:
            spec = fig.add_gridspec(ncols=1, nrows=2, height_ratios=[1,1])

        ax = []
        ax.append(fig.add_subplot(spec[0,0]))
        ax.append(fig.add_subplot(spec[1,0], sharex=ax[0]))
        plt.setp(ax[0].get_xticklabels(), visible=False)

        if plotCohere:
            plt.setp(ax[1].get_xticklabels(), visible=False)
            ax.append(fig.add_subplot(spec[2,0], sharex=ax[0]))

    # Coherence
    if coher_nd is None:
        coher_nd = np.ones_like(freq)

    # Make the plots
    if gainUnc_mag is None:
        if dB:
            ax[0].semilogx(freq, 20*np.log10(gain_mag), fmt, label = label)
            ax[0].set_ylabel('Gain (dB)')
        else:
            ax[0].semilogx(freq, gain_mag, fmt, label = label)
            ax[0].set_ylabel('Gain (mag)')
    else:
        ax[0].set_xscale("log", nonposx='clip')

        if dB:
            gainUnc_dB = [-20 * np.log10(1 - gainUnc_mag/gain_mag), 20 * np.log10(1 +  gainUnc_mag/gain_mag)]

            ax[0].errorbar(freq, 20*np.log10(gain_mag),  yerr = gainUnc_dB, fmt = fmt, elinewidth = 0, capsize = 2, label = label)
            ax[0].set_ylabel('Gain (dB)')
        else:
            ax[0].errorbar(freq, gain_mag,  yerr = gainUnc_mag, fmt = fmt, elinewidth = 0, capsize = 2, label = label)
            ax[0].set_ylabel('Gain (mag)')

    ax[0].grid(True)

    ax[1].semilogx(freq, phase_deg, fmt, label = label)
    ax[1].yaxis.set_major_locator(plticker.MultipleLocator(180))
    ax[1].grid(True)
    ax[1].set_ylabel('Phase (deg)')

    # Plot the Coherence
    if coher_nd is not None:
        ax[-1].semilogx(freq, coher_nd, fmt, label = label)
        ax[-1].grid(True)
        ax[-1].set_ylabel('Coherence (nd)')
        ax[-1].set_ylim(0, 1)

    ax[-1].set_xlabel('Freq (Hz)')
    ax[-1].legend(loc = 'right')

    return fig

# SISO BodeMag Plot
def PlotBodeMag(freq, gain_mag, coher_nd = None, gainUnc_mag = None, fig = None, fmt = '', label='', dB = True):

    plotCohere = True
    if coher_nd is None:
        plotCohere = False

    if isinstance(fig, matplotlib.figure.Figure):
        ax = fig.get_axes()

    elif isinstance(fig, int) and plt.fignum_exists(fig):
        fig = plt.figure(fig) # Get the figure handle by label
        ax = fig.get_axes()
    else:
        fig = plt.figure(constrained_layout = True, num = fig)

        if plotCohere:
            spec = fig.add_gridspec(ncols=1, nrows=2, height_ratios=[2,1])
        else:
            spec = fig.add_gridspec(ncols=1, nrows=1, height_ratios=[1])

        ax = []
        ax.append(fig.add_subplot(spec[0,0]))

        if plotCohere:
            plt.setp(ax[0].get_xticklabels(), visible=False)
            ax.append(fig.add_subplot(spec[1,0], sharex=ax[0]))

    # Coherence
    if coher_nd is None:
        coher_nd = np.ones_like(freq)

    # Make the plots
    if gainUnc_mag is None:
        if dB:
            ax[0].semilogx(freq, 20*np.log10(gain_mag), fmt, label = label)
            ax[0].set_ylabel('Gain (dB)')
        else:
            ax[0].semilogx(freq, gain_mag, fmt, label = label)
            ax[0].set_ylabel('Gain (mag)')
    else:
        ax[0].set_xscale("log", nonposx='clip')

        if dB:
            gainUnc_dB = [-20 * np.log10(1 - gainUnc_mag/gain_mag), 20 * np.log10(1 +  gainUnc_mag/gain_mag)]

            ax[0].errorbar(freq, 20*np.log10(gain_mag),  yerr = gainUnc_dB, fmt = fmt, elinewidth = 0, capsize = 2, label = label)
            ax[0].set_ylabel('Gain (dB)')
        else:
            ax[0].errorbar(freq, gain_mag,  yerr = gainUnc_mag, fmt = fmt, elinewidth = 0, capsize = 2, label = label)
            ax[0].set_ylabel('Gain (mag)')

    ax[0].grid(True)

    # Plot the Coherence
    if coher_nd is not None:
        ax[-1].semilogx(freq, coher_nd, fmt, label = label)
        ax[-1].grid(True)
        ax[-1].set_ylabel('Coherence (nd)')
        ax[-1].set_ylim(0, 1)

    ax[-1].set_xlabel('Freq (Hz)')
    ax[-1].legend(loc = 'right')

    return fig


# SISO Nyquist Plot
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
            if unc.imag == 0:
                uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc, 2*unc, color=p[-1].get_color(), alpha=0.25)
            else:
                uncPatch = patch.Ellipse((nom.real, nom.imag), 2*unc.real, 2*unc.imag, color=p[-1].get_color(), alpha=0.25)
            ax[0].add_artist(uncPatch)
            #ax[0].set_label(label)

    ax[0].grid(True)
    ax[0].set_xlabel('Real (nd)')
    ax[0].set_ylabel('Imag (nd)')
    ax[0].legend(loc = 'right')

    return fig


# Singular Value / Critical Distance Plot
def PlotSigma(freq, sigma, err = None, coher_nd = None, fig = None, fmt = '', label=''):

    plotCohere = True
    if coher_nd is None:
        plotCohere = False

    if isinstance(fig, matplotlib.figure.Figure):
        ax = fig.get_axes()

    elif isinstance(fig, int) and plt.fignum_exists(fig):
        fig = plt.figure(fig) # Get the figure handle by label
        ax = fig.get_axes()
    else:
        fig = plt.figure(constrained_layout = True, num = fig)
        if plotCohere:
            spec = fig.add_gridspec(ncols=1, nrows=2, height_ratios=[2,1])
        else:
            spec = fig.add_gridspec(ncols=1, nrows=1, height_ratios=[1])

        ax = []
        ax.append(fig.add_subplot(spec[0,0]))

        if plotCohere:
            ax.append(fig.add_subplot(spec[1,0], sharex=ax[0]))
            plt.setp(ax[0].get_xticklabels(), visible=False)


    # Make the plots, # Plot the uncertainty as errorbars
    if err is None:
        ax[0].plot(freq, sigma, fmt, label = label)
    else:
        ax[0].errorbar(freq, sigma, yerr = err, fmt = fmt, elinewidth = 0, capsize = 2, label = label, uplims = True)

    ax[0].grid(True)
    ax[0].set_xlim(left = 0.0)
    ax[0].set_ylim(bottom = 0.0)
    ax[0].set_ylabel('Singular Value (nd)')

    # Plot the Coherence
    if coher_nd is not None:
        ax[-1].plot(freq, coher_nd, fmt, label = label)
        ax[-1].grid(True)
        ax[-1].set_ylabel('Coherence (nd)')
        ax[-1].set_ylim(0, 1)

    ax[-1].legend(loc = 'right')
    ax[-1].set_xlabel('Freq (Hz)')

    return fig
