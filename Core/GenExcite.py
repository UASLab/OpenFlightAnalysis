"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np

# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps


#%% Step, Pulse, Doublet, 3-2-1-1
def Discrete():

    return

#%% Linear and Log Sweeps/Chirps
def Chirp(freqInit_rps, freqFinal_rps, time_s, ampInit = 1.0, ampFinal = 1.0, freqType = 'linear', ampType = 'linear', initZero = 1):
    # Check Inputs
    timeRate_s = time_s[2] - time_s[1]
    freqMaxLimit_rps = (1/(2*timeRate_s) * hz2rps)


    if freqInit_rps > freqMaxLimit_rps:
        print('Chirp - The initial  frequency is too high for the frame rate')
        freqInit_rps = freqMaxLimit_rps

    if freqFinal_rps > freqMaxLimit_rps:
        print('Chirp - The final desired frequency is too high for the frame rate')
        freqFinal_rps = freqMaxLimit_rps

    # Number of time samples
    numSamples = len(time_s)

    # End time
    timeFinal_s = time_s[-1:]

    # Phase variation and Frequency variation
    if freqType in 'linear':
        freq_rps = ((freqFinal_rps-freqInit_rps)/(2*timeFinal_s) * time_s) + freqInit_rps
        phase_rad = freq_rps * time_s

    elif freqType in 'log10':
        phase_rad = ((timeFinal_s*freqInit_rps) / np.log10(freqFinal_rps/freqInit_rps)) * ((freqFinal_rps/freqInit_rps)**(time_s/timeFinal_s) - 1)
        freq_rps = phase_rad / time_s

    elif freqType in 'ln':
        phase_rad = ((timeFinal_s*freqInit_rps) / np.log(freqFinal_rps/freqInit_rps)) * ((freqFinal_rps/freqInit_rps)**(time_s/timeFinal_s) - 1)
        freq_rps = phase_rad / time_s

    else:
        print('Chirp - Unkown frequency variation type')


    # Amplitude variation
    iSample = np.linspace(0, numSamples-1, numSamples)
    if ampType in 'linear':
        amp = ampInit + iSample * ((ampFinal - ampInit)/(numSamples - 1))

    elif ampType in 'log10':
        amp = 10**(np.log10(ampInit) + iSample * (np.log10(ampFinal) - np.log10(ampInit)) / (numSamples-1))

    elif ampType in 'ln':
        amp = np.exp(1)**(np.log(ampInit) + iSample * (np.log(ampFinal) - np.log(ampInit)) / (numSamples-1))

    else:
        print('Chirp - Unkown amplitude variation type')


    # Generate chirp time history
    signal = amp * np.sin(phase_rad)


    # Ensure a zero final value
    if initZero:
        # Find the index imediately before the last zero crossing
        iForceZero = np.where(np.abs(np.diff(np.sign(signal))))[0][-1] + 1

        # Hold zero after the last zero crossing
        signal[iForceZero:] = 0.0


    # Return
    return (signal, amp, freq_rps)



#%% Peak Minimal Optimal Multisine
def MultiSine(freqElem_rps, ampElem_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = True, initZero = True, normalize = None, costType = 'Schoeder'):

    #Reference:
    # "Synthesis of Low-Peak-Factor Signals and Binary Sequences with Low
    # Autocorrelation", Schroeder, IEEE Transactions on Information Theory
    # Jan 1970.
    #
    # "Tailored Excitation for Multivariable Stability Margin Measurement
    # Applied to the X-31A Nonlinear Simulation"  NASA TM-113085
    # John Bosworth and John Burken

    #
    # "Multiple Input Design for Real-Time Parameter Estimation"
    # Eugene A. Morelli, 2003


    if costType.lower() == 'schroeder': # Schroeder Wave
        phaseElem_rad = SchroederPhase(ampElem_nd, phaseInit_rad, boundPhase)

    elif costType.lower() in ['oms', 'norm2', 'squaresum', 'max']: # Optimal Multisine Wave for minimum peak factor
        if costType.lower() == 'oms':
            costType = 'norm2'

        phaseElem_rad = OptimalPhase(freqElem_rps, ampElem_nd, sigIndx, time_s, phaseInit_rad, boundPhase, costType)


    # Generate the signals
    [sigList, sigElem] = MultiSineAssemble(freqElem_rps, phaseElem_rad, ampElem_nd, time_s, sigIndx)

    # Offset the phase components to yield near zero initial and final values
    # for each of the signals, based on Morrelli.  This is optional.
    if initZero:
        for i in range(10): # Do this a few times to improve the results
            # Phase shift required for each of the frequency components
            phaseElem_rad += PhaseShift(sigList, time_s, freqElem_rps, sigIndx)

            # Recompute the signals with the phases
            [sigList, sigElem] = MultiSineAssemble(freqElem_rps, phaseElem_rad, ampElem_nd, time_s, sigIndx)


    # Re-scale and re-assemble to achieve desired normalization
    if normalize == 'peak':
        # unity peak-to-peak amplitude on each channel
        for iSig, sig in enumerate(sigList):
            iElem = sigIndx[iSig]
            ampElem_nd[iElem] *= 2.0 / (max(sig) - min(sig))

        [sigList, sigElem] = MultiSineAssemble(freqElem_rps, phaseElem_rad, ampElem_nd, time_s, sigIndx)

    elif normalize == 'rms':
        # unity Root-Mean-Square amplitude on each channel
        for iSig, sig in enumerate(sigList):
            iElem = sigIndx[iSig]
            ampElem_nd[iElem] *= 1.0 / np.sqrt(np.mean(sig**2))

        [sigList, sigElem] = MultiSineAssemble(freqElem_rps, phaseElem_rad, ampElem_nd, time_s, sigIndx)

    return np.asarray(sigList), phaseElem_rad, sigElem



#%% SchroederPhase
def SchroederPhase(ampElem_nd, phaseInit_rad = 0, boundPhase = 1):

    #Reference:
    # "Synthesis of Low-Peak-Factor Signals and Binary Sequences with Low
    # Autocorrelation", Schroeder, IEEE Transactions on Information Theory
    # Jan 1970.

    # Compute the relative signal power
    sigPowerRel = (ampElem_nd / max(ampElem_nd))**2 / len(ampElem_nd)

    # Initialize the phase elements
    phaseElem_rad = np.zeros_like(ampElem_nd)
    phaseElem_rad[0] = phaseInit_rad

    # Compute the Schroeder phase shifts
    # Compute phases  (Reference 1, Equation 11)
    numElem = len(phaseElem_rad)
    for iElem in range(0, numElem):
        sumVal = 0;
        for indxL in range(0, iElem-1):
            sumVal = sumVal + ((iElem - indxL) * sigPowerRel[indxL])

        phaseElem_rad[iElem] = phaseElem_rad[0] - 2*np.pi * sumVal

    # Bound Phases
    # Correct phases to be in the range [0, 2*pi), optional
    if boundPhase:
        phaseElem_rad = np.mod(phaseElem_rad, 2*np.pi);


    return phaseElem_rad


#%% Optimal Phase
def OptimalPhase(freqElem_rps, ampElem_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, costType = 'norm2'):
    #Reference:
    # "Multiple Input Design for Real-Time Parameter Estimation"
    # Eugene A. Morelli, 2003

    from scipy.optimize import minimize

    if sigIndx is None:
        sigIndx = np.ones_like(ampElem_nd)

    # Initial Guess for Phase based on Schroeder
    phaseElem_rad = SchroederPhase(ampElem_nd, phaseInit_rad, boundPhase = 1)
    #phaseElem_rad = np.zeros_like(ampElem_nd)
    #phaseElem_rad = np.random.normal(0.0, np.pi, (ampElem_nd.shape))

    # Define the Cost Function for the Optimization
    def OMSCostFunc(phaseElem_rad, freqElem_rps, ampElem_nd, time_s, sigIndx, costType = 'norm2'):
        sigList, sigElem = MultiSineAssemble(freqElem_rps, phaseElem_rad, ampElem_nd, time_s, sigIndx)
        peakfactor = PeakFactor(sigList)

        # Cost Type
        cost = []
        if costType.lower() == 'norm2':
            cost = np.linalg.norm(peakfactor, 2)

        elif costType.lower() == 'squaresum':
            cost = sum(peakfactor**2)

        elif costType.lower() == 'max':
            cost = max(peakfactor)

        return cost

    # Compute the optimal phases
    optMethod = 'BFGS'
    #optOptions = {'maxiter': 100, 'disp': True}
    optOptions = {'maxiter': 400, 'disp': True}
    #optOptions = {'disp': True}

    optResult = minimize(OMSCostFunc, phaseElem_rad, args = (freqElem_rps, ampElem_nd, time_s, sigIndx, costType), method = optMethod, options = optOptions)
    phaseElem_rad = np.copy(optResult.x)


    # Bound Phases
    # Correct phases to be in the range [0, 2*pi), optional
    if boundPhase:
        phaseElem_rad = np.mod(phaseElem_rad, 2*np.pi);


    return phaseElem_rad


#%% MultiSineAssemble
def MultiSineAssemble(freqElem_rps, phaseElem_rad, ampElem_nd, time_s, sigIndx = None):

    # Default Values and Constants
    if sigIndx is None:
      sigIndx = np.ones_like(freqElem_rps)

    # Check Inputs
    if (len(freqElem_rps) != len(phaseElem_rad)) or (len(freqElem_rps) != len(ampElem_nd)):
        print('MultiSineAssemble - Inputs for signal power, frequency, and phase must have the same length')

    # Generate each signal component
    numChan = len(sigIndx)
    numElem = len(freqElem_rps)
    sigElem = np.zeros((numElem, len(time_s)))
    for iElem in range(0, numElem):
        sigElem[iElem] = ampElem_nd[iElem] * np.sin(freqElem_rps[iElem] * time_s + phaseElem_rad[iElem])


    # Combine signal components into signals
    sigList = []
    for iChan in range(0, numChan):
        iElem = sigIndx[iChan]
        sig = sum(sigElem[iElem])
        sigList.append(sig)

    return sigList, sigElem

#%%
def MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, freqRate_hz, numCycles = 1, freqStepDes_rps = 0, methodSW = 'zipper'):


    ## Check Inputs
    if len(freqMinDes_rps) is not len(freqMaxDes_rps):
        print('MultiSineComponents - The min and max frequency inputs must be the same length')

    # Number of channels
    numChan = len(freqMinDes_rps)

    freqMaxLimit_rps = freqRate_hz / 2 * hz2rps
    if any(freqMaxDes_rps > freqMaxLimit_rps):
        print('MultiSineComponents - The maximum desired frequency is too high for the frame rate');
        freqMaxDes_rps = freqMaxLimit_rps;


    ## Refine the frequency selection to avoid leakage, based on Bosworth
    # Convert frequencies from rad/s to Hz
    freqMinDes_hz = freqMinDes_rps * rps2hz
    freqMaxDes_hz = freqMaxDes_rps * rps2hz

    # Time vector is based on completing the desired number of cycles for the
    # min frequency component, must be divisible by the frame rate
    timeDur_s = (round((numCycles / min(freqMinDes_hz))*freqRate_hz)) / freqRate_hz
    time_s =  np.linspace(0, timeDur_s, int(timeDur_s*freqRate_hz) + 1)


    # Frequency sequence step size
    freqStepMin_hz = 1/freqRate_hz # Absolute Minimum (closest) step size
    freqStepMax_hz = min(freqMaxDes_hz - freqMinDes_hz) # Max - Min would be a huge step size

    freqStepDes_hz = freqStepDes_rps * rps2hz
    # freqStep_hz = round(freqStepDes_hz / freqStepMin_hz) * freqStepMin_hz
    freqStep_hz = freqStepDes_hz

    # Clip if required
    freqStep_hz = np.clip(freqStep_hz, freqStepMin_hz, freqStepMax_hz)

    # Adjust the min frequency based on the max time
    freqMin_hz = numCycles / timeDur_s
    freqMax_hz = np.max(freqMaxDes_hz)

    # Frequencies of all the components
    freqElem_hz = np.arange(freqMin_hz, freqMax_hz, freqStep_hz)
    freqMax_hz = np.max(freqElem_hz)
    numElem = len(freqElem_hz)
    freqElem_rps = freqElem_hz * hz2rps

    ## Distribute the frequency components into the signals
    # Distribution methods
    if methodSW in ['zipper', 'zip']:
        # Zippered distribution
        numElem =  int(np.floor(numElem/numChan) * numChan) # truncate
        sigIndx = np.zeros((numChan, int(numElem/numChan)), dtype=int)

        for iSignal in range(0, numChan):
          sigIndx[iSignal, ] = range(iSignal, numElem, numChan)

    else:
        print('MultiSineComponent - Distribution method type not understood')

    return freqElem_rps, sigIndx, time_s


#%% PhaseShift
def PhaseShift(sigList, time_s, freqElem_rps, sigIndx):

    #Reference:
    # "Multiple Input Design for Real-Time Parameter Estimation"
    # Eugene A. Morelli, 2003
    #

    ## Check Inputs
    # Number of signals and components
    numChan = len(sigIndx)

    ## Time shift per signal
    # Find the timeShift require so that each channel start near zero, then compute the phase for each element
    phaseShift_rad = np.zeros_like(freqElem_rps)
    for iChan in range(0, numChan):
        iSig = sigIndx[iChan]
        sig = sigList[iChan]

        # Find the first zero crossing
        iZero = np.where(np.abs(np.diff(np.sign(sig))))[0][0]

        # Refine the solution via interpolation around the sign switch, and return the time shift
        timeShift_s = np.interp(0, sig[iZero:iZero+2], time_s[iZero:iZero+2])

        # Find the phase shift associated with the time shift for each frequency component in the combined signals
        phaseShift_rad[iSig] = timeShift_s * freqElem_rps[iSig]


    return phaseShift_rad

#%% PeakFactor before and/or after
def PeakFactor(sigList):

    ## Calculate the peak factor
    numChan = len(sigList)
    peakFactor = np.zeros(numChan)
    for iChan in range(0, numChan):
        sig = sigList[iChan]
        peakFactor[iChan] = (max(sig) - min(sig)) / (2*np.sqrt(np.dot(sig, sig) / len(sig)))


    return peakFactor
