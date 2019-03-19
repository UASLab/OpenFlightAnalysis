"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""


import numpy as np
import matplotlib.pyplot as plt

import GenExcite



# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps


#%% Define the frequency selection and distribution of the frequencies into the signals
numChan = 3
timeRate_s = 1/50
timeDur_s = 10.0
numCycles = 1

freqMinDes_rps = (numCycles/timeDur_s) * hz2rps * np.ones(numChan)
#freqMaxDes_rps = (0.5/timeRate_s) * hz2rps *  np.ones(numChan)
freqMaxDes_rps = 10 * hz2rps *  np.ones(numChan)
freqStepDes_rps = 0.1 * hz2rps
methodSW = 'zip' # "zippered" component distribution

freqElem_rps, sigIndx, time_s = GenExcite.MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, timeRate_s, numCycles, freqStepDes_rps, methodSW)
timeDur_s = time_s[-1] - time_s[0]

## Generate Schoeder MultiSine Signal
ampRelElem_nd = np.ones_like(freqElem_rps) ## Relative Signal Amplitude, create flat

sigList, phaseElem_rad, sigElem, ampElem_nd = GenExcite.Schroeder(freqElem_rps, ampRelElem_nd, sigIndx, time_s, phaseInit_rad = 0, boundPhase = 1, initZero = 1);


plt.figure()
for iChan in range(0, numChan):
    plt.plot(time_s, sigList[iChan]); plt.grid()

plt.figure()
for iChan in range(0, numChan):
    iPlt = iChan + 1
    plt.subplot(numChan,1,iPlt); plt.grid()
    plt.plot(time_s, sigList[iChan])


#%%
## Results
peakFactorRel = GenExcite.PeakFactor(sigList) / np.sqrt(2)
print(peakFactorRel)

## PSD
freqRate_rps = 1/timeRate_s * hz2rps
winType = [];
smoothFactor = 5;

#[xxPsd, xFft, freq_rps] = PsdEst(sigList, freqRate_rps, winType, smoothFactor);
#PsdPlot(freq_rps, xxPsd, 'rad/sec', [], []);
#
## Chrip-Z
#xxPsd_CZ = NaN(max(sum(signalDist)), size(signalDist, 2));
#xChirp_CZ = NaN(max(sum(signalDist)), size(signalDist, 2));
#freq_rps_CZ = NaN(max(sum(signalDist)), size(signalDist, 2));
#for indxSignal = 1:size(sigList, 1)
#    sigSel = signalDist(:, indxSignal) == 1;
#    freqVec_rps = freqElem_rps(sigSel);
#    sigVec = sigList(sigSel);
#    vecElem = 1:length(freqVec_rps);
#
#    [xxPsd_CZ(vecElem, indxSignal), xChirp_CZ(vecElem, indxSignal), freq_rps_CZ(vecElem, indxSignal)] = ChirpZEst(sigVec, freqVec_rps, freqRate_rps, winType, smoothFactor);
#
#
#PsdPlot(freq_rps_CZ, xxPsd_CZ, 'rad/sec', [], []);
#

#%% Create the output for JSON config
import json
timeFinal_s = time_s[-1]
timeStart_s = time_s[0]

jsonMulti = []
for iChan in range(0, numChan):
    iElem = sigIndx[iChan]

    dictChan = {}
    dictChan['Type'] = 'MultiSine'
    dictChan['Duration'] = timeDur_s
    dictChan['Frequency'] = list(freqElem_rps[iElem])
    dictChan['Phase'] = list(phaseElem_rad[iElem])
    dictChan['Amplitude'] = list(ampRelElem_nd[iElem])

    jsonMulti.append(dictChan)

#print(json.dumps(jsonMulti))
