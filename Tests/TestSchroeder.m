% Test Shroeder MultiSine Generation
%
% Notes:
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% TODO: Add comments and convert to validation form
clear all;

% Constants
hz2rps = 2*pi;
rps2hz = 1/hz2rps;


%% Define the frequency selection and distribution of the frequencies into the signals
numSignals = 3;
timeRate_s = 1/50;
timeDur_s = 10.0;
numCycles = 1;

freqMinDes_rps = (numCycles/timeDur_s) * hz2rps * ones(numSignals, 1);
freqMaxDes_rps = 8.25 * hz2rps * ones(numSignals, 1);
freqStepDes_rps = 0.2 * hz2rps;
methodSW = 'zip'; % "zippered" component distribution

[freqComp_rps, time_s, signalDist] = MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, timeRate_s, numCycles, freqStepDes_rps, methodSW);


%% Relative Signal Power
signalPowerRel = NaN(size(freqComp_rps));
for indxSig = 1:numSignals
    sigSel = find(signalDist(:,indxSig) == 1);
    signalPowerRel(sigSel) = ones(length(sigSel), 1) ./ length(sigSel); % Flat-spectrum, each signal has unity power
end


%% Generate Schoeder MultiSine Signal
phaseComp1_rad = [];
boundSW = [];
normalSW = [];
forceZeroSW = 1;

[signals, phaseComp_rad, signalComp] = MultiSineSchroeder(freqComp_rps, signalPowerRel, time_s, signalDist, phaseComp1_rad, boundSW, normalSW, forceZeroSW);


%% Modify the Excitation Signal
gains = 1 * ones(numSignals, 1);
numRepeat = [];
seperateSW = [];
timeLead_s = 1*timeRate_s;
timeTrail_s = 1*timeRate_s;

[signals, time_s] = ModifyExcitation(signals, time_s, gains, numRepeat, seperateSW, timeLead_s, timeTrail_s);


%% Results
peakFactorRel = PeakFactor(signals)/sqrt(2)

figure; plot(time_s, signals); grid on;

% PSD
freqRate_rps = 1/timeRate_s * hz2rps;
winType = 'rect';
smoothFactor = 1;

% [xxP, xDft, freq_rps] = SpectEstFFT(signals, freqRate_rps, winType, smoothFactor);
% SpectPlot(freq_rps, xxP, 'rad/sec', [], []);

% Chrip-Z
xxP_CZ = NaN(max(sum(signalDist)), size(signalDist, 2));
xDft_CZ = NaN(max(sum(signalDist)), size(signalDist, 2));
freq_rps_CZ = NaN(max(sum(signalDist)), size(signalDist, 2));
for indxSignal = 1:size(signals, 1)
    sigSel = signalDist(:, indxSignal) == 1;
    freqVec_rps = freqComp_rps(sigSel);
    sigVec = signals(indxSignal, :);
    vecElem = 1:length(freqVec_rps);

    [xxP_CZ(vecElem, indxSignal), xDft_CZ(vecElem, indxSignal), freq_rps_CZ(vecElem, indxSignal)] = SpectEst(sigVec, freqRate_rps, freqVec_rps, winType, smoothFactor, 'ChirpZ');
end

[xxP_CZ, xDft_CZ, freq_rps_CZ] = SpectEst(signals, freqComp_rps, freqRate_rps, winType, smoothFactor, 'ChirpZ');
SpectPlot(freq_rps_CZ, xxP_CZ, 'rad/sec', [], []);

return

%% Create the output for JSON config
for indxSig = 1:size(signalDist, 2)
    sigSel = find(signalDist(:, indxSig) == 1);
    
    disp(['"Type": "MultiSine", "Duration": ', jsonencode(timeDur_s), ','])
    disp(['"Frequency": ', jsonencode(freqComp_rps(sigSel)), ','])
    disp(['"Phase": ', jsonencode(phaseComp_rad(sigSel)), ','])
    
    scale = sqrt(0.5 / length(sigSel)); % Goldy3 scales the amplitude vector
    ampRel(sigSel) = sqrt(0.5 .* signalPowerRel(sigSel)) / scale; % Goldy3 uses the scaled amplitude vector as input
    
    disp(['"Amplitude": ', jsonencode(ampRel(sigSel)), ','])
    disp([' '])
end