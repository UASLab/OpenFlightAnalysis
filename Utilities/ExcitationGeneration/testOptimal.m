%
%
%Notes:
%
%
%Dependency:
% MultiSineComponents
% MultiSineOptimal
% ModifyExcitation
% PeakFactor
% PsdEst
% PsdPlot
%

%Version History: Verison 1.1
% 11/02/2006  C. Regan     Initial Release (v1.0)
% 02/05/2007  C. Regan     Added call to MultiSineComponents, added switch settings (v1.1)
%


%% FIXME: Add comments and convert to validation form
clear all;

% Constants
hz2rps = 2*pi;
rps2hz = 1/hz2rps;


%% Define the frequency selection and distribution of the frequencies into the signals
numSignals = 2;
timeRate_s = 1/50;
timeDur_s = 10.0;
numCycles = 1;

freqMinDes_rps = (numCycles/timeDur_s) * hz2rps * ones(numSignals, 1);
freqMaxDes_rps = (0.5/timeRate_s) * hz2rps * ones(numSignals, 1);
freqStepDes_rps = [];
methodSW = 'zip'; % "zippered" component distribution

[freqComp_rps, time_s, signalDist] = MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, timeRate_s, numCycles, freqStepDes_rps, methodSW);


%% Relative Signal Power
signalPowerRel = NaN(size(freqComp_rps));
for indxSig = 1:numSignals
    sigSel = find(signalDist(:,indxSig) == 1);
    signalPowerRel(sigSel) = ones(length(sigSel), 1) ./ length(sigSel); % Flat-spectrum, each signal has unity power
end


%% Generate Optimal MultiSine Signal
phaseComp1_rad = [];
normalSW = [];
forceZeroSW = 1;
costType = 'norm2';

[signals, phaseComp_rad, signalComp] = MultiSineOptimal(freqComp_rps, signalPowerRel, time_s, signalDist, phaseComp1_rad, normalSW, forceZeroSW, costType);


%% Modify the Excitation Signal
gains = 1 * ones(numSignals, 1);
numRepeat = [];
seperateSW = [];
timeLead_s = 1*timeRate_s;
timeTrail_s = 1*timeRate_s;

[signals, time_s] = ModifyExcitation(signals, time_s, gains, numRepeat, seperateSW, timeLead_s, timeTrail_s);


%% Results
peakFactorRel = PeakFactor(signals)/sqrt(2)

figure(1); plot(time_s, signals); grid on;

% PSD
freqRate_rps = 1/timeRate_s * hz2rps;
winType = [];
smoothFactor = 3;

[xxPsd, xFft, freq_rps] = PsdEst(signals, freqRate_rps, winType, smoothFactor);
PsdPlot(freq_rps, xxPsd, 'rad/sec', [], []);

% Chrip-Z
xxPsd_CZ = NaN(max(sum(signalDist)), size(signalDist, 2));
xChirp_CZ = NaN(max(sum(signalDist)), size(signalDist, 2));
freq_rps_CZ = NaN(max(sum(signalDist)), size(signalDist, 2));
for indxSignal = 1:size(signals, 1)
    sigSel = signalDist(:, indxSignal) == 1;
    freqVec_rps = freqComp_rps(sigSel);
    sigVec = signals(sigSel);
    vecElem = 1:length(freqVec_rps);

    [xxPsd_CZ(vecElem, indxSignal), xChirp_CZ(vecElem, indxSignal), freq_rps_CZ(vecElem, indxSignal)] = ChirpZEst(sigVec, freqVec_rps, freqRate_rps, winType, smoothFactor);
end

PsdPlot(freq_rps_CZ, xxPsd_CZ, 'rad/sec', [], []);


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
