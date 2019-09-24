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
structMultiSine.numChan = 3;
structMultiSine.timeRate_s = 1/50;
structMultiSine.timeDur_s = 10.0;
structMultiSine.numCycles = 2;

freqMinDes_hz = structMultiSine.numCycles / structMultiSine.timeDur_s;
freqMaxDes_hz = 6.0;

structMultiSine.freqRange_rps = repmat([freqMinDes_hz, freqMaxDes_hz], [structMultiSine.numChan, 1]) * hz2rps;
structMultiSine.freqStepDes_rps = (1 / 10) * hz2rps;
methodSW = 'zip'; % "zippered" component distribution

structMultiSine = MultiSineComponents(structMultiSine, methodSW);


%% Amplitude
structMultiSine.ampChan_nd = {};
for iChan = 1:structMultiSine.numChan
    structMultiSine.ampChan_nd{iChan} = ones(size(structMultiSine.freqChan_rps{iChan})); % Flat-spectrum, each signal has unity amplitude
end


%% Generate Schoeder MultiSine Signal
phaseComp1_rad = [];
boundSW = [];
normalSW = true;
forceZeroSW = true;

[structMultiSine] = MultiSineSchroeder(structMultiSine, phaseComp1_rad, boundSW, normalSW, forceZeroSW);


%% Modify the Excitation Signal
numRepeat = [];
timeLead_s = 1*structMultiSine.timeRate_s;
timeTrail_s = 1*structMultiSine.timeRate_s;

[structMultiSine] = ModifyExcitation(structMultiSine, numRepeat, timeLead_s, timeTrail_s);


%% Results
signals = structMultiSine.signals;
peakFactorRel = PeakFactor(signals)/sqrt(2)

figure;
for iChan = 1:structMultiSine.numChan
    subplot(structMultiSine.numChan, 1, iChan);
    plot(structMultiSine.time_s, structMultiSine.signals(iChan, :)); grid on;
end

% PSD
structMultiSine.freqRate_rps = 1/structMultiSine.timeRate_s * hz2rps;
winType = 'rect';
smoothFactor = 1;

% [xxP, xDft, freq_rps] = SpectEst(signals, freqRate_rps, [], winType, smoothFactor, 'FFT');
% SpectPlot(freq_rps, xxP, 'rad/sec', [], []);

% Chrip-Z
xxP_CZ = {};
xDft_CZ = {};
freq_rps_CZ = {};
for iChan = 1:structMultiSine.numChan
    [xxP_CZ{iChan}, xDft_CZ{iChan}, freq_rps_CZ{iChan}] = SpectEst(structMultiSine.signals, structMultiSine.freqRate_rps, structMultiSine.freqChan_rps{iChan}, winType, smoothFactor, 'ChirpZ');
end

% SpectPlot(freq_rps_CZ, xxP_CZ, 'rad/sec', [], []);
figure;
for iChan = 1:structMultiSine.numChan
    subplot(structMultiSine.numChan, 1, iChan);
    semilogx(freq_rps_CZ{iChan}, Mag2DB( xxP_CZ{iChan} ), '*-'); grid on;
end


return

%% Create the output for JSON config
waveName = 'Schroeder';
WaveDef = [];
for iChan = 1:structMultiSine.numChan
    name = [waveName '_' num2str(iChan)];
    WaveDef.(name).Type = 'MultiSine';
    WaveDef.(name).Duration = structMultiSine.timeDur_s;
    WaveDef.(name).Frequency = structMultiSine.freqChan_rps{iChan};
    WaveDef.(name).Phase = structMultiSine.phaseChan_rad{iChan};
    WaveDef.(name).Amplitude = structMultiSine.ampChan_nd{iChan};
end

jsonStr = jsonencode(WaveDef);

% Pretty Print
jsonStr = strrep(jsonStr, '},"', sprintf('},\r"'));
jsonStr = strrep(jsonStr, ':{', sprintf(':{\r\t'));
jsonStr = strrep(jsonStr, ',"', sprintf(',\r\t"'));
jsonStr = strrep(jsonStr, '}}', sprintf('}\r}'));
jsonStr = strrep(jsonStr, ']}', sprintf(']\r}'));
jsonStr = strrep(jsonStr, ':', sprintf(': '));
jsonStr = strrep(jsonStr, ',', sprintf(', '));

disp(jsonStr)
