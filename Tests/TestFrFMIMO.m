% Test Optimal MultiSine Generation
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

run('UltraStick25e_System')

% Constants
hz2rps = 2*pi;
rps2hz = 1/hz2rps;


%% Define the frequency selection and distribution of the frequencies into the signals
structMultiSine.numChan = 3;
structMultiSine.timeRate_s = 1/50;
structMultiSine.timeDur_s = 10.0;
structMultiSine.numCycles = 1;

freqMinDes_hz = structMultiSine.numCycles / structMultiSine.timeDur_s;
freqMaxDes_hz = 10.2;

structMultiSine.freqRange_rps = repmat([freqMinDes_hz, freqMaxDes_hz], [structMultiSine.numChan, 1]) * hz2rps;
structMultiSine.freqStepDes_rps = (2 / 10) * hz2rps;
methodSW = 'zip'; % "zippered" component distribution

structMultiSine = MultiSineComponents(structMultiSine, methodSW);


%% Amplitude
structMultiSine.ampChan_nd = {};
for iChan = 1:structMultiSine.numChan
    structMultiSine.ampChan_nd{iChan} = ones(size(structMultiSine.freqChan_rps{iChan})); % Flat-spectrum, each signal has unity amplitude
end


%% Generate MultiSine Signal
phaseComp1_rad = [];
boundSW = [];
normalSW = true;
forceZeroSW = true;

[structMultiSine] = MultiSineSchroeder(structMultiSine, phaseComp1_rad, boundSW, normalSW, forceZeroSW);

%% Simulate the Excitation in the Linear CL model
t = structMultiSine.time_s;

exc = structMultiSine.signals;
ref = zeros(size(exc));
u = [ref; exc];

y = lsim(sysCtrlCL, u, t)';

iExcList = 4:6;
iOutList = 7:9;

fb = y(iOutList, :);
ff = zeros(size(fb));
v = ff - fb;

%%
% PSD
optFrf.dftType = 'ChirpZ';
optFrf.scaleType = 'spectrum';
optFrf.freqRate = 50 * hz2rps;
optFrf.freq = structMultiSine.freqChan_rps;

optFrf.optWin.type = 'rect';
% optFrf.optWin.taperRatio = 0.0;

optFrf.optSmooth.type = 'rect';
optFrf.optSmooth.len = 3;


evFrf = FreqRespEst(exc, v, optFrf);


optPlot.freqUnits = 'Hz';
for iIn = 1:length(iExcList)
    iExc = iExcList(iIn);
    
    [evFrf{iIn}.gain_dB, evFrf{iIn}.phase_deg] = GainPhase(evFrf{iIn}.T);
    [gain_mag, phase_deg, w_rps] = bode(sysCtrlCL(iOutList, iExc), evFrf{iIn}.freq);
    gain_dB = Mag2DB(gain_mag);
    
    for iOut = 1:length(iOutList)
        figure;
        subplot(3,1,1);
        semilogx(w_rps, gain_dB(iOut,:), 'o'); hold on;
        semilogx(evFrf{iIn}.freq, evFrf{iIn}.gain_dB(iOut,:));
        subplot(3,1,2);
        semilogx(w_rps, phase_deg(iOut,:), 'o'); hold on;
        semilogx(evFrf{iIn}.freq, evFrf{iIn}.phase_deg(iOut,:));
        subplot(3,1,3);
        semilogx(w_rps, ones(size(w_rps)), 'o'); hold on;
        semilogx(evFrf{iIn}.freq, evFrf{iIn}.coher(iOut,:));
    end
    
end