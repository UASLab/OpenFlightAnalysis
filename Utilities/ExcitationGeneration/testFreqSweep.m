% 
%
%Notes:
% 
%
%Dependency:
% FreqSweep
% ModifyExcitation
% PeakFactor
%

%Version History: Verison 1.0
% 11/02/2006  C. Regan     Initial Release (v1.0)
%


%% FIXME: Add comments and convert to validation form
clear all;

% Constants
hz2rps = 2*pi;
rps2hz = 1/hz2rps;

% User supplied inputs
timeLength_s = 120;
timeRate_s = 1/200;


freqInit_rps = 20.3;
freqFinal_rps = 0.5;
time_s = 0:timeRate_s:timeLength_s;
amplInit = 1;
amplFinal = 1;
freqType = [];
amplType = [];
forceZeroSW = 1;

[signal, ampl, freq_rps] = FreqSweep(freqInit_rps, freqFinal_rps, time_s, amplInit, amplFinal, freqType, amplType, forceZeroSW);


gains = 1;
numRepeat = [];
seperateSW = [];
timeLead_s = [];
timeTrail_s = [];

[signals, time_s] = ModifyExcitation(signal, time_s, gains, numRepeat, seperateSW, timeLead_s, timeTrail_s);


PeakFactor(signals)/sqrt(2)
figure; plot(time_s, signals); grid on;


%signalPSD = psd(spectrum.welch, signals, 'Fs', 1/timeRate_s);
signalPSD = psd(signals);
figure; plot(signalPSD); grid on; hold on;
