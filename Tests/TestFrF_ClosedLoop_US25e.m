% Test UltraStick25e FRD estimation - closed-loop model
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
structMultiSine.numCycles = 5;

freqMinDes_hz = 1.0 / structMultiSine.timeDur_s;
freqMaxDes_hz = 10.2;

structMultiSine.freqRange_rps = repmat([freqMinDes_hz, freqMaxDes_hz], [structMultiSine.numChan, 1]) * hz2rps;
structMultiSine.freqStepDes_rps = (1 / 10) * hz2rps;
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

numRef = 3;
numExcCntrl = numRef;
numExcSurf = 7;
numDist = 6;
numTime = length(t);

ref = zeros(numRef, numTime);
excCtrl = structMultiSine.signals;
excSurf = zeros(numExcSurf, numTime);
dist = zeros(numDist, numTime);
u = [ref; excCtrl; excSurf; dist];

y = lsim(sysCtrlCL, u, t)';

iExcList = 4:6;
iOutList = 13:15;

e = u(iExcList, :);
fb = y(iOutList, :);
ff = zeros(numExcCntrl, numTime);
v = (ff - fb) + e;

% Check Correlation
% [R,P] = corrcoef([e;v]');

%%
% Linear Model Response
sysLin_frd = frd(sysCtrlCL(iOutList, iExcList), linspace(0.4, 120, 500), 'rad/s');
[T, w_rps] = freqresp(sysLin_frd);
[Gain_mag, Phase_deg] = bode(sysLin_frd); Gain_dB = Mag2DB(Gain_mag);

% FR Estimation
FRF.Opt = [];
FRF.Opt.DftType = 'ChirpZ';
FRF.Opt.ScaleType = 'spectrum';
FRF.Opt.Ts = 1/50;
FRF.Opt.Frequency = structMultiSine.freqChan_rps;

FRF.Opt.Window.Type = 'rect';
% FRF.Opt.Window.TaperRatio = 0.1;

FRF.Opt.Smooth.Type = 'rect';
FRF.Opt.Smooth.Length = 3;

FRF.Opt.Interp.FreqInterp = sort(horzcat(FRF.Opt.Frequency{:}));
FRF.Opt.Interp.Type = 'linear';
FRF.Opt.MIMO = true;

[evFrf, evFrf_MIMO] = FreqRespEst(e, v, FRF);
[ebFrf, ebFrf_MIMO] = FreqRespEst(e, fb, FRF);

% Tvb = Teb * inv(Tev)
vbFrf_MIMO.FRD = ebFrf_MIMO.FRD * inv(evFrf_MIMO.FRD);
vbFrf_MIMO.Coherence = evFrf_MIMO.Coherence;


[vbFrf_MIMO.Gain_mag, vbFrf_MIMO.Phase_deg] = bode(vbFrf_MIMO.FRD);
vbFrf_MIMO.Gain_dB = Mag2DB(vbFrf_MIMO.Gain_mag);
% vbFrf_MIMO.Phase_deg = unwrap(vbFrf_MIMO.Phase_deg * d2r, [], 3) * r2d;


%%
% Linear Model Response
% sysL_Cntrl = getLoopTransfer(sysCtrlCL, sysCtrlExc.InputName(2:2:end), 1);

% sysLin_frd = frd(sysCtrlL, linspace(0.4, 120, 500), 'rad/s');
sysLin_frd = frd(sysL_Cntrl, linspace(0.4, 120, 500), 'rad/s');
[T, w_rps] = freqresp(sysLin_frd);
[Gain_mag, Phase_deg] = bode(sysLin_frd); Gain_dB = Mag2DB(Gain_mag);


%%
optPlot.FreqUnits = 'Hz';
for iIn = 1:length(iExcList)
    for iOut = 1:length(iOutList)
        figure;
        subplot(3,1,1);
        semilogx(w_rps, squeeze(Gain_dB(iOut, iIn, :)), 'k'); hold on; grid on;
        semilogx(vbFrf_MIMO.FRD.Frequency, squeeze(vbFrf_MIMO.Gain_dB(iOut,iIn,:)), '-r');
        subplot(3,1,2);
        semilogx(w_rps, squeeze(Phase_deg(iOut, iIn, :)), 'k'); hold on; grid on;
        semilogx(vbFrf_MIMO.FRD.Frequency, squeeze(vbFrf_MIMO.Phase_deg(iOut,iIn,:)), '-r');
        subplot(3,1,3);
        semilogx(w_rps, ones(size(w_rps)), 'k'); hold on; grid on;
        semilogx(vbFrf_MIMO.FRD.Frequency, squeeze(vbFrf_MIMO.Coherence(iOut,iIn,:)), '-r');
        legend({'Linear Model', 'Excited/Estimated System', 'Interpolated', 'MIMO'});
        
        subplot(3,1,1);
        title(['Bode Plot: ', sysLin_frd.InputName{iIn}, ' to ', sysLin_frd.OutputName{iOut}]);
    end
end

%%
figure();
hBode = bodeplot(sysLin_frd); hold on; grid on;
bodeplot(vbFrf_MIMO.FRD);
legend({'Linear Model', 'Excited/Estimated Interpolated MIMO'});
setoptions(hBode, 'FreqUnits', 'Hz');
% setoptions(hBode, 'IOGrouping', 'all');

figure();
hNyq = nyquistplot(sysLin_frd); hold on; grid on;
nyquistplot(vbFrf_MIMO.FRD);
legend({'Linear Model', 'Excited/Estimated Interpolated MIMO'});
setoptions(hNyq, 'FreqUnits', 'Hz');
setoptions(hNyq, 'MagUnits', 'abs');
% setoptions(hNyq, 'IOGrouping', 'all');

figure();
hSig = sigmaplot(sysLin_frd, [], 2); hold on; grid on;
sigmaplot(vbFrf_MIMO.FRD, [], 2);
legend({'Linear Model', 'Excited/Estimated Interpolated MIMO'});
setoptions(hSig, 'FreqUnits', 'Hz');
setoptions(hSig, 'MagUnits', 'abs');
