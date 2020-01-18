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
structMultiSine.numCycles = 3;

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

exc = structMultiSine.signals;
ref = zeros(size(exc));
u = [ref; exc];

y = lsim(sysOL, u, t)';

iExcList = 4:6;
iOutList = 1:3;

e = u(iExcList, :);
v = y(iOutList, :);

% Check Correlation
% [R,P] = corrcoef([e;v]');

%%
% Linear Model Response
sysLin_frd = frd(sysOL(iOutList, iExcList), linspace(0.4, 120, 500), 'rad/s');
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

% [txy, f] = tfestimate(exc(1,:), v(1,:), [], [], FRF.Opt.Frequency{1}, 50*2*pi);
% figure(); semilogx(f, unwrap(angle(txy)) * 180/pi);

[evFrf, evFrf_MIMO] = FreqRespEst(exc, v, FRF);
for iIn = 1:length(iExcList)
    [evFrf{iIn}.Gain_mag, evFrf{iIn}.Phase_deg] = bode(evFrf{iIn}.FRD);
    
    evFrf{iIn}.Gain_dB = Mag2DB(evFrf{iIn}.Gain_mag);
%     evFrf{iIn}.Phase_deg = unwrap(evFrf{iIn}.Phase_deg * d2r, [], 2) * r2d;
    
    [evFrf{iIn}.GainInterp_mag, evFrf{iIn}.PhaseInterp_deg] = bode(evFrf{iIn}.Interp.FRD);
    
    evFrf{iIn}.GainInterp_dB = Mag2DB(evFrf{iIn}.GainInterp_mag);
%     evFrf{iIn}.PhaseInterp_deg = unwrap(evFrf{iIn}.PhaseInterp_deg * d2r, [], 2) * r2d;
end


[evFrf_MIMO.Gain_mag, evFrf_MIMO.Phase_deg] = bode(evFrf_MIMO.FRD);
evFrf_MIMO.Gain_dB = Mag2DB(evFrf_MIMO.Gain_mag);
% evFrf_MIMO.Phase_deg = unwrap(evFrf_MIMO.Phase_deg * d2r, [], 3) * r2d;

%%
optPlot.FreqUnits = 'Hz';
for iIn = 1:length(iExcList)
    iExc = iExcList(iIn);

    for iOut = 1:length(iOutList)
        figure;
        subplot(3,1,1);
        semilogx(w_rps, squeeze(Gain_dB(iOut, iIn, :)), 'k'); hold on; grid on;
        semilogx(evFrf{iIn}.FRD.Frequency, evFrf{iIn}.Gain_dB(iOut,:), '*b');
        semilogx(evFrf{iIn}.Interp.FRD.Frequency, evFrf{iIn}.GainInterp_dB(iOut,:), '-r');
        semilogx(evFrf_MIMO.FRD.Frequency, squeeze(evFrf_MIMO.Gain_dB(iOut,iIn,:)), '-.m');
        subplot(3,1,2);
        semilogx(w_rps, squeeze(Phase_deg(iOut, iIn, :)), 'k'); hold on; grid on;
        semilogx(evFrf{iIn}.FRD.Frequency, evFrf{iIn}.Phase_deg(iOut,:), '*b');
        semilogx(evFrf{iIn}.Interp.FRD.Frequency, evFrf{iIn}.PhaseInterp_deg(iOut,:), '-r');
        semilogx(evFrf_MIMO.FRD.Frequency, squeeze(evFrf_MIMO.Phase_deg(iOut,iIn,:)), '-.m');
        subplot(3,1,3);
        semilogx(w_rps, ones(size(w_rps)), 'k'); hold on; grid on;
        semilogx(evFrf{iIn}.FRD.Frequency, evFrf{iIn}.Coherence(iOut,:), '*b');
        semilogx(evFrf{iIn}.Interp.FRD.Frequency, evFrf{iIn}.Interp.Coherence(iOut,:), '-r');
        semilogx(evFrf_MIMO.FRD.Frequency, squeeze(evFrf_MIMO.Coherence(iOut,iIn,:)), '-.m');
        legend({'Linear Model', 'Excited/Estimated System', 'Interpolated', 'MIMO'});
        
        subplot(3,1,1);
        title(['Bode Plot: ', sysOL.InputName{iIn}, ' to ', sysOL.OutputName{iOut}]);
    end
end

%%
figure();
hNyq = nyquistplot(sysLin_frd); hold on; grid on;
nyquistplot(evFrf_MIMO.FRD);
legend({'Linear Model', 'Excited/Estimated Interpolated MIMO'});
setoptions(hNyq, 'FreqUnits', 'Hz');
setoptions(hNyq, 'MagUnits', 'abs');
% setoptions(hNyq, 'IOGrouping', 'all');


figure();
hSig = sigmaplot(sysLin_frd, [], 2); hold on; grid on;
sigmaplot(evFrf_MIMO.FRD, [], 2);
legend({'Linear Model', 'Excited/Estimated Interpolated MIMO'});
setoptions(hSig, 'FreqUnits', 'Hz');
setoptions(hSig, 'MagUnits', 'abs');
