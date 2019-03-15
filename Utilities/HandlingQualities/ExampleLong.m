function [] = ExampleLong(stkQ_in, qBody_dps, nz_g, theta_deg, alpha_deg, vTrue_fps, category, plotTitleDesc)
% Example of HQ routines for short period longitudinal axis analysis.
%
%Usage:  [] = ExampleLong(stkQ_in, qBody_dps, nz_g, theta_deg, alpha_deg, vTrue_fps, category, plotTitleDesc);
%
%Inputs:
% stkQ_in       - pitch stick input (inch)
% qBody_dps     - pitch rate, body axis (deg/sec)
% nz_g          - normal load at center of rotation (g)
% theta_deg     - attitude angle (deg)
% alpha_deg     - angle of attack (deg)
% vTrue_fps     - true velocity (ft/sec)
% category      - flight phase category designation
% plotTitleDesc - descriptor for plot titles []
%
%Outputs:
% 
%
%Notes:
% Inputs are from a frequency response data.
%
%Dependency:
% BandwidthEst
% BodePlot
% TimeDelayEst
% TransFunc
% Loes
% NealSmith
% ShortTermPitch
% SmithGeddes
%
%Reference:
% MIL-STD-1797A, Appendix A, Paragraph 4.2.1.2
%

%Version History: Version 1.1
% 03/07/2006  C. Regan     Initial Release (v1.0)
% 03/27/2007  C. Regan     Updated Calls to 'TransFunc' (v1.1)
%


%% Check I/O Arguments
error(nargchk(7, 8, nargin, 'struct'))
if nargin < 8
    plotTitleDesc = [];
end

error(nargoutchk(0, 0, nargout, 'struct'))


%% Default Values and Constants
% Constants
hz2rps = 2*pi;
rps2hz = 1/hz2rps;
d2r = pi/180;
grav_fps2 = 32.174;

%% Check Inputs
% Inputs as column vectors
stkQ_in = stkQ_in(:);
qBody_dps = qBody_dps(:);
nz_g = nz_g(:);
theta_deg = theta_deg(:);
alpha_deg = alpha_deg(:);


%% Calculate frequency responses (q, nz, theta)
freqRate_rps = 80 * hz2rps;
freqVec_rps = logspace(-2, 2, 200);
coherLimit = 0.75;

[freqQ_rps, gainQ_dB, phaseQ_deg, coherQ] = TransFunc(stkQ_in, qBody_dps, freqVec_rps, freqRate_rps, 'cosi', []);
plotTitleBodeQ = {'Frequency Response'; [plotTitleDesc ': q/pitch stick']};
BodePlot(freqQ_rps, gainQ_dB, phaseQ_deg, coherQ, coherLimit, plotTitleBodeQ);

[freqNz_rps, gainNz_dB, phaseNz_deg, coherNz] = TransFunc(stkQ_in, nz_g, freqVec_rps, freqRate_rps, 'cosi', []);
plotTitleBodeNz = {'Frequency Response'; [plotTitleDesc ': nz/pitch stick']};
BodePlot(freqNz_rps, gainNz_dB, phaseNz_deg, coherNz, coherLimit, plotTitleBodeNz);

[freqTheta_rps, gainTheta_dB, phaseTheta_deg, coherTheta] = TransFunc(stkQ_in, theta_deg, freqVec_rps, freqRate_rps, 'cosi', []);
plotTitleBodeTheta = {'Frequency Response'; [plotTitleDesc ': theta/pitch stick']};
BodePlot(freqTheta_rps, gainTheta_dB, phaseTheta_deg, coherTheta, coherLimit, plotTitleBodeTheta);


%%
% LOES comparisons will be performed at certain frequencies, interpolate
freqLoes_rps = logspace(-1, 1, 20);

freqLoesQ_rps = freqLoes_rps;
gainLoesQ_dB = interp1(freqQ_rps, gainQ_dB, freqLoesQ_rps, 'spline');
phaseLoesQ_deg = interp1(freqQ_rps, phaseQ_deg, freqLoesQ_rps, 'spline');
coherLoesQ = interp1(freqQ_rps, coherQ, freqLoesQ_rps, 'spline');

freqLoesNz_rps = freqLoes_rps;
gainLoesNz_dB = interp1(freqNz_rps, gainNz_dB, freqLoesNz_rps, 'spline');
phaseLoesNz_deg = interp1(freqNz_rps, phaseNz_deg, freqLoesNz_rps, 'spline');
coherLoesNz = interp1(freqNz_rps, coherNz, freqLoesNz_rps, 'spline');

freqLoesTheta_rps = freqLoes_rps;
gainLoesTheta_dB = interp1(freqTheta_rps, gainTheta_dB, freqLoesTheta_rps, 'spline');
phaseLoesTheta_deg = interp1(freqTheta_rps, phaseTheta_deg, freqLoesTheta_rps, 'spline');
coherLoesTheta = interp1(freqTheta_rps, coherTheta, freqLoesTheta_rps, 'spline');


%% LOES
% Calculate the Normal load per angle of attack slope
nzAlphaFit = polyfit((alpha_deg * d2r), nz_g, 1);
% nzFit_g = polyval(nzAlphaFit, (alpha_deg * d2r));
nzAlpha_gpr = nzAlphaFit(1);

% Approximate Lalpha
Lalpha = (nzAlpha_gpr * grav_fps2) ./ vTrue_fps;

% LOES fits
% Q/Stick
% tfParamsKnownQ = [NaN, NaN, NaN, 0.0, Lalpha];
tfParamsKnownQ = [NaN, NaN, NaN, 0.0, NaN];
[tfParamsQ] = Loes(freqLoesQ_rps, gainLoesQ_dB, phaseLoesQ_deg, 1, tfParamsKnownQ, [], [], plotTitleDesc);

% Nz/Stick
tfParamsKnownNz = [NaN, NaN, NaN, 0.0];
[tfParamsNz] = Loes(freqLoesNz_rps, gainLoesNz_dB, phaseLoesNz_deg, 2, tfParamsKnownNz, [], [], plotTitleDesc);

% Q/Stick and Nz/Stick (High Order)
% tfParamsKnownHoQNz = [NaN, NaN, NaN, 0.0, NaN, NaN, 0.0, NaN, NaN, tfParamsQ.z, tfParamsQ.w];
tfParamsKnownHoQNz = [NaN, NaN, NaN, 0.0, NaN, NaN, 0.0, NaN, NaN, NaN, NaN];
[tfParamsHoQNz] = Loes(freqLoesQ_rps, gainLoesQ_dB, phaseLoesQ_deg, 3, tfParamsKnownHoQNz, gainLoesNz_dB, phaseLoesNz_deg, plotTitleDesc);

% Q/Stick and Nz/Stick (Low Order)
% tfParamsKnownQNz = [NaN, Lalpha, 0.0, NaN, 0.0, NaN, NaN];
tfParamsKnownQNz = [NaN, NaN, 0.0, NaN, 0.0, NaN, NaN];
[tfParamsQNz] = Loes(freqLoesQ_rps, gainLoesQ_dB, phaseLoesQ_deg, 4, tfParamsKnownQNz, gainLoesNz_dB, phaseLoesNz_deg, plotTitleDesc);


%% Neal-Smith Criterion and Plots
% Calculate the bandwidth (theta/pitch stick)
freqBW_rps = BandwidthEst(freqTheta_rps, gainTheta_dB, phaseTheta_deg);
respType = 'theta';

[pilotLead_deg, resPeak_dB, freqCL_rps, gainCL_dB, phaseCL_deg] = NealSmith(freqTheta_rps, gainTheta_dB, phaseTheta_deg, freqBW_rps, [], [], respType, plotTitleDesc);


%% ReCalculate frequency responses (theta, nz)
% freqRate_rps = 80 * hz2rps;
% freqVec_rps = [0.01 40];
% coherLimit = 0.75;
% 
% [freqTheta_rps, gainTheta_dB, phaseTheta_deg, coherTheta] = TransFunc(stkQ_in, theta_deg, freqVec_rps, freqRate_rps, 'cosi', []);
% plotTitleBodeTheta = {'Frequency Response'; [plotTitleDesc ': theta/pitch stick']};
% BodePlot(freqTheta_rps, gainTheta_dB, phaseTheta_deg, coherTheta, coherLimit, plotTitleBodeTheta);
% 
% [freqNz_rps, gainNz_dB, phaseNz_deg, coherNz] = TransFunc(stkQ_in, nz_g, freqVec_rps, freqRate_rps, 'cosi', []);
% plotTitleBodeNz = {'Frequency Response'; [plotTitleDesc ': nz/pitch stick']};
% BodePlot(freqNz_rps, gainNz_dB, phaseNz_deg, coherNz, coherLimit, plotTitleBodeNz);


%% Short-Term Pitch Response Criterion
freqSP_rps = tfParamsQNz.wsp;
dampSP = tfParamsQNz.zsp;
tTheta2_s = tfParamsHoQNz.Ttheta2;

% Calculate the estimated equivalent time delay (theta/pitch stick)
tDelay_s = TimeDelayEst(freqTheta_rps, phaseTheta_deg);

% Test Criterion and Plots
ShortTermPitch(freqSP_rps, dampSP, tTheta2_s, nzAlpha_gpr, freqBW_rps, tDelay_s, category, plotTitleDesc);


%% Smith-Geddes Criterion and Plots
respType = 'long';
[freqPilotBW_rps, phasePilotBW_deg, gainSlope_db_oct, aveCH, phaseNzBW_deg] = SmithGeddes(freqTheta_rps, gainTheta_dB, phaseTheta_deg, phaseNz_deg, respType, plotTitleDesc);


%% Check Outputs
