function [] = ExampleLat(stkP_in, pBody_dps, phi_deg, ny_g, category, plotTitleDesc)
% Example of HQ routines for lateral axis analysis.
%
%Usage:  [] = ExampleLat(stkP_in, pBody_dps, phi_deg, ny_g, category, plotTitleDesc);
%
%Inputs:
% stkP_in       - roll stick input (inch)
% pBody_dps     - roll rate, body axis (deg/sec)
% phi_deg       - bank angle (deg)
% ny_g          - side load at center of rotation (g)
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
% BodePlot
% TransFunc
% Loes
% SmithGeddes
%

%Version History: Version 1.1
% 03/07/2006  C. Regan     Initial Release (v1.0)
% 03/27/2007  C. Regan     Updated Calls to 'TransFunc' (v1.1)
%


%% Check I/O Arguments
error(nargchk(5, 6, nargin, 'struct'))
if nargin < 6
    plotTitleDesc = [];
end

error(nargoutchk(0, 0, nargout, 'struct'))


%% Default Values and Constants
% Constants
hz2rps = 2*pi;
rps2hz = 1/hz2rps;


%% Check Inputs
% Inputs as column vectors
stkP_in = stkP_in(:);
pBody_dps = pBody_dps(:);
phi_deg = phi_deg(:);
ny_g = ny_g(:);


%% Calculate frequency responses (p, phi)
freqRate_rps = 80 * hz2rps;
freqVec_rps = [0.01 20];
coherLimit = 0.75;

[freqP_rps, gainP_dB, phaseP_deg, coherP] = TransFunc(stkP_in, pBody_dps, freqVec_rps, freqRate_rps, 'cosi', []);
[freqPhi_rps, gainPhi_dB, phasePhi_deg, coherPhi] = TransFunc(stkP_in, phi_deg, freqVec_rps, freqRate_rps, 'cosi', []);

plotTitleBodeP = {'Frequency Response'; [plotTitleDesc ': p/roll stick']};
BodePlot(freqP_rps, gainP_dB, phaseP_deg, coherP, coherLimit, plotTitleBodeP);

plotTitleBodePhi = {'Frequency Response'; [plotTitleDesc ': phi/roll stick']};
BodePlot(freqPhi_rps, gainPhi_dB, phasePhi_deg, coherPhi, coherLimit, plotTitleBodePhi);


%% Loes
% LOES fits
[tfParamsP] = Loes(freqP_rps, gainP_dB, phaseP_deg, 7, [], [], [], plotTitleDesc);
[tfParamsHoP] = Loes(freqP_rps, gainP_dB, phaseP_deg, 8, [], [], [], plotTitleDesc);
[tfParamsHoPhi] = Loes(freqPhi_rps, gainPhi_dB, phasePhi_deg, 11, [], [], [], plotTitleDesc);
[tfParamsPhi] = Loes(freqPhi_rps, gainPhi_dB, phasePhi_deg, 12, [], [], [], plotTitleDesc);

%% ReCalculate frequency responses (phi, ny)
freqRate_rps = 80 * hz2rps;
freqVec_rps = [0.5 20];
coherLimit = 0.75;

[freqPhi_rps, gainPhi_dB, phasePhi_deg, coherPhi] = TransFunc(stkP_in, phi_deg, freqVec_rps, freqRate_rps, 'cosi', []);
[freqNy_rps, gainNy_dB, phaseNy_deg, coherNy] = TransFunc(stkP_in, ny_g, freqVec_rps, freqRate_rps, 'cosi', []);

plotTitleBodePhi = {'Frequency Response'; [plotTitleDesc ': phi/roll stick']};
BodePlot(freqPhi_rps, gainPhi_dB, phasePhi_deg, coherPhi, coherLimit, plotTitleBodePhi);

plotTitleBodeNy = {'Frequency Response'; [plotTitleDesc ': ny/roll stick']};
BodePlot(freqNy_rps, gainNy_dB, phaseNy_deg, coherNy, coherLimit, plotTitleBodeNy);


%% Smith-Geddes Criterion and Plots
respType = 'latdir';
[freqPilotBW_rps, phasePilotBW_deg, gainSlope_db_oct, aveCH, phaseNyBW_deg] = SmithGeddes(freqPhi_rps, gainPhi_dB, phasePhi_deg, phaseNy_deg, respType, plotTitleDesc);


%% Check Outputs
