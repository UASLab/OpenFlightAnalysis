function [] = ExampleDir(stkR_in, beta_deg, category, plotTitleDesc)
% Example of HQ routines for directional axis analysis.
%
%Usage:  [] = ExampleDir(stkR_in, beta_deg, category, plotTitleDesc);
%
%Inputs:
% stkR_in       - yaw pedal input (inch)
% beta_deg      - sideslip angle (deg)
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
%

%Version History: Version 1.1
% 03/07/2006  C. Regan     Initial Release (v1.0)
% 03/27/2007  C. Regan     Updated Calls to 'TransFunc' (v1.1)
%


%% Check I/O Arguments
error(nargchk(3, 4, nargin, 'struct'))
if nargin < 4
    plotTitleDesc = [];
end

error(nargoutchk(0, 0, nargout, 'struct'))


%% Default Values and Constants
% Constants
hz2rps = 2*pi;
rps2hz = 1/hz2rps;


%% Check Inputs
% Inputs as column vectors
stkR_in = stkR_in(:);
beta_deg = beta_deg(:);


%% Calculate frequency responses (beta)
freqRate_rps = 80 * hz2rps;
freqVec_rps = [0.5 15];
coherLimit = 0.75;

[freqBeta_rps, gainBeta_dB, phaseBeta_deg, coherBeta] = TransFunc(stkR_in, beta_deg, freqVec_rps, freqRate_rps, 'cosi', []);

plotTitleBodeBeta = {'Frequency Response'; [plotTitleDesc ': beta/yaw pedal']};
BodePlot(freqBeta_rps, gainBeta_dB, phaseBeta_deg, coherBeta, coherLimit, plotTitleBodeBeta);


%% LOES
% LOES fits
[tfParamsBeta] = Loes(freqBeta_rps, gainBeta_dB, phaseBeta_deg, 9, [], [], [], plotTitleDesc);
[tfParamsHoBeta] = Loes(freqBeta_rps, gainBeta_dB, phaseBeta_deg, 10, [], [], [], plotTitleDesc);


%% Check Outputs
