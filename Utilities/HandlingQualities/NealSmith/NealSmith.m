function [pilotLead_deg, resPeak_dB, freqCL_rps, gainCL_dB, phaseCL_deg] = NealSmith(freq_rps, gain_dB, phase_deg, freqBW_rps, pilotLag_sec, droop_dB, respType, plotTitleDesc, saveName)
% Neal/Smith handling qualities criterion.
%
%Usage:  [pilotLead_deg, resPeak_dB, freqCL_rps, gainCL_dB, phaseCL_deg] = NealSmith(freq_rps, gain_dB, phase_deg, freqBW_rps, pilotLag_sec, droop_dB, respType, plotTitleDesc, saveName);
%
%Inputs:
% freq_rps      - frequencies corresponding to gain and phase (rad/sec)
% gain_dB       - magnitude of frequency response (dB) (Q/DEP optional)
% phase_deg     - phase of frequency response (deg) (Q/DEP optional)
% freqBW_rps    - bandwidth frequency (rad/sec) [3.0]
% pilotLag_sec  - pilot Lag (sec) [0.3]
% droop_dB      - droop value (dB) [-3.0]
% respType      - 'q' if using Q/STICK freq response ['theta']
% plotTitleDesc - plot title description (and switch) []
% saveName      - file to save the figure []
%
%Outputs:
% pilotLead_deg - Required pilot lead (deg)
% resPeak_dB    - Resonance peak resulting (dB)
% freqCL_rps    - Frequency vector close-loop frequency response
% gainCL_dB     - Magnitude of close-loop frequency response
% phaseCL_deg   - Phase of close-loop frequency response
%
%Notes:
% Based on input frequency response of pitch attitude to stick (THA/STICK)
%   or, optionally, pitch rate to stick (Q/STICK).
% Output vectors for freq, gain, phase contain one more point than input vectors.
%
%Dependency:
% NealSmithComp
% NealSmithCrit
% NealSmithPlotFR
% NealSmithPlotLevels
% NealSmithPlotNichols
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release, created based on Keith Wichman's routines (v1.0)
%


%% Check I/O Arguments
error(nargchk(3, 9, nargin, 'struct'))
if nargin < 9, saveName= [];
    if nargin < 8, plotTitleDesc = []; end
    if nargin < 7, respType = []; end
    if nargin < 6, droop_dB = []; end
    if nargin < 5, pilotLag_sec = []; end
    if nargin < 4, freqBW_rps = []; end
end

error(nargoutchk(0, 5, nargout, 'struct'))


%% Default Values and Constants
if isempty(freqBW_rps), freqBW_rps = 3.0; end
if isempty(pilotLag_sec), pilotLag_sec = 0.3; end
if isempty(droop_dB), droop_dB = -3.0; end
if isempty(respType), respType = 'theta'; end

d2r = pi/180;
r2d = 180/pi;


%% Check Inputs
% Inputs as column vectors
freq_rps = freq_rps(:);
gain_dB = gain_dB(:);
phase_deg = phase_deg(:);

% Unwrap the phase angle
phase_deg = unwrap(phase_deg * d2r) * r2d;


%% Compute the Neal/Smith Criteria paramters
[pilotLead_deg, resPeak_dB, freqCL_rps, gainCL_dB, phaseCL_deg, gainOL_dB, phaseOL_deg] = NealSmithComp(freq_rps, gain_dB, phase_deg, freqBW_rps, pilotLag_sec, droop_dB, respType);


%% Determine Levels
[level1Bound, level2Bound, level] = NealSmithCrit('new', pilotLead_deg, resPeak_dB);

levelGainBound(:,1) = [min(freqCL_rps), freqBW_rps, freqBW_rps];
levelGainBound(:,2) = [droop_dB, droop_dB, min(gainCL_dB)];

levelPhaseBound(:,1) = [min(freqCL_rps), freqBW_rps, freqBW_rps];
levelPhaseBound(:,2) = [-90, -90, min(phaseCL_deg)];


%% Plot
if ~isempty(plotTitleDesc)
    figure;
    
    subplot(2, 2, 1);
    plotTitleFR = {'Neal-Smith - Closed-Loop Response Requirements'; plotTitleDesc};
    plotTextGain = {['BW freq: ' num2str(freqBW_rps) ' (rps)']; ['Droop: ' num2str(droop_dB) ' (dB)']};
    NealSmithPlotFR(freqCL_rps, gainCL_dB, [], levelGainBound, 'gain', plotTitleFR, plotTextGain);

    subplot(2, 2, 3);
    plotTextPhase = {['BW freq: ' num2str(freqBW_rps) ' (rps)']; ['Phase: ' num2str(-90.0) ' (deg)']};
    NealSmithPlotFR(freqCL_rps, [], phaseCL_deg, levelPhaseBound, 'phase', plotTitleFR, plotTextPhase);

    subplot(2, 2, 4);
    plotTitleLevels = {'Neal-Smith Requirements'; plotTitleDesc};
    plotTextLevels = {['Pilot Lead: ' num2str(pilotLead_deg) ' (deg)']; ['Res Peak: ' num2str(resPeak_dB) ' (dB)']};
    NealSmithPlotLevels(pilotLead_deg, resPeak_dB, level1Bound, level2Bound, level, plotTitleLevels, plotTextLevels);

    subplot(2, 2, 2);
    plotTitleNichols = {'Neal-Smith Requirements'; plotTitleDesc};
    plotTextNichols = [];
    NealSmithPlotNichols(freqCL_rps, gainCL_dB, phaseCL_deg, gainOL_dB, phaseOL_deg, freqBW_rps, droop_dB, plotTitleNichols, plotTextNichols);

    figHandle = gcf;

    % Save the plot as a fig
    if saveName
        saveas(figHandle, saveName, 'fig');
    end
end


%% Check Outputs
