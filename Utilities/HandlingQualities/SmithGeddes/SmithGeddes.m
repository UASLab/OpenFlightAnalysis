function [freqPilotBW_rps, phasePilotBW_deg, gainSlope_db_oct, aveCH, phase2BW_deg] = SmithGeddes(freq_rps, gain_dB, phase_deg, phase2_deg, respType, plotTitleDesc, saveName)
% Smith/Geddes handling qualities criterion.
%
%Usage:  [freqPilotBW_rps, phasePilotBW_deg, gainSlope_db_oct, aveCH, phase2BW_deg] = SmithGeddes(freq_rps, gain_dB, phase_deg, phase2_deg, respType, plotTitleDesc, saveName);
%
%Inputs:
% freq_rps      - frequency of the frequency response (rad/sec)
% gain_dB       - magnitude of the frequency response (dB)
% phase_deg     - phase of frequency responce (deg)
% phase2_deg    - phase of nz/dep (ny/dap) frequency responce (deg) []
% respType      - axis of interest: 'long' or 'latdir' ['long']
% plotTitleDesc - plot title description (and switch) []
% saveName      - file to save the figure []
%
%Outputs:
% freqPilotBW_rps  - pilot bandwidth frequency (ref: R. Smith report)
% phasePilotBW_deg - phase at pilot bandwidth frequency (deg)
% gainSlope_db_oct - slope of the gain between 1 and 6 rps (dB/octave)
% aveCH            - average Cooper-Harper Rating
% phase2BW_deg     - bandwidth phase of nz/dep (ny/dap) FR (deg) []
%
%Notes:
% Input frequency response is for theta/pitchStick or phi/rollStick.
% The optional phase frequency response input is normalLoad/pitchStick
%   the normal load is at the pilot station (g).
% Pitch attitude criterion function 'phasePilotBW_deg',
%   should be greater than -130 degrees for Level 1 Accel criterion
%   function 'phase2BW_deg', should be between -130 and -180 degrees
%   for Level 1.
%
%Dependency:
% SmithGeddesACH
% SmithGeddesComp
% SmithGeddesCrit
% SmithGeddesPlotACH
% SmithGeddesPlotFR
% SmithGeddesPlotLevels
%
%Reference:
% AFWAL-TR-81-3090
% 1993 SAE presentation
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(3, 7, nargin, 'struct'))
if nargin < 7, saveName = [];
    if nargin < 6, plotTitleDesc = []; end
    if nargin < 5, respType = 'long'; end
    if nargin < 4, phase2_deg = []; end
end

error(nargoutchk(0, 5, nargout, 'struct'))


%% Default Values and Constants
if isempty(respType), respType = 'long'; end

r2d = 180/pi;
d2r = pi/180;


%% Check Inputs
% Inputs as column vectors
freq_rps = freq_rps(:);
gain_dB = gain_dB(:);
phase_deg = phase_deg(:);
phase2_deg = phase2_deg(:);

% Unwrap the phase angle
phase_deg = unwrap(phase_deg * d2r) * r2d;
phase2_deg = unwrap(phase2_deg * d2r) * r2d;


%% Calculate the Smith-Geddes Criterion Parameters
[freqPilotBW_rps, phasePilotBW_deg, gainSlope_db_oct, phase2BW_deg, gainFitPoints] = SmithGeddesComp(freq_rps, gain_dB, phase_deg, phase2_deg, respType);


%% Average Cooper-Harper level
[levelCH, levelCHLow, levelCHUp, aveCH] = SmithGeddesACH(phasePilotBW_deg);


%% Levels
[level1Bound, level2Bound, level1PIOBound, level2PIOBound] = SmithGeddesCrit('normal');



%% Plot
if ~isempty(plotTitleDesc)
    figure;

    subplot(2, 2, 1);
    plotTitleFR = {'Smith-Geddes Requirements'; plotTitleDesc};
    plotTextGain = {['slope: ' num2str(gainSlope_db_oct) ' (dB/oct)']; ['freqBW: ' num2str(freqPilotBW_rps) ' (rps)']};
    SmithGeddesPlotFR(freq_rps, gain_dB, phase_deg, freqPilotBW_rps, gainFitPoints, 'gain', respType, plotTitleFR, plotTextGain);

    subplot(2, 2, 3);
    plotTextPhase = {['freqBW: ' num2str(freqPilotBW_rps) ' (rps)']};
    SmithGeddesPlotFR(freq_rps, gain_dB, phase_deg, freqPilotBW_rps, [], 'phase', respType, plotTitleFR, plotTextPhase);

    subplot(2, 2, 2);
    plotTitleACH = {'Smith-Geddes Requirements'; plotTitleDesc};
    plotTextACH = {['BW phase: ' num2str(phasePilotBW_deg) ' (deg)']; ['Average CH: ' num2str(aveCH)]};
    SmithGeddesPlotACH(phasePilotBW_deg, aveCH, levelCH, levelCHLow, levelCHUp, plotTitleACH, plotTextACH);

    subplot(2, 2, 4);
    plotTitleLevels = {'Smith-Geddes Requirements'; plotTitleDesc};
    plotTextLevels = {['Pilot BW phase: ' num2str(phasePilotBW_deg) ' (deg)']; ['Parameter BW phase: ' num2str(phase2BW_deg) ' (deg)']};
    SmithGeddesPlotLevels(phasePilotBW_deg, phase2BW_deg, level2Bound, level1Bound, level2PIOBound, level1PIOBound, plotTitleLevels, plotTextLevels);

    figHandle = gcf;

    % Save the plot as a fig
    if saveName
        saveas(figHandle, saveName, 'fig');
    end
end


%% Check Outputs
