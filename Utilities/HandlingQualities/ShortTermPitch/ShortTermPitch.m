function [figHandle] = ShortTermPitch(freqSP_rps, dampSP, tTheta2_s, nzAlpha_gpr, freqBW_rps, tDelay_s, category, plotTitleDesc, saveFile)
% Plot summary of "Short-term pitch response" criterion, MIL-STD-1797A.
%
%Usage:  [figHandle] = ShortTermPitch(freqSP_rps, dampSP, tTheta2_s, nzAlpha_gpr, freqBW_rps, tDelay_s, category, plotTitleDesc, saveFile)
%
%Inputs:
% freqSP_rps    - short-period frequency (rad/sec)
% dampSP        - short-period damping
% tTheta2_s     - time constant for the lag between flight path and pitch attitude response (sec)
% nzAlpha_gpr   - normal load per angle of attack (g/rad)
% freqBW_rps    - bandwidth of 'theta/dep' transfer function (rad/sec)
% tDelay_s      - estimated equivalent time delay (sec)
% category      - flight phase category designation
% plotTitleDesc - descriptor for plot titles []
% saveFile      - file to save figure (and switch) []
%
%Outputs:
% figHandle - handle object for the summary figure
%
%Notes:
% This function simply consolidates all the basic short-period requirements
% citeria and plotting routines.
%
%Dependency:
% BandwidthCrit
% BandwidthPlot
% CapCrit
% CapPlot
% PitchRespCrit
% PitchRespPlot
% ShortPeriodCrit
% ShortPeriodPlot
%
%Notes:
% 
%
%Reference:
% MIL-STD-1797A, Appendix A, Paragraph 4.2.1.2
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(7, 9, nargin, 'struct'))
if nargin < 9, saveFile = [];
    if nargin < 8, plotTitleDesc = []; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs
% Inputs as column vectors
freqSP_rps = freqSP_rps(:);
dampSP = dampSP(:);
tTheta2_s = tTheta2_s(:);
nzAlpha_gpr = nzAlpha_gpr(:);
freqBW_rps = freqBW_rps(:);
tDelay_s = tDelay_s(:);


%% Determine Levels
[cap, levelCAP, level1CAP, level2CAP, level3CAP] = CapCrit(freqSP_rps, dampSP, nzAlpha_gpr, category);
[levelSP, level1SP, level2SP, level3SP] = ShortPeriodCrit(nzAlpha_gpr, freqSP_rps, category);
[thetaLag_rad, levelPR, level1PR, level2PR, level3PR] = PitchRespCrit(freqSP_rps, dampSP, tTheta2_s, category);
[levelBW, level1BW, level2BW, level3BW] = BandwidthCrit(freqBW_rps, tDelay_s, category);


%% Plot
if ~isempty(plotTitleDesc)
    figure;

    subplot(2, 2, 1);
    plotTitleCAP = {['MIL-STD-1797A - Category ' category ' - CAP Requirements']; plotTitleDesc};
    plotTextCAP = {['SP damp: ' num2str(dampSP)]; ['CAP: ' num2str(cap)]};
    CapPlot(dampSP, cap, level1CAP, level2CAP, level3CAP, levelCAP, plotTitleCAP, plotTextCAP);

    subplot(2, 2, 3);
    plotTitleSP = {['MIL-STD-1797A - Category ' category ' - Alternate Short-Period Requirements']; plotTitleDesc};
    plotTextSP = {['Nz/AOA: ' num2str(nzAlpha_gpr)]; ['SP freq: ' num2str(freqSP_rps)]};
    ShortPeriodPlot(nzAlpha_gpr, freqSP_rps, level1SP, level2SP, level3SP, levelSP, plotTitleSP, plotTextSP);

    subplot(2, 2, 2);
    plotTitlePR = {['MIL-STD-1797A - Category ' category ' - Pitch Response Requirements']; plotTitleDesc};
    plotTextPR = {['SP damp: ' num2str(dampSP)]; ['SP freq: ' num2str(freqSP_rps)]; ['tTheta2: ' num2str(tTheta2_s)]};
    PitchRespPlot(dampSP, thetaLag_rad, level1PR, level2PR, level3PR, levelPR, plotTitlePR, plotTextPR);

    subplot(2, 2, 4);
    plotTitleBW = {['MIL-STD-1797A - Category ' category ' - Bandwidth Requirements']; plotTitleDesc};
    plotTextBW = {['BW freq: ' num2str(freqBW_rps)]; ['Delay: ' num2str(tDelay_s)]};
    BandwidthPlot(freqBW_rps, tDelay_s, level1BW, level2BW, level3BW, levelBW, plotTitleBW, plotTextBW);

    figHandle = gcf;

    % Save the plot as a fig
    if saveFile
        saveas(figHandle, saveFile, 'fig');
    end
end


%% Check Outputs
