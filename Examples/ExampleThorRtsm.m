% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
%
% Author: Chris Regan

%% Constants
r2d = 180/pi;
d2r = 1/r2d;
in2m = 0.0254;
hz2rps = 2*pi;

%%
% pathRoot = fullfile('G:', 'Shared drives', 'UAVLab', 'Flight Data');
pathRoot = fullfile('/mnt', 'Data', 'chris', 'Documents', 'FlightArchive');


%% Segment Defintions
makeFltName = @(seg) [seg.vehName, 'FLT', seg.fltNum];

segDef = {};

iSeg = 1;
segDef{iSeg}.vehName = 'Thor'; segDef{iSeg}.fltNum = '126';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg});
segDef{iSeg}.time_us = [830130591, 840130591 + 2e6/50];

iSeg = iSeg+1;
segDef{iSeg}.vehName = 'Thor'; segDef{iSeg}.fltNum = '128';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg});
segDef{iSeg}.time_us = [959859721, 985320803];

iSeg = iSeg+1;
segDef{iSeg}.vehName = 'Thor'; segDef{iSeg}.fltNum = '128';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg});
segDef{iSeg}.time_us = [847254621, 881436255];


% Create the Data, Config, and Def fullfiles, add to segDef
makeDataPath = @(pathRoot, seg) fullfile(pathRoot, seg.vehName, seg.fltName, [seg.fltName '.mat']);
makeConfigPath = @(pathRoot, seg) fullfile(pathRoot, seg.vehName, seg.fltName, [lower(seg.vehName) '.json']);
makeDefPath = @(pathRoot, seg) fullfile(pathRoot, seg.vehName, seg.fltName, [lower(seg.vehName) '_def.json']);

numSeg = length(segDef);
for iSeg = 1:numSeg
    segDef{iSeg}.fileData = makeDataPath(pathRoot, segDef{iSeg});
    segDef{iSeg}.fileConfig = makeConfigPath(pathRoot, segDef{iSeg});
    segDef{iSeg}.fileDef = makeDefPath(pathRoot, segDef{iSeg});
end

%% Load Data
[segData, fltData] = LoadData(segDef);

%%
for iSeg = 1:numSeg
    segData{iSeg}.Control.cmdYaw_pidFF = zeros(size(segData{iSeg}.Control.cmdPitch_pidFF));
    segData{iSeg}.Control.cmdYaw_pidFB = segData{iSeg}.wB_I_rps(3,:);
end

%% Plot Ins and Outs

%% Frequency Response

iSeg = 1;

t = segData{iSeg}.time_s;
v_exc = [segData{iSeg}.Excitation.cmdRoll_rps; segData{iSeg}.Excitation.cmdPitch_rps; segData{iSeg}.Excitation.cmdYaw_rps];

v_ff = [segData{iSeg}.Control.cmdRoll_pidFF; segData{iSeg}.Control.cmdPitch_pidFF; segData{iSeg}.Control.cmdYaw_pidFF];
v_fb = [segData{iSeg}.Control.cmdRoll_pidFB; segData{iSeg}.Control.cmdPitch_pidFB; segData{iSeg}.Control.cmdYaw_pidFB];

v = [segData{iSeg}.Control.cmdRoll_rps; segData{iSeg}.Control.cmdPitch_rps; segData{iSeg}.Control.cmdYaw_rps];

freq = {fltData.ThorFLT126.config.Excitation.OMS_RTSM_1.Frequency, fltData.ThorFLT126.config.Excitation.OMS_RTSM_2.Frequency, fltData.ThorFLT126.config.Excitation.OMS_RTSM_3.Frequency}; % [rad/s]

[numIn, ~] = size(v_exc);
[numOut, ~] = size(v_fb);

%%
optFrf.dftType = 'ChirpZ';
optFrf.scaleType = 'spectrum';
optFrf.freqRate = 50 * hz2rps;
optFrf.freq = freq;

optFrf.optWin.type = 'tukey';
optFrf.optWin.taperRatio = 0.1;

optFrf.optSmooth.type = 'rect';
optFrf.optSmooth.len = 3;

% Merge Frf at different frequencies
% Interpolate or fit then interpolate to get a common frequency basis
optFrf.freqE = sort(vertcat(optFrf.freq{:}));

% Compute Frequency Response
[evFrf] = FreqRespEst(v_exc, v, optFrf);
[ebFrf] = FreqRespEst(v_exc, v_fb, optFrf);


optPlot.freqUnits = 'Hz';
for iFrf = 1:length(evFrf)
    %     [figSpectIn] = SpectPlot(xyFrf{iFrf}.inSpect, optPlot);
    %     [figSpectOut] = SpectPlot(xyFrf{iFrf}.outSpect, optPlot);
    
    [figBode] = BodePlot(evFrf{iFrf}, optPlot);
    [figBode] = BodePlot(ebFrf{iFrf}, optPlot);
    
    %     [figNyquist] = NyquistPlot(evFrf{iFrf}, optPlot);
    %     [figNyquist] = NyquistPlot(ebFrf{iFrf}, optPlot);
end

%%
numFreqE = length(optFrf.freqE );
evT = nan(numOut, numIn, numFreqE);
ebT = nan(numOut, numIn, numFreqE);
for iFrf = 1:length(evFrf)
    evT(:, iFrf, :) = evFrf{iFrf}.T;
    ebT(:, iFrf, :) = ebFrf{iFrf}.T;
    
    L.coher(:, iFrf, :) = ebFrf{iFrf}.coher;
end

L.freq = optFrf.freqE;
L.T = nan(size(evT));
for iFreq = 1:numFreqE
    L.T(:, :, iFreq) = ebT(:, :, iFreq) * inv(evT(:, :, iFreq));
end

[L.gain_dB, L.phase_deg] = GainPhase(L.T);

for iIn = 1:numIn
    
    %     [figNyquist] = NyquistPlot(evFrf{iFrf}, optPlot);
    
    figure;
    subplot(3,1,1)
    plot(L.freq, squeeze(L.gain_dB(:, iIn, :)));
    subplot(3,1,2)
    plot(L.freq, squeeze(L.phase_deg(:, iIn, :)));
    subplot(3,1,3)
    plot(L.freq, squeeze(L.coher(:, iIn, :)));
    
%     figure;
%     subplot(3,1,1:2)
%     plot(real(squeeze(L.T(:, iIn, :)))', imag(squeeze(L.T(:, iIn, :)))');
%     subplot(3,1,3)
%     plot(L.freq, squeeze(L.coher(:, iIn, :)));
end
