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
% pathRoot = fullfile('/mnt', 'Data', 'chris', 'Documents', 'FlightArchive');
% pathRoot = fullfile('/', 'home', 'rega0051', 'FlightArchive');
pathRoot = fullfile('O:', 'Shared drives', 'UAVLab', 'Flight Data');


%% Segment Defintions
makeFltName = @(seg, segType) [seg.vehName, segType, seg.fltNum];

segDef = {};

iSeg = 1;
% segDef{iSeg}.vehName = 'Huginn'; segDef{iSeg}.fltNum = '03';
% segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
% segDef{iSeg}.time_us = [693478512, 708458622]; % 23 m/s
% 
% iSeg = iSeg + 1;
% segDef{iSeg}.vehName = 'Huginn'; segDef{iSeg}.fltNum = '03';
% segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
% segDef{iSeg}.time_us = [865897708, 878157589]; % 26 m/s
% 
% iSeg = iSeg + 1;
% segDef{iSeg}.vehName = 'Huginn'; segDef{iSeg}.fltNum = '04';
% segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
% segDef{iSeg}.time_us = [884703586, 896345033]; % 29 m/s
% 
% iSeg = iSeg + 1;
% segDef{iSeg}.vehName = 'Huginn'; segDef{iSeg}.fltNum = '04';
% segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
% segDef{iSeg}.time_us = [998753749, 1010974466]; %32 m/s
% 
% iSeg = iSeg + 1;
segDef{iSeg}.vehName = 'Huginn'; segDef{iSeg}.fltNum = '05';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
segDef{iSeg}.time_us = [582428501, 594428501]; % 26 m/s

iSeg = iSeg + 1;
segDef{iSeg}.vehName = 'Huginn'; segDef{iSeg}.fltNum = '05';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
segDef{iSeg}.time_us = [799508311, 811508311]; % 29 m/s

iSeg = iSeg + 1;
segDef{iSeg}.vehName = 'Huginn'; segDef{iSeg}.fltNum = '06';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
segDef{iSeg}.time_us = [955842068, 967842068]; % 32 m/s

iSeg = iSeg + 1;
segDef{iSeg}.vehName = 'Huginn'; segDef{iSeg}.fltNum = '06';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
segDef{iSeg}.time_us = [1122559648, 1134559648]; %23 m/s

% Create the Data, Config, and Def fullfiles, add to segDef
makeDataPath = @(pathRoot, seg) fullfile(pathRoot, seg.vehName, seg.fltName, [seg.fltName '.mat']);
makeConfigPath = @(pathRoot, seg) fullfile(pathRoot, seg.vehName, seg.fltName, [lower(seg.vehName) '.json']);
makeDefPath = @(pathRoot, seg) fullfile(pathRoot, seg.vehName, seg.fltName, [lower(seg.vehName) '_def.json']);

numSeg = length(segDef);
for iSeg = 1:numSeg
%     segDef{iSeg}.time_us(2) = segDef{iSeg}.time_us(1) + 12e6;
    segDef{iSeg}.fileData = makeDataPath(pathRoot, segDef{iSeg});
    segDef{iSeg}.fileConfig = makeConfigPath(pathRoot, segDef{iSeg});
    segDef{iSeg}.fileDef = makeDefPath(pathRoot, segDef{iSeg});
end

%% Load Data
[segData, fltData] = LoadRaptrs(segDef);


%%
logVers = [];
for iSeg = 1:numSeg
    
    if isfield(fltData.(segDef{iSeg}.fltName).config.Excitation, 'WaveDef')
        logVers(iSeg) = 2;
        
        segData{iSeg}.Control.cmdBend_pid_rpsFF = zeros(size(segData{iSeg}.Control.cmdPitch_PID_rpsFF));
        segData{iSeg}.Control.cmdBend_pid_rpsFB = zeros(size(segData{iSeg}.Control.cmdPitch_PID_rpsFF));
    else
        logVers(iSeg) = 1;
        
        segData{iSeg}.Control.cmdBend_PID_ndFF = zeros(size(segData{iSeg}.Control.cmdPitch_PID_rpsFF));
        segData{iSeg}.Control.cmdBend_PID_ndFB = zeros(size(segData{iSeg}.Control.cmdPitch_PID_rpsFF));
    end
end


%% Frequency Response
t = {};
v_exc = {};
v_ff = {};
v_fb = {};
v = {};
freq = {};

for iSeg = 1:numSeg

    t{iSeg} = segData{iSeg}.time_s;
    v_exc{iSeg} = [segData{iSeg}.Excitation.cmdRoll_rps; segData{iSeg}.Excitation.cmdPitch_rps; segData{iSeg}.Excitation.cmdBend_nd];
    
    if logVers(iSeg) == 2
        v_ff{iSeg} = [segData{iSeg}.Control.cmdRoll_pid_rpsFF; segData{iSeg}.Control.cmdPitch_pid_rpsFF; segData{iSeg}.Control.cmdBend_pid_rpsFF];
        v_fb{iSeg} = [segData{iSeg}.Control.cmdRoll_pid_rpsFB; segData{iSeg}.Control.cmdPitch_pid_rpsFB; segData{iSeg}.Control.cmdBend_pid_rpsFB];
        
        v{iSeg} = v_exc{iSeg} + [segData{iSeg}.Control.cmdRoll_rps; segData{iSeg}.Control.cmdPitch_rps; segData{iSeg}.Control.cmdBend_nd];
        
        freq{iSeg} = {fltData.(segDef{iSeg}.fltName).config.Excitation.WaveDef.OMS_1.Frequency, fltData.(segDef{iSeg}.fltName).config.Excitation.WaveDef.OMS_2.Frequency, fltData.(segDef{iSeg}.fltName).config.Excitation.WaveDef.OMS_3.Frequency}; % [rad/s]
    
    else
        v_ff{iSeg} = [segData{iSeg}.Control.cmdRoll_PID_rpsFF; segData{iSeg}.Control.cmdPitch_PID_rpsFF; segData{iSeg}.Control.cmdBend_PID_ndFF];
        v_fb{iSeg} = [segData{iSeg}.Control.cmdRoll_PID_rpsFB; segData{iSeg}.Control.cmdPitch_PID_rpsFB; segData{iSeg}.Control.cmdBend_PID_ndFB];
        
        v{iSeg} = [segData{iSeg}.Control.cmdRoll_rps; segData{iSeg}.Control.cmdPitch_rps; segData{iSeg}.Control.cmdBend_nd];
        
        freq{iSeg} = {fltData.(segDef{iSeg}.fltName).config.Excitation.OMS_RTSM_1.Frequency, fltData.(segDef{iSeg}.fltName).config.Excitation.OMS_RTSM_2.Frequency, fltData.(segDef{iSeg}.fltName).config.Excitation.OMS_RTSM_3.Frequency}; % [rad/s]
    end
end

[numIn, ~] = size(v_exc{1});
[numOut, ~] = size(v_fb{1});


%% Plot Ins and Outs
for iSeg = 1:numSeg
    figure;
    subplot(4,1,1);
    plot(t{iSeg} - t{iSeg}(1), v_exc{iSeg}); grid on; hold on;
    subplot(4,1,2);
    plot(t{iSeg} - t{iSeg}(1), v{iSeg}); grid on; hold on;
    subplot(4,1,3);
    plot(t{iSeg} - t{iSeg}(1), v_ff{iSeg}); grid on; hold on;
    subplot(4,1,4);
    plot(t{iSeg} - t{iSeg}(1), v_fb{iSeg}); grid on; hold on;
end


%%
optFrf.dftType = 'ChirpZ';
optFrf.scaleType = 'spectrum';
optFrf.freqRate = 50 * hz2rps;

optFrf.optWin.type = 'rect';
% optFrf.optWin.taperRatio = 0.0;

optFrf.optSmooth.type = 'rect';
optFrf.optSmooth.len = 3;

evFrf = {};
ebFrf = {};
vbFrf = {};

Tev = {};
Teb = {};
Tvb = {};
Cev = {};
Ceb = {};
Cvb = {};

for iSeg = 1:numSeg
    % Compute Frequency Response
    optFrf.freq = freq{iSeg};
    
    % Merge Frf at different frequencies
    % Interpolate or fit then interpolate to get a common frequency basis
    optFrf.freqE = sort(vertcat(optFrf.freq{:}));
    
    evFrf{iSeg} = FreqRespEst(v_exc{iSeg}, v{iSeg}, optFrf);
    ebFrf{iSeg} = FreqRespEst(v_exc{iSeg}, v_fb{iSeg}, optFrf);
    
    % Tvb = Teb * inv(Tev)
    Tev{iSeg} = zeros(numOut, numIn, length(optFrf.freqE));
    Teb{iSeg} = zeros(numOut, numIn, length(optFrf.freqE));
    
    for iIn = 1:numIn
        Tev{iSeg}(:, iIn, :) = evFrf{iSeg}{iIn}.T;
        Cev{iSeg}(:, iIn, :) = evFrf{iSeg}{iIn}.coher;
        Teb{iSeg}(:, iIn, :) = ebFrf{iSeg}{iIn}.T;
        Ceb{iSeg}(:, iIn, :) = ebFrf{iSeg}{iIn}.coher;
    end
    
    for iFreq = 1:length(optFrf.freqE)
        Tvb{iSeg}(:, :, iFreq) = Teb{iSeg}(:, :, iFreq) * inv(Tev{iSeg}(:, :, iFreq));
        Cvb{iSeg}(:, :, iFreq) = Cev{iSeg}(:, :, iFreq);
    end
    
    vbFrf{iSeg}.freq = optFrf.freqE;
    vbFrf{iSeg}.T = Tvb{iSeg};
    vbFrf{iSeg}.coher = Cvb{iSeg};
    vbFrf{iSeg}.sys = frd(vbFrf{iSeg}.T, vbFrf{iSeg}.freq, 1/(optFrf.freqRate / hz2rps));

end

% margin(vbFrf{iSeg}.sys)


%%
iIn = 3;

optSpect.dftType = 'ChirpZ';
optSpect.scaleType = 'spectrum';
optSpect.freqRate = 50 * hz2rps;

optSpect.freq = freq{iSeg}{iIn};

optSpect.optWin.type = 'rect';
% optSpect.optWin.taperRatio = 0.0;

optSpect.optSmooth.type = 'rect';
optSpect.optSmooth.len = 3;

x = v_exc{iSeg}(iIn,:);
% y = v{iSeg}(iIn,:);
y = v_fb{iSeg}(1,:);

% xSpect = SpectEst(x, optSpect);
% SpectPlot(xSpect, optPlot);
xyFrf = FreqRespEst(x, y, optSpect);

optPlot.freqUnits = 'Hz';
BodePlot(xyFrf, optPlot);


%% Sim
% UltraStick25e_System;


%%
optPlot.freqUnits = 'Hz';
for iSeg = 1:4
%         [figBode] = BodePlot(vbFrf{iSeg}, optPlot);
        
    for iFrf = 1:length(evFrf{iSeg})
%         [figSpectIn] = SpectPlot(ebFrf{iSeg}{iFrf}.inSpect, optPlot);
%         [figSpectOut] = SpectPlot(ebFrf{iSeg}{iFrf}.outSpect, optPlot);
%         [figSpectOut] = SpectPlot(evFrf{iSeg}{iFrf}.outSpect, optPlot);
        
%         [figBode] = BodePlot(ebFrf{iSeg}{iFrf}, optPlot);
%         [figBode] = BodePlot(evFrf{iSeg}{iFrf}, optPlot);
        
%         [figNyquist] = NyquistPlot(ebFrf{iFrf}, optPlot);
%         [figNyquist] = NyquistPlot(evFrf{iFrf}, optPlot);
%         [figNyquist] = NyquistPlot(vbFrf{iFrf}, optPlot);
    end
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
