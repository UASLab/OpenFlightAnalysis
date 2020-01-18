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
segDef{iSeg}.vehName = 'Thor'; segDef{iSeg}.fltNum = '128';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
segDef{iSeg}.time_us = [700283749, 712283749];

iSeg = iSeg + 1;
segDef{iSeg}.vehName = 'Thor'; segDef{iSeg}.fltNum = '128';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
segDef{iSeg}.time_us = [714405473, 746405473];

iSeg = iSeg + 1;
segDef{iSeg}.vehName = 'Thor'; segDef{iSeg}.fltNum = '129';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
segDef{iSeg}.time_us = [1461077095, 1473077095];

iSeg = iSeg + 1;
segDef{iSeg}.vehName = 'Thor'; segDef{iSeg}.fltNum = '01';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'SIM_PID');
segDef{iSeg}.time_us = [227979999, 239979999];


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
[segData, fltData] = LoadRaptrs(segDef);


%%
logVers = [];
for iSeg = 1:numSeg
    if isfield(fltData.(segDef{iSeg}.fltName).config.Excitation, 'WaveDef')
        logVers(iSeg) = 2;
        
        segData{iSeg}.Control.cmdYaw_pid_rpsFF = zeros(size(segData{iSeg}.Control.cmdPitch_pid_rpsFF));
        segData{iSeg}.Control.cmdYaw_pid_rpsFB = segData{iSeg}.Control.Test.SCAS_Att.cmdYaw_damp_rps;
    else
        logVers(iSeg) = 1;
        
        segData{iSeg}.Control.cmdYaw_pidFF = zeros(size(segData{iSeg}.Control.cmdPitch_pidFF));
        segData{iSeg}.Control.cmdYaw_pidFB = segData{iSeg}.Control.cmdYaw_damp_rps;
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
    v_exc{iSeg} = [segData{iSeg}.Excitation.cmdRoll_rps; segData{iSeg}.Excitation.cmdPitch_rps; segData{iSeg}.Excitation.cmdYaw_rps];
    
    if logVers(iSeg) == 2
        v_ff{iSeg} = [segData{iSeg}.Control.cmdRoll_pid_rpsFF; segData{iSeg}.Control.cmdPitch_pid_rpsFF; segData{iSeg}.Control.cmdYaw_pid_rpsFF];
        v_fb{iSeg} = [segData{iSeg}.Control.cmdRoll_pid_rpsFB; segData{iSeg}.Control.cmdPitch_pid_rpsFB; segData{iSeg}.Control.cmdYaw_pid_rpsFB];
        
        v{iSeg} = v_exc{iSeg} + [segData{iSeg}.Control.cmdRoll_rps; segData{iSeg}.Control.cmdPitch_rps; segData{iSeg}.Control.cmdYaw_rps];
        
        freq{iSeg} = {fltData.(segDef{iSeg}.fltName).config.Excitation.WaveDef.OMS_1.Frequency, fltData.(segDef{iSeg}.fltName).config.Excitation.WaveDef.OMS_2.Frequency, fltData.(segDef{iSeg}.fltName).config.Excitation.WaveDef.OMS_3.Frequency}; % [rad/s]
        
    else
        v_ff{iSeg} = [segData{iSeg}.Control.cmdRoll_pidFF; segData{iSeg}.Control.cmdPitch_pidFF; segData{iSeg}.Control.cmdYaw_pidFF];
        v_fb{iSeg} = [segData{iSeg}.Control.cmdRoll_pidFB; segData{iSeg}.Control.cmdPitch_pidFB; segData{iSeg}.Control.cmdYaw_pidFB];
        
        v{iSeg} = [segData{iSeg}.Control.cmdRoll_rps; segData{iSeg}.Control.cmdPitch_rps; segData{iSeg}.Control.cmdYaw_rps];
        
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
% FR Estimation
FRF.Opt = [];
FRF.Opt.DftType = 'ChirpZ';
FRF.Opt.ScaleType = 'spectrum';
FRF.Opt.Ts = 1/50;

FRF.Opt.Window.Type = 'rect';
% FRF.Opt.Window.TaperRatio = 0.1;

FRF.Opt.Smooth.Type = 'rect';
FRF.Opt.Smooth.Length = 3;

FRF.Opt.Interp.Type = 'linear';
FRF.Opt.MIMO = true;


evFrf = {};
ebFrf = {};
vbFrf = {};

evFrf_MIMO = {};
ebFrf_MIMO = {};
vbFrf_MIMO = {};

for iSeg = 1:numSeg
    % Compute Frequency Response
    FRF.Opt.Frequency = freq{iSeg};
    
    % Merge Frf at different frequencies
    % Interpolate or fit then interpolate to get a common frequency basis
    FRF.Opt.Interp.FreqInterp = sort(vertcat(FRF.Opt.Frequency{:}));
    
    [evFrf{iSeg}, evFrf_MIMO{iSeg}] = FreqRespEst(v_exc{iSeg}, v{iSeg}, FRF);
    [ebFrf{iSeg}, ebFrf_MIMO{iSeg}] = FreqRespEst(v_exc{iSeg}, v_fb{iSeg}, FRF);
    
    % Tvb = Teb * inv(Tev)
    vbFrf_MIMO{iSeg}.FRD = ebFrf_MIMO{iSeg}.FRD * inv(evFrf_MIMO{iSeg}.FRD);
    vbFrf_MIMO{iSeg}.Coherence = evFrf_MIMO{iSeg}.Coherence;
end

% margin(vbFrf_MIMO{iSeg}.FRD(1,1))

return;


%% Sim
UltraStick25e_System;

% Linear Model Response
sysLin_frd = frd(sysCtrlL, linspace(0.4, 120, 500), 'rad/s');


%%
for iSeg = 1:numSeg
    [numOut, numIn] = size(vbFrf_MIMO{iSeg}.FRD);
    numFreq = length(vbFrf_MIMO{iSeg}.FRD.Frequency);
    
    % ID Model
    vbFrf_MIMO{iSeg}.IDFRD = idfrd(vbFrf_MIMO{iSeg}.FRD);
%     vbFrf_MIMO{iSeg}.IDFRD.SpectrumData = 0.1 + 0.0 * abs(vbFrf_MIMO{iSeg}.FRD.ResponseData);
%     vbFrf_MIMO{iSeg}.IDFRD.NoiseCovariance = 0.1 + 0.0 * abs(vbFrf_MIMO{iSeg}.FRD.ResponseData);
    vbFrf_MIMO{iSeg}.IDFRD.CovarianceData = zeros(numOut, numIn, numFreq, 2, 2);
    
    % Unc Model
    vbFrf_MIMO{iSeg}.UFRD = ufrd(vbFrf_MIMO{iSeg}.FRD);
%     vbFrf_MIMO{iSeg}.UFRD. = 
    
    for iIn = 1:numIn
        for iOut = 1:numOut
            for iFreq = 1:numFreq
                vbFrf_MIMO{iSeg}.IDFRD.CovarianceData(iOut, iIn, iFreq, :, :) = 0.1 * eye(2);
            end
        end
    end
end


%% Plots
for iIn = 1:length(evFrf{iSeg})
    for iOut = 1:length(evFrf{iSeg})
        figure();
        legList = {'Linear Model'};
        hBode = bodeplot(sysLin_frd(iOut,iIn)); hold on; grid on;
        for iSeg = 1:numSeg
            legList{end+1} = segDef{iSeg}.fltName;

            bodeplot(vbFrf_MIMO{iSeg}.IDFRD(iOut, iIn));
            showConfidence(hBode, 1);
        end
        setoptions(hBode, 'FreqUnits', 'Hz');
%         setoptions(hBode, 'MagUnits', 'abs');
        
        legend(legList);
    end
end

%% 
for iIn = 1:length(evFrf{iSeg})
    for iOut = 1:length(evFrf{iSeg})
        figure();
        legList = {'Linear Model'};
        hNyq = nyquistplot(sysLin_frd(iOut,iIn)); hold on; grid on;
        for iSeg = 1:numSeg
            legList{end+1} = segDef{iSeg}.fltName;

            nyquistplot(vbFrf_MIMO{iSeg}.IDFRD(iOut, iIn));
%             showConfidence(hNyq, 1);
        end
        setoptions(hNyq, 'FreqUnits', 'Hz');
        
        legend(legList);
    end
end

%% 
figure();
legList = {'Linear Model'};
hSig = sigmaplot(sysLin_frd, [], 2); hold on; grid on;
for iSeg = 1:numSeg
    legList{end+1} = segDef{iSeg}.fltName;
    
    sigmaplot(vbFrf_MIMO{iSeg}.IDFRD, [], 2);
%     showConfidence(hSig, 1);
end
setoptions(hSig, 'FreqUnits', 'Hz');
setoptions(hSig, 'MagUnits', 'abs');

legend(legList);

%%
opts = robOptions('VaryFrequency','on', 'Display','on');
[stabmarg,wcu,info] = robstab(vbFrf_MIMO{iSeg}.IDFRD, {0.1,10}, opts);