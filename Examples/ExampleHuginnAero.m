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
segDef{iSeg}.vehName = 'Huginn'; segDef{iSeg}.fltNum = '05';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
segDef{iSeg}.time_us = [622279236, 634279236]; % 26 m/s
segDef{iSeg}.segName = [segDef{iSeg}.fltName, ' AeroSym1 26 m/s'];

iSeg = iSeg + 1;
segDef{iSeg}.vehName = 'Huginn'; segDef{iSeg}.fltNum = '05';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg}, 'FLT');
segDef{iSeg}.time_us = [687832534, 699832534]; % 26 m/s
segDef{iSeg}.segName = [segDef{iSeg}.fltName, ' AeroSym2 26 m/s'];


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
    aZ = [segData{iSeg}.aCenterFwdIMU_IMU_mps2(2,:) - segData{iSeg}.aCenterFwdIMU_IMU_mps2(2,1); ...
        segData{iSeg}.aCenterAftIMU_IMU_mps2(2,:) - segData{iSeg}.aCenterAftIMU_IMU_mps2(2,1); ...
        segData{iSeg}.aLeftMidIMU_IMU_mps2(2,:) - segData{iSeg}.aLeftMidIMU_IMU_mps2(2,1); ...
        segData{iSeg}.aLeftFwdIMU_IMU_mps2(2,:) - segData{iSeg}.aLeftFwdIMU_IMU_mps2(2,1); ...
        segData{iSeg}.aLeftAftIMU_IMU_mps2(2,:) - segData{iSeg}.aLeftAftIMU_IMU_mps2(2,1); ...
        segData{iSeg}.aRightMidIMU_IMU_mps2(2,:) - segData{iSeg}.aRightMidIMU_IMU_mps2(2,1); ...
        segData{iSeg}.aRightFwdIMU_IMU_mps2(2,:) - segData{iSeg}.aRightFwdIMU_IMU_mps2(2,1); ...
        segData{iSeg}.aRightAftIMU_IMU_mps2(2,:) - segData{iSeg}.aRightAftIMU_IMU_mps2(2,1)];
    
    aCoefEta1 = [456.1, -565.1, -289.9, 605.4, 472.4, -292, 605.6, 472.2] * 0.3048;
    measEta1 = aCoefEta1 * (aZ - mean(aZ)) * 1e-3;
    
    aCoefEta1dt = [2.956, -2.998, -1.141, 5.974, 5.349, -1.149, 5.974, 5.348] * 0.3048;
    measEta1dt = aCoefEta1dt * (aZ - mean(aZ)) * 1e-3;

    segData{iSeg}.sensBend_mps2 = measEta1;
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
    
    u_exc{iSeg} = [segData{iSeg}.Excitation.cmdTE1L_rad + segData{iSeg}.Excitation.cmdTE1R_rad; 
        segData{iSeg}.Excitation.cmdTE2L_rad + segData{iSeg}.Excitation.cmdTE2R_rad; 
        segData{iSeg}.Excitation.cmdTE3L_rad + segData{iSeg}.Excitation.cmdTE3R_rad; 
        segData{iSeg}.Excitation.cmdTE4L_rad + segData{iSeg}.Excitation.cmdTE4R_rad; 
        segData{iSeg}.Excitation.cmdTE5L_rad + segData{iSeg}.Excitation.cmdTE5R_rad; 
        segData{iSeg}.Excitation.cmdLEL_rad + segData{iSeg}.Excitation.cmdLER_rad ];
    

    u_cmd{iSeg} = [segData{iSeg}.Control.cmdTE1L_rad + segData{iSeg}.Control.cmdTE1R_rad; 
        segData{iSeg}.Control.cmdTE2L_rad + segData{iSeg}.Control.cmdTE2R_rad; 
        segData{iSeg}.Control.cmdTE3L_rad + segData{iSeg}.Control.cmdTE3R_rad; 
        segData{iSeg}.Control.cmdTE4L_rad + segData{iSeg}.Control.cmdTE4R_rad; 
        segData{iSeg}.Control.cmdTE5L_rad + segData{iSeg}.Control.cmdTE5R_rad; 
        segData{iSeg}.Control.cmdLEL_rad + segData{iSeg}.Control.cmdLER_rad ];
    
    u_cmd{iSeg}(~any(u_exc{iSeg}, 2), :) = [];
    u_exc{iSeg}(~any(u_exc{iSeg}, 2), :) = [];
    
    u_ctl{iSeg} = u_cmd{iSeg} -  u_exc{iSeg};
    
    y{iSeg} = [segData{iSeg}.wB_I_rps(1,:); segData{iSeg}.wB_I_rps(2,:); segData{iSeg}.sensBend_mps2];
    
    freq{iSeg} = {fltData.(segDef{iSeg}.fltName).config.Excitation.OMS_Surf_1.Frequency, ...
        fltData.(segDef{iSeg}.fltName).config.Excitation.OMS_Surf_2.Frequency, ...
        fltData.(segDef{iSeg}.fltName).config.Excitation.OMS_Surf_3.Frequency}; % [rad/s]
end

[numIn, ~] = size(u_exc{1});
[numOut, ~] = size(y{1});


%% Plot Ins and Outs
for iSeg = 1:numSeg
    figure;
    subplot(4,1,1);
    plot(t{iSeg} - t{iSeg}(1), u_exc{iSeg}); grid on; hold on;
    subplot(4,1,2);
    plot(t{iSeg} - t{iSeg}(1), u_ctl{iSeg}); grid on; hold on;
    subplot(4,1,3);
    plot(t{iSeg} - t{iSeg}(1), u_cmd{iSeg}); grid on; hold on;
    subplot(4,1,4);
    plot(t{iSeg} - t{iSeg}(1), y{iSeg}); grid on; hold on;
end


%%
FRF.Opt = [];
FRF.Opt.DftType = 'ChirpZ';
FRF.Opt.ScaleType = 'spectrum';
FRF.Opt.Ts = 1/50;

FRF.Opt.Window.Type = 'cosi';
FRF.Opt.Window.TaperRatio = 0.5;

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
    
    [euFrf{iSeg}, euFrf_MIMO{iSeg}] = FreqRespEst(u_exc{iSeg}, u_ctl{iSeg}, FRF);
    [eyFrf{iSeg}, eyFrf_MIMO{iSeg}] = FreqRespEst(u_exc{iSeg}, y{iSeg}, FRF);
    
    % Tvb = Teb * inv(Tev)
%     uyFrf_MIMO{iSeg}.FRD = eyFrf_MIMO{iSeg}.FRD * inv(euFrf_MIMO{iSeg}.FRD);
%     uyFrf_MIMO{iSeg}.Coherence = eyFrf_MIMO{iSeg}.Coherence;
end

% margin(vbFrf_MIMO{iSeg}.FRD(1,1))

return;

%% Sim
Huginn_System;

% Linear Model Response
fLin = linspace(0.4, 120, 500);
sysLin_frd = frd(sysCtrlL, fLin, 'rad/s');


%% Bode
for iIn = 1:numIn
    for iOut = 1:numOut
        figure();
        legList = {};
        for iSeg = 1:numSeg
            legList{end+1} = segDef{iSeg}.segName;

%             [gLin, pLin, ~] = bode(sysLin_frd(iOut,iIn));
%             gLin = Mag2DB(squeeze(gLin));
%             pLin = squeeze(pLin);
%             
%             cLin = ones(size(fLin));
            
            fMimo = eyFrf_MIMO{iSeg}.FRD.Frequency;
            [gMimo, pMimo] = GainPhase(squeeze(eyFrf_MIMO{iSeg}.FRD.ResponseData(iOut,iIn,:)));
            cMimo = squeeze(eyFrf_MIMO{iSeg}.Coherence(iOut,iIn,:));
            
            subplot(3,1,1);
%             semilogx(fLin, gLin, 'k'); grid on; hold on;
            semilogx(fMimo, gMimo, '*-r'); grid on; hold on;

            subplot(3,1,2);
%             semilogx(fLin, pLin, 'k'); grid on; hold on;
            semilogx(fMimo, pMimo, '*-r'); grid on; hold on;

            subplot(3,1,3);
%             semilogx(fLin, cLin, 'k'); grid on; hold on;
            semilogx(fMimo, cMimo, '*-r'); grid on; hold on;
            linkaxes(findobj(gcf, 'Type', 'axes'), 'x');
            
            subplot(3,1,1);
            title(['Bode Plot: ', num2str(iIn), ' to ', num2str(iOut)]);
        end
        
        legend(legList);
    end
end

%% Nyquist
for iIn = 1:numIn
    for iOut = 1:numOut
        figure();
        legList = {'Linear Model'};
        hNyq = nyquistplot(sysLin_frd(iOut,iIn)); hold on; grid on;
        for iSeg = 1:numSeg
            legList{end+1} = segDef{iSeg}.fltName;

%             hNyq = nyquistplot(vbFrf{iSeg}{iIn}.FRD(iOut, :), '--*'); hold on;
            nyquistplot(vbFrf_MIMO{iSeg}.FRD(iOut, iIn), '-*');
        end
        setoptions(hNyq, 'FreqUnits', 'Hz');
        
        legend(legList);
    end
end