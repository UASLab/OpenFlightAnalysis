
%% Constants
r2d = 180/pi;
d2r = 1/r2d;
in2m = 0.0254;


%% Load Flight Data and Configuration Parameters
% basePath = 'D:\Flight Archive\Mjolnir';
% basePath = '/mnt/sdb1/Flight Archive/Mjolnir';
basePath = 'O:\Team Drives\UAVLab\Flight Data\Thor';

clear flt;

flt{1}.param.vehName = 'Thor'; flt{1}.param.fltNum = '120'; 
flt{1}.param.apType = 'Goldy3'; flt{1}.param.config = '2';

flt{2}.param.vehName = 'Thor'; flt{2}.param.fltNum = '121'; 
flt{2}.param.apType = 'Goldy3'; flt{2}.param.config = '2';

numFlt = length(flt);
for iFlt = 1:numFlt
    % Load Flight Data, pack into structures
    flt{iFlt}.param.fltName = [flt{iFlt}.param.vehName 'FLT' flt{iFlt}.param.fltNum];
    flt{iFlt}.param.fltFileH5 = fullfile(basePath, flt{iFlt}.param.fltName, [flt{iFlt}.param.fltName '.h5']);
    flt{iFlt}.param.fltFile = strrep(flt{iFlt}.param.fltFileH5, '.h5', '.mat');
    
    if ~exist(flt{iFlt}.param.fltFile, 'file')
        h5Convert2Mat(flt{iFlt}.param.fltFileH5);
    end
    
    [flt{iFlt}.raw] = LoadFlight(flt{iFlt}.param.fltFile, flt{iFlt}.param.apType);
    [flt{iFlt}.sig] = StructFlight(flt{iFlt}.raw, flt{iFlt}.param.apType, flt{iFlt}.param.config);
    
    % Load Configuration Parameters
    flt{iFlt}.param = LoadConfigGoldy3(flt{iFlt}.param);
    
    % Plot Flight Overview
    if 0
        PlotOverviewGoldy3(flt{iFlt}.sig);
    end
    
    if 1
        % Find Test Segments
        disp(flt{iFlt}.param.fltName)
        for iTest = 0:max(flt{iFlt}.sig.Mission.testID)
            indxSeg = find((flt{iFlt}.sig.Mission.excitEngage == 1) & (flt{iFlt}.sig.Mission.testID == iTest));
            if ~isempty(indxSeg)
                tSegMin_s = flt{iFlt}.sig.time_s(min(indxSeg));
                tSegMax_s = flt{iFlt}.sig.time_s(max(indxSeg));
            
                disp(['iExcite = ' num2str(iTest) '; seg.excite{iExcite}.time = ' mat2str([tSegMin_s, tSegMax_s]) '; seg.excite{iExcite}.indx = SegmentTime2Index(time_s, seg.excite{iExcite}.time);']);
            end
        end
    end
end

%% Load Flight Maneuver Segments
for iFlt = 1:numFlt
    [flt{iFlt}.seg] = LoadFlightSegDef(flt{iFlt}.param, flt{iFlt}.sig.time_s);
end


%% Create Segments
clear param;

% Copy Flight Parameter Set to Segment Parameter Set
indxFlt = 1; iSeg = 1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.excite{1}.indx(1) + [0:50*12];

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.excite{5}.indx(1) + [0:50*12];

indxFlt = 2; iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.excite{1}.indx(1) + [0:50*12];

% Number of Segments
numSeg = length(param);

%% Copy Raw data into Segment Structures, Vectorize
clear seg;
[param, seg] = Raw2Seg_Goldy3(flt, param);


%% RTSM
winType = 'rect';
smoothFactor = 15;
coherLimit = [];

freqVec = linspace(0.62831853071795862, 157.07963267948966, 250);

for iSeg = 1:numSeg
    t = seg{iSeg}.time_s - seg{iSeg}.time_s(1);
    
    iLoop = 1;
    x{iSeg, iLoop} = seg{iSeg}.excitePhi_rad;
    y{iSeg, iLoop} = (seg{iSeg}.refPhi_rad - seg{iSeg}.excitePhi_rad) - seg{iSeg}.sNav_BL_rad(1,:);
    
    iLoop = 2;
    x{iSeg, iLoop} = seg{iSeg}.exciteTheta_rad;
    y{iSeg, iLoop} = (seg{iSeg}.refTheta_rad - seg{iSeg}.exciteTheta_rad) - seg{iSeg}.sNav_BL_rad(2,:);
end


for iSeg = 1:numSeg
    freqRate = 2*pi / mode(diff(t));
    
    for iLoop = 1:2

        [freq, gain_dB, phase_deg, xyC, xyT, xxP, yyP, xyP] = TransFunc(x{iSeg, indxLoop}, y{iSeg, indxLoop}, [0.01, 50] * 2*pi, freqRate, winType, smoothFactor, coherLimit);
        % [freq, gain_dB, phase_deg, xyC, xyT, xxP, yyP, xyP] = TransFuncChirpZ(x{iSeg, iLoop}, y{iSeg, iLoop}, freqVec(iLoop:2:end), freqRate, winType, smoothFactor, coherLimit);
        
        sysL = -xyT ./ (1 + xyT);
        [gainL_dB, phaseL_deg] = GainPhase(sysL);
        
        % figure; semilogy(freq, xxP, freq, yyP);
        BodePlot(freq, gain_dB, phase_deg, xyC);
%         NyquistPlot(freq, sysL, xyC);
    end
end

return
%% Save
save('Mjolnir_AirspeedCal.mat', 'param', 'seg', 'optParam');
