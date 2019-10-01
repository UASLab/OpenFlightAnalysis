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
pathRoot = fullfile('G:', 'Shared drives', 'UAVLab', 'Flight Data');


%% Segment Defintions
makeFltName = @(seg) [seg.vehName, 'FLT', seg.fltNum];

segDef = {};

iSeg = 1;
segDef{iSeg}.vehName = 'Thor'; segDef{iSeg}.fltNum = '126';
segDef{iSeg}.fltName = makeFltName(segDef{iSeg});
segDef{iSeg}.time_us = [829130591, 842470942];

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

%% Plot Ins and Outs

%% Frequency Response

optSpect.dftType = 'ChirpZ';
optSpect.freqRate = 50 * hz2rps;
optSpect.freq = [1:10] * hz2rps;

optSpect.scaleType = 'density';


x = segData{1}.Excitation.cmdRoll_rps;
y = segData{1}.Control.cmdRoll_rps;

optSpect.optWin.len = length(x);
[freq, xyT, xyC, xxP, yyP, xyP] = FreqRespEst(x, y, optSpect);


