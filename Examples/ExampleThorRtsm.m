
%% Constants
r2d = 180/pi;
d2r = 1/r2d;
in2m = 0.0254;


%%

pathRoot = fullfile('G:', 'Shared drives', 'UAVLab', 'Flight Data');



%% Load Config
fileConfig = fullfile(pathData, ['thor' '.json']);
fileFltDef = fullfile(pathData, ['thor_def' '.json']);


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
[fltData, segData] = LoadData(segDef);



clear flt;

flt{1}.param.vehName = 'Thor'; flt{1}.param.fltNum = '120'; 
flt{1}.param.apType = 'Goldy3'; flt{1}.param.config = '2';

flt{2}.param.vehName = 'Thor'; flt{2}.param.fltNum = '121'; 
flt{2}.param.apType = 'Goldy3'; flt{2}.param.config = '2';




