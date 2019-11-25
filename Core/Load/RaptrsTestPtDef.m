function [testPtDef] = RaptrsTestPtDef(fltData)
% Find the Test Segments of interest
%
%Inputs:
% fltData - RAPTRS Log and Config Structures
%
%Outputs:
% testPtDef - Structure of segments
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan

%% Check I/O Arguments
narginchk(1, 1);
nargoutchk(1, 1);

%% Find Longest Segment of SOC Engaged
time_us = fltData.logData.Sensors.Fmu.Time_us;
socEng = fltData.logData.Mission.socEngage;

socStart = find([0, diff(socEng)]);
socEnd = find([diff(1-socEng), 1]);

segLen = socEnd - socStart;

[~, iMax] = max(segLen);

iSocEng = socStart(iMax):socEnd(iMax);
socEngSegDef = struct('time_us', time_us([socStart(iMax),socEnd(iMax)]));


%% Find Excitation Segments
time_us = fltData.logData.Sensors.Fmu.Time_us(iSocEng);
excEngage = fltData.logData.Mission.excitEngage(iSocEng);

excStart = find([0, diff(excEngage)]);
excEnd = find([diff(1-excEngage), 0]);

try testIndx = fltData.logData.Mission.testID(iSocEng(excStart+1)) + 1; end
try testIndx = fltData.logData.Mission.testPtID(iSocEng(excStart)) + 1; end

excSegDef = fltData.config.Mission_Manager.Test_Points(testIndx);
for iExc = 1:length(excSegDef)
    excSegDef(iExc).time_us = time_us([excStart(iExc), excEnd(iExc)]);
end

%%
testPtDef.SocEngage = socEngSegDef;
testPtDef.Excitations = excSegDef;

