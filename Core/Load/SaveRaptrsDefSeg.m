function [] = SaveRaptrsDefSeg(testPtDef, fileDef)
% Save the RAPTRS Def file
%
%Inputs:
%
%Outputs:
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan

%% Check I/O Arguments
narginchk(1, 2);
nargoutchk(0, 0);

%% Read the Existing Def
if exist(fileDef, 'file')
    dataDef = ReadConfig(fileDef);
else
    dataDef = struct('Configuration', [], 'Segments', []);
end


%% Create the output for JSON config
dataDef.Segments = testPtDef;
jsonStr = jsonencode(dataDef);

% Pretty Print
jsonStr = strrep(jsonStr, '[{', sprintf('[\r\n\t{'));
jsonStr = strrep(jsonStr, '},"', sprintf('},\r\n"'));
% jsonStr = strrep(jsonStr, ':{', sprintf(':{\r\n\t'));
% jsonStr = strrep(jsonStr, ',"', sprintf(',\r\n\t"'));
% jsonStr = strrep(jsonStr, '}}', sprintf('}\r\n}'));
jsonStr = strrep(jsonStr, '},{', sprintf('},\r\n\t{'));
jsonStr = strrep(jsonStr, ']}]', sprintf(']}\r\n]'));
jsonStr = strrep(jsonStr, ':', sprintf(': '));
jsonStr = strrep(jsonStr, ',', sprintf(', '));
jsonStr = strrep(jsonStr, ', "Segments"', sprintf(',\r\n"Segments"'));

% disp(jsonStr)

%% Save
fidDef = fopen(fileDef, 'w');
if fidDef == -1
    error([mfilename ' - Cannot create JSON file : ' fileDef]);
end
fwrite(fidDef, jsonStr, 'char');
fclose(fidDef);
