function [structFDAT] = StructFlight(structRaw, fileType, fileVers)
% Description of Function
%
%Inputs:
% structRaw   - Raw flight structure
% fileType    - Type of flight system
% fileVers    - Version of flight system []
%
%Outputs:
% structFDAT - Structure of file variables
%
%Notes:
% Any Special Notes.
%

%Version History: Version 1.1
% 09/23/2016  Name     Initial Release (v1.0)
% 10/17/2016  Name     Simplified Parsing (v1.1)
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(2, 3);
if nargin < 3
    fileVers = [];
end

% Default Values

% Check the number of outputs
nargoutchk(0, 1);


%% Constants


%% Load the appropriate function for the type of flight computer
switch lower(fileType)
    case {'goldy1'}
        [structFDAT] = StructFlightGoldy1(structRaw, fileVers);        
    case {'goldy2'}
        [structFDAT] = StructFlightGoldy2(structRaw);
    case {'goldy3'}
        [structFDAT] = StructFlightGoldy3(structRaw, fileVers);
    case {'px4'}
        [structFDAT] = StructFlightPX4(structRaw);
    case {'apm'}
        [structFDAT] = StructFlightAPM(structRaw);
    otherwise
end


%% Outputs
