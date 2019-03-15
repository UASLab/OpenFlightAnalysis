function [structLoad] = LoadFlight(fileLoad, fileType, fileVers)
% Description of Function
%
%Inputs:
% fileLoad    - File to load
% fileType    - Type of flight system
%
%Outputs:
% structLoad - Structure of file variables
%
%Notes:
% Any Special Notes.
%

%Version History: Version 1.0
% 09/23/2016  Name     Initial Release (v1.0)
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(2, 2);

% Default Values

% Check the number of outputs
nargoutchk(0, 1);


%% Constants

%% Load the appropriate function for the type of flight computer
switch lower(fileType)
    case {'goldy1'}
        [structLoad] = LoadFlightGoldy1(fileLoad);        
    case {'goldy2'}
        [structLoad] = LoadFlightGoldy2(fileLoad);
    case {'goldy3'}
        [structLoad] = LoadFlightGoldy3(fileLoad);
    case {'px4_ulog'}
        [structLoad] = LoadFlightUlog(fileLoad);
    case {'px4_sdlog2'}
        [structLoad] = LoadFlightSdlog2(fileLoad);
    otherwise
end


%% Outputs

