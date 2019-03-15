function [structLoad] = LoadFlightGoldy1(fileLoad)
% Load Goldy1 flight data in a structure
%
%Inputs:
% fileLoad    - File to load
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
narginchk(1, 1);

% Check the number of outputs
nargoutchk(0, 1);


%% Constants


%% Computation
structLoad = load(fileLoad);

%% Outputs

