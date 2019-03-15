function [xInt] = IntegrateSignal(x, x0, time)
% Perform an integration of a signal.
%
%Inputs:
% x       - Discrete signal data
% x0      - Initial integrated signal value [0]
% time    - Time Step or Vector [1]
%
%Outputs:
% xInt - integrated signal
%
%Notes:

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release (v1.0)
%
quad

%% Check I/O Arguments
% Check the number of inputs
narginchk(1, 3);
if nargin < 3, time = [];
    if nargin < 2, x0 = []; end
end

% Default Values
if isempty(x0), x0 = 0; end
if isempty(time), time = 1; end

% Check the number of outputs
nargoutchk(0, 1);


%% Check Inputs
sizeSig = size(x);

% Create a time vector
if length(time) == 1
    time = (0:dt:(sizeSig(2)-1))';
end

% Set a flag to take the transpose
if sizeSig(1) > sizeSig(2)
    tranFlag = 1;
    x = x';
    sizeSig = size(x);
else
    tranFlag = 0;
end

%% Compute the Rotation Matrices
% Pre-Allocate Vector
xInt = NaN(sizeSig);

% Time Step
timeDiff = [NaN, diff(time)];

% Initialize the Output
xInt(:, 1) = x0;

for indx = 2 : sizeSig(2)
    timeStep = timeDiff(:, indx);
    xInt(:, indx) = xInt(:, indx - 1) + timeStep * x(:, indx);
end


%% Outputs
if tranFlag == 1; xInt = xInt'; end

