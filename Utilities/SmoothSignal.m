function [xSmooth] = SmoothSignal(x, method, windowSize)
% Smoothing function based on convolution of a kernel over the data.
%
%Usage:  [xFilt] = SmoothSignal(x, method, windowSize);
%
%Inputs:
% x          - input time history
% method     - method description ['rect']
% windowSize - total width of the kernel [1]
%
%Outputs:
% xSmooth - smoothed time history
%
%Notes:
% FIXIT 'conv' function has built-in padding options
% 
%

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
narginchk(1, 3)
if nargin < 3
    windowSize = [];
    if nargin < 2; method = []; end
end

nargoutchk(0, 1)


%% Default Values and Constants
if isempty(method), method = 'rect'; end
if isempty(windowSize), windowSize = 1; end


%% Check Inputs
[widthX, lenX] = size(x);

% windowSize must be greater than 1
if windowSize <= 1
    xSmooth = x;
    return;
end

% Transpose
if widthX > lenX
    transposeFlag = 1;
    x = x';
    [widthX, lenX] = size(x);
else
    transposeFlag = 0;
end


%% Define the Convolution Kernel
switch method
    case 'rect'
        kernel = ones(widthX, windowSize) / windowSize;
    otherwise % TODO: add more smoothing kernels
        warning('MATLAB:SmoothData:method', 'Rectangular is the only known kernel for smoothing') % FIXME: message ID
        kernel = ones(widthX, windowSize) / windowSize;
end


%% Perform the Smoothing
% Pad the ends by half the width of filter
padLen = floor(windowSize/2);

padLo = x(1)*ones(widthX, padLen);
padHi = x(end)*ones(widthX, padLen);

% Pad the ends prior to convolution
xPad = [padLo, x, padHi];

% Convolve the filter over the data
for indxRow = 1:widthX
    xConv(indxRow, :) = conv(xPad(indxRow, :), kernel(indxRow, :));
end

% Remove the padding
xSmooth = xConv(:, 1 + 2*padLen : end - 2*padLen);


%% Check Output
% Transpose
if transposeFlag == 1
    xSmooth = xSmooth';
end
