function [xSmooth] = SmoothFunc(x, optSmooth)
% Smoothing function based on convolution of a kernel over the data.
%
%Usage:  [xSmooth] = SmoothFunc(x, optSmooth);
%
%Inputs:
% x          - input time history
% optSmooth
%   type - method description ['rect']
%   len  - total width of the kernel [1]
%
%Outputs:
% xSmooth - smoothed time history
% 

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
narginchk(1, 2)
if nargin < 2
    optSmooth = struct();
end
if ~isfield(optSmooth, 'type'), optSmooth.type = []; end
if ~isfield(optSmooth, 'len'), optSmooth.len = []; end

nargoutchk(0, 1)


%% Default Values and Constants
if isempty(optSmooth.type), optSmooth.type = 'rect'; end
if isempty(optSmooth.len), optSmooth.len = 1; end


%% Check Inputs
[widthX, ~] = size(x);

% optSmooth.len must be greater than 1
if optSmooth.len <= 1
    xSmooth = x;
    return;
end


%% Define the Convolution Kernel
switch lower(optSmooth.type)
    case 'rect'
        kernel = ones(1, optSmooth.len) / optSmooth.len;
    otherwise % TODO: add more smoothing kernels
        warning('MATLAB:SmoothData:method', 'Rectangular is the only known kernel for smoothing') % FIXME: message ID
        kernel = ones(1, optSmooth.len) / optSmooth.len;
end


%% Perform the Smoothing
% Pad the start and end with values
lenPad = floor(optSmooth.len/2);
xPad = [repmat(x(:,1), 1, lenPad), x, repmat(x(:,end), 1, lenPad)];

% Convolve the kernel over the data
for i = 1:widthX
    xSmooth(i,:) = conv(xPad(i,:), kernel, 'same');
end

xSmooth = xSmooth(:, 1+lenPad : end-lenPad);

%% Check Output
