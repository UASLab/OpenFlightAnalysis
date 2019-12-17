function [xSmooth] = SmoothFunc(x, Opt)
% Smoothing function based on convolution of a kernel over the data.
%
%Usage:  [xSmooth] = SmoothFunc(x, optSmooth);
%
%Inputs:
% x          - input time history
% optSmooth
%   Type - method description ['rect']
%   Length  - total width of the kernel [1]
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
    Opt = struct();
end
if ~isfield(Opt, 'Type'), Opt.Type = []; end
if ~isfield(Opt, 'Length'), Opt.Length = []; end

nargoutchk(0, 1);


%% Default Values and Constants
if isempty(Opt.Type), Opt.Type = 'rect'; end
if isempty(Opt.Length), Opt.Length = 1; end


%% Check Inputs
[widthX, ~] = size(x);

% optSmooth.Length must be greater than 1
if Opt.Length <= 1
    xSmooth = x;
    return;
end


%% Define the Convolution Kernel
switch lower(Opt.Type)
    case 'rect'
        kernel = ones(1, Opt.Length) / Opt.Length;
    otherwise % TODO: add more smoothing kernels
        warning('MATLAB:SmoothData:method', 'Rectangular is the only known kernel for smoothing') % FIXME: message ID
        kernel = ones(1, Opt.Length) / Opt.Length;
end


%% Perform the Smoothing
% Pad the start and end with values
lenPad = floor(Opt.Length/2);
xPad = [repmat(x(:,1), 1, lenPad), x, repmat(x(:,end), 1, lenPad)];

% Convolve the kernel over the data
for i = 1:widthX
    xSmooth(i,:) = conv(xPad(i,:), kernel, 'same');
end

xSmooth = xSmooth(:, 1+lenPad : end-lenPad);

%% Check Output
