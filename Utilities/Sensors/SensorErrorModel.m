function [xTrue] = SensorErrorModel(xMeas, param)
% Compute the "True" Measure based on the error model parameters.
%
%Inputs:
% xMeas   - measures
% param   - error model parameters [errorType = 'None']
%
%Outputs:
% xTrue   - true
%
%Notes:
% There are no internal unit conversions.
% Generally, Flight data analysis uses the "-" version of the models, simulation would use "+"
%

%Version History: Version 1.2
% 10/14/2016  C. Regan     Initial Release (v1.0)
% 10/17/2016  C. Regan     Changed to simple and faster input parsing (v1.1)
% 10/25/2016  C. Regan     Fixed 'inversebias+' and 'inversebias-' types (v1.2)
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(1, 2);
if nargin < 2
    param = [];
end

% Default Values
if isempty(param), param.errorType = 'None'; end

% Check the number of outputs
nargoutchk(0, 1);


%% Compute the True Measure
lenSeg = length(xMeas);

switch lower(param.errorType)
    case {'none'} % None
        xTrue = xMeas;
    case {'delay'} % time delay
        indxSeg = 1:length(xMeas);
        xTrue = interp1(indxSeg, xMeas', indxSeg - param.data, 'linear', 'extrap')';
    case {'bias+'} % Bias
        xTrue = xMeas + repmat(param.bias, 1, length(xMeas));
    case {'bias-'} % Bias
        xTrue = xMeas - repmat(param.bias, 1, length(xMeas));
    case {'scalebias+'} % Scale and Bias
        xTrue = param.K * xMeas + repmat(param.bias, 1, length(xMeas));
    case {'quad+'} % Quad, Scale, and Bias
        xTrue = param.Quad^2 * xMeas + param.K^2 * xMeas + repmat(param.bias, 1, length(xMeas));
    case {'scalebias-'} % Scale and Bias
        xTrue = param.K \ (xMeas - repmat(param.bias, 1, length(xMeas)));
    case {'gyro+'} % Scale and Bias with linear acceleration sensitivity
        xTrue = param.K * (xMeas + param.G * param.aTrue) + repmat(param.bias, 1, length(xMeas));
    case {'gyro-'} % Scale and Bias with linear acceleration sensitivity
        xTrue = param.K \ (xMeas - repmat(param.bias, 1, length(xMeas))) - param.G * param.aTrue;
    case {'inversebias+'}
        xTrue = param.K ./ xMeas + repmat(param.bias, 1, length(xMeas));
    case {'inverse2bias+'}
        xTrue = param.K ./ xMeas.^2 + repmat(param.bias, 1, length(xMeas));
    case {'inversesqrtbias+'}
        xTrue = param.K ./ sqrt(xMeas) + repmat(param.bias, 1, length(xMeas));
    case {'inversebias-'}
        xTrue = param.K ./ (xMeas - repmat(param.bias, 1, length(xMeas)));
        
    otherwise
        disp('Unkown Option');
end


