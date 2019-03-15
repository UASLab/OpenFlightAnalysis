function [gain_dB, phase_deg] = GainPhase(xyT)
% Compute the gain and phase from transfer function data.
%
%Usage:  [gain_dB, phase_deg] = GainPhase(xyT);
%
%Inputs:
% xyT - complex transfer function
%
%Outputs:
% gain_dB   - magnitude of transfer function (db)
% phase_deg - phase angle of transfer function (deg)
%
%Notes:
% 
%
%Dependency:
% Mag2DB
%

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
narginchk(1, 1);
nargoutchk(0, 2);


%% Default Values and Constants
% Constants
r2d = 180/pi;


%% Check Inputs
[widthT, lenT] = size(xyT);

% Transpose
if widthT > lenT
    transposeFlag = 1;
    xyT = xyT';
else
    transposeFlag = 0;
end


%% Compute magnitude and phase from complex transfer function
gain_dB = Mag2DB(abs(xyT));
phase_deg = angle(xyT) * r2d;
%phase_deg = atan2(imag(xyT), real(xyT)) * r2d; % Equivalent
%phase_deg = imag(log(xyT)) * r2d; % Equivalent but slower


%% Check Outputs
% Transpose
if transposeFlag == 1
    gain_dB = gain_dB';
    phase_deg = phase_deg';
end
