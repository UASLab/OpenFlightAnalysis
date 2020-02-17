function [FRF, FRF_MIMO] = FreqRespNoiseEst(x, y, FRF)
% Estimate the transfer function (y/x) between two time histories.
%
%Usage:  [FRF] = FreqRespEst(x, y, FRF);
%
%Inputs:
% x            - input time history
% y            - output time history
% FRF.Opt
%   Spect
%     Ts        - sample rate (see Note)
%     Frequency - frequencies of interest (see Note)
%   Window      - Window Options
%   Smooth      - Smoothing Options
%   Interp
%     FreqInterp
%     Type
%   MIMO
%   FreqNull
%
%Outputs:
% FRF
%   Spect
%     Input
%     Output
%   FRD
%     Frequency    - frequency of transfer function (see Note)
%     ResponseData - complex transfer function
%   FRD_Null
%   PowerIn       - input Power
%   PowerOut      - output Power
%   PowerCross    - cross-spectral Power of the ouput/input
%   Coherence     - magnitude squared coherence
%
%Notes:
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(2, 3);
if nargin < 3
    FRF.Opt = struct();
end

nargoutchk(1, 2);

%% Default Values and Constants
if ~isfield(FRF.Opt, 'DftType'), FRF.Opt.DftType = []; end

if ~isfield(FRF.Opt, 'Ts'), FRF.Opt.Ts = []; end
if isempty(FRF.Opt.Ts), FRF.Opt.Ts = 1; end

if ~isfield(FRF.Opt, 'ScaleType'), FRF.Opt.ScaleType = []; end
if isempty(FRF.Opt.ScaleType), FRF.Opt.ScaleType = 'spectrum'; end

if ~isfield(FRF.Opt, 'Window'), FRF.Opt.Window = struct(); end

if ~isfield(FRF.Opt, 'MIMO'), FRF.Opt.MIMO = []; end
if isempty(FRF.Opt.MIMO), FRF.Opt.MIMO = false; end



%% Check Inputs


%% Approximate the frequency response function
if iscell(FRF.Opt.Frequency) % Cell Array of SIMO systems
    numIn = length(FRF.Opt.Frequency);
    FRF_Save = FRF;
    
    FRF = cell(size(FRF.Opt.Frequency));
    for iFrf = 1:numIn
        FRF{iFrf}.Opt = FRF_Save.Opt;
        FRF{iFrf}.Opt.Frequency = FRF_Save.Opt.Frequency{iFrf};
        
        FRF{iFrf} = FreqRespNoiseEst(x(iFrf,:), y, FRF{iFrf});
    end
    
else % SIMO at one frequency set
    % Add x and y to frf structure
    FRF.TH.Input = x;
    FRF.TH.Output = y;
    
    % Compute the DFT and Spectrum for x and y
    Spect.Opt = FRF.Opt;
    [FRF.Spect.Input] = SpectEst(FRF.TH.Input, Spect);
    [FRF.Spect.Output] = SpectEst(FRF.TH.Output, Spect);
    
    
    % Add x and y specturm to frf structure
    FRF.PowerIn = FRF.Spect.Input.Power;
    FRF.PowerOut = FRF.Spect.Output.Power;
    
    % Compute cross spectrum, Scale is doubled because one-sided DFTs
    FRF.PowerCross = 2*FRF.Spect.Input.Scale * FRF.Spect.Input.DFT .* conj(FRF.Spect.Output.DFT);
    
    
    % Null Frequency Components
    SpectNull.Opt =  FRF.Opt;
    SpectNull.Opt.Frequency = FRF.Opt.FreqNull;
    [FRF.Spect.InNull] = SpectEst(FRF.TH.Input, SpectNull);
    [FRF.Spect.OutNull] = SpectEst(FRF.TH.Output, SpectNull);
    FRF.PowerInNull = FRF.Spect.InNull.Power;
    FRF.PowerOutNull = FRF.Spect.OutNull.Power;
    
    %% Compute complex transfer function approximation
    T = FreqRespEstCmplx(FRF.PowerIn, FRF.PowerCross);
    T = reshape(T, [], 1, length(FRF.Opt.Frequency)); % Shift dims so that the FRF is Ny X Nu X Nf
    
    FRF.FRD = frd(T, FRF.Opt.Frequency, FRF.Opt.Ts);
    
    % Smooth the PSDs
    if isfield(FRF.Opt, 'Smooth')
        FRF.PowerCrossRaw = FRF.PowerCross;
        FRF.PowerCross = SmoothFunc(FRF.PowerCrossRaw, FRF.Opt.Smooth);
    end
    
    % Compute magnitude squared coherence
    FRF.Coherence = Coherence(FRF.PowerCross, FRF.PowerIn, FRF.PowerOut);
    
    % Interpolate, as a SIMO
    if isfield(FRF.Opt, 'Interp')
        if ~isfield(FRF.Opt.Interp, 'FreqInterp')
            error([mfilename ' - FreqInterp must be specified.']);
        end
        FRF.Interp = InterpFRD(FRF);
        
        T = FreqRespEstCmplx(FRF.Interp.PowerIn, FRF.Interp.PowerCross);
        T = reshape(T, [], 1, length(FRF.Interp.Frequency)); % Shift dims so that the FRF is Ny X Nu X Nf
        FRF.Interp.FRD = frd(T, FRF.Interp.Frequency, FRF.Opt.Ts);
        
        FRF.Interp.Coherence = Coherence(FRF.Interp.PowerCross, FRF.Interp.PowerIn, FRF.Interp.PowerOut);

        Unc = FreqRespEstCmplx(FRF.Interp.PowerIn, abs(FRF.Interp.PowerCrossNull));
        Unc = reshape(Unc, [], 1, length(FRF.Interp.Frequency)); % Shift dims so that the FRF is Ny X Nu X Nf
        FRF.Interp.Unc = frd(Unc, FRF.Interp.Frequency, FRF.Opt.Ts);
    end
    
end % SIMO


% Create the MIMO model
% The FRF must be a set of SIMO models with common Frequencies
if (iscell(FRF) == true)
    if ((FRF{1}.Opt.MIMO == true) && isfield(FRF{1}, 'Interp'))
        
        FRF_MIMO.Frequency = FRF{1}.Interp.Frequency;
        
        [numOut, ~] = size(FRF{1}.Interp.FRD);
        numIn = length(FRF);
        numFreq = length(FRF_MIMO.Frequency);
        
        FRF_MIMO.PowerIn = zeros(1, numIn, numFreq);
        FRF_MIMO.PowerOut = zeros(numOut, numIn, numFreq);
        FRF_MIMO.PowerCross = zeros(numOut, numIn, numFreq);
        FRF_MIMO.Coherence = zeros(numOut, numIn, numFreq);
        
        % Null        
        FRF_MIMO.PowerInNull = zeros(1, numIn, numFreq);
        FRF_MIMO.PowerOutNull = zeros(numOut, numIn, numFreq);
        FRF_MIMO.PowerCrossNull = zeros(numOut, numIn, numFreq);
        
        for iIn = 1:numIn
            FRF_MIMO.PowerIn(1, iIn, :) = FRF{iIn}.Interp.PowerIn;
            FRF_MIMO.PowerOut(:, iIn, :) = FRF{iIn}.Interp.PowerOut;
            FRF_MIMO.PowerCross(:, iIn, :) = FRF{iIn}.Interp.PowerCross;
            FRF_MIMO.Coherence(:, iIn, :) = FRF{iIn}.Interp.Coherence;
            
            FRF_MIMO.PowerInNull(1, iIn, :) = FRF{iIn}.Interp.PowerInNull;
            FRF_MIMO.PowerOutNull(:, iIn, :) = FRF{iIn}.Interp.PowerOutNull;
            FRF_MIMO.PowerCrossNull(:, iIn, :) = FRF{iIn}.Interp.PowerCrossNull;
        end
        
        FRF_MIMO.FRD = frd(FreqRespEstCmplx(FRF_MIMO.PowerIn, FRF_MIMO.PowerCross), FRF_MIMO.Frequency, FRF{1}.Opt.Ts);
        FRF_MIMO.Unc = frd(FreqRespEstCmplx(FRF_MIMO.PowerIn, abs(FRF_MIMO.PowerCrossNull)), FRF_MIMO.Frequency, FRF{1}.Opt.Ts);
        
    end
end

end % function

% Interpolate or fit then interpolate to get a common frequency basis
function [Interp] = InterpFRD(FRF)

if ~isfield(FRF.Opt.Interp, 'Type'), FRF.Opt.Interp.Type = []; end
if isempty(FRF.Opt.Interp.Type), FRF.Opt.Interp.Type = 'linear'; end
interpType = FRF.Opt.Interp.Type; % 'linear', 'nearest', 'next', 'previous', 'pchip', 'cubic', 'v5cubic', 'makima', or 'spline'

if ~isfield(FRF.Opt.Interp, 'Extrap'), FRF.Opt.Interp.Extrap = []; end
if isempty(FRF.Opt.Interp.Extrap), FRF.Opt.Interp.Extrap = 'extrap'; end
extrapType = FRF.Opt.Interp.Extrap; % 'extrap', nan

Interp.Frequency = FRF.Opt.Interp.FreqInterp;

Interp.PowerIn = interp1(FRF.FRD.Frequency, FRF.PowerIn', Interp.Frequency, interpType, extrapType)';
Interp.PowerOut = interp1(FRF.FRD.Frequency, FRF.PowerOut', Interp.Frequency, interpType, extrapType)';
%     Interp.PowerCross = interp1(FRF.FRD.Frequency, FRF.PowerCross', Interp.Frequency, interpType, extrapType)';
Interp.PowerCross = interpPolar(FRF.FRD.Frequency, FRF.PowerCross', Interp.Frequency, interpType, extrapType)';


if isfield(FRF, 'PowerInNull')
    Interp.PowerInNull = interp1(FRF.Spect.InNull.Frequency, FRF.PowerInNull', Interp.Frequency, interpType, extrapType)';
    Interp.PowerOutNull = interp1(FRF.Spect.OutNull.Frequency, FRF.PowerOutNull', Interp.Frequency, interpType, extrapType)';

    inDft = interpPolar(FRF.Spect.Input.Frequency, FRF.Spect.Input.DFT', Interp.Frequency, 'linear', 'extrap')';
    outNullDft = interpPolar(FRF.Spect.OutNull.Frequency, FRF.Spect.OutNull.DFT', Interp.Frequency, 'linear', 'extrap')';
    Interp.PowerCrossNull = 2*FRF.Spect.Input.Scale * inDft' .* conj(outNullDft);
end
end

function [vq] = interpPolar(x, v, xq, method, extrapolation)

rho = abs(v);
theta = angle(v);

rhoInterp = interp1(x, rho, xq, method, extrapolation);
thetaInterp = interp1(x, theta, xq, method, extrapolation);

vq = rhoInterp .* (cos(thetaInterp) + 1i * sin(thetaInterp));

end
