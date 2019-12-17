function [FRF, FRF_MIMO] = FreqRespEst(x, y, FRF)
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
%
%Outputs:
% FRF
%   Spect
%     Input
%     Output
%   FRD
%     Frequency    - frequency of transfer function (see Note)
%     ResponseData - complex transfer function
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
        
        FRF{iFrf} = FreqRespEst(x(iFrf,:), y, FRF{iFrf});
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
    end
    
end


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
        
        for iIn = 1:numIn
            FRF_MIMO.PowerIn(1, iIn, :) = FRF{iIn}.Interp.PowerIn;
            FRF_MIMO.PowerOut(:, iIn, :) = FRF{iIn}.Interp.PowerOut;
            FRF_MIMO.PowerCross(:, iIn, :) = FRF{iIn}.Interp.PowerCross;
            FRF_MIMO.Coherence(:, iIn, :) = FRF{iIn}.Interp.Coherence;
        end
        
        FRF_MIMO.FRD = frd(FreqRespEstCmplx(FRF_MIMO.PowerIn, FRF_MIMO.PowerCross), FRF_MIMO.Frequency, FRF{1}.Opt.Ts);
        
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
    Interp.PowerCross = interp1(FRF.FRD.Frequency, FRF.PowerCross', Interp.Frequency, interpType, extrapType)';

    T = FreqRespEstCmplx(Interp.PowerIn, Interp.PowerCross);
    T = reshape(T, [], 1, length(Interp.Frequency)); % Shift dims so that the FRF is Ny X Nu X Nf
    Interp.FRD = frd(T, Interp.Frequency, FRF.Opt.Ts);

    Interp.Coherence = Coherence(Interp.PowerCross, Interp.PowerIn, Interp.PowerOut);
end
