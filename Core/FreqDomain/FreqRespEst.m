function [frf] = FreqRespEst(x, y, optFrf)
% Estimate the transfer function (y/x) between two time histories.
%
%Usage:  [frf] = FreqRespEst(x, y, optFrf);
%
%Inputs:
% x            - assumed input time history
% y            - assumed output time history
% optFrf
%   optSpect
%     freqRate     - sample rate (see Note)
%     freq         - frequencies of interest (see Note)
%     optWin       - Window Options
%     optSmooth    - Smoothing Options
%
%Outputs:
% frf
%   freq      - frequency of transfer function (see Note)
%   T       - complex transfer function
%   cohor       - magnitude squared coherence
%   crossP       - cross-spectral Power of the ouput/input
%   inP       - input Power
%   outP       - output Power
%   inSpect
%   outSpect
%
%Notes:
% 'freq' and 'freqRate' must have the same units; 'freq' will have the
% units of the frequency inputs.
% The response can be optionally limited to a specified frequency range
% and/or coherence threshold.
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
    optFrf = struct();
end

nargoutchk(0, 1);

%% Default Values and Constants
if ~isfield(optFrf, 'dftType'), optFrf.dftType = []; end
if ~isfield(optFrf, 'freqRate'), optFrf.freqRate = []; end
if ~isfield(optFrf, 'scaleType'), optFrf.scaleType = []; end
if ~isfield(optFrf, 'freqE'), optFrf.freqE = []; end

if ~isfield(optFrf, 'optWin'), optFrf.optWin = struct(); end

if isempty(optFrf.freqRate), optFrf.freqRate = 1; end
if isempty(optFrf.scaleType), optFrf.scaleType = 'spectrum'; end


%% Check Inputs


%% Approximate the frequency response function
if iscell(optFrf.freq) % SIMO at Multiple sets of frequencies
    numFrf = length(optFrf.freq);
    optFrfSet = optFrf;
    
    frf = cell(size(optFrf.freq));
    for iFrf = 1:numFrf
        
        frf{iFrf}.freq = optFrf.freq{iFrf};
        frf{iFrf}.inSig = x(iFrf,:);
        frf{iFrf}.outSig = y;
        
        optFrfSet.freq = frf{iFrf}.freq;
        frf{iFrf} = FreqRespEst(x(iFrf,:), y, optFrfSet);
    
        % Merge Frf at different frequencies
        % Interpolate or fit then interpolate to get a common frequency basis
        if ~isempty(optFrf.freqE)
            interpType = 'linear'; % 'linear', 'nearest', 'next', 'previous', 'pchip', 'cubic', 'v5cubic', 'makima', or 'spline'
            extrapType = 'extrap'; % 'extrap', nan
            
            frf{iFrf}.inP = interp1(frf{iFrf}.freq, frf{iFrf}.inP', optFrf.freqE, interpType, extrapType)';
            frf{iFrf}.outP = interp1(frf{iFrf}.freq, frf{iFrf}.outP', optFrf.freqE, interpType, extrapType)';
            frf{iFrf}.crossP = interp1(frf{iFrf}.freq, frf{iFrf}.crossP', optFrf.freqE, interpType, extrapType)';

            frf{iFrf}.T = FreqRespEstCmplx(frf{iFrf}.inP, frf{iFrf}.crossP);
            frf{iFrf}.coher = Coherence(frf{iFrf}.crossP, frf{iFrf}.inP, frf{iFrf}.outP);
            
            frf{iFrf}.freq = optFrf.freqE;
        end
    end

else % SIMO at one frequency set

    % Add x and y to frf structure
    frf.freq = optFrf.freq;
    frf.inSig = x;
    frf.outSig = y;
        
    % Compute the DFT and Spectrum for x and y
    [frf.inSpect] = SpectEst(frf.inSig, optFrf);
    [frf.outSpect] = SpectEst(frf.outSig, optFrf);
    
    % Add x and y specturm to frf structure
    frf.inP = frf.inSpect.P;
    frf.outP = frf.outSpect.P;
    
    % Compute cross spectrum, Scale is doubled because one-sided DFTs
    frf.crossP = 2*frf.inSpect.scale * conj(frf.inSpect.dft) .* frf.outSpect.dft;

    %% Compute complex transfer function approximation
    frf.T = FreqRespEstCmplx(frf.inP, frf.crossP);
    
    % Smooth the PSDs
    if isfield(optFrf, 'optSmooth')
        frf.crossPRaw = frf.crossP;
        frf.crossP = SmoothFunc(frf.crossPRaw, optFrf.optSmooth);
    end
    
    % Compute magnitude squared coherence
    frf.coher = Coherence(frf.crossP, frf.inP, frf.outP);

end
