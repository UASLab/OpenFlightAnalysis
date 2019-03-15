function [signal, ampl, freq_rps] = FreqSweep(freqInit_rps, freqFinal_rps, time_s, amplInit, amplFinal, freqType, amplType, forceZeroSW)
% Generate a frequency sweep time history.
%
%Usage:  [signal, ampl, freq_rps] = FreqSweep(freqInit_rps, freqFinal_rps, time_s, amplInit, amplFinal, freqType, amplType, forceZeroSW);
%
%Inputs:
% freqInit_rps  - initial frequency (rad/sec)
% freqFinal_rps - final frequency   (rad/sec)
% time_s        - time history vector (sec)
% amplInit      - initial amplitude [1]
% amplFinal     - final amplitude [1]
% freqType      - type of frequency variation with time ['linear']
% amplType      - type of amplitude variation with time ['linear']
% forceZeroSW   - switch to force final zero []
%
%Outputs:
% signal   - time history of frequency sweep
% ampl     - amplitude time history of output signal
% freq_rps - frequency time history of output signal (rad/s)
%
%Notes:
% The sequence holds zero after the last zero crossing.
% amplType and freqType can be either 'linear', {'log', 'log10'}, or 'ln'
% Also see: Signal Processing Toolbox function 'chirp'
%

%Version History: Version 1.2
% 03/07/2006  C. Regan     Initial Release (v1.0)
% 06/08/2006  C. Regan     Fixed frequency distributions and added 'ln' types (v1.1)
% 11/07/2006  C. Regan     Added I/O argument checking (v1.2)
%


%% Check I/O Arguments
error(nargchk(3, 8, nargin, 'struct'))
if nargin < 8, forceZeroSW = [];
    if nargin < 7, amplType = []; end
    if nargin < 6, freqType = []; end
    if nargin < 5, amplFinal = []; end
    if nargin < 4, amplInit = []; end
end

error(nargoutchk(0, 3, nargout, 'struct'))


%% Default Values and Constants
if isempty(amplInit), amplInit = 1; end
if isempty(amplFinal), amplFinal = 1; end
if isempty(freqType), freqType = 'linear'; end
if isempty(amplType), amplType = 'linear'; end

% Constants
hz2rps = 2*pi;


%% Check Inputs
timeRate_s = time_s(2) - time_s(1);
freqMaxLimit_rps = (1/(2*timeRate_s) * hz2rps);

if freqInit_rps > freqMaxLimit_rps
    warning([mfilename ' - The initial desired frequency is too high for the frame rate']);
    freqInit_rps = freqMaxLimit_rps;
end
if freqFinal_rps > freqMaxLimit_rps
    warning([mfilename ' - The final desired frequency is too high for the frame rate']);
    freqFinal_rps = freqMaxLimit_rps;
end


%% Phase variation and Frequency variation
% Number of time samples
numSamples = length(time_s);

% End time
timeFinal_s = time_s(end);

switch freqType
    case 'linear'
        freq_rps = ((freqFinal_rps-freqInit_rps)/(2*timeFinal_s) * time_s) + freqInit_rps;
        phase_rad = freq_rps .* time_s;
        
    case {'log', 'log10'}
        phase_rad = ((timeFinal_s*freqInit_rps)/log10(freqFinal_rps/freqInit_rps)) * ((freqFinal_rps/freqInit_rps).^(time_s/timeFinal_s) - 1);
        freq_rps = phase_rad ./ time_s;
                
    case 'ln'
        phase_rad = ((timeFinal_s*freqInit_rps)/log(freqFinal_rps/freqInit_rps)) * ((freqFinal_rps/freqInit_rps).^(time_s/timeFinal_s) - 1);
        freq_rps = phase_rad ./ time_s;
        
    otherwise
        warning([mfilename '- Unkown frequency sweep type']);
end


%% Amplitude variation
switch amplType
    case 'linear'
        %ampl = linspace(amplInit, amplFinal, numSamples);
        ampl = amplInit + (0:numSamples-1)*((amplFinal - amplInit)/(numSamples - 1));

    case {'log', 'log10', 'ln'}
        %ampl = logspace(log10(amplInit), log10(amplFinal), numSamples);
        ampl = 10.^(log10(amplInit) + (0:numSamples-1)*(log10(amplFinal)-log10(amplInit))/(numSamples-1));

    otherwise
        warning([mfilename '- Unkown amplitude sweep type']);
end


%% Generate frequency sweep time history
signal = ampl .* sin(phase_rad);


%% Ensure a zero final value
if forceZeroSW
    % Stop sweep at last zero crossing and hold zero
    if signal(end) > 0
        indxCross = max(find(signal <= 0 ));
    else
        indxCross = max(find(signal > 0 ));
    end
    signal(indxCross + 1 : end) = 0;
end


%% Check Outputs
