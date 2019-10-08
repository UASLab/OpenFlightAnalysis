function [dft, freq] = FFT(x, freqRate)
% Compute the DFT of time history data using FFT.

[~, lenX] = size(x);

% The output will be half the length of the input, input should be even length
lenDft = 2*floor(lenX/2);

% Compute FFT of x
dft  = fft(xWin, lenDft, 2);

% Form Frequency Vector
freq = (freqRate/lenDft) * (1:length(dft));

end