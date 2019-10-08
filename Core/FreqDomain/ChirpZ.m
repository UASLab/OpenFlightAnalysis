function [dft, freq] = ChirpZ(x, freqRate, freqVec)
% Compute the DFT of time history data with Chirp-Z.

% Min and Max Frequencies
freqMin = min(freqVec);
freqMax = max(freqVec);

% Number of frequency points
mChirp = length(freqVec);
freqStep = (freqMax - freqMin) / (mChirp - 1);
freq = (0 : mChirp-1) * freqStep + freqMin;

% Ratio between points
wChirp = exp(-1i * (2*pi / (mChirp - 1)) * (freqMax - freqMin) / freqRate);

% Starting point
aChirp = exp(1i * 2*pi * freqMin / freqRate);

% ChirpZ
dft = czt(x', mChirp, wChirp, aChirp)';
