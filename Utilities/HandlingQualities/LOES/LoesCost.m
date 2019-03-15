function [cost] = LoesCost(freq_rps, gain_dB, phase_deg, typeTF, tfParamsValue, weightG, weightP, gain2_dB, phase2_deg)
% Calculates the cost function for the LOES fit.
%
%Usage:  [cost] = LoesCost(freq_rps, gain_dB, phase_deg, typeTF, tfParamsValue, weightG, weightP, gain2_dB, phase2_deg);
%
%Inputs:
% freq_rps      - frequency of frequency response (rad/sec)
% gain_dB       - magnitude of 1st input frequency response (dB)
% phase_deg     - phase of 1st input frequency response (deg)
% typeTF        - transfer function type; see: 'LoesTFparams'
% tfParamsValue - transfer function parameters
% weightG       - cost weight for gain error [1]
% weightP       - cost weight for phase error [0.0175]
% gain2_dB      - magnitude of 2nd input frequency response (dB) []
% phase2_deg    - phase of 2nd input frequency response (deg) []
%
%Outputs:
% cost - weighted cost of the LOES fit
%
%Notes:
% 
%
%Dependency:
% LoesFunc
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(5, 9, nargin, 'struct'))
if nargin < 9, phase2_deg = [];
    if nargin < 8, gain2_dB = []; end
    if nargin < 7, weightP = []; end
    if nargin < 6, weightG = []; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants
if isempty(weightP), weightP = 0.0175; end
if isempty(weightG), weightG = 1; end


%% Check Inputs
% Inputs as column vectors
freq_rps = freq_rps(:);
gain_dB = gain_dB(:);
phase_deg = phase_deg(:);
gain2_dB = gain2_dB(:);
phase2_deg = phase2_deg(:);


%% Calculate the frequency response of the transfer function and its cost
switch typeTF
    case {3, 4, 5}
        % Call the function
        [gainLoes_dB, phaseLoes_deg, gain2Loes_dB, phase2Loes_deg] = LoesFunc(typeTF, tfParamsValue, freq_rps);

        % Calculate the weighted cost function
        cost1 = weightG*sum((gain_dB - gainLoes_dB).^2) + weightP*sum((phase_deg - phaseLoes_deg).^2);
        cost2 = weightG*sum((gain2_dB - gain2Loes_dB).^2) + weightP*sum((phase2_deg - phase2Loes_deg).^2);
        cost = (cost1 + cost2)/2;

    otherwise
        % Call the function
        [gainLoes_dB, phaseLoes_deg] = LoesFunc(typeTF, tfParamsValue, freq_rps);

        % Calculate the weighted cost function
        cost = weightG*sum((gain_dB - gainLoes_dB).^2) + weightP*sum((phase_deg - phaseLoes_deg).^2);

end

% Scale output to be comparable for length
% and compatible with LaManna's early runs
costNorm = 20*(cost/length(freq_rps));


%% Check Outputs
