function [level, level1Bound, level2Bound, level3Bound] = BandwidthCrit(freqBW_rps, tDelay_s, category)
% Bandwidth criterion rating level and boundaries.
%
%Usage:  [level, level1Bound, level2Bound, level3Bound] = BandwidthCrit(freqBW_rps, tDelay_s, category);
%
%Inputs: 
% freqBW_rps - bandwidth of 'theta/dep' transfer function (rad/sec)
% tDelay_s   - estimated equivalent time delay (sec)
% category   - flight phase category designation
%
%Outputs:
% level       - level rating
% level1Bound - level 1 bound
% level2Bound - level 2 bound
% level3Bound - level 3 bound
%
%Notes:
% 
%
%Reference:
% MIL-STD-1797A, Appendix A, Paragraph 4.2.1.2, Section D
% "Bandwidth - A Criterion For Highly Augmented Aircraft" Hoh and Hodgkinson
% AGARD-CP-333
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release, based on Pat Stoliker's routines(v1.0)
%


%% Check I/O Arguments
error(nargchk(3, 3, nargin, 'struct'))
error(nargoutchk(1, 4, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs
% Inputs as column vectors
freqBW_rps = freqBW_rps(:);
tDelay_s = tDelay_s(:);


%% Level boudaries, (x,y) coordinates, counterclockwise loop
% TODO: Check these values
switch category
    case 'A' % CategoryA
        % Level 1 boundaries
        level1Bound(:,1) = [11.2, 8.8, 8.16, 7.6, 7.08, 6.8, 6.6, 6.54, 6.5, 6.5];
        level1Bound(:,2) = [0.0, 0.06, 0.055, 0.05, 0.045, 0.04, 0.035, 0.03, 0.02, 0.0];

        % Level 2 boundaries
        level2Bound(:,1) = [3.4, 2.6, 2.08, 1.76, 1.40, 1.16, 1.0, 1.0];
        level2Bound(:,2) = [0.15, 0.14, 0.13, 0.12, 0.11, 0.10, 0.085, 0.0];

        % Level 3 boundaries
        level3Bound(:,1) = NaN;
        level3Bound(:,2) = NaN;

    otherwise
        warning('Unkown category'); % FIXME: warning ID missing
end


%% Deterine the level achieved
% FIXME: Zeroed for lack of a real solution
level = zeros(length(freqBW_rps), 1);


%% Check Outputs
