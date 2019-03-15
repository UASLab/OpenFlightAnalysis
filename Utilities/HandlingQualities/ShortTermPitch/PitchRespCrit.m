function [thetaLag_rad, level, level1Bound, level2Bound, level3Bound] = PitchRespCrit(freqSP_rps, dampSP, tTheta2_s, category)
% Pitch response criterion level rating and boundaries.
%
%Usage:  [thetaLag_rad, level, level1Bound, level2Bound, level3Bound] = PitchRespCrit(freqSP_rps, dampSP, tTheta2_s, category);
%
%Inputs:
% freqSP_rps - short-period frequency (rad/sec)
% dampSP     - short-period damping
% tTheta2_s  - time constant for the lag between flight path and pitch attitude response (sec)
% category   - flight phase category designation
%
%Outputs:
% thetaLag_rad - lag between flight path and pitch attitude response (rad)
% level        - level rating
% level1Bound  - level 1 bound
% level2Bound  - level 2 bound
% level3Bound  - level 3 bound
%
%Notes:
% 
%
%Reference:
% MIL-STD-1797A, Appendix A, Paragraph 4.2.1.2, Section A
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(4, 4, nargin, 'struct'))
error(nargoutchk(2, 5, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs
% Inputs as column vectors
freqSP_rps = freqSP_rps(:);
dampSP = dampSP(:);
tTheta2_s = tTheta2_s(:);


%% Calculate the yaxis value (wsp * tTheta2)
thetaLag_rad = freqSP_rps .* tTheta2_s;


%% Level boudaries, (x,y) coordinates, counterclockwise loop
% TODO: Check these values
switch category
    case 'A' % CategoryA
        % Level 1 boundaries
        level1Bound(:,1) = [1.3, 1.3, 0.35, 0.35];
        level1Bound(:,2) = [10, 1.5, 1.5, 10];

        % Level 2 boundaries
        level2Bound(:,1) = [2, 2, 0.25, 0.25];
        level2Bound(:,2) = [10, 1, 1, 10];

        % Level 3 boundaries
        level3Bound(:,1) = [5, 5, 0.1, 0.1];
        level3Bound(:,2) = [10, 0.1, 0.1, 10];

    case 'B' % CategoryB
        % Level 1 boundaries
        level1Bound(:,1) = [2, 2, 0.3, 0.3];
        level1Bound(:,2) = [10, 1, 1, 10];

        % Level 2 boundaries
        level2Bound(:,1) = [2, 2, 0.2, 0.2];
        level2Bound(:,2) = [10, 0.6, 0.6, 10];

        % Level 3 boundaries
        level3Bound(:,1) = [5, 5, 0.1, 0.1];
        level3Bound(:,2) = [10, 0.1, 0.1, 10];

    case 'C' % CategoryC
        % Level 1 boundaries
        level1Bound(:,1) = [1.3, 1.3, 0.35, 0.35];
        level1Bound(:,2) = [10, 1.25, 1.25, 10];

        % Level 2 boundaries
        level2Bound(:,1) = [2, 2, 0.4, 0.25, 0.25];
        level2Bound(:,2) = [10, 0.7, 0.7, 1.1, 10];

        % Level 3 boundaries
        level3Bound(:,1) = [5, 5, 0.1, 0.1];
        level3Bound(:,2) = [10, 0.1, 0.1, 10];

    otherwise
        warning('Unkown category'); % FIXME: warning ID missing
end


%% Deterine the level achieved
% FIXME: This could be improved to be more general, and readable
level = zeros(length(thetaLag_rad),1);

% Test for Level 3
level(dampSP > min(level3Bound(:,1))) = 3;

% Test for Level 2
level(...
    (dampSP > min(level2Bound(:,1))) & (dampSP < max(level2Bound(:,1))) & ...
    (thetaLag_rad > min(level2Bound(:,2))) & (thetaLag_rad < max(level2Bound(:,2)))...
    ) = 2;

% Test for Level 1
level(...
    (dampSP > min(level1Bound(:,1))) & (dampSP < max(level1Bound(:,1))) & ...
    (thetaLag_rad > min(level1Bound(:,2))) & (thetaLag_rad < max(level1Bound(:,2)))...
    ) = 1;


%% Check Outputs
