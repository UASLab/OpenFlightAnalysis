function [level, level1Bound, level2Bound, level3Bound] = ShortPeriodCrit(nzAlpha_gpr, freqSP_rps, category)
% Short period criterion rating and boundaries.
%
%Usage:  [level, level1Bound, level2Bound, level3Bound] = ShortPeriodCrit(nzAlpha_gpr, freqSP_rps, category)
%
%Inputs:
% nzAlpha_gpr - normal load per angle of attack (g/rad)
% freqSP_rps  - short-period frequency (rad/sec)
% category    - flight phase category designation
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
% MIL-STD-1797A, Appendix A, Paragraph 4.2.1.2, Section A
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(3, 3, nargin, 'struct'))
error(nargoutchk(1, 4, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs
% Inputs as column vectors
nzAlpha_gpr = nzAlpha_gpr(:);
freqSP_rps = freqSP_rps(:);


%% Level boudaries, (x,y) coordinates, counterclockwise loop
% TODO: Check these values
switch category
    case 'A' % CategoryA
        % Level 1 boundaries
        level1Bound(:,1) = [100.0  , 0.278, 3.5714, 100.0];
        level1Bound(:,2) = [18.9737, 1.00 , 1.00  , 5.2915];
        
        % Level 2 boundaries
        level2Bound(:,1) = [100.0  , 0.10, NaN, 0.10, 2.25, 100.0];
        level2Bound(:,2) = [31.6228, 1.00, NaN, 0.60, 0.60, 4.00];
        
        % Level 3 boundaries
        level3Bound(:,1) = [100.0, 0.10];
        level3Bound(:,2) = [4.00,  0.126];
        
    otherwise
        warning('Unkown category'); % FIXME: warning ID missing
end


%% Deterine the level achieved
% FIXME: Zeroed for lack of a real solution
level = zeros(length(nzAlpha_gpr),1);


%% Check Outputs
