function [out1, out2] = FunctionTemplate(in1, in2, in3)
% OpenFlightAnalysis - FunctionTemplate
%   Description blah blah blah....
%
% Inputs:
%   in1 -
%   in2 -
%   in3 -
%
% Outputs:
%   out1 -
%   out2 -
%
% Notes:
%
% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
%
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(1, 3)
if nargin < 2, in2 = [];
    if nargin < 3, in3 = []; end
end

nargoutchk(1, 2)

%% Default Values and Constants
if isempty(in2), in2 = 2; end
if isempty(in3), in3 = 3; end

%% Check Inputs


%% Constants

%%
out1 = in1;
out2 = in2;


%% Check Outputs


end
