function [out1, out2] = FunctionTemplate(in1, in2, in3)
% Description of Function
%
% The following are valid function calls
% [out1] = FunctionTemplate(in1);
% [~, out2] = FunctionTemplate(in1, in2, in3);
% [out1, out2] = FunctionTemplate(in1, [], in3);
%
%Inputs:
% in1    - Description of required input
% in2    - Description of required input [default]
% in3    - Description of optional input [default]
%
%Outputs:
% out1 - Description of Output1
% out2 - Description of Output2
%
%Notes:
% Any Special Notes.
%

%Version History: Version 1.0
% mm/dd/yyyy  Name     Initial Release (v1.0)
%


%% Check I/O Arguments
% Check Inputs, create if absent
narginchk(1, 3);
if nargin < 3, in3 = [];
    if nargin < 2, in2 = []; end
end

% Check Outputs
nargoutchk(0, 2);


%% Default Values and Constants
if isempty(in3), in3 = 1; end
if isempty(in2), in2 = 2; end

r2d = 180/pi;

%% Computation
out1 = in1;
out2 = in2 + in3;


%% Check Outputs

