function I_B_kgm2 = MassPropShift(rCG_B_m, mass_kg, IRef_B_kgm2)
% OpenFlightAnalysis - MassPropShift
%  Compute the Inertia due to a CG change.
%
% Inputs:
%  rCG_B_m   - Mass CG shift distance
%  mass_kg   - mass
%  IRef_B_kgm2    - Reference Inertia
%
% Outputs:
%  I_B_kgm2     - New Inertia Tensor
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details

% Author: Chris Regan

%% Check I/O Arguments
% Check the number of inputs
narginchk(1, 3);
if nargin < 3, IRef_B = [];
    if nargin < 2, mass_kg = []; end
end

% Default Values
if isempty(mass_kg), mass_kg = 1.0; end
if isempty(IRef_B_kgm2), IRef_B_kgm2 = zeros(3); end

% Check the number of outputs
nargoutchk(0, 1);

% Calculate the New Inertia % I_CG = I_Ref + m(dot(r, r)*I + (r*r')), where r is from Ref to CG
I_B_kgm2 = IRef_B_kgm2 + mass_kg * (dot(rCG_B_m, rCG_B_m)*eye(3) + rCG_B_m * rCG_B_m');

