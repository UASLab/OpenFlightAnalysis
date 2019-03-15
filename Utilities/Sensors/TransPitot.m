function [v_BA_B_mps] = TransPitot(v_PA_P_mps, w_BA_B_rps, s_PB_rad, r_PB_B_m)
% Transform Pitot Measurements from Probe location to Body.
%
% %Inputs:
% v_PA_P_mps - Velocity of the Probe wrt Atm in Probe Coordinates [P/A]P (m/s2)
% w_BA_B_rps - Rotation rate of the Body Frame wrt Atm in Body Coordinates [B/A]B (rad/s)
% s_PB_rad   - Orientation (321) of the Probe Frame wrt Body Frame [P/B] (rad)
% r_PB_B_m   - Position of the Probe Frame wrt Body Frame in Body Coordinates [P/B]B (m)
%
%Outputs:
% v_BA_B_mps - Velocity of the Body wrt Atm in Body Coordinates [B/A]B (m/s2)
% v_BA_L_mps - Velocity of the Body wrt Atm in Local Level Coordinates [B/A]L (m/s2)
%
%Notes:
%

%Version History: Version 1.2
% 09/20/2016  C. Regan     Initial Release (v1.0)
% 10/17/2016  C. Regan     Changed to simple and faster input parsing (v1.1)
% 10/25/2016  C. Regan     Rempved transformation from B to L (v1.2)
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(4, 4);

% Default Values
    
% Check the number of outputs
nargoutchk(0, 1);


%% Constants


%% Check Inputs
[~, numSamp] = size(v_PA_P_mps);

% Parameterize transformation from P to B
sSeq_PB_rad = '321';
T_B2P = RotMat(sSeq_PB_rad, s_PB_rad, 'rad');
T_P2B = T_B2P';

v_PB_B_mps = [0; 0; 0]; % Velocity of the Probe wrt the Body Frame [P/B]B (m/s)
v_BP_B_mps = -v_PB_B_mps;

r_BP_B_m = -r_PB_B_m;

%%
v_BA_B_mps = NaN(3, numSamp);
for indx = 1:numSamp
    % Transform from P to B
    v_PA_B_mps = T_P2B * v_PA_P_mps(:,indx);
    v_BA_B_mps(:,indx) = RelVel(v_PA_B_mps, w_BA_B_rps(:,indx), r_BP_B_m, v_BP_B_mps);
end


%% Outputs


