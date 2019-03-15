function [vEst_BA_B_mps, vEst_BA_L_mps] = Press5HoleCal(p5TipMeas_Pa, p5AlphaMeas_Pa, p5BetaMeas_Pa, s_BL_rad, param)
% Compute the Pitot Respose with a given sensor error model.
%
% %Inputs:
% pDiffMeas_Pa    - Magnitude of the Differential Pressure of the Probe wrt Atm [P/A] (Pa)
% w_BA_B_rps      - Rotation rate of the Body Frame wrt Atm in Body Coordinates [B/A]B (rad/s)
% s_BL_rad        - Orientation (321) of the Body Frame wrt Local Level Frame [B/L] (rad)
% param           - Structure
%  (pPitotDiff_Pa)- Pitot Probe Error Model Parameters
%  (lenSeg)       - Length of current segment
%  (s_BL_rad.seq) - Orientation Sequence for the L to B rotation
%  (s_PB_rad.data)- Orientation (321) of the Probe Frame wrt Body Frame [P/B] (rad)
%  (r_PB_B_m.data)     - Position of the Probe Frame wrt Body Frame in Body Coordinates [P/B]B (m)
%
%Outputs:
% vEst_BA_B_mps - Velocity of the Body wrt Atm in Body Coordinates [B/A]B (m/s2)
% vEst_BA_L_mps - Velocity of the Body wrt Atm in Local Level Coordinates [B/A]L (m/s2)
%
%Notes:
%

%Version History: Version 1.0
% 09/20/2016  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(5, 5);

% Default Values

% Check the number of outputs
nargoutchk(0, 2);


%% Constants
r2d = 180/pi;


%% Check Inputs
[~, numSamp] = size(p5TipMeas_Pa);


%% Apply Error Models, Estimate the "true" measures from the Error Models
% Apply Pitot-Static Error Models
[p5Tip_Pa] = SensorErrorModel(p5TipMeas_Pa, param.p5Tip_Pa);
[p5Alpha_deg] = SensorErrorModel(p5AlphaMeas_Pa, param.p5Alpha_Pa);
[p5Beta_deg] = SensorErrorModel(p5BetaMeas_Pa, param.p5Beta_Pa);


%% Transform Pitot Measures
%
vMag_PA_mps = sqrt(2/1.225 * abs(p5Tip_Pa));

% FIXIT
v_PA_P_mps = NaN(3, size(vMag_PA_mps, 2));
% v_PA_P_mps(1,:) = vMag_PA_mps ./ (cosd(p5Alpha_deg) .* cosd(p5Beta_deg));
v_PA_P_mps(1,:) = vMag_PA_mps;
v_PA_P_mps(2,:) = vMag_PA_mps .* sind(p5Beta_deg);
v_PA_P_mps(3,:) = v_PA_P_mps(1,:) .* tand(p5Alpha_deg);

% Assume the rotation rate of the atmosphere is negligible
w_AL_L_rps = zeros(3, length(vMag_PA_mps));

% Compute the Rotation rate of the Probe wrt the Atm
% w_BA_B_rps = w_BL_B_rps + T_BL * w_AL_L_rps;
% w_BA_B_rps = w_BL_B_rps + w_AL_L_rps; % FIXIT - should have transformation from L to B
w_BA_B_rps = zeros(3, length(vMag_PA_mps));

% Translate and Rotate the Pitot measurement to the Body frame
[vEst_BA_B_mps] = TransPitot(v_PA_P_mps, w_BA_B_rps, param.s_5B_rad.data, param.r_PB_B_m.data);

% Transform Coordinates from B to L
vEst_BA_L_mps = NaN(3, numSamp);
for indx = 1:numSamp
    
    % Compute the NED velocity
    T_L2B = RotMat(param.s_BL_rad.seq, s_BL_rad(:,indx), 'rad');
    T_B2L = T_L2B';
    
    vEst_BA_L_mps(:,indx) = T_B2L * vEst_BA_B_mps(:,indx);
end
