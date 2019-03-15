function [vEst_BA_B_mps, vEst_BA_L_mps] = PitotCal(pDiffMeas_Pa, s_BL_rad, param)
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
narginchk(3, 3);

% Default Values
    
% Check the number of outputs
nargoutchk(0, 2);


%% Constants


%% Check Inputs
[~, numSamp] = size(pDiffMeas_Pa);


%% Apply Error Models, Estimate the "true" measures from the Error Models
% Apply Pitot-Static Error Models
[pDiff_Pa] = SensorErrorModel(pDiffMeas_Pa, param.pPitotDiff_Pa);


%% Transform Pitot Measures
% Assume the measured airspeed is in the Probe X-direction
vMag_PA_mps = sqrt(2/1.225 * abs(pDiff_Pa));
v_PA_P_mps = [vMag_PA_mps; zeros(2, length(vMag_PA_mps))];

% Assume the rotation rate of the atmosphere is negligible
w_AL_L_rps = zeros(3, length(vMag_PA_mps));

% Compute the Rotation rate of the Probe wrt the Atm
% w_BA_B_rps = w_BL_B_rps + T_BL * w_AL_L_rps;
% w_BA_B_rps = w_BL_B_rps + w_AL_L_rps; % FIXIT - should have transformation from L to B
w_BA_B_rps = zeros(3, length(vMag_PA_mps));

% Translate and Rotate the Pitot measurement to the Body frame
[vEst_BA_B_mps] = TransPitot(v_PA_P_mps, w_BA_B_rps, param.s_PB_rad.data, param.r_PB_B_m.data);

% Transform Coordinates from B to L
vEst_BA_L_mps = NaN(3, numSamp);
for indx = 1:numSamp

    % Compute the NED velocity 
    T_L2B = RotMat(param.s_BL_rad.seq, s_BL_rad(:,indx), 'rad');
    T_B2L = T_L2B';
    
    vEst_BA_L_mps(:,indx) = T_B2L * vEst_BA_B_mps(:,indx);
end
