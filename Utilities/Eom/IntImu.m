function [s_BL_rad, v_BE_L_mps, r_BE_L_m, w_BL_B_rps, a_BE_B_mps2] = IntImu(time_s, w_ImuI_Imu_rps, a_ImuI_Imu_mps2, sInit_BL_rad, vInit_BE_L_mps, rInit_BE_L_m, r_ImB_B_m, s_ImB_rad)
% Compute the Euler Angles and NED Velocities from IMU signals, assumes flat Earth
%
%Inputs:
% time_s
% w_ImuI_Imu_rps
% a_ImuI_Imu_mps2
% sInit_BL_rad
% vInit_BE_L_mps
% rInit_BL_L_m
% r_ImB_B_m          - Location of the IMU package wrt the Body Frame [Imu/B]B (m)
% s_ImB_rad          - Orientation of the IMU package wrt Body Frame [Imu/B] (rad)
%
%Outputs:
% s_BL_rad   - 
% v_BE_L_mps - 
% r_BE_L_m - 
% w_BL_B_rps - 
% a_BL_B_mps2 - 
%
%Notes:
%

%Version History: Version 1.1
% 10/03/2016  C. Regan     Initial Release (v1.0)
% 10/17/2016  C. Regan     Changed to simple and faster input parsing (v1.1)
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(3, 8);
if nargin < 8, s_ImB_rad = [];
    if nargin < 7, r_ImB_B_m = []; end
    if nargin < 6, rInit_BE_L_m = []; end
    if nargin < 5, vInit_BE_L_mps = []; end
    if nargin < 4, sInit_BL_rad = []; end
end

% Default Values
if isempty(sInit_BL_rad), sInit_BL_rad = [0; 0; 0]; end
if isempty(vInit_BE_L_mps), vInit_BE_L_mps = [0; 0; 0]; end
if isempty(rInit_BE_L_m), rInit_BE_L_m = [0; 0; 0]; end
if isempty(r_ImB_B_m), r_ImB_B_m = [0; 0; 0]; end
if isempty(s_ImB_rad), s_ImB_rad = [0; 0; 0]; end

% Check the number of outputs
nargoutchk(0, 5);


%% Constants
accelGrav_mps2 = 9.807;


%% Check Inputs
[~, numSamp] = size(w_ImuI_Imu_rps);

sSeq_ImB_rad = '321'; % FIXIT - should be input
sSeq_BL_rad = '321'; % FIXIT - should be input


%%
timeStep_s = [NaN, diff(time_s)];

v_ImB_B_mps = [0; 0; 0]; % Velocity of the IMU package wrt the Body Frame [Imu/B]B (m/s)
a_ImB_B_mps2 = [0; 0; 0]; % Acceleration of the IMU package wrt the Body Frame [Imu/B]B (m/s^2)

% Imu to Body Coordinate Transformation

T_B2Im = RotMat(sSeq_ImB_rad, s_ImB_rad, 'rad'); % [T]Imu/B
T_Im2B = T_B2Im';

% Transformation from I frame to L frame
aGrav_BI_L_mps2 = [0; 0; -1] * accelGrav_mps2; % L frame Acceleration wrt I frame, for flat earth this is simply gravity
w_LI_I_rps = [0; 0; 0]; % L frame rotation rate wrt I frame, this is usually negligible - FIXIT
T_LI = eye(3); % Transformation from I frame to L frame, FIXIT - Use Position and RotMat, could be added to the loop

% Pre-Allocate Arrays and Initialize
w_BL_B_rps = NaN(3, numSamp);
a_BE_B_mps2 = NaN(3, numSamp);
s_BL_rad = NaN(3, numSamp); s_BL_rad(:,1) = sInit_BL_rad;
v_BE_L_mps = NaN(3, numSamp); v_BE_L_mps(:, 1) = vInit_BE_L_mps;
r_BE_L_m = NaN(3, numSamp); r_BE_L_m(:, 1) = rInit_BE_L_m;

T_L2B = NaN(3, 3, numSamp); T_L2B(:,:,1) = RotMat(sSeq_BL_rad, s_BL_rad(:,1), 'rad');
T_B2L = NaN(3, 3, numSamp); T_B2L(:,:,1) = T_L2B(:,:,1)';

for indx = 2:numSamp
    % Transform Imu rates to Body rates, note it is treated as a "free vector" on a rigid body
    w_BI_Imu_rps = w_ImuI_Imu_rps(:,indx);
    
    % Change Coordinates from Imu to Body
    w_BI_B_rps = T_Im2B * w_BI_Imu_rps;
    
    % Reference transformation from Inertial to L
    w_LI_B_rps = T_B2L(:,:,indx-1) * T_LI * w_LI_I_rps;
    w_BL_B_rps(:,indx) = w_BI_B_rps - w_LI_B_rps;
    
    % Derivative of Gyro
    wTrueDot_ImuI_Imu_rps = diff(w_ImuI_Imu_rps(:, indx-1:indx), [], 2) / timeStep_s(indx);
    
    % Relative Acceleration of the IMU frame wrt Body frame
    a_BI_Imu_mps2 = RelAccel(a_ImuI_Imu_mps2(:,indx), w_ImuI_Imu_rps(:,indx), wTrueDot_ImuI_Imu_rps, r_ImB_B_m, a_ImB_B_mps2, v_ImB_B_mps);
    
    % Change coordinates from Imu to B
    a_BI_B_mps2 = T_Im2B * a_BI_Imu_mps2;

    %% Integrate the Gyro to Compute the Euler Angles
    % Transformation Matrix from Euler Rates to Body Rates
    T_BodyEuler = RotRateMat(sSeq_BL_rad, s_BL_rad(:,indx-1), 'rad');
    
    % Rotational Kinematics, Transform Body Rates to Euler Rates
    sDot_BI_rad = T_BodyEuler \ w_BL_B_rps(:, indx); 
    
    % Integrate Euler Rates to Orientation (321) of Body wrt L
    s_BL_rad(:,indx) = s_BL_rad(:,indx-1) + sDot_BI_rad * timeStep_s(indx);

    % Fix Heading angle wrapping
    if s_BL_rad(3,indx) > pi
        s_BL_rad(3,indx) = s_BL_rad(3,indx) - 2*pi;
    elseif s_BL_rad(3,indx) < -pi
        s_BL_rad(3,indx) = s_BL_rad(3,indx) + 2*pi;
    end
    
    %% Compute Velocity, Integration in the Inertial frame (L frame is good for flat earth - FIXIT)
    % Rotation Matrix
    T_L2B(:,:,indx) = RotMat(sSeq_BL_rad, s_BL_rad(:,indx), 'rad');
    T_B2L(:,:,indx) = T_L2B(:,:,indx)';
        
    % Transfer from I to L, Remove Gravity
    a_EI_B_mps2 = [0; 0; 0];
    a_BE_B_mps2(:,indx) = a_EI_B_mps2 + a_BI_B_mps2 - T_L2B(:,:,indx) * aGrav_BI_L_mps2;
    
    % Change Coordinates from B to L
    a_BE_L_mps2 = T_B2L(:,:,indx) * a_BE_B_mps2(:,indx);
    
    % Velocity Integration
    v_BE_L_mps(:,indx) = v_BE_L_mps(:,indx-1) + a_BE_L_mps2 * timeStep_s(indx);
    
    %% Position Integration
    r_BE_L_m(:,indx) = r_BE_L_m(:,indx-1) + v_BE_L_mps(:,indx) * timeStep_s(indx);
    
end


%% Outputs


