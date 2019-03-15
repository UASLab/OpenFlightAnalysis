
%% Constants
r2d = 180/pi;
d2r = 1/r2d;
in2m = 0.0254;


%% Load Flight Data and Configuration Parameters
flt{1}.param.vehName = 'Geri';
flt{1}.param.apType = 'Goldy1';
flt{1}.param.config = '1';
flt{1}.param.fltNum = 13;
flt{1}.param.fltFile = 'R:\a-uav-lab\COMMON\Projects\PAAW\Technical\mAEWing1\Test and Analysis\Flight Tests\GeriFLT13\GeriFLT13.mat';

flt{2}.param.vehName = 'Geri';
flt{2}.param.apType = 'Goldy1';
flt{2}.param.config = '1';
flt{2}.param.fltNum = 14;
flt{2}.param.fltFile = 'R:\a-uav-lab\COMMON\Projects\PAAW\Technical\mAEWing1\Test and Analysis\Flight Tests\GeriFLT14\GeriFLT14.mat';

numFlt = length(flt);
for iFlt = 1:numFlt
    % Load Flight Data, pack into structures
    [flt{iFlt}.raw] = LoadFlight(flt{iFlt}.param.fltFile, flt{iFlt}.param.apType);
    [flt{iFlt}.sig] = StructFlight(flt{iFlt}.raw, flt{iFlt}.param.apType);

    % Load Configuration Parameters
    flt{iFlt}.param = LoadConfigGoldy1(flt{iFlt}.param);

    % Plot Flight Overview
    if 0
        PlotOverviewGoldy1(flt{iFlt}.sig);
    end
end

%% Load Flight Maneuver Segments
for iFlt = 1:numFlt
    [flt{iFlt}.seg] = LoadFlightSegDef(flt{iFlt}.param, flt{iFlt}.sig.time_s);
end


%% Create Segments
% Copy Flight Parameter Set to Segment Parameter Set
param{1} = flt{1}.param;
param{1}.indxFlt = 1;
param{1}.indxSeg = flt{1}.seg.flt.indx;

param{2} = flt{2}.param;
param{2}.indxFlt = 2;
param{2}.indxSeg = flt{2}.seg.flt.indx;

% Number of Segments
numSeg = length(param);

%% Copy Raw data into Segment Structures, Vectorize
[param, seg] = Raw2Seg(flt, param);

for iSeg = 1:numSeg
    % Initial Conditions for Integration
    param{iSeg}.s_BL_rad.init = seg{iSeg}.sNav_BL_rad(:,1); % Initialize Euler Angles with EKF solution
    param{iSeg}.s_BL_rad.seq = '321';
    param{iSeg}.v_BL_L_mps.init = seg{iSeg}.vNav_BL_L_mps(:,1); % Initialize NED velocities with EKF solution
    param{iSeg}.r_BL_L_m.init = [0, 0, 0];
end

%% Convert GPS and Nav solution position to L frame
for iSeg = 1:numSeg
    seg{iSeg}.rNav_BE_E_m = NaN(3, param{iSeg}.lenSeg);
    seg{iSeg}.rNav_BL_L_m = NaN(3, param{iSeg}.lenSeg);
    seg{iSeg}.r_GpsE_E_m = NaN(3, param{iSeg}.lenSeg);
    seg{iSeg}.r_GpsL_L_m = NaN(3, param{iSeg}.lenSeg);
    for indx = 1:param{iSeg}.lenSeg
        % Change Nav Solution to L frame
        seg{iSeg}.rNav_BL_L_m(:, indx) = D2L(seg{iSeg}.rNav_BD_D_ddm(:,1), seg{iSeg}.rNav_BD_D_ddm(:,indx));
        
        % Change GPS Solution to L frame
        seg{iSeg}.r_GpsL_L_m(:, indx) = D2L(seg{iSeg}.rMeas_GpsD_D_ddm(:,1), seg{iSeg}.rMeas_GpsD_D_ddm(:,indx));
    end
end

%% Pre-Optimization
for iSeg = 1:numSeg
    % Apply Error Models, Estimate the "true" measures from the Error Models
    % Apply IMU Error Models
    [seg{iSeg}.a_ImuI_Imu_mps2] = SensorErrorModel(seg{iSeg}.aMeas_ImuI_Imu_mps2, param{iSeg}.a_ImuI_Imu_mps2);
    param{iSeg}.w_ImuI_Imu_rps.aTrue = seg{iSeg}.a_ImuI_Imu_mps2;
    [seg{iSeg}.w_ImuI_Imu_rps] = SensorErrorModel(seg{iSeg}.wMeas_ImuI_Imu_rps, param{iSeg}.w_ImuI_Imu_rps);
    
    % Apply GPS Error Models
    [seg{iSeg}.r_GpsD_D_ddm] = SensorErrorModel(seg{iSeg}.rMeas_GpsD_D_ddm, param{iSeg}.r_GpsD_D_ddm);
    [seg{iSeg}.v_GpsL_L_mps] = SensorErrorModel(seg{iSeg}.vMeas_GpsL_L_mps, param{iSeg}.v_GpsL_L_mps);
    
    % Integrate the IMU data to calculate the Euler Angles, Inertial Velocities, and Change in Position
    % Store as '1' value to recall later
    [seg{iSeg}.s1_BL_rad, seg{iSeg}.v1_BL_L_mps, seg{iSeg}.r1_BL_L_m, seg{iSeg}.w1_BL_B_rps, seg{iSeg}.a1_BL_B_mps2] = IntImu(seg{iSeg}.time_s, seg{iSeg}.w_ImuI_Imu_rps, seg{iSeg}.a_ImuI_Imu_mps2, param{iSeg}.s_BL_rad.init, param{iSeg}.v_BL_L_mps.init, param{iSeg}.r_BL_L_m.init, param{iSeg}.r_ImuB_B_m.data, param{iSeg}.s_ImuB_rad.data);
end

%% Estimate the IMU Parameters
% Establish the opimization 'free' parameters
optParam0 = param{iSeg}.s_ImuB_rad.data; % IMU Installation Angle
for iSeg = 1:numSeg
    indxStart = 6*(iSeg - 1) + 3;
    optParam0(indxStart + [1:3]) = param{iSeg}.a_ImuI_Imu_mps2.bias; % Accel Bias
    optParam0(indxStart + [4:6]) = param{iSeg}.w_ImuI_Imu_rps.bias; % Gyro Bias
end

% Setup the Optimizer
optProb.objective = @(optParam) ImuCost(seg, param, optParam);
optProb.x0 = optParam0;
optProb.lb = optParam0 - 2;
optProb.ub = optParam0 + 2;
optProb.solver = 'fmincon';
optProb.options = optimoptions(optProb.solver);
optProb.options.Algorithm = 'sqp';
% optProb.options.MaxIter = 5;
% optProb.options.Display = 'iter';
optProb.options.Display = 'iter-detailed';
optProb.options.PlotFcns = { @optimplotfval };
optProb.options.UseParallel = true;

% Call the Optimizer
% [cost] = ImuCost(seg, param, optParam0);
% cost = optProb.objective(optParam0);
if isempty(gcp('nocreate')), optPool = parpool('local', 4); end
[optParam, fval, exitflag, output] = fmincon(optProb);
delete(gcp('nocreate'));

% Unpack the parameter results
for iSeg = 1:numSeg
    param{iSeg}.s_ImuB_rad.data = optParam(1:3); % IMU Orientation
    
    indxStart = 6*(iSeg - 1) + 3;
    param{iSeg}.a_ImuI_Imu_mps2.bias = optParam(indxStart + [1:3]);
    param{iSeg}.w_ImuI_Imu_rps.bias = optParam(indxStart + [4:6]);
end

for iSeg = 1:numSeg
    % Compute the final trajectory - FIXIT - shouldn't need to redo...
    % Apply IMU Error Models
    [seg{iSeg}.a_ImuI_Imu_mps2] = SensorErrorModel(seg{iSeg}.aMeas_ImuI_Imu_mps2, param{iSeg}.a_ImuI_Imu_mps2);
    param{iSeg}.w_ImuI_Imu_rps.aTrue = seg{iSeg}.a_ImuI_Imu_mps2;
    [seg{iSeg}.w_ImuI_Imu_rps] = SensorErrorModel(seg{iSeg}.wMeas_ImuI_Imu_rps, param{iSeg}.w_ImuI_Imu_rps);
    
    % Integrate the IMU data to calculate the Euler Angles, Inertial Velocities, and Change in Position
    [seg{iSeg}.s_BL_rad, seg{iSeg}.v_BL_L_mps, seg{iSeg}.r_BL_L_m, seg{iSeg}.w_BL_B_rps, seg{iSeg}.a_BL_B_mps2] = IntImu(seg{iSeg}.time_s, seg{iSeg}.w_ImuI_Imu_rps, seg{iSeg}.a_ImuI_Imu_mps2, param{iSeg}.s_BL_rad.init, param{iSeg}.v_BL_L_mps.init, param{iSeg}.r_BL_L_m.init, param{iSeg}.r_ImuB_B_m.data, param{iSeg}.s_ImuB_rad.data);
end

%%
% Compare against EKF Euler Angles
optPlotCell.xNames = {[], [], 'Time(s)'};
optPlotCell.yNames = {'Bank Angle (deg)', 'Attitude Angle (deg)', 'Heading Angle (deg)'};
optPlotCell.legNames = {{'EKF', 'Integration', 'Optimized'}, {}, {}};
optPlotCell.titleStr = 'Euler Angles';

for iSeg = 1:numSeg
    PlotCell({seg{iSeg}.time_s - seg{iSeg}.time_s(1)}, ...
        {{seg{iSeg}.sNav_BL_rad(1,:) * r2d, seg{iSeg}.s1_BL_rad(1,:) * r2d, seg{iSeg}.s_BL_rad(1,:) * r2d};
        {seg{iSeg}.sNav_BL_rad(2,:) * r2d, seg{iSeg}.s1_BL_rad(2,:) * r2d, seg{iSeg}.s_BL_rad(2,:) * r2d};
        {seg{iSeg}.sNav_BL_rad(3,:) * r2d, seg{iSeg}.s1_BL_rad(3,:) * r2d, seg{iSeg}.s_BL_rad(3,:) * r2d}}, ...
        optPlotCell);
end
%%
% Compare against EKF NED Rates
optPlotCell.yNames = {'vNorth (m/s)', 'vEast (m/s)', 'vDown (m/s)'};
optPlotCell.legNames = {{'EKF', 'GPS', 'Integration', 'Optimized'}, {}, {}};
optPlotCell.titleStr = 'Inertial Velocities';

for iSeg = 1:numSeg
    PlotCell({seg{iSeg}.time_s - seg{iSeg}.time_s(1)}, ...
        {{seg{iSeg}.vNav_BL_L_mps(1,:), seg{iSeg}.vMeas_GpsL_L_mps(1,:), seg{iSeg}.v1_BL_L_mps(1,:), seg{iSeg}.v_BL_L_mps(1,:)};
        {seg{iSeg}.vNav_BL_L_mps(2,:), seg{iSeg}.vMeas_GpsL_L_mps(2,:), seg{iSeg}.v1_BL_L_mps(2,:), seg{iSeg}.v_BL_L_mps(2,:)};
        {seg{iSeg}.vNav_BL_L_mps(3,:), seg{iSeg}.vMeas_GpsL_L_mps(3,:), seg{iSeg}.v1_BL_L_mps(3,:), seg{iSeg}.v_BL_L_mps(3,:)}}, ...
        optPlotCell);
end
%%
% Compare against EKF NED Positions
optPlotCell.yNames = {'North (m)', 'East (m)', 'Down (m)'};
optPlotCell.titleStr = 'Inertial Position';

for iSeg = 1:numSeg
    PlotCell({seg{iSeg}.time_s - seg{iSeg}.time_s(1)}, ...
        {{seg{iSeg}.rNav_BL_L_m(1,:), seg{iSeg}.r_GpsL_L_m(1,:), seg{iSeg}.r1_BL_L_m(1,:), seg{iSeg}.r_BL_L_m(1,:)};
        {seg{iSeg}.rNav_BL_L_m(2,:), seg{iSeg}.r_GpsL_L_m(2,:), seg{iSeg}.r1_BL_L_m(2,:), seg{iSeg}.r_BL_L_m(2,:)};
        {seg{iSeg}.rNav_BL_L_m(3,:), seg{iSeg}.r_GpsL_L_m(3,:), seg{iSeg}.r1_BL_L_m(3,:), seg{iSeg}.r_BL_L_m(3,:)}}, ...
        optPlotCell);
end
return
%% Transform Pitot Measures
for iSeg = 1:numSeg
    % Apply Pitot-Static Error Models
    [seg{iSeg}.vMag_PA_mps] = SensorErrorModel(seg{iSeg}.vMeasMag_PA_mps, param{iSeg}.v_PA_P_mps);
    [seg{iSeg}.rMag_PA_m] = SensorErrorModel(seg{iSeg}.rMeasMag_PA_m, param{iSeg}.r_PA_P_m);
    
    % Assume the measured airspeed is in the Probe X-direction
    seg{iSeg}.v_PA_P_mps = [seg{iSeg}.vMag_PA_mps; zeros(2, param{iSeg}.lenSeg)];
    
    % Assume the rotation rate of the atmosphere is negligible
    seg{iSeg}.w_AL_L_rps = zeros(3, param{iSeg}.lenSeg);
    
    % Compute the Rotation rate of the Probe wrt the Atm
    % seg{iSeg}.w_BA_B_rps = seg{iSeg}.w_BL_B_rps + T_BL * seg{iSeg}.w_AL_L_rps;
    seg{iSeg}.w_BA_B_rps = seg{iSeg}.w_BL_B_rps + seg{iSeg}.w_AL_L_rps; % FIXIT - should have transformation from L to B
    
    % Translate and Rotate the Pitot measurement to the Body frame
    [seg{iSeg}.vTemp_BA_B_mps, seg{iSeg}.vTemp_BA_L_mps] = TransPitot(seg{iSeg}.v_PA_P_mps, seg{iSeg}.w_BA_B_rps, seg{iSeg}.s_BL_rad, param{iSeg}.s_PB_rad.data, param{iSeg}.r_PB_B_m.data);
    
    % Compute the Magnitude of the Velocity at the Body frame
    seg{iSeg}.vMag_BA_mps = sqrt((seg{iSeg}.vTemp_BA_B_mps(1,:).^2 + seg{iSeg}.vTemp_BA_B_mps(2,:).^2 + seg{iSeg}.vTemp_BA_B_mps(3,:).^2));
end

%% Estimate the Wind
for iSeg = 1:numSeg
    % Magnitude of the Inertial Velocity
    seg{iSeg}.vMag_BL_mps = sqrt(seg{iSeg}.vNav_BL_L_mps(1,:).^2 + seg{iSeg}.vNav_BL_L_mps(2,:).^2 + seg{iSeg}.vNav_BL_L_mps(3,:).^2);
    
    % Subtract the Estimated Body Airspeed from the Inertial Velocity
    seg{iSeg}.vEst_AL_L_mps = seg{iSeg}.vNav_BL_L_mps - seg{iSeg}.vTemp_BA_L_mps;
    seg{iSeg}.vEstMag_AL_mps = sqrt((seg{iSeg}.vEst_AL_L_mps(1,:).^2 + seg{iSeg}.vEst_AL_L_mps(2,:).^2 + seg{iSeg}.vEst_AL_L_mps(3,:).^2));
    
    % Compute Heading (to) angle
    seg{iSeg}.sEst_AL_rad = NaN(3, param{iSeg}.lenSeg);
    seg{iSeg}.sEst_AL_rad(3,:) = atan2(seg{iSeg}.vEst_AL_L_mps(2,:), seg{iSeg}.vEst_AL_L_mps(1,:));
    
    % Compute the Mean of the Wind Estimate, in NED
    param{iSeg}.vEst_AL_L_mps.mean = nanmean(seg{iSeg}.vEst_AL_L_mps, 2);
    
    
    disp(['Wind Estimate: '])
    disp(param{iSeg}.vEst_AL_L_mps.mean)
end

%%
% Plot Wind Estimate
optPlotCell.xNames = {[], [], 'Time(s)'};
optPlotCell.yNames = {'vNorth (m/s)', 'vEast (m/s)', 'vDown (m/s)'};
optPlotCell.legNames = {{'Inertial', 'Airspeed', 'Wind'}, {}, {}};
optPlotCell.titleStr = 'Wind Speed Estimate';

for iSeg = 1:numSeg
    PlotCell({seg{iSeg}.time_s}, ...
        {{seg{iSeg}.vNav_BL_L_mps(1,:), seg{iSeg}.vTemp_BA_L_mps(1,:), seg{iSeg}.vEst_AL_L_mps(1,:)}, ...
        {seg{iSeg}.vNav_BL_L_mps(2,:), seg{iSeg}.vTemp_BA_L_mps(2,:), seg{iSeg}.vEst_AL_L_mps(2,:)}, ...
        {seg{iSeg}.vNav_BL_L_mps(3,:), seg{iSeg}.vTemp_BA_L_mps(3,:), seg{iSeg}.vEst_AL_L_mps(3,:)}}, ...
        optPlotCell);
end

% Plot Airspeed Magnitude
optPlotCell.xNames = {'Time(s)'};
optPlotCell.yNames = {'vMag (m/s)'};
optPlotCell.legNames = {{'Meas', 'Body', 'Inertial', 'Wind', 'Inertial - Airspeed'}, {}, {}};
optPlotCell.titleStr = 'Airspeed Magnitude';

for iSeg = 1:numSeg
    PlotCell({seg{iSeg}.time_s}, ...
        {{seg{iSeg}.vMeasMag_PA_mps, seg{iSeg}.vMag_BA_mps, seg{iSeg}.vMag_BL_mps, seg{iSeg}.vEstMag_AL_mps, abs(seg{iSeg}.vMag_BL_mps - seg{iSeg}.vMag_BA_mps)}}, ...
        optPlotCell);
end

% Plot Wind Speed Estimate
optPlotCell.xNames = {[], [], 'Time(s)'};
optPlotCell.yNames = {'v NED (m/s)', 'vMag (m/s)', 'heading to (deg)'};
optPlotCell.legNames = {{'vNorth', 'vEast', 'vDown'}, {}, {}};
optPlotCell.titleStr = 'Wind Speed Estimate';

for iSeg = 1:numSeg
    PlotCell({seg{iSeg}.time_s}, ...
        {{seg{iSeg}.vEst_AL_L_mps(1,:), seg{iSeg}.vEst_AL_L_mps(2,:), seg{iSeg}.vEst_AL_L_mps(3,:)}, ...
        {seg{iSeg}.vEstMag_AL_mps}, ...
        {seg{iSeg}.sEst_AL_rad(3,:) * r2d}}, ...
        optPlotCell);
end

%% Input an Estimate for the Wind and Compute
for iSeg = 1:numSeg
    % Compute the Airspeeds based on the Wind Estimate
    seg{iSeg}.v_BA_L_mps = seg{iSeg}.vNav_BL_L_mps - repmat(param{iSeg}.vEst_AL_L_mps.mean, 1, param{iSeg}.lenSeg);
    
    % Transform the L Coordinates to the B Coordinates
    seg{iSeg}.v_BA_B_mps = NaN(3, param{iSeg}.lenSeg);
    for indx = 1:param{iSeg}.lenSeg
        T_BL = RotMat(param{iSeg}.s_BL_rad.seq, seg{iSeg}.s_BL_rad(:,indx), 'rad');
        
        seg{iSeg}.v_BA_B_mps(:,indx) = T_BL * seg{iSeg}.v_BA_L_mps(:,indx);
    end
    
    % Magnitude of the Velocity
    seg{iSeg}.vMag_BA_B_mps = sqrt(seg{iSeg}.v_BA_B_mps(1,:).^2 + seg{iSeg}.v_BA_B_mps(2,:).^2 + seg{iSeg}.v_BA_B_mps(3,:).^2);
    
    % Airspeed Error
    nanmean(sqrt(abs(seg{iSeg}.vMag_BA_B_mps.^2 - seg{iSeg}.vMeasMag_PA_mps.^2)))
    
    % Compute Inflow Angles, Angle of Attack and Angle of Sideslip
    seg{iSeg}.alpha_BA_deg = atan2d(seg{iSeg}.v_BA_B_mps(3,:), seg{iSeg}.v_BA_B_mps(1,:));
    seg{iSeg}.beta_BA_deg = asin(seg{iSeg}.v_BA_B_mps(2,:) ./ seg{iSeg}.vMag_BA_B_mps);
end

for iSeg = 1:numSeg
    % Plot inflow conditions
    optPlotCell.xNames = {[], [], 'Time(s)'};
    optPlotCell.yNames = {'vMag (m/s)', 'Alpha (deg)', 'Beta (deg)'};
    optPlotCell.legNames = {{'Measured', 'Estimated'}, {}, {}};
    optPlotCell.titleStr = 'Inflow Conditions';
end

for iSeg = 1:numSeg
    PlotCell({seg{iSeg}.time_s}, ...
        {{seg{iSeg}.vMeasMag_PA_mps, seg{iSeg}.vMag_BA_B_mps}, ...
        {seg{iSeg}.alpha_BA_deg}, ...
        {seg{iSeg}.beta_BA_deg}}, ...
        optPlotCell);
end

for iSeg = 1:numSeg
    % Plot Airspeed Error Estimate
    figure;
    plot(seg{iSeg}.vMeasMag_PA_mps, seg{iSeg}.vMag_BA_B_mps - seg{iSeg}.vMeasMag_PA_mps, '*');
    xlabel('Measured Airspeed (m/s)')
    ylabel('Airpeed Error Estimate (m/s)');
    hold on;
end
