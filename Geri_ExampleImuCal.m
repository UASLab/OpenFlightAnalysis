
%% Constants
r2d = 180/pi;
d2r = 1/r2d;
in2m = 0.0254;


%% Load Flight Data and Configuration Parameters
pathDataRoot = 'R:\a-uav-lab\COMMON\Projects\PAAW\Technical\mAEWing1\Test and Analysis\Flight Tests\';

indxFlt = 1;
flt{indxFlt}.param.vehName = 'Geri';
flt{indxFlt}.param.apType = 'Goldy1';
flt{indxFlt}.param.fltNum = 2;
flt{indxFlt}.param.config = '1';

indxFlt = indxFlt + 1;
flt{indxFlt}.param.vehName = 'Geri';
flt{indxFlt}.param.apType = 'Goldy1';
flt{indxFlt}.param.fltNum = 4;
flt{indxFlt}.param.config = '1';

indxFlt = indxFlt + 1;
flt{indxFlt}.param.vehName = 'Geri';
flt{indxFlt}.param.apType = 'Goldy1';
flt{indxFlt}.param.fltNum = 5;
flt{indxFlt}.param.config = '1';

numFlt = length(flt);
for iFlt = 1:numFlt
    % Generate the file name and full file path
    flt{indxFlt}.param.fltName = [flt{indxFlt}.param.vehName, 'FLT', num2str(flt{indxFlt}.param.fltNum, '%0.2d')];
    flt{indxFlt}.param.fltFile = fullfile(pathDataRoot, [flt{indxFlt}.param.fltName, '\', flt{indxFlt}.param.fltName, '.mat']);

    % Load Configuration Parameters
    flt{iFlt}.param = LoadConfigGoldy1(flt{iFlt}.param);
    
    % Load Flight Data, pack into structures
    [flt{iFlt}.raw] = LoadFlight(flt{iFlt}.param.fltFile, flt{iFlt}.param.apType);
    [flt{iFlt}.sig] = StructFlight(flt{iFlt}.raw, flt{iFlt}.param.apType, flt{iFlt}.param.apVers);

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
iSeg = 1; indxFlt = 3;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{1}.indx;

iSeg = iSeg+1; indxFlt = 3;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{2}.indx;

iSeg = iSeg+1; indxFlt = 3;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{3}.indx;

iSeg = iSeg+1; indxFlt = 3;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
% param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{4}.indx;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn4approach.indx;

% Number of Segments
numSeg = length(param);

%% Copy Raw data into Segment Structures, Vectorize
[param, seg] = Raw2Seg(flt, param);

%% Pre-Optimization
for iSeg = 1:numSeg
    % Apply Error Models, Estimate the "true" measures from the Error Models
    % Apply IMU Error Models
    [seg{iSeg}.a_ImuI_Imu_mps2] = SensorErrorModel(seg{iSeg}.aMeas_ImuI_Imu_mps2, param{iSeg}.a_ImuI_Imu_mps2);
    param{iSeg}.w_ImuI_Imu_rps.aTrue = seg{iSeg}.a_ImuI_Imu_mps2;
    [seg{iSeg}.w_ImuI_Imu_rps] = SensorErrorModel(seg{iSeg}.wMeas_ImuI_Imu_rps, param{iSeg}.w_ImuI_Imu_rps);
    
    % Apply GPS Error Models
    [seg{iSeg}.r_GpsE_D_ddm] = SensorErrorModel(seg{iSeg}.rMeas_GpsE_D_ddm, param{iSeg}.r_GpsE_D_ddm);
    [seg{iSeg}.v_GpsE_L_mps] = SensorErrorModel(seg{iSeg}.vMeas_GpsE_L_mps, param{iSeg}.v_GpsE_L_mps);
    
    %% Convert GPS and Nav solution position to L frame
    seg{iSeg}.rNav_BE_E_m = NaN(3, param{iSeg}.lenSeg);
    seg{iSeg}.rNav_BE_L_m = NaN(3, param{iSeg}.lenSeg);
    seg{iSeg}.r_GpsE_E_m = NaN(3, param{iSeg}.lenSeg);
    seg{iSeg}.r_GpsE_L_m = NaN(3, param{iSeg}.lenSeg);
    for indx = 1:param{iSeg}.lenSeg
        % Change Nav Solution to L frame
        seg{iSeg}.rNav_BE_L_m(:, indx) = D2L(seg{iSeg}.rNav_BE_D_ddm(:,1), seg{iSeg}.rNav_BE_D_ddm(:,indx));
        
        % Change GPS Solution to L frame
        seg{iSeg}.r_GpsE_L_m(:, indx) = D2L(seg{iSeg}.r_GpsE_D_ddm(:,1), seg{iSeg}.r_GpsE_D_ddm(:,indx));
    end
    
    % Initial Conditions for Integration
    param{iSeg}.s_BL_rad.init = seg{iSeg}.sNav_BL_rad(:,1); % Initialize Euler Angles with EKF solution
    param{iSeg}.v_BE_L_mps.init = seg{iSeg}.vNav_BE_L_mps(:,1); % Initialize NED velocities with EKF solution
    param{iSeg}.r_BE_L_m.init = [0, 0, 0];
    
    % Integrate the IMU data to calculate the Euler Angles, Inertial Velocities, and Change in Position
    % Store as '1' value to recall later
    [seg{iSeg}.s1_BL_rad, seg{iSeg}.v1_BE_L_mps, seg{iSeg}.r1_BE_L_m, seg{iSeg}.w1_BL_B_rps, seg{iSeg}.a1_BE_B_mps2] = IntImu(seg{iSeg}.time_s, seg{iSeg}.w_ImuI_Imu_rps, seg{iSeg}.a_ImuI_Imu_mps2, param{iSeg}.s_BL_rad.init, param{iSeg}.v_BE_L_mps.init, param{iSeg}.r_BE_L_m.init, param{iSeg}.r_ImuB_B_m.data, param{iSeg}.s_ImuB_rad.data);
end

%% Estimate the IMU Parameters
% Establish the opimization 'free' parameters
iOptStatic = 1;
optParam.static{iOptStatic}.field = 's_ImuB_rad.data'; % IMU Installation Angle
optParam.static{iOptStatic}.len = 3;
optParam.static{iOptStatic}.ub = 5*d2r*[1;1;1]; optParam.static{iOptStatic}.lb = -optParam.static{iOptStatic}.ub;

iOptSeg = 1;
optParam.seg{iOptSeg}.field = 's_BL_rad.init'; % Initial Orientation
optParam.seg{iOptSeg}.len = 3;

iOptSeg = iOptSeg + 1;
optParam.seg{iOptSeg}.field = 'v_BE_L_mps.init'; % Initial Inertial Velocity
optParam.seg{iOptSeg}.len = 3;

iOptSeg = iOptSeg + 1;
optParam.seg{iOptSeg}.field = 'a_ImuI_Imu_mps2.bias'; % Accel Bias
optParam.seg{iOptSeg}.len = 3;
optParam.seg{iOptSeg}.ub = 1*[1;1;1]; optParam.seg{iOptSeg}.lb = -optParam.seg{iOptSeg}.ub;

iOptSeg = iOptSeg + 1;
optParam.seg{iOptSeg}.field = 'w_ImuI_Imu_rps.bias'; % Gyro Bias
optParam.seg{iOptSeg}.len = 3;
optParam.seg{iOptSeg}.ub = 0.1*[1;1;1]; optParam.seg{iOptSeg}.lb = -optParam.seg{iOptSeg}.ub;


% Pack the optParam into a vector
[optParam, param] = DefOptParam(optParam, param, 'pack');
optParamVal = optParam.val;


% Setup the Optimizer
optProb.objective = @(optParamVal) ImuCalCost(seg, param, optParam, optParamVal);
optProb.x0 = optParam.val;
optProb.lb = optParam.lb;
optProb.ub = optParam.ub;
optProb.solver = 'fmincon';
optProb.options = optimoptions(optProb.solver);
optProb.options.Algorithm = 'sqp';
% optProb.options.MaxIter = 2;
% optProb.options.Display = 'iter';
optProb.options.Display = 'iter-detailed';
optProb.options.PlotFcns = { @optimplotfval };
optProb.options.UseParallel = true;

% Call the Optimizer
% [cost] = ImuCalCost(seg, param, optParam, optParamVal);
% cost = optProb.objective(optParamVal);
[optParamVal, fval, exitflag, output] = fmincon(optProb);


% Unpack the parameter results
optParam.val = optParamVal;
[optParam, param] = DefOptParam(optParam, param, 'unpack');

% Compute the final trajectory - FIXIT - shouldn't need to redo...
[~, seg, param] = ImuCalCost(seg, param, optParam, optParamVal);


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

% Compare against EKF NED Rates
optPlotCell.yNames = {'vNorth (m/s)', 'vEast (m/s)', 'vDown (m/s)'};
optPlotCell.legNames = {{'EKF', 'GPS', 'Integration', 'Optimized'}, {}, {}};
optPlotCell.titleStr = 'Inertial Velocities';

for iSeg = 1:numSeg
    PlotCell({seg{iSeg}.time_s - seg{iSeg}.time_s(1)}, ...
        {{seg{iSeg}.vNav_BE_L_mps(1,:), seg{iSeg}.vMeas_GpsE_L_mps(1,:), seg{iSeg}.v1_BE_L_mps(1,:), seg{iSeg}.v_BE_L_mps(1,:)};
        {seg{iSeg}.vNav_BE_L_mps(2,:), seg{iSeg}.vMeas_GpsE_L_mps(2,:), seg{iSeg}.v1_BE_L_mps(2,:), seg{iSeg}.v_BE_L_mps(2,:)};
        {seg{iSeg}.vNav_BE_L_mps(3,:), seg{iSeg}.vMeas_GpsE_L_mps(3,:), seg{iSeg}.v1_BE_L_mps(3,:), seg{iSeg}.v_BE_L_mps(3,:)}}, ...
        optPlotCell);
end

% Compare against EKF NED Positions
optPlotCell.yNames = {'North (m)', 'East (m)', 'Down (m)'};
optPlotCell.titleStr = 'Inertial Position';

for iSeg = 1:numSeg
    PlotCell({seg{iSeg}.time_s - seg{iSeg}.time_s(1)}, ...
        {{seg{iSeg}.rNav_BE_L_m(1,:), seg{iSeg}.r_GpsE_L_m(1,:), seg{iSeg}.r1_BE_L_m(1,:), seg{iSeg}.r_BE_L_m(1,:)};
        {seg{iSeg}.rNav_BE_L_m(2,:), seg{iSeg}.r_GpsE_L_m(2,:), seg{iSeg}.r1_BE_L_m(2,:), seg{iSeg}.r_BE_L_m(2,:)};
        {seg{iSeg}.rNav_BE_L_m(3,:), seg{iSeg}.r_GpsE_L_m(3,:), seg{iSeg}.r1_BE_L_m(3,:), seg{iSeg}.r_BE_L_m(3,:)}}, ...
        optPlotCell);
end
return
%% Save
save('GeriFLT04_ImuCal.mat', 'param', 'seg', 'optParam');
