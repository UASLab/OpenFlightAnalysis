
%% Constants
r2d = 180/pi;
d2r = 1/r2d;
in2m = 0.0254;


%% Load Flight Data and Configuration Parameters
% basePath = 'D:\Flight Archive\Mjolnir';
% basePath = '/mnt/sdb1/Flight Archive/Mjolnir';
basePath = 'E:\Flight Archive\Mjolnir';

flt{1}.param.vehName = 'Mjolnir';
flt{1}.param.apType = 'Goldy3';
flt{1}.param.config = '1';
flt{1}.param.fltNum = 1;
flt{1}.param.fltFile = fullfile(basePath, 'MjolnirFLT01', 'MjolnirFLT01.mat');

flt{2}.param.vehName = 'Mjolnir';
flt{2}.param.apType = 'Goldy3';
flt{2}.param.config = '1';
flt{2}.param.fltNum = 2;
flt{2}.param.fltFile = fullfile(basePath, 'MjolnirFLT02', 'MjolnirFLT02.mat');

flt{3}.param.vehName = 'Mjolnir';
flt{3}.param.apType = 'Goldy3';
flt{3}.param.config = '1';
flt{3}.param.fltNum = 3;
flt{3}.param.fltFile = fullfile(basePath, 'MjolnirFLT03', 'MjolnirFLT03.mat');

flt{4}.param.vehName = 'Mjolnir';
flt{4}.param.apType = 'Goldy3';
flt{4}.param.config = '1';
flt{4}.param.fltNum = 4;
flt{4}.param.fltFile = fullfile(basePath, 'MjolnirFLT04', 'MjolnirFLT04.mat');

flt{5}.param.vehName = 'Mjolnir';
flt{5}.param.apType = 'Goldy3';
flt{5}.param.config = '1';
flt{5}.param.fltNum = 5;
flt{5}.param.fltFile = fullfile(basePath, 'MjolnirFLT05', 'MjolnirFLT05.mat');

numFlt = length(flt);
for iFlt = 1:numFlt
    % Load Flight Data, pack into structures
    [flt{iFlt}.raw] = LoadFlight(flt{iFlt}.param.fltFile, flt{iFlt}.param.apType);
    [flt{iFlt}.sig] = StructFlight(flt{iFlt}.raw, flt{iFlt}.param.apType);

    % Load Configuration Parameters
    flt{iFlt}.param = LoadConfigGoldy3(flt{iFlt}.param);

    % Plot Flight Overview
    if 0
        PlotOverviewGoldy3(flt{iFlt}.sig);
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
param{iSeg}.indxSeg = flt{indxFlt}.seg.aircal.indx;

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{1}.indx;

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{2}.indx;

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{3}.indx;

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{4}.indx;

iSeg = iSeg+1; indxFlt = 4;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.aircal.indx;

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{1}.indx;

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{2}.indx;

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{3}.indx;

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{4}.indx;

iSeg = iSeg+1; indxFlt = 5;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.aircal.indx;

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{1}.indx;

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{2}.indx;

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{3}.indx;

iSeg = iSeg+1;
param{iSeg} = flt{indxFlt}.param;
param{iSeg}.indxFlt = indxFlt;
param{iSeg}.indxSeg = flt{indxFlt}.seg.turn{4}.indx;

% Number of Segments
numSeg = length(param);

%% Copy Raw data into Segment Structures, Vectorize
[param, seg] = Raw2Seg_Goldy3(flt, param);


%% Convert GPS and Nav solution position to L frame
for iSeg = 1:numSeg
    seg{iSeg}.rNav_BE_E_m = NaN(3, param{iSeg}.lenSeg);
    seg{iSeg}.rNav_BE_L_m = NaN(3, param{iSeg}.lenSeg);
    seg{iSeg}.r_GpsE_E_m = NaN(3, param{iSeg}.lenSeg);
    seg{iSeg}.r_GpsE_L_m = NaN(3, param{iSeg}.lenSeg);
    for indx = 1:param{iSeg}.lenSeg
        % Change Nav Solution to L frame
        seg{iSeg}.rNav_BE_L_m(:, indx) = D2L(seg{iSeg}.rNav_BE_D_ddm(:,1), seg{iSeg}.rNav_BE_D_ddm(:,indx));
        
        % Change GPS Solution to L frame
        seg{iSeg}.r_GpsE_L_m(:, indx) = D2L(seg{iSeg}.rMeas_GpsE_D_ddm(:,1), seg{iSeg}.rMeas_GpsE_D_ddm(:,indx));
    end
end

%% Pre-Optimization, Initial Guess for the Wind
for iSeg = 1:numSeg
    % Over-ride Default Error Model, Optional
    param{iSeg}.v_PA_P_mps.errorType = 'ScaleBias+';
    param{iSeg}.v_PA_P_mps.bias = 0;
    param{iSeg}.v_PA_P_mps.K = 1.0;
    
    % Transform Pitot Measures
    [seg{iSeg}.vTemp_BA_B_mps, seg{iSeg}.vTemp_BA_L_mps] = ...
        AirCal(seg{iSeg}.vMeasMag_PA_mps, seg{iSeg}.sNav_BL_rad, param{iSeg});

    % Magnitude of the Inertial Velocity
    seg{iSeg}.vMag_BE_mps = sqrt(seg{iSeg}.vNav_BE_L_mps(1,:).^2 + seg{iSeg}.vNav_BE_L_mps(2,:).^2 + seg{iSeg}.vNav_BE_L_mps(3,:).^2);
    
    % Subtract the Estimated Body Airspeed from the Inertial Velocity
    seg{iSeg}.v_AE_L_mps = seg{iSeg}.vNav_BE_L_mps - seg{iSeg}.vTemp_BA_L_mps;

    % Compute the Mean of the Wind Estimate, in NED
    param{iSeg}.v_AE_L_mps.mean = nanmean(seg{iSeg}.v_AE_L_mps, 2);
end


%% Estimate the Wind and Pitot Parameters
% Establish the opimization 'free' parameters
% Tuning Parameters that are Static (same for all segments)
indxOptStatic = 1;
% optParam.static{indxOptStatic}.field = 's_PB_rad.data';
% optParam.static{indxOptStatic}.len = 3;
% optParam.static{indxOptStatic}.ub = 10*d2r*[1;1;1]; optParam.static{indxOptStatic}.lb = -optParam.static{indxOptStatic}.ub;

% indxOptStatic = indxOptStatic + 1;
optParam.static{indxOptStatic}.field = 'v_PA_P_mps.K';
optParam.static{indxOptStatic}.len = 1;

indxOptStatic = indxOptStatic + 1;
optParam.static{indxOptStatic}.field = 'v_PA_P_mps.bias';
optParam.static{indxOptStatic}.len = 1;


% Tuning Parameters for each Segment
indxOptSeg = 1;
optParam.seg{indxOptSeg}.field = 'v_AE_L_mps.mean';
% optParam.seg{indxOptSeg}.ub = 10*[1;1;1]; optParam.seg{indxOptSeg}.lb = -optParam.seg{indxOptSeg}.ub;
optParam.seg{indxOptSeg}.len = 3;


% Pack the optParam into a vector
[optParam, param] = DefOptParam(optParam, param, 'pack');


% Setup the Optimizer
optProb.objective = @(optParamVal) double(AirCalCost(seg, param, optParam, optParamVal));
optProb.x0 = optParam.val;
optProb.lb = optParam.lb;
optProb.ub = optParam.ub;

optProb.solver = 'fmincon';
optProb.options = optimoptions(optProb.solver);
optProb.options.Algorithm = 'active-set';
optProb.options.Display = 'iter-detailed';
optProb.options.PlotFcns = { @optimplotfval };
optProb.options.UseParallel = true;

% Call the Optimizer
% [cost] = AirCalCost(seg, param, optParam, optParam.val);
% cost = optProb.objective(optParam.val);
[optParamVal, fval, exitflag, output] = fmincon(optProb);


% Unpack the parameter results
optParam.val = optParamVal;
[optParam, param] = DefOptParam(optParam, param, 'unpack');

% Compute the final solution - FIXIT - shouldn't need to redo...
[~, seg, param] = AirCalCost(seg, param, optParam, optParamVal);

%% Compute the Wing Velocity and Errors
for iSeg = 1:numSeg
    % Recompute the instantaneous Wind Velocity
    % Wind Velocity, Subtract the Estimated Body Airspeed from the Inertial Velocity
    seg{iSeg}.v_AE_L_mps = seg{iSeg}.vEst_BE_L_mps - seg{iSeg}.vCal_BA_L_mps; % param{iSeg}.v_AE_L_mps.mean
    
    % Compute the True Air Velocity, True = Nav - MeanWind
    seg{iSeg}.v_BA_L_mps = seg{iSeg}.vNav_BE_L_mps - seg{iSeg}.v_AE_L_mps;
    
    % Rotate the B/A velocity to the B-frame
    seg{iSeg}.v_BA_B_mps = NaN(3, param{iSeg}.lenSeg);
    for indx = 1:param{iSeg}.lenSeg

        T_L2B = RotMat(param{iSeg}.s_BL_rad.seq, seg{iSeg}.sNav_BL_rad(:,indx), 'rad');
        seg{iSeg}.v_BA_B_mps(:,indx) = T_L2B * seg{iSeg}.v_BA_L_mps(:,indx);
    end

    % Magnitude of the Wind Speed
    seg{iSeg}.vMag_AE_mps = sqrt(seg{iSeg}.vMean_AE_L_mps(1,:).^2 + seg{iSeg}.vMean_AE_L_mps(2,:).^2 + seg{iSeg}.vMean_AE_L_mps(3,:).^2);
	
    % Magnitude of the Airspeed
    seg{iSeg}.vCalMag_BA_mps = sqrt(seg{iSeg}.vCal_BA_B_mps(1,:).^2 + seg{iSeg}.vCal_BA_B_mps(2,:).^2 + seg{iSeg}.vCal_BA_B_mps(3,:).^2);
    seg{iSeg}.vMag_BA_mps = sqrt(seg{iSeg}.v_BA_B_mps(1,:).^2 + seg{iSeg}.v_BA_B_mps(2,:).^2 + seg{iSeg}.v_BA_B_mps(3,:).^2);
    
    % Groudspeed Error
    seg{iSeg}.vError_BE_L_mps = seg{iSeg}.vNav_BE_L_mps - seg{iSeg}.vEst_BE_L_mps;
    seg{iSeg}.vMagError_BE_mps = sqrt(seg{iSeg}.vError_BE_L_mps(1,:).^2 + seg{iSeg}.vError_BE_L_mps(2,:).^2 + seg{iSeg}.vError_BE_L_mps(3,:).^2);
    
    % Airspeed Error
    seg{iSeg}.vCalError_BA_B_mps = seg{iSeg}.v_BA_B_mps - seg{iSeg}.vCal_BA_B_mps;
    seg{iSeg}.vCalErrorMag_BA_mps = sqrt(seg{iSeg}.vCalError_BA_B_mps(1,:).^2 + seg{iSeg}.vCalError_BA_B_mps(2,:).^2 + seg{iSeg}.vCalError_BA_B_mps(3,:).^2);
    
    % seg{iSeg}.vCalMagError_BA_mps = seg{iSeg}.vMag_BA_mps - seg{iSeg}.vMeasMag_PA_mps;
    
    % Compute Inflow Angles, Angle of Attack and Angle of Sideslip, these
    % should be zero with the assumption that Pitot only measures magnitude
    % so we assign it only to the X-axis
    seg{iSeg}.alphaCal_BA_deg = atan2d(seg{iSeg}.vCal_BA_B_mps(3,:), seg{iSeg}.vCal_BA_B_mps(1,:));
    seg{iSeg}.betaCal_BA_deg = asind(seg{iSeg}.vCal_BA_B_mps(2,:) ./ seg{iSeg}.vCalMag_BA_mps);

    % Compute Inflow Angles, Angle of Attack and Angle of Sideslip, these
    % are based on the NED velocities (wind corrected) and the NavFilter based orientation
    seg{iSeg}.alpha_BA_deg = atan2d(seg{iSeg}.v_BA_B_mps(3,:), seg{iSeg}.v_BA_B_mps(1,:));
    seg{iSeg}.beta_BA_deg = asind(seg{iSeg}.v_BA_B_mps(2,:) ./ seg{iSeg}.vMag_BA_mps);
    
end

%%

% Plot Ground Speed Estimate
optPlotCell.xNames = {[], [], 'Time(s)'};
optPlotCell.yNames = {'vNorth (m/s)', 'vEast (m/s)', 'vDown (m/s)'};
optPlotCell.legNames = repmat({{'GPS', 'Optimized Inertial', 'Optimized Air'}}, 3, 1);
optPlotCell.titleStr = 'Ground Speed Estimate';

for iSeg = 1:numSeg
    PlotCell({seg{iSeg}.time_s}, ...
        {{seg{iSeg}.vMeas_GpsE_L_mps(1,:), seg{iSeg}.vNav_BE_L_mps(1,:), seg{iSeg}.vEst_BE_L_mps(1,:)}, ...
        {seg{iSeg}.vMeas_GpsE_L_mps(2,:), seg{iSeg}.vNav_BE_L_mps(2,:), seg{iSeg}.vEst_BE_L_mps(2,:)}, ...
        {seg{iSeg}.vMeas_GpsE_L_mps(3,:), seg{iSeg}.vNav_BE_L_mps(3,:), seg{iSeg}.vEst_BE_L_mps(3,:)}}, ...
        optPlotCell);
end


%%
% Plot Airspeed Magnitude
% optPlotCell.xNames = {'Time(s)'};
% optPlotCell.yNames = {'vMag (m/s)'};
% optPlotCell.legNames = {{'Airspeed Meas', 'Airspeed Optimized', 'Inertial', 'Wind'}};
% optPlotCell.titleStr = 'Airspeed Magnitude';
% 
% for iSeg = 1:numSeg
%     PlotCell({seg{iSeg}.time_s}, ...
%         {{seg{iSeg}.vMeasMag_PA_mps, seg{iSeg}.vMag_BA_mps, seg{iSeg}.vMag_BE_mps, seg{iSeg}.vMag_AE_mps}}, ...
%         optPlotCell);
% end


%% Plot Inflow Conditions
for iSeg = 1:numSeg
    % Plot inflow conditions
    optPlotCell.xNames = {[], [], 'Time(s)'};
    optPlotCell.yNames = {'vMag (m/s)', 'Alpha (deg)', 'Beta (deg)'};
    optPlotCell.legNames = {{'Measured', 'Estimated', 'Inertial Derived, Mean Wind Compensated'},{'Measured', 'Estimated'},{'Measured', 'Estimated'}};
    optPlotCell.titleStr = 'Inflow Conditions';
end

for iSeg = 1:numSeg
    PlotCell({seg{iSeg}.time_s}, ...
        {{seg{iSeg}.vMeasMag_PA_mps, seg{iSeg}.vCalMag_BA_mps, seg{iSeg}.vMag_BA_mps}, ...
        {seg{iSeg}.alphaCal_BA_deg, seg{iSeg}.alpha_BA_deg}, ...
        {seg{iSeg}.betaCal_BA_deg, seg{iSeg}.beta_BA_deg}}, ...
        optPlotCell);
    hold on;
end


%% Airspeed Error Estimate
figure;
for iSeg = 1:numSeg
    % Plot Airspeed Error Estimate
    subplot(2,1,1)
    plot(seg{iSeg}.vMeasMag_PA_mps, seg{iSeg}.vMag_BA_mps, '*'); grid on;
    ylabel('Airpeed Estimate (m/s)');
    hold on;
    
    subplot(2,1,2)
    plot(seg{iSeg}.vMeasMag_PA_mps, seg{iSeg}.vCalErrorMag_BA_mps, '*'); grid on;
    ylabel('Airpeed Error RSS Estimate (m/s)');
    xlabel('Measured Airspeed (m/s)')
    hold on;
end
linkaxes(findobj(gcf, 'Type', 'axes'), 'x');

v = 0:1:50; 
subplot(2,1,1);
plot(v, max(AirCal(v, seg{1}.sNav_BL_rad, param{1})), 'k');

return
%% Save
save('Mjolnir_AirspeedCal.mat', 'param', 'seg', 'optParam');
