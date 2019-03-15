
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
flt{iFlt}.param.apVers = '10';

indxFlt = indxFlt + 1;
flt{indxFlt}.param.vehName = 'Geri';
flt{indxFlt}.param.apType = 'Goldy1';
flt{indxFlt}.param.fltNum = 4;
flt{indxFlt}.param.config = '1';
flt{iFlt}.param.apVers = '10';

indxFlt = indxFlt + 1;
flt{indxFlt}.param.vehName = 'Geri';
flt{indxFlt}.param.apType = 'Goldy1';
flt{indxFlt}.param.fltNum = 5;
flt{indxFlt}.param.config = '1';
flt{iFlt}.param.apVers = '10';

numFlt = length(flt);
for iFlt = 1:numFlt
    % Generate the file name and full file path
    flt{iFlt}.param.fltName = [flt{iFlt}.param.vehName, 'FLT', num2str(flt{iFlt}.param.fltNum, '%0.2d')];
    flt{iFlt}.param.fltFile = fullfile(pathDataRoot, [flt{iFlt}.param.fltName, '\', flt{iFlt}.param.fltName, '.mat']);

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
iSeg = 1; iFlt = 1;
param{iSeg} = flt{iFlt}.param;
param{iSeg}.indxFlt = iFlt;
param{iSeg}.indxSeg = flt{iFlt}.seg.turn{2}.indx;

iSeg = iSeg+1; iFlt = 1;
param{iSeg} = flt{iFlt}.param;
param{iSeg}.indxFlt = iFlt;
param{iSeg}.indxSeg = flt{iFlt}.seg.turn{3}.indx;

iSeg = iSeg+1; iFlt = 2;
param{iSeg} = flt{iFlt}.param;
param{iSeg}.indxFlt = iFlt;
param{iSeg}.indxSeg = flt{iFlt}.seg.turn{2}.indx;

iSeg = iSeg+1; iFlt = 2;
param{iSeg} = flt{iFlt}.param;
param{iSeg}.indxFlt = iFlt;
param{iSeg}.indxSeg = flt{iFlt}.seg.turn{3}.indx;

iSeg = iSeg+1; iFlt = 3;
param{iSeg} = flt{iFlt}.param;
param{iSeg}.indxFlt = iFlt;
param{iSeg}.indxSeg = flt{iFlt}.seg.turn{2}.indx;

iSeg = iSeg+1; iFlt = 3;
param{iSeg} = flt{iFlt}.param;
param{iSeg}.indxFlt = iFlt;
param{iSeg}.indxSeg = flt{iFlt}.seg.turn{3}.indx;

% Number of Segments
numSeg = length(param);


%% Load Flight Data and Configuration Parameters
[param, seg] = Raw2Seg(flt, param);


%% Pre-Optimization, Initial Guess for the Wind
for iSeg = 1:numSeg
    % Transform Pitot Measures
    param{iSeg}.v_PA_P_mps.errorType = 'ScaleBias+';
    param{iSeg}.v_PA_P_mps.bias = -3.8;
    param{iSeg}.v_PA_P_mps.K = 1.3;

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
iOptStatic = 1;
% optParam.static{iOptStatic}.field = 's_PB_rad.data';
% optParam.static{iOptStatic}.len = 3;
% optParam.static{iOptStatic}.ub = 10*d2r*[1;1;1]; optParam.static{iOptStatic}.lb = -optParam.static{iOptStatic}.ub;

% iOptStatic = iOptStatic + 1;
optParam.static{iOptStatic}.field = 'v_PA_P_mps.K';
optParam.static{iOptStatic}.len = 1;
% optParam.static{iOptStatic}.ub = 10000; optParam.static{iOptStatic}.lb = -optParam.static{iOptStatic}.ub;

% iOptStatic = iOptStatic + 1;
% optParam.static{iOptStatic}.field = 'v_PA_P_mps.Quad';
% optParam.static{iOptStatic}.len = 1;

iOptStatic = iOptStatic + 1;
optParam.static{iOptStatic}.field = 'v_PA_P_mps.bias';
optParam.static{iOptStatic}.len = 1;

% Tuning Parameters for each Segment
iOptSeg = 1;
optParam.seg{iOptSeg}.field = 'v_AE_L_mps.mean';
% optParam.seg{iOptSeg}.ub = 10*[1;1;1]; optParam.seg{iOptSeg}.lb = -optParam.seg{iOptSeg}.ub;
optParam.seg{iOptSeg}.len = 3;


% Pack the optParam into a vector
[optParam, param] = DefOptParam(optParam, param, 'pack');
optParamVal = optParam.val;


% Setup the Optimizer
optProb.objective = @(optParamVal) AirCalCost(seg, param, optParam, optParamVal);
optProb.x0 = optParam.val;
optProb.lb = optParam.lb;
optProb.ub = optParam.ub;
optProb.solver = 'fmincon';
optProb.options = optimoptions(optProb.solver);
optProb.options.Algorithm = 'sqp';
optProb.options.MaxIter = 2;
% optProb.options.Display = 'iter';
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
    seg{iSeg}.v_AE_L_mps = seg{iSeg}.vEst_BE_L_mps - seg{iSeg}.vCal_BA_L_mps;
    
    % Compute the True Air Velocity, True = Nav - MeanWind
    seg{iSeg}.v_BA_L_mps = seg{iSeg}.vNav_BE_L_mps - seg{iSeg}.v_AE_L_mps;
%     seg{iSeg}.v_BA_L_mps = seg{iSeg}.vEst_BE_L_mps - seg{iSeg}.v_AE_L_mps;
    
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
    
    % Compute Inflow Angles, Angle of Attack and Angle of Sideslip
    seg{iSeg}.alpha_BA_deg = atan2d(seg{iSeg}.vCal_BA_B_mps(3,:), seg{iSeg}.vCal_BA_B_mps(1,:));
    seg{iSeg}.beta_BA_deg = asind(seg{iSeg}.vCal_BA_B_mps(2,:) ./ seg{iSeg}.vCalMag_BA_mps);
    
end

%%
% Plot Ground Speed Estimate
optPlotCell.xNames = {[], [], 'Time(s)'};
optPlotCell.yNames = {'vNorth (m/s)', 'vEast (m/s)', 'vDown (m/s)'};
optPlotCell.legNames = {{'GPS', 'Optimized Inertial', 'Optimized Air'}, {}, {}};
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
% optPlotCell.legNames = {{'Airspeed Meas', 'Airspeed Optimized', 'Inertial', 'Wind'}, {}, {}};
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
    optPlotCell.yNames = {'vMag (m/s)', 'Alpha (deg', 'Beta (deg)'};
    optPlotCell.legNames = {{'Measured', 'Calibrated', 'Inertial Derived, Mean Wind Compensated'},{},{}};
    optPlotCell.titleStr = 'Inflow Conditions';
end

for iSeg = 1:numSeg
    PlotCell({seg{iSeg}.time_s}, ...
        {{seg{iSeg}.vMeasMag_PA_mps, seg{iSeg}.vCalMag_BA_mps, seg{iSeg}.vMag_BA_mps}, ...
        {seg{iSeg}.alpha_BA_deg}, ...
        {seg{iSeg}.beta_BA_deg}}, ...
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

v = 10:0.1:35; 
subplot(2,1,1);
plot(v, SensorErrorModel(v,  param{iSeg}.v_PA_P_mps), 'k');

%% Save
% save('GeriFLT_AirCal.mat', 'param', 'seg', 'optParam');
