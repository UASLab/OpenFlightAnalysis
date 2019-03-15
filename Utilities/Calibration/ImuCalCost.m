function [cost, seg, param] = ImuCalCost(seg, param, optParam, optParamVal)

% Number of segments to use in the cost function
numSeg = length(seg);

% Copy the seg structure
seg = seg;

%% Parse the Parameters
optParam.val = optParamVal;
[~, param] = DefOptParam(optParam, param, 'unpack');

%%
parfor iSeg = 1:numSeg
    % Compute the Integrated IMU solution with error model parameters
    [seg{iSeg}.s_BL_rad, seg{iSeg}.v_BE_L_mps, seg{iSeg}.r_BE_L_m, seg{iSeg}.w_BL_B_rps, seg{iSeg}.a_BL_B_mps2] = ...
        ImuCal(seg{iSeg}.time_s, seg{iSeg}.aMeas_ImuI_Imu_mps2, seg{iSeg}.wMeas_ImuI_Imu_rps, param{iSeg});
    
    % Apply GPS Error Models
    param{iSeg}.frameDelay_cnt.errorType = param{iSeg}.tDelay_Gps_s.errorType;
    param{iSeg}.frameDelay_cnt.data = param{iSeg}.tDelay_Gps_s.data / seg{iSeg}.tStep_s;
    
    [seg{iSeg}.rEstDelay_GpsE_L_m] = SensorErrorModel(seg{iSeg}.rEst_GpsE_L_m, param{iSeg}.frameDelay_cnt);
    [seg{iSeg}.vEstDelay_GpsE_L_mps] = SensorErrorModel(seg{iSeg}.vEst_GpsE_L_mps, param{iSeg}.frameDelay_cnt);
        
    % Calculate the Cost for each segment
    costSeg(iSeg) = norm(seg{iSeg}.rEstDelay_GpsE_L_m - seg{iSeg}.r_BE_L_m, 2) / param{iSeg}.lenSeg;
end

% Total Cost
cost = norm(costSeg, 2);

