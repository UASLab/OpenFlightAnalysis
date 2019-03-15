function [cost, seg, param] = AirCalCost(seg, param, optParam, optParamVal)

% Number of segments to use in the cost function
numSeg = length(seg);

% Copy the seg structure
seg = seg;

%% Parse the Parameters
optParam.val = optParamVal;
[~, param] = DefOptParam(optParam, param, 'unpack');

%%
parfor iSeg = 1:numSeg
    % Compute the Airspeed solution with error model parameters
    [seg{iSeg}.vCal_BA_B_mps, seg{iSeg}.vCal_BA_L_mps] = ...
        AirCal(seg{iSeg}.vMeasMag_PA_mps, seg{iSeg}.sNav_BL_rad, param{iSeg});
    
    % Compute the Ground Speeds
    % Wind Estimate, assume constant at mean value
    seg{iSeg}.vMean_AE_L_mps = repmat(param{iSeg}.v_AE_L_mps.mean, 1, param{iSeg}.lenSeg);
    
    % Compute the Groudspeeds from the Corrected Airspeeds using the Wind Estimate
    seg{iSeg}.vEst_BE_L_mps = seg{iSeg}.vMean_AE_L_mps + seg{iSeg}.vCal_BA_L_mps;

    % Cost for each segment
    costSeg(iSeg) = norm(seg{iSeg}.vNav_BE_L_mps - seg{iSeg}.vEst_BE_L_mps, 2) / param{iSeg}.lenSeg;
end

% Total Cost
cost = norm(costSeg, 2);

