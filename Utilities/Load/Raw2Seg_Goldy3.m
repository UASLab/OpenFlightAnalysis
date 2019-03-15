function [param, seg] = Raw2Seg_Goldy3(flt, param)

% Copy Param
param = param;

% Number of Segments
numSeg = length(param);

for iSeg = 1:numSeg
    % Store the length of the segment
    param{iSeg}.lenSeg = length(param{iSeg}.indxSeg);
    
    % Associate and Vectorize the flight data, based on the time slices
    seg{iSeg}.time_s = flt{param{iSeg}.indxFlt}.sig.time_s(param{iSeg}.indxSeg);
    seg{iSeg}.tStep_s = median(diff(seg{iSeg}.time_s));
    
    % Associate IMU data
    seg{iSeg}.wBias_rps = double(flt{param{iSeg}.indxFlt}.sig.NavFilter.wBias_rps(:, param{iSeg}.indxSeg)); % Gyroscope bias estimates (rad/s)
    seg{iSeg}.aBias_mps2 = double(flt{param{iSeg}.indxFlt}.sig.NavFilter.aBias_mps2(:, param{iSeg}.indxSeg)); % Accelerometer bias estimates [Accel/I]Accel (m/s^2)
    
    seg{iSeg}.wMeas_ImuI_Imu_rps = double(flt{param{iSeg}.indxFlt}.sig.Imu.w_rps(:, param{iSeg}.indxSeg)); % Gyroscope measurements [Gyro/I]Gyro (rad/s)
    seg{iSeg}.aMeas_ImuI_Imu_mps2 = double(flt{param{iSeg}.indxFlt}.sig.Imu.a_mps2(:, param{iSeg}.indxSeg)); % Accelerometer measurements [Accel/I]Accel (m/s^2)
    
    % Associate EKF Navigation data
    seg{iSeg}.sNav_BL_rad = double(flt{param{iSeg}.indxFlt}.sig.NavFilter.s_rad(:, param{iSeg}.indxSeg));
    param{iSeg}.sNav_BL_rad.seq = '321';
    seg{iSeg}.vNav_BE_L_mps = double(flt{param{iSeg}.indxFlt}.sig.NavFilter.v_BE_L_mps(:, param{iSeg}.indxSeg));
    seg{iSeg}.rNav_BE_D_ddm = flt{param{iSeg}.indxFlt}.sig.NavFilter.r_BE_G_ddm(:, param{iSeg}.indxSeg); % Nav Position Solution [GPS/D]D (deg,deg,m)
    
    % Associate Pitot-Static Airspeed measurement
    seg{iSeg}.pPitotDiff_Pa = double(flt{param{iSeg}.indxFlt}.sig.Pitot.pDiff_Pa(param{iSeg}.indxSeg));
    seg{iSeg}.pPitotStatic_Pa = double(flt{param{iSeg}.indxFlt}.sig.Pitot.pStatic_Pa(param{iSeg}.indxSeg));
    
    seg{iSeg}.vMeasMag_PA_mps = double(flt{param{iSeg}.indxFlt}.sig.Airdata.vIas_mps(param{iSeg}.indxSeg));
    seg{iSeg}.rMeasMag_PA_m = double(flt{param{iSeg}.indxFlt}.sig.Airdata.alt_m(param{iSeg}.indxSeg));
    
    % Associate GPS Measurements
    seg{iSeg}.rMeas_GpsE_D_ddm = flt{param{iSeg}.indxFlt}.sig.Gps.r_BE_G_ddm(:, param{iSeg}.indxSeg); % GPS Position Measurements [GPS/D]D (deg,deg,m)
    seg{iSeg}.vMeas_GpsE_L_mps = flt{param{iSeg}.indxFlt}.sig.Gps.v_BE_L_mps(:, param{iSeg}.indxSeg); % GPS Velocity Measurements [GPS/L]L (m/s)
    
    % Control
    seg{iSeg}.refPhi_rad = double(flt{param{iSeg}.indxFlt}.sig.Control.refPhi_rad(:, param{iSeg}.indxSeg));
    seg{iSeg}.refTheta_rad = double(flt{param{iSeg}.indxFlt}.sig.Control.refTheta_rad(:, param{iSeg}.indxSeg));
    
    seg{iSeg}.cmdEff = double(flt{param{iSeg}.indxFlt}.sig.Control.cmdEff(:, param{iSeg}.indxSeg));

    % Excitation
    seg{iSeg}.excitePhi_rad = double(flt{param{iSeg}.indxFlt}.sig.Excitation.RTSM.refPhi_rad(:, param{iSeg}.indxSeg));
    seg{iSeg}.exciteTheta_rad = double(flt{param{iSeg}.indxFlt}.sig.Excitation.RTSM.refTheta_rad(:, param{iSeg}.indxSeg));
    
    
%     seg{iSeg}.rMeas_GpsE_D_ddm = [flt{param{iSeg}.indxFlt}.sig.gps.lat_deg(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.gps.lon_deg(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.gps.alt_m(param{iSeg}.indxSeg)]'; % GPS Position Measurements [GPS/D]D (deg,deg,m)
%     seg{iSeg}.vMeas_GpsE_L_mps = [flt{param{iSeg}.indxFlt}.sig.gps.velN_mps(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.gps.velE_mps(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.gps.velD_mps(param{iSeg}.indxSeg)]'; % GPS Velocity Measurements [GPS/L]L (m/s)
end
