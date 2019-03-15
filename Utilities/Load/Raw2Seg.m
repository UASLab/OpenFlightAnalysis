function [param, seg] = Raw2Seg(flt, param)

% Copy Param
param = param;

% Number of Segments
numSeg = length(param);

for iSeg = 1:numSeg
    % Store the length of the segment
    param{iSeg}.lenSeg = length(param{iSeg}.indxSeg);
    
    % Associate and Vectorize the flight data, based on the time slices
    seg{iSeg}.time_s = flt{param{iSeg}.indxFlt}.sig.time_s(param{iSeg}.indxSeg)';
    
    % Associate IMU data
    seg{iSeg}.wBias_rps = [flt{param{iSeg}.indxFlt}.sig.nav.pBias_rps(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.nav.qBias_rps(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.nav.rBias_rps(param{iSeg}.indxSeg)]'; % Gyroscope bias estimates (rad/s)
    seg{iSeg}.aBias_mps2 = [flt{param{iSeg}.indxFlt}.sig.nav.accelXBias_mps2(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.nav.accelYBias_mps2(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.nav.accelZBias_mps2(param{iSeg}.indxSeg)]'; % Accelerometer bias estimates [Accel/I]Accel (m/s^2)
    
    seg{iSeg}.wMeas_ImuI_Imu_rps = [flt{param{iSeg}.indxFlt}.sig.imu.p_rps(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.imu.q_rps(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.imu.r_rps(param{iSeg}.indxSeg)]' - seg{iSeg}.wBias_rps; % Gyroscope measurements [Gyro/I]Gyro (rad/s)
    seg{iSeg}.aMeas_ImuI_Imu_mps2 = [flt{param{iSeg}.indxFlt}.sig.imu.accelX_mps2(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.imu.accelY_mps2(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.imu.accelZ_mps2(param{iSeg}.indxSeg)]' - seg{iSeg}.aBias_mps2; % Accelerometer measurements [Accel/I]Accel (m/s^2)
    
    % Associate EKF Navigation data
    seg{iSeg}.sNav_BL_rad = [flt{param{iSeg}.indxFlt}.sig.imu.phi_rad(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.imu.theta_rad(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.imu.psi_rad(param{iSeg}.indxSeg)]';
    seg{iSeg}.vNav_BE_L_mps = [flt{param{iSeg}.indxFlt}.sig.nav.velN_mps(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.nav.velE_mps(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.nav.velD_mps(param{iSeg}.indxSeg)]';
    seg{iSeg}.rNav_BE_D_ddm = [flt{param{iSeg}.indxFlt}.sig.nav.lat_deg(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.nav.lon_deg(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.nav.alt_m(param{iSeg}.indxSeg)]'; % Nav Position Solution [GPS/D]D (deg,deg,m)
    
    % Associate Pitot-Static Airspeed measurement
    seg{iSeg}.vMeasMag_PA_mps = flt{param{iSeg}.indxFlt}.sig.aird.ias_mps(param{iSeg}.indxSeg)';
    seg{iSeg}.rMeasMag_PA_m = flt{param{iSeg}.indxFlt}.sig.aird.h_m(param{iSeg}.indxSeg)';
    
    % Associate GPS Measurements
    seg{iSeg}.rMeas_GpsE_D_ddm = [flt{param{iSeg}.indxFlt}.sig.gps.lat_deg(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.gps.lon_deg(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.gps.alt_m(param{iSeg}.indxSeg)]'; % GPS Position Measurements [GPS/D]D (deg,deg,m)
    seg{iSeg}.vMeas_GpsE_L_mps = [flt{param{iSeg}.indxFlt}.sig.gps.velN_mps(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.gps.velE_mps(param{iSeg}.indxSeg), flt{param{iSeg}.indxFlt}.sig.gps.velD_mps(param{iSeg}.indxSeg)]'; % GPS Velocity Measurements [GPS/L]L (m/s)
end
