function [structFDAT] = StructFlightGoldy3(structRaw, fileVers)
% Create the FDAT structure from the Goldy3 Raw structure
%
%Inputs:
% structRaw   - Raw flight structure
%
%Outputs:
% structFDAT - Structure of file variables
%
%Notes:
% Any Special Notes.
%

%Version History: Version 1.0
% 01/16/2018  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(1, 2);

% Default Values
    
% Check the number of outputs
nargoutchk(0, 1);


%% Constants
r2d = 180/pi;
uT2G = 0.01;


%% Load the appropriate function for the type of flight computer
structFDAT.raw = structRaw;
structFDAT = structFDAT.raw;

switch fileVers
    case '1'
        % Time
        structFDAT.time_s = double(structRaw.Fmu.Time_us) / 1e6;
        
        % Effectors
        
        % Pitot-Static
        structFDAT.Pitot.pDiff_Pa = structRaw.Pitot_0.Diff.Pressure_Pa;
        structFDAT.Pitot.pStatic_Pa = structRaw.Pitot_0.Static.Pressure_Pa;
        structFDAT.Pitot.temp_C = mean([structRaw.Pitot_0.Diff.Temp_C; structRaw.Pitot_0.Static.Temp_C]);
        
        % Airdata
        structFDAT.Airdata.vIas_mps = structRaw.Airdata.vIas_mps;
        structFDAT.Airdata.alt_m = structRaw.Airdata.alt_m;
        structFDAT.Airdata.temp_C = structRaw.Airdata.temp_C;
        
        % Controllers
        
        % GPS
        structFDAT.Gps.updateFlag_nd = [0, diff(structRaw.Gps_0.Sec)]>0;
        structFDAT.Gps.r_BE_G_ddm = structRaw.Gps_0.LLA .* [r2d; r2d; 1];
        structFDAT.Gps.v_BE_L_mps = structRaw.Gps_0.NEDVelocity_ms;
        
        
        % Navigation Estimation (EKF)
        structFDAT.NavFilter.aBias_mps2 = structRaw.NavFilter.AccelBias_mss;
        structFDAT.NavFilter.wBias_rps = structRaw.NavFilter.GyroBias_rads;
        
        structFDAT.NavFilter.r_BE_G_ddm = structRaw.NavFilter.LLA .* [r2d; r2d; 1];
        structFDAT.NavFilter.v_BE_L_mps = structRaw.NavFilter.NEDVelocity_ms;
        
        structFDAT.NavFilter.a_mps2 = structRaw.Mpu9250.Accel_mss;
        structFDAT.NavFilter.w_rps = structRaw.Mpu9250.Gyro_rads(1,:);
        structFDAT.NavFilter.s_rad = structRaw.NavFilter.Euler_rad;
        
        % IMU
        structFDAT.Imu.a_mps2 = structRaw.Mpu9250.Accel_mss - structRaw.NavFilter.AccelBias_mss;
        structFDAT.Imu.w_rps = structRaw.Mpu9250.Gyro_rads - structFDAT.NavFilter.wBias_rps;
        structFDAT.Imu.mag_uT = structRaw.Mpu9250.Mag_uT;
        
        % Instrumentation
        structFDAT.P5Hole.pTip_Pa = structRaw.presTip_Pa.Pressure_Pa;
        structFDAT.P5Hole.pStatic_Pa = structRaw.presStatic_Pa.Pressure_Pa;
        structFDAT.P5Hole.pAlpha_Pa = structRaw.presAlpha_Pa.Pressure_Pa;
        structFDAT.P5Hole.pBeta_Pa = structRaw.presBeta_Pa.Pressure_Pa;
        structFDAT.P5Hole.temp_C = mean([structRaw.presTip_Pa.Temp_C; structRaw.presStatic_Pa.Temp_C; structRaw.presAlpha_Pa.Temp_C; structRaw.presBeta_Pa.Temp_C]);
        
        % Mission
        
        % Pilot Commands
        structFDAT.Pilot = structRaw.SbusRx_0;
        
        
        % System Identification and Excitation
        
    case '2'
        
        % Time
        structFDAT.time_s = double(structRaw.Sensors.Fmu.Time_us) / 1e6;
        
        % Effectors
        
        % Pitot-Static
        structFDAT.Pitot.pDiff_Pa = structRaw.Sensors.Swift.Differential.Pressure_Pa;
        structFDAT.Pitot.pStatic_Pa = structRaw.Sensors.Swift.Static.Pressure_Pa;
        structFDAT.Pitot.temp_C = mean([structRaw.Sensors.Swift.Differential.Temperature_C; structRaw.Sensors.Swift.Static.Temperature_C]);
        
        % Airdata
        structFDAT.Airdata.vIas_mps = structRaw.Sensor_Processing.vIAS_ms;
        structFDAT.Airdata.alt_m = structRaw.Sensor_Processing.hBaro_m;
        
        % Controllers
        structFDAT.Control.refPhi_rad = structRaw.Control.refPhi_rad;
        structFDAT.Control.refTheta_rad = structRaw.Control.refTheta_rad;
        structFDAT.Control.refPsi_rad = structRaw.Control.refPsi_rad;
        structFDAT.Control.refV_ms = structRaw.Control.refV_ms;
        structFDAT.Control.refH_m = structRaw.Control.refH_m;
        
        structFDAT.Control.cmdEff = [structRaw.Control.cmdMotor_nd; structRaw.Control.cmdElev_rad; structRaw.Control.cmdRud_rad; structRaw.Control.cmdAilL_rad; structRaw.Control.cmdFlapL_rad; structRaw.Control.cmdFlapR_rad; structRaw.Control.cmdAilR_rad]; 
        
        % GPS
        structFDAT.Gps.r_BE_G_ddm = [structRaw.Sensors.uBlox.Latitude_rad * r2d; structRaw.Sensors.uBlox.Longitude_rad * r2d; structRaw.Sensors.uBlox.Altitude_m];
        structFDAT.Gps.v_BE_L_mps = [structRaw.Sensors.uBlox.NorthVelocity_ms; structRaw.Sensors.uBlox.EastVelocity_ms; structRaw.Sensors.uBlox.DownVelocity_ms];
        
        % Navigation Estimation (EKF)
        structFDAT.NavFilter.aBias_mps2 = [structRaw.Sensor_Processing.AccelXBias_mss; structRaw.Sensor_Processing.AccelYBias_mss; structRaw.Sensor_Processing.AccelZBias_mss];
        structFDAT.NavFilter.wBias_rps = [structRaw.Sensor_Processing.GyroXBias_rads; structRaw.Sensor_Processing.GyroYBias_rads; structRaw.Sensor_Processing.GyroZBias_rads];
        
        structFDAT.NavFilter.r_BE_G_ddm = [structRaw.Sensor_Processing.Latitude_rad * r2d; structRaw.Sensor_Processing.Longitude_rad * r2d; structRaw.Sensor_Processing.Altitude_m];
        structFDAT.NavFilter.v_BE_L_mps = [structRaw.Sensor_Processing.NorthVelocity_ms; structRaw.Sensor_Processing.EastVelocity_ms; structRaw.Sensor_Processing.DownVelocity_ms];
        
        structFDAT.NavFilter.a_mps2 = [structRaw.Sensor_Processing.AccelX_mss; structRaw.Sensor_Processing.AccelY_mss; structRaw.Sensor_Processing.AccelZ_mss];
        structFDAT.NavFilter.w_rps = [structRaw.Sensor_Processing.GyroX_rads; structRaw.Sensor_Processing.GyroY_rads; structRaw.Sensor_Processing.GyroZ_rads];
        structFDAT.NavFilter.s_rad = [structRaw.Sensor_Processing.Roll_rad; structRaw.Sensor_Processing.Pitch_rad; structRaw.Sensor_Processing.Heading_rad];
        
        % IMU
        structFDAT.Imu.a_mps2 = structFDAT.NavFilter.a_mps2 - structFDAT.NavFilter.aBias_mps2;
        structFDAT.Imu.w_rps = structFDAT.NavFilter.w_rps - structFDAT.NavFilter.wBias_rps;
        structFDAT.Imu.mag_uT = [structRaw.Sensors.Fmu.Mpu9250.MagX_uT; structRaw.Sensors.Fmu.Mpu9250.MagY_uT; structRaw.Sensors.Fmu.Mpu9250.MagZ_uT];
        
        % Mission
        structFDAT.Mission.socEngage = structRaw.Mission.socEngage;
        structFDAT.Mission.ctrlSel = structRaw.Mission.ctrlSel;
        structFDAT.Mission.testID = structRaw.Mission.testID;
        structFDAT.Mission.excitEngage = structRaw.Mission.excitEngage;
        
        % Pilot Commands
        structFDAT.Pilot = structRaw.Sensors.Sbus;
        
        % System Identification and Excitation
        
end

%% Outputs


