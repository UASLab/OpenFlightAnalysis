function [s_BL_rad, v_BE_L_mps, r_BE_L_m, w_BL_B_rps, a_BE_B_mps2] = ImuCal(time_s, aMeas_ImuI_Imu_mps2, wMeas_ImuI_Imu_rps, param)

    %% Apply Error Models, Estimate the "true" measures from the Error Models
    % Apply Gyro Error Models
    [a_ImuI_Imu_mps2] = SensorErrorModel(aMeas_ImuI_Imu_mps2, param.a_ImuI_Imu_mps2);
    
    % Apply the Accelerometer Error Models
    param.w_ImuI_Imu_rps.aTrue = a_ImuI_Imu_mps2; % Use the "true" accel for the gyro linear sensitivity
    [w_ImuI_Imu_rps] = SensorErrorModel(wMeas_ImuI_Imu_rps, param.w_ImuI_Imu_rps);

    % Integrate the IMU data to calculate the Euler Angles, Inertial Velocities, and Change in Position
    [s_BL_rad, v_BE_L_mps, r_BE_L_m, w_BL_B_rps, a_BE_B_mps2] = ...
        IntImu(time_s, w_ImuI_Imu_rps, a_ImuI_Imu_mps2, param.s_BL_rad.init, param.v_BE_L_mps.init, param.r_BE_L_m.init, param.r_ImuB_B_m.data, param.s_ImuB_rad.data);
