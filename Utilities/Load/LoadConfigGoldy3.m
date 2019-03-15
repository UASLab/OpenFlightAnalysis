function param = LoadConfigGoldy3(param)

%% Constants
r2d = 180/pi;
d2r = 1/r2d;
in2m = 0.0254;

%%
switch lower(param.vehName)
    case {'mjolnir'}
        switch lower(param.config)
            case {'1'}
                param.s_BL_rad.seq = '321';
                
                % Position and Orientation of Imu (Accel and Gyro)
                param.r_ImuB_B_m.data = [-5; 0; 0] * in2m; % Location of the IMU package wrt the Body Frame [Imu/B]B (m) FIXIT - Use Actual
                param.s_ImuB_rad.data = [0; 0; 0]; % Orientation (321) of the IMU package wrt Body Frame [Imu/B] (rad) FIXIT - Use Actual
                param.s_ImuB_rad.seq = '321'; % Orientation sequence definition
                
                % Accel Error Model ( a_meas = sf * a_true + bias )
                param.a_ImuI_Imu_mps2.errorType = 'ScaleBias-';
                param.a_ImuI_Imu_mps2.bias = [0; 0; 0];
                param.a_ImuI_Imu_mps2.K = eye(3);
                param.a_ImuI_Imu_mps2.vSigma = (1.0) * eye(3); % Noise Covariance
                
                % Gyro Error Model ( w_meas = sf * (w_true + G*a_true) + bias )
                param.w_ImuI_Imu_rps.errorType = 'Gyro-';
                param.w_ImuI_Imu_rps.bias = [0; 0; 0]; % Bias
                param.w_ImuI_Imu_rps.K = eye(3); % Scale Factor (Matrix)
                param.w_ImuI_Imu_rps.vSigma = (0.3 * d2r) * eye(3); % Noise Covariance
                param.w_ImuI_Imu_rps.G = zeros(3); % Linear Acceleration Sensitivity
                
                
                % Position and Orientation of Pitot-Probe
                param.r_PB_B_m.data = [16; -20; 0] * in2m; % Location of the Pitot probe wrt the Body Frame [P/B]B (m)
                param.s_PB_rad.data = [0; 0; 0]; % Orientation (321) of the Pitot probe wrt Body Frame [P/B] (rad)
                param.s_PB_rad.seq = '321'; % Orientation sequence definition
                
                % Pitot Error Model - FIXIT - Check Form
                param.v_PA_P_mps.errorType = 'ScaleBias+';
                param.v_PA_P_mps.bias = 0;
                param.v_PA_P_mps.K = 1;
                param.v_PA_P_mps.vSigma = 0.5; % Noise Covariance - FIXIT
                
                param.pPitotDiff_Pa.errorType = 'ScaleBias+';
                param.pPitotDiff_Pa.bias = 0;
                param.pPitotDiff_Pa.K = 1;
                param.pPitotDiff_Pa.vSigma = 0.5; % Noise Covariance - FIXIT
                
                % Static Error Model ( h_meas = K * h_true + bias ) - FIXIT - Check Form
                param.r_PA_P_m.errorType = 'ScaleBias+';
                param.r_PA_P_m.bias = 0;
                param.r_PA_P_m.K = 1;
                param.r_PA_P_m.vSigma = 1; % Noise Covariance - FIXIT
                
                param.pPitotStatic_Pa.errorType = 'ScaleBias+';
                param.pPitotStatic_Pa.bias = 0;
                param.pPitotStatic_Pa.K = 1;
                param.pPitotStatic_Pa.vSigma = 1; % Noise Covariance - FIXIT
                
                % Position and Orientation of 5Hole-Probe
                param.r_5B_B_m.data = [16; 20; 0] * in2m; % Location of the 5Hole probe wrt the Body Frame [5/B]B (m)
                param.s_5B_rad.data = [0; 0; 0]; % Orientation (321) of the 5Hole probe wrt Body Frame [5/B] (rad)
                param.s_5B_rad.seq = '321'; % Orientation sequence definition
                
                % Pitot Error Model - FIXIT - Check Form            
                param.p5Tip_Pa.errorType = 'ScaleBias+';
                param.p5Tip_Pa.bias = 0;
                param.p5Tip_Pa.K = 1;
                param.p5Tip_Pa.vSigma = 0.5; % Noise Covariance - FIXIT
                
                % Static Error Model - FIXIT - Check Form
                param.p5Static_Pa.errorType = 'ScaleBias+';
                param.p5Static_Pa.bias = 0;
                param.p5Static_Pa.K = 1;
                param.p5Static_Pa.vSigma = 1; % Noise Covariance - FIXIT
                
                % Alpha Error Model - FIXIT - Check Form
                param.p5Alpha_Pa.errorType = 'ScaleBias+';
                param.p5Alpha_Pa.bias = 0;
                param.p5Alpha_Pa.K = 0.0642;
                param.p5Alpha_Pa.vSigma = 1; % Noise Covariance - FIXIT
                
                % Alpha Error Model - FIXIT - Check Form
                param.p5Beta_Pa.errorType = 'ScaleBias+';
                param.p5Beta_Pa.bias = 0;
                param.p5Beta_Pa.K = 0.0642;
                param.p5Beta_Pa.vSigma = 1; % Noise Covariance - FIXIT
                
                % Position of GPS Antenna
                param.r_GpsB_B_m.data = [-12; 0; -2] * in2m; % Location of the GPS Ant wrt the Body Frame [Gps/B]B (m)
                
                % GPS Data Delay
                param.tDelay_Gps_s.errorType = 'Delay';
                param.tDelay_Gps_s.data = -0.250; % GPS time delay relative to IMU, negative indicates the log data for GPS is more delayed
                
                % GPS Position Error Model ( r_meas = r_true + bias )
                param.r_GpsE_D_ddm.errorType = 'bias-';
                param.r_GpsE_D_ddm.bias = [0; 0; 0]; % GPS Position Bias
                param.r_GpsE_D_ddm.vSigma = diag([0.5, 0.5, 0.5]); % GPS Position Noise Covariance
                
                % GPS Velocity Error Model ( v_meas = v_true + bias )
                param.v_GpsE_L_mps.errorType = 'bias-';
                param.v_GpsE_L_mps.bias = [0; 0; 0]; % GPS Position Bias
                param.v_GpsE_L_mps.vSigma = diag([3, 3, 5]); % GPS Position Noise Covariance
                
            otherwise
                disp('Configuration for aircraft unknown')
                
        end
    case {'thor'}
        switch lower(param.config)
            case {'2'}
                param.s_BL_rad.seq = '321';
                
                % Position and Orientation of Imu (Accel and Gyro)
                param.r_ImuB_B_m.data = [-2; 0; 0] * in2m; % Location of the IMU package wrt the Body Frame [Imu/B]B (m) FIXIT - Use Actual
                param.s_ImuB_rad.data = [0; 0; 0]; % Orientation (321) of the IMU package wrt Body Frame [Imu/B] (rad) FIXIT - Use Actual
                param.s_ImuB_rad.seq = '321'; % Orientation sequence definition
                
                % Accel Error Model ( a_meas = sf * a_true + bias )
                param.a_ImuI_Imu_mps2.errorType = 'ScaleBias-';
                param.a_ImuI_Imu_mps2.bias = [0; 0; 0];
                param.a_ImuI_Imu_mps2.K = eye(3);
                param.a_ImuI_Imu_mps2.vSigma = (1.0) * eye(3); % Noise Covariance
                
                % Gyro Error Model ( w_meas = sf * (w_true + G*a_true) + bias )
                param.w_ImuI_Imu_rps.errorType = 'Gyro-';
                param.w_ImuI_Imu_rps.bias = [0; 0; 0]; % Bias
                param.w_ImuI_Imu_rps.K = eye(3); % Scale Factor (Matrix)
                param.w_ImuI_Imu_rps.vSigma = (0.3 * d2r) * eye(3); % Noise Covariance
                param.w_ImuI_Imu_rps.G = zeros(3); % Linear Acceleration Sensitivity
                
                
                % Position and Orientation of Pitot-Probe
                param.r_PB_B_m.data = [10; -16; 0] * in2m; % Location of the Pitot probe wrt the Body Frame [P/B]B (m)
                param.s_PB_rad.data = [0; 0; 0]; % Orientation (321) of the Pitot probe wrt Body Frame [P/B] (rad)
                param.s_PB_rad.seq = '321'; % Orientation sequence definition
                
                % Pitot Error Model - FIXIT - Check Form
                param.v_PA_P_mps.errorType = 'ScaleBias+';
                param.v_PA_P_mps.bias = 0;
                param.v_PA_P_mps.K = 1;
                param.v_PA_P_mps.vSigma = 0.5; % Noise Covariance - FIXIT
                
                param.pPitotDiff_Pa.errorType = 'ScaleBias+';
                param.pPitotDiff_Pa.bias = 0;
                param.pPitotDiff_Pa.K = 1;
                param.pPitotDiff_Pa.vSigma = 0.5; % Noise Covariance - FIXIT
                
                % Static Error Model ( h_meas = K * h_true + bias ) - FIXIT - Check Form
                param.r_PA_P_m.errorType = 'ScaleBias+';
                param.r_PA_P_m.bias = 0;
                param.r_PA_P_m.K = 1;
                param.r_PA_P_m.vSigma = 1; % Noise Covariance - FIXIT
                
                param.pPitotStatic_Pa.errorType = 'ScaleBias+';
                param.pPitotStatic_Pa.bias = 0;
                param.pPitotStatic_Pa.K = 1;
                param.pPitotStatic_Pa.vSigma = 1; % Noise Covariance - FIXIT
       
                % Position of GPS Antenna
                param.r_GpsB_B_m.data = [-8; 0; -1.5] * in2m; % Location of the GPS Ant wrt the Body Frame [Gps/B]B (m)
                
                % GPS Data Delay
                param.tDelay_Gps_s.errorType = 'Delay';
                param.tDelay_Gps_s.data = -0.250; % GPS time delay relative to IMU, negative indicates the log data for GPS is more delayed
                
                % GPS Position Error Model ( r_meas = r_true + bias )
                param.r_GpsE_D_ddm.errorType = 'bias-';
                param.r_GpsE_D_ddm.bias = [0; 0; 0]; % GPS Position Bias
                param.r_GpsE_D_ddm.vSigma = diag([0.5, 0.5, 0.5]); % GPS Position Noise Covariance
                
                % GPS Velocity Error Model ( v_meas = v_true + bias )
                param.v_GpsE_L_mps.errorType = 'bias-';
                param.v_GpsE_L_mps.bias = [0; 0; 0]; % GPS Position Bias
                param.v_GpsE_L_mps.vSigma = diag([3, 3, 5]); % GPS Position Noise Covariance
                
            otherwise
                disp('Configuration for aircraft unknown')
                
        end
    otherwise
        disp('Aircraft Name unknown');
        
end