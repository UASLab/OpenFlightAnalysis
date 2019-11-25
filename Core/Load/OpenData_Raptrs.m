function [oData] = OpenData_Raptrs(logData, config)
% Create the oData structure from the Goldy3/Raptrs structures
%
%Inputs:
% logData - Raw flight log structure
% config - Flight Configuration structure
%
%Outputs:
% oData - Structure of file variables
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan

%% Check I/O Arguments and Defaults
narginchk(1, 2);
nargoutchk(1, 1);


%% Constants
r2d = 180/pi;
uT2G = 0.01;


%% Load the appropriate function for the type of flight computer
oData = struct();
        
% Time
oData.time_us = double(logData.Sensors.Fmu.Time_us);
oData.time_s = oData.time_us / 1e6;
oData.dt_s = [0, diff(oData.time_s)];

%% Sensors
if isfield(logData.Sensor_Processing, 'Standard')
    logData.Sensor_Processing = logData.Sensor_Processing.Standard;
end


% Pitot-Static
if isfield(logData.Sensors, 'Swift')
    oData.pTip_Pa = logData.Sensors.Swift.Differential.Pressure_Pa;
    oData.pStatic_Pa = logData.Sensors.Swift.Static.Pressure_Pa;
    oData.temp_C = mean([logData.Sensors.Swift.Differential.Temperature_C; logData.Sensors.Swift.Static.Temperature_C]);
    
elseif isfield(logData.Sensors, 'Pitot')
    oData.pTip_Pa = logData.Sensors.Pitot.Differential.Pressure_Pa;
    oData.pStatic_Pa = logData.Sensors.Pitot.Static.Pressure_Pa;
    oData.temp_C = mean([logData.Sensors.Pitot.Differential.Temperature_C; logData.Sensors.Pitot.Static.Temperature_C]);
    
end

% 5-Hole
if isfield(logData.Sensors, 'FiveHole')
    if isfield(logData.Sensors.FiveHole, 'Alpha1')
        oData.pAlpha1_Pa = logData.Sensors.FiveHole.Alpha1.Pressure_Pa;
        oData.pAlpha2_Pa = logData.Sensors.FiveHole.Alpha2.Pressure_Pa;
        oData.pBeta1_Pa = logData.Sensors.FiveHole.Beta1.Pressure_Pa;
        oData.pBeta2_Pa = logData.Sensors.FiveHole.Beta2.Pressure_Pa;
        oData.pStatic_Pa = logData.Sensors.FiveHole.Static.Pressure_Pa;
        oData.pTip_Pa = logData.Sensors.FiveHole.Tip.Pressure_Pa;
        oData.tempProbe_C = mean([
            logData.Sensors.FiveHole.Alpha1.Temperature_C;
            logData.Sensors.FiveHole.Alpha2.Temperature_C;
            logData.Sensors.FiveHole.Beta1.Temperature_C;
            logData.Sensors.FiveHole.Beta2.Temperature_C;
            logData.Sensors.FiveHole.Static.Temperature_C;
            logData.Sensors.FiveHole.Tip.Temperature_C]);
        
    elseif isfield(logData.Sensors.FiveHole, 'PresAlpha')
        oData.pAlpha_Pa = logData.Sensors.FiveHole.PresAlpha.Pressure_Pa;
        oData.pBeta_Pa = logData.Sensors.FiveHole.PresBeta.Pressure_Pa;
        oData.pStatic_Pa = logData.Sensors.FiveHole.Static.Pressure_Pa;
        oData.pTip_Pa = logData.Sensors.FiveHole.Tip.Pressure_Pa;
        oData.tempProbe_C = np.mean([
            logData.Sensors.FiveHole.PresAlpha.Temperature_C;
            logData.Sensors.FiveHole.PresBeta.Temperature_C;
            logData.Sensors.FiveHole.Static.Temperature_C;
            logData.Sensors.FiveHole.Tip.Temperature_C]);
    end
end

% Airdata
oData.vIas_mps = logData.Sensor_Processing.vIAS_ms;
oData.altBaro_m = logData.Sensor_Processing.hBaro_m;

% Copy the active Baseline controller data up to Control/Baseline and Control
% Then copy active Test controller data up to Control/Test and Control
if isfield(config.Mission_Manager, 'Baseline_Select_Switch')
    baseCntrlNames = fieldnames(config.Mission_Manager.Baseline_Select_Switch.ControlSel);
    
    testCtrlNames = {};
    for iTest = 1:length(config.Mission_Manager.Test_Points)
        testCtrlNames{iTest} = config.Mission_Manager.Test_Points(iTest).Control;
    end
    
    for i = 1:length(oData.time_s)
        setName = 'Baseline';
        groupName = baseCntrlNames{logData.Mission.baseCtrlSel(i) + 1};
        
        % Fix the GroupName - If duplicates are listed MATLAB add "_#" to the end
        groupNameList = fieldnames(config.Control.GroupDef);
        while length(groupName) > 0
            if any(contains(groupNameList, groupName))
                break;
            end
            groupName = groupName(1:end-1); % Strip the last character off
        end
        
        levelNames = fieldnames(config.Control.GroupDef.(groupName));
        for iLevel = 1:length(levelNames)
            cntrlName = config.Control.GroupDef.(groupName).(levelNames{iLevel});
            cntrlName = strrep(cntrlName, '-', '_');
            sgnlNameList = fieldnames(logData.Control.(setName).(cntrlName));
            for iSgnl = 1:length(sgnlNameList)
                sgnlName = sgnlNameList{iSgnl};
                
                logData.Control.(setName).(sgnlName)(i) = logData.Control.(setName).(cntrlName).(sgnlName)(i);
                logData.Control.(sgnlName)(i) = logData.Control.(setName).(cntrlName).(sgnlName)(i);
            end
        end
        
        if (logData.Mission.testCtrlMode(i) >= 1) % Test is either Armed or Engaged
            setName = 'Test';
            groupName = testCtrlNames{logData.Mission.testPtID(i) + 1};
            
            levelNames = fieldnames(config.Control.GroupDef.(groupName));
            for iLevel = 1:length(levelNames)
                cntrlName = config.Control.GroupDef.(groupName).(levelNames{iLevel});
                cntrlName = strrep(cntrlName, '-', '_');
                sgnlNameList = fieldnames(logData.Control.(setName).(cntrlName));
                for iSgnl = 1:length(sgnlNameList)
                    sgnlName = sgnlNameList{iSgnl};
                    
                    logData.Control.(setName).(sgnlName)(i) = logData.Control.(setName).(cntrlName).(sgnlName)(i);
                    
                    if (logData.Mission.testCtrlMode(i) == 3) % Test is Engaged
                        logData.Control.(sgnlName)(i) = logData.Control.(setName).(cntrlName).(sgnlName)(i);
                    end
                end
            end
        end
    end
else
    
end
      
oData.Control = logData.Control;

% Get the Effectors List from config
effList = {};
for iEff = 1:length(config.Effectors)
    if strcmp(config.Effectors{iEff}.Type, 'Node')
        for iEffNode = 1:length(config.Effectors{iEff}.Effectors)
            [~, effList{end+1}] = fileparts(config.Effectors{iEff}.Effectors(iEffNode).Input);
        end
    else
        [~, effList{end+1}] = fileparts(config.Effectors{iEff}.Input);
    end
end

for iEff = 1:length(effList)
    oData.Effectors.(effList{iEff}) = logData.Control.(effList{iEff});
end

% GPS
oData.rGps_D_ddm = [logData.Sensors.uBlox.Latitude_rad * r2d; logData.Sensors.uBlox.Longitude_rad * r2d; logData.Sensors.uBlox.Altitude_m];
oData.vGps_L_mps = [logData.Sensors.uBlox.NorthVelocity_ms; logData.Sensors.uBlox.EastVelocity_ms; logData.Sensors.uBlox.DownVelocity_ms];

% Navigation Estimation (EKF)
oData.rB_D_ddm = [logData.Sensor_Processing.Latitude_rad * r2d; logData.Sensor_Processing.Longitude_rad * r2d; logData.Sensor_Processing.Altitude_m];
oData.vB_L_mps = [logData.Sensor_Processing.NorthVelocity_ms; logData.Sensor_Processing.EastVelocity_ms; logData.Sensor_Processing.DownVelocity_ms];

oData.aB_I_mps2 = [logData.Sensor_Processing.AccelX_mss; logData.Sensor_Processing.AccelY_mss; logData.Sensor_Processing.AccelZ_mss];
oData.wB_I_rps = [logData.Sensor_Processing.GyroX_rads; logData.Sensor_Processing.GyroY_rads; logData.Sensor_Processing.GyroZ_rads];
oData.sB_L_rad = [logData.Sensor_Processing.Roll_rad; logData.Sensor_Processing.Pitch_rad; logData.Sensor_Processing.Heading_rad];

% IMU
oData.aImu_I_mps2 = [logData.Sensors.Fmu.Mpu9250.AccelX_mss; logData.Sensors.Fmu.Mpu9250.AccelY_mss; logData.Sensors.Fmu.Mpu9250.AccelZ_mss];
oData.wImu_I_rps = [logData.Sensors.Fmu.Mpu9250.GyroX_rads; logData.Sensors.Fmu.Mpu9250.GyroY_rads; logData.Sensors.Fmu.Mpu9250.GyroZ_rads];
oData.magImu_L_uT = [logData.Sensors.Fmu.Mpu9250.MagX_uT; logData.Sensors.Fmu.Mpu9250.MagY_uT; logData.Sensors.Fmu.Mpu9250.MagZ_uT];

if isfield(logData.Sensors, 'Imu')
    imuList = fields(logData.Sensors.Imu);
    for iImu = 1:length(imuList)
        oData.('a'+imuName+'IMU_IMU_mps2') = [logData.Sensors.Imu.(imuName).AccelX_mss; logData.Sensors.Imu.(imuName).AccelY_mss; logData.Sensors.Imu.(imuName).AccelZ_mss];
        oData.('w'+imuName+'IMU_IMU_rps') = [logData.Sensors.Imu.(imuName).GyroX_rads; logData.Sensors.Imu.(imuName).GyroY_rads; logData.Sensors.Imu.(imuName).GyroZ_rads];
    end
end

% Mission
oData.Mission = logData.Mission;

% Pilot Commands
oData.Pilot = logData.Sensors.Sbus;

% System Identification and Excitation
excList = fields(logData.Excitation);

oData.Excitation = struct();
for iExc = 1:length(excList)
    excName = excList{iExc};
    
    sigList = fields(logData.Excitation.(excName));
    
    for iSig = 1:length(sigList)
        sigName = sigList{iSig};
        sigVal = logData.Excitation.(excName).(sigName);
        if isfield(oData.Excitation, sigName)
            oData.Excitation.(sigName) = oData.Excitation.(sigName) + sigVal;
        else
            oData.Excitation.(sigName) = sigVal;
        end
    end
end


% Make sure the base values are available from the excitation
% if ~isfield(oData, 'Control')
%     oData.Control = struct();
% end
% 
% sigList = fields(oData.Excitation);
% 
% for iSig = 1:length(sigList)
%     sigName = sigList{iSig};
% 
%     if ~isfield(oData.Control, sigName)
%         try oData.Control.(sigName) = logData.Control.(sigName); end
%         try oData.Control.(sigName) = logData.Control.Test.(sigName); end
%     end
% 
% end

%% Outputs


