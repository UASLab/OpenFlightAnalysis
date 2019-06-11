#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Louis Mueller
University of Minnesota UAV Lab

Description:
Read and rename flight data .h5 file.  

"""
import h5py
import json
import csv

import numpy as np


#%% Load HDF5 into Dictionary
def Load_h5(filename):
    with h5py.File(filename, 'r') as f:
        data = LoadRecursive_h5(f, '/')
    return data

def LoadRecursive_h5(f, basePath):
    data = {}
    for key, item in f[basePath].items():
        if isinstance(item, h5py._hl.dataset.Dataset):
            data[key] = item.value
            data[key] = data[key].flatten()
        elif isinstance(item, h5py._hl.group.Group):
            data[key] = LoadRecursive_h5(f, basePath + key + '/')
    return data
    


#%% Read/Write Json files into Dictionary
def JsonRead(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
        f.close()
    return data

def JsonWrite(filename, data):
    with open(filename, 'w') as f:
        json.dump(data, f, indent = 4, ensure_ascii=False)
        f.close()
    return 1
    


#%% HDF5 Read
def Log_RAPTRS(filename, fileConfig):
    
    h5Data = Load_h5(filename) # RAPTRS log data as hdf5     
    sysConfig = JsonRead(fileConfig)
    oData = OpenData_RAPTRS(h5Data, sysConfig)
    
    return oData, h5Data

#%% Convert to OpenData Format
def OpenData_RAPTRS(h5Data, sysConfig, oData = {}):
    
    r2d = 180/np.pi;

    # TIME
    oData['time_us'] = h5Data['Sensors']['Fmu']['Time_us']
    oData['time_s'] = oData['time_us'] * 1e-6
    oData['timeDiff_s'] = np.diff(oData['time_s'], prepend=oData['time_s'][0])
    
    # IMU (a, w, mag)
    oData['aImu_I_mps2'] = np.array([h5Data['Sensors']['Fmu']['Mpu9250']['AccelX_mss'],h5Data['Sensors']['Fmu']['Mpu9250']['AccelY_mss'],h5Data['Sensors']['Fmu']['Mpu9250']['AccelZ_mss']])
    oData['wImu_I_rps'] = np.array([h5Data['Sensors']['Fmu']['Mpu9250']['GyroX_rads'],h5Data['Sensors']['Fmu']['Mpu9250']['GyroY_rads'],h5Data['Sensors']['Fmu']['Mpu9250']['GyroZ_rads']])
    oData['magImu_L_uT'] = np.array([h5Data['Sensors']['Fmu']['Mpu9250']['MagX_uT'],h5Data['Sensors']['Fmu']['Mpu9250']['MagY_uT'],h5Data['Sensors']['Fmu']['Mpu9250']['MagZ_uT']])
    
    
    if 'Imu' in h5Data['Sensors']:
        for imuName in h5Data['Sensors']['Imu'].keys():
            if imuName in h5Data['Sensors']['Imu']:
                oData['a'+imuName+'IMU_IMU_mps2'] = np.array([h5Data['Sensors']['Imu'][imuName]['AccelX_mss'],h5Data['Sensors']['Imu'][imuName]['AccelY_mss'],h5Data['Sensors']['Imu'][imuName]['AccelZ_mss']])
                oData['w'+imuName+'IMU_IMU_rps'] = np.array([h5Data['Sensors']['Imu'][imuName]['GyroX_rads'],h5Data['Sensors']['Imu'][imuName]['GyroY_rads'],h5Data['Sensors']['Imu'][imuName]['GyroZ_rads']])
    
    # Thor Pitot-Static
    if 'Swift' in h5Data['Sensors']:
        oData['pTip_Pa'] = h5Data['Sensors']['Swift']['Differential']['Pressure_Pa']
        oData['pStatic_Pa'] = h5Data['Sensors']['Swift']['Static']['Pressure_Pa']
        oData['tempProbe_C'] = np.mean([h5Data['Sensors']['Swift']['Differential']['Temperature_C'], h5Data['Sensors']['Swift']['Static']['Temperature_C']], axis=0)

    if 'Pitot' in h5Data['Sensors']:
        oData['pTip_Pa'] = h5Data['Sensors']['Pitot']['Differential']['Pressure_Pa']
        oData['pStatic_Pa'] = h5Data['Sensors']['Pitot']['Static']['Pressure_Pa']
        oData['tempProbe_C'] = np.mean([h5Data['Sensors']['Pitot']['Differential']['Temperature_C'], h5Data['Sensors']['Pitot']['Static']['Temperature_C']], axis=0)

    
    # Huginn Pitot-Static
    if '5Hole' in h5Data['Sensors']:
        if 'Alpha1' in h5Data['Sensors']['5Hole']: # For Huginn
            oData['pAlpha1_Pa'] = h5Data['Sensors']['5Hole']['Alpha1']['Pressure_Pa']
            oData['pAlpha2_Pa'] = h5Data['Sensors']['5Hole']['Alpha2']['Pressure_Pa']
            oData['pBeta1_Pa'] = h5Data['Sensors']['5Hole']['Beta1']['Pressure_Pa']
            oData['pBeta2_Pa'] = h5Data['Sensors']['5Hole']['Beta2']['Pressure_Pa']
            oData['pStatic_Pa'] = h5Data['Sensors']['5Hole']['Static']['Pressure_Pa']
            oData['pTip_Pa'] = h5Data['Sensors']['5Hole']['Tip']['Pressure_Pa']
            oData['tempProbe_C'] = np.mean(
                    [h5Data['Sensors']['5Hole']['Alpha1']['Temperature_C'], 
                     h5Data['Sensors']['5Hole']['Alpha2']['Temperature_C'], 
                     h5Data['Sensors']['5Hole']['Beta1']['Temperature_C'], 
                     h5Data['Sensors']['5Hole']['Beta2']['Temperature_C'], 
                     h5Data['Sensors']['5Hole']['Static']['Temperature_C'], 
                     h5Data['Sensors']['5Hole']['Tip']['Temperature_C']], axis=0)

            
    # Mjolnir Pitot-Static
        if 'PresAlpha' in h5Data['Sensors']['5Hole']: # For Mjolnir
            oData['pAlpha_Pa'] = h5Data['Sensors']['5Hole']['PresAlpha']['Pressure_Pa']
            oData['pBeta_Pa'] = h5Data['Sensors']['5Hole']['PresBeta']['Pressure_Pa']
            oData['pStatic_Pa'] = h5Data['Sensors']['5Hole']['Static']['Pressure_Pa']
            oData['pTip_Pa'] = h5Data['Sensors']['5Hole']['Tip']['Pressure_Pa']
            oData['tempProbe_C'] = np.mean(
                    [h5Data['Sensors']['5Hole']['PresAlpha']['Temperature_C'],
                     h5Data['Sensors']['5Hole']['PresBeta']['Temperature_C'],
                     h5Data['Sensors']['5Hole']['Static']['Temperature_C'],
                     h5Data['Sensors']['5Hole']['Tip']['Temperature_C']], axis=0)
    
    # Airdata
    oData['vIas_mps'] = h5Data['Sensor-Processing']['vIAS_ms']
    oData['altBaro_m'] = h5Data['Sensor-Processing']['hBaro_m']
    
    # Controllers
    oData['refPhi_rad'] = h5Data['Control']['refPhi_rad']
    oData['refTheta_rad'] = h5Data['Control']['refTheta_rad']
    if 'refPsi_rad' in h5Data['Control']:     
        oData['refPsi_rad'] = h5Data['Control']['refPsi_rad']
    oData['refV_mps'] = h5Data['Control']['refV_ms']
    oData['refH_m'] = h5Data['Control']['refH_m']
    
    # Effectors
    # Get the list of effectors
    # sysConfig['Effectors']
    effList = ['cmdMotor_nd', 'cmdElev_rad', 'cmdRud_rad', 'cmdAilL_rad', 'cmdAilR_rad', 'cmdFlapL_rad', 'cmdFlapR_rad', 
               'cmdTE1L_rad', 'cmdTE1R_rad', 'cmdTE2L_rad', 'cmdTE2R_rad', 'cmdTE3L_rad', 'cmdTE3R_rad', 'cmdTE4L_rad', 'cmdTE4R_rad', 'cmdTE5L_rad', 'cmdTE5R_rad', 'cmdLEL_rad', 'cmdLER_rad']
    
    oData['Effectors'] = {}
    for eff in effList:
        if eff in h5Data['Control']:
             oData['Effectors'][eff] = h5Data['Control'][eff]
    
    # GPS
    oData['rGps_D_ddm'] = np.array([h5Data['Sensors']['uBlox']['Latitude_rad'] * r2d, h5Data['Sensors']['uBlox']['Longitude_rad'] * r2d, h5Data['Sensors']['uBlox']['Altitude_m']])
    oData['vGps_L_mps'] = np.array([h5Data['Sensors']['uBlox']['NorthVelocity_ms'], h5Data['Sensors']['uBlox']['EastVelocity_ms'], h5Data['Sensors']['uBlox']['DownVelocity_ms']])
    
    # EKF
    oData['rB_D_ddm'] = np.array([h5Data['Sensor-Processing']['Latitude_rad'] * r2d, h5Data['Sensor-Processing']['Longitude_rad'] * r2d, h5Data['Sensor-Processing']['Altitude_m']])
    oData['vB_L_mps'] = np.array([h5Data['Sensor-Processing']['NorthVelocity_ms'], h5Data['Sensor-Processing']['EastVelocity_ms'], h5Data['Sensor-Processing']['DownVelocity_ms']])
    
    oData['aB_I_mps2'] = np.array([h5Data['Sensor-Processing']['AccelX_mss'], h5Data['Sensor-Processing']['AccelY_mss'], h5Data['Sensor-Processing']['AccelZ_mss']])
    oData['wB_I_rps'] = np.array([h5Data['Sensor-Processing']['GyroX_rads'], h5Data['Sensor-Processing']['GyroY_rads'], h5Data['Sensor-Processing']['GyroZ_rads']])
    oData['sB_L_rad'] = np.array([h5Data['Sensor-Processing']['Roll_rad'], h5Data['Sensor-Processing']['Pitch_rad'], h5Data['Sensor-Processing']['Heading_rad']])
    
    # Mission
    oData['socEngage'] = h5Data['Mission']['socEngage']
    oData['ctrlSel'] = h5Data['Mission']['ctrlSel']
    oData['testID'] = h5Data['Mission']['testID']
    oData['exciteEngage'] = h5Data['Mission']['excitEngage']
    
    # Power
    oData['pwrFmu_V'] = h5Data['Sensors']['Fmu']['Voltage']['Input_V']
    oData['pwrFmuReg_V'] = h5Data['Sensors']['Fmu']['Voltage']['Regulated_V']
    
    if 'currAvionics' in h5Data['Sensors']['Power']:
        oData['pwrAvionics_A'] = h5Data['Sensors']['Power']['currAvionics']['CalibratedValue']
        oData['pwrAvionics_V'] = h5Data['Sensors']['Power']['voltAvionics']['CalibratedValue']
    
    if 'currPropLeft' in h5Data['Sensors']['Power']:
        oData['pwrPropLeft_A'] = h5Data['Sensors']['Power']['currPropLeft']['CalibratedValue']
        oData['pwrPropLeft_V'] = h5Data['Sensors']['Power']['voltPropLeft']['CalibratedValue']
    
    if 'currPropRight' in h5Data['Sensors']['Power']:
        oData['pwrPropRight_A'] = h5Data['Sensors']['Power']['currPropRight']['CalibratedValue']
        oData['pwrPropRight_V'] = h5Data['Sensors']['Power']['voltPropRight']['CalibratedValue']
        
    
    # Excitations
    oData['Excitation'] = {}
    for k, v in h5Data['Excitation'].items():
        for sigName, sigVal in v.items():
            if sigName in oData['Excitation']:
                oData['Excitation'][sigName] += sigVal
            else:
                oData['Excitation'][sigName] = sigVal

    # Make sure the base values are available from the excitation
    sigExc = oData['Excitation'].keys()
    for sigName in sigExc:
        if sigName not in oData:
            oData[sigName] = h5Data['Control'][sigName]
    
    return oData


#%% HDF5 Read
def Log_JSBSim(filename):
    
    simData = Load_CSV(filename)
    oData = OpenData_RAPTRS(simData)
    
    return oData, simData


#%% Read JSBSim CSV log
def Load_CSV(filename):
    
    simData = {}
    
    with open(filename, 'r') as csvFile:
        reader = csv.reader(csvFile)
        count = 0

        for row in reader:
            if count == 0:
                names = row
                for i in range(0, len(row)):
                    simData[names[i]] = np.array([])
            else:
                for i in range(0, len(row)):
                    simData[names[i]] = np.append(simData[names[i]], float(row[i]))
    
            count += 1
    
    csvFile.close()
    
    return simData
    
#%%
def OpenData_JSBSim(simData, oData = {}):
    
    # Time
    oData['time_s'] = simData['Time'] # !!Unsure on units!!
    
    # GPS
    oData['alt_true_m'] = simData['/fdm/jsbsim/sensor/gps/alt_true_m']
    oData['lat_true_rad'] = simData['/fdm/jsbsim/sensor/gps/lat_true_rad']
    oData['long_true_rad'] = simData['/fdm/jsbsim/sensor/gps/long_true_rad']
    oData['v_true_mps'] = np.array([simData['/fdm/jsbsim/sensor/gps/vEast_true_mps'], simData['/fdm/jsbsim/sensor/gps/vNorth_true_mps'], simData['/fdm/jsbsim/sensor/gps/vDown_true_mps']])
    
    # IMU
    oData['accel_true_fps2'] = np.array([simData['/fdm/jsbsim/sensor/imu/accelX_true_fps2'], simData['/fdm/jsbsim/sensor/imu/accelY_true_fps2'], simData['/fdm/jsbsim/sensor/imu/accelZ_true_fps2']])
    #oData['gyro_true_rps'] = np.array([simData['/fdm/jsbsim/sensor/imu/gyroX_true_rps'], simData['/fdm/jsbsim/sensor/imu/gyroY_true_rps'], simData['/fdm/jsbsim/sensor/imu/gyroZ_true_rps']])
    
    # Pitot
    oData['presStatic_true_pa'] = simData['/fdm/jsbsim/sensor/pitot/presStatic_true_Pa']
    oData['presTip_true_pa'] = simData['/fdm/jsbsim/sensor/pitot/presTip_true_Pa']
    oData['temp_true_C'] = simData['/fdm/jsbsim/sensor/pitot/temp_true_C']
    
    # Altitude
    oData['alt_AGL_ft'] = simData['Altitude AGL (ft)']
    oData['alt_ASL_ft'] = simData['Altitude ASL (ft)']
    
    # Alpha and Beta
    oData['alpha_deg'] = simData['Alpha (deg)']
    oData['beta_deg'] = simData['Beta (deg)']
    
    # Body Acceleration
    oData['accel_body_units'] = np.array([simData['BodyAccel_X'], simData['BodyAccel_Y'], simData['BodyAccel_Z']]) # !!Unsure on units!!
    
    # Moments of Inertia
    oData['I_xx_units'] = simData['I_{xx}'] # !!Unsure on units!!
    oData['I_xy_units'] = simData['I_{xy}'] # !!Unsure on units!!
    oData['I_xz_units'] = simData['I_{xz}'] # !!Unsure on units!!
    oData['I_yx_units'] = simData['I_{yx}'] # !!Unsure on units!!
    oData['I_yy_units'] = simData['I_{yy}'] # !!Unsure on units!!
    oData['I_yz_units'] = simData['I_{yz}'] # !!Unsure on units!!
    oData['I_zx_units'] = simData['I_{zx}'] # !!Unsure on units!!
    oData['I_zy_units'] = simData['I_{zy}'] # !!Unsure on units!!
    oData['I_zz_units'] = simData['I_{zz}'] # !!Unsure on units!!
    
    # P, Q, R
    oData['p_deg/s'] = simData['P (deg/s)']
    oData['q_deg/s'] = simData['Q (deg/s)']
    oData['r_deg/s'] = simData['R (deg/s)']
    
    # Pdot, Qdot, Rdot
    oData['pdot_deg/s2'] = simData['P dot (deg/s^2)']
    oData['qdot_deg/s2'] = simData['Q dot (deg/s^2)']
    oData['rdot_deg/s2'] = simData['R dot (deg/s^2)']
    
    # Qbar
    oData['qbar_psf'] = simData['q bar (psf)']
    
    # Wind
    oData['wind_fps'] = np.array([simData['Wind V_{East} (ft/s)'],simData['Wind V_{North} (ft/s)'],simData['Wind V_{Down} (ft/s)']])
    
    return(oData)






