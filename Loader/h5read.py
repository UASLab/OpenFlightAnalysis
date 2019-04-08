#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 23 12:15:07 2018

@author: louismueller
"""
#%% Imports
import h5py
import numpy as np
import math

r2d = 180/math.pi;


#%% File Path/Name
def h5read(filename):

    #filename = '/Users/louismueller/Documents/UAV_LAB/FlightAnalysis/FlightData/Thor/ThorFlt121.h5'
    
    
    #%% Load Functions
            
    def load_dict_from_hdf5(filename):
        with h5py.File(filename, 'r') as h5file:
            return recursively_load_dict_contents_from_group(h5file, '/')
    
            
    def recursively_load_dict_contents_from_group(h5file, path):
        ans = {}
        for key, item in h5file[path].items():
            if isinstance(item, h5py._hl.dataset.Dataset):
                ans[key] = item.value
                ans[key] = ans[key].flatten()
            elif isinstance(item, h5py._hl.group.Group):
                ans[key] = recursively_load_dict_contents_from_group(h5file, path + key + '/')
        return ans
    
    h5dat = load_dict_from_hdf5(filename) # dictionary 
    

    FSTRUCT = {}
    
    #%% TIME
    FSTRUCT['Time_us'] = h5dat['Sensors']['Fmu']['Time_us']
    
    #%%IMU (a, w, mag)
    
    FSTRUCT['aIMU_FMU_mps2'] = np.array([h5dat['Sensors']['Fmu']['Mpu9250']['AccelX_mss'],h5dat['Sensors']['Fmu']['Mpu9250']['AccelY_mss'],h5dat['Sensors']['Fmu']['Mpu9250']['AccelZ_mss']])
    FSTRUCT['wIMU_FMU_rads'] = np.array([h5dat['Sensors']['Fmu']['Mpu9250']['GyroX_rads'],h5dat['Sensors']['Fmu']['Mpu9250']['GyroY_rads'],h5dat['Sensors']['Fmu']['Mpu9250']['GyroZ_rads']])
    FSTRUCT['magIMU_FMU_uT'] = np.array([h5dat['Sensors']['Fmu']['Mpu9250']['MagX_uT'],h5dat['Sensors']['Fmu']['Mpu9250']['MagY_uT'],h5dat['Sensors']['Fmu']['Mpu9250']['MagZ_uT']])
    
    
    if 'Imu' in h5dat['Sensors']:
        MuninIMU = ['CenterAft', 'CenterFwd', 'LeftAft', 'LeftFwd', 'LeftMid', 'RightAft', 'RightFwd', 'RightMid']
        for location in MuninIMU:
            if location in h5dat['Sensors']['Imu']:
                FSTRUCT['a'+location+'IMU_IMU_mps2'] = np.array([h5dat['Sensors']['Imu'][location]['AccelX_mss'],h5dat['Sensors']['Fmu']['Mpu9250']['AccelY_mss'],h5dat['Sensors']['Fmu']['Mpu9250']['AccelZ_mss']])
                FSTRUCT['w'+location+'IMU_IMU_rads'] = np.array([h5dat['Sensors']['Imu'][location]['GyroX_rads'],h5dat['Sensors']['Fmu']['Mpu9250']['GyroY_rads'],h5dat['Sensors']['Fmu']['Mpu9250']['GyroZ_rads']])
                FSTRUCT['mag'+location+'IMU_IMU_uT'] = np.array([h5dat['Sensors']['Imu'][location]['MagX_uT'],h5dat['Sensors']['Fmu']['Mpu9250']['MagY_uT'],h5dat['Sensors']['Fmu']['Mpu9250']['MagZ_uT']])
        
    #%% Pitot-Static
                
    # Thor Pitot-Static
    if 'Swift' in h5dat['Sensors']:
        FSTRUCT['pDiff_Pa'] = h5dat['Sensors']['Swift']['Differential']['Pressure_Pa']
        FSTRUCT['pStatic_Pa'] = h5dat['Sensors']['Swift']['Static']['Pressure_Pa']
        FSTRUCT['pitotDiffTemp_C'] = h5dat['Sensors']['Swift']['Differential']['Temperature_C']
        FSTRUCT['pitotStaticTemp_C'] = h5dat['Sensors']['Swift']['Static']['Temperature_C']
        
    # Hugin Pitot-Static
    if '5Hole' in h5dat['Sensors']:
        if 'Alpha1' in h5dat['Sensors']['5Hole']: # For Huginn
            FSTRUCT['pAlpha1_5H_pa'] = h5dat['Sensors']['5Hole']['Alpha1']['Pressure_Pa']
            FSTRUCT['pAlpha2_5H_pa'] = h5dat['Sensors']['5Hole']['Alpha2']['Pressure_Pa']
            FSTRUCT['pBeta1_5H_pa'] = h5dat['Sensors']['5Hole']['Beta1']['Pressure_Pa']
            FSTRUCT['pBeta2_5H_pa'] = h5dat['Sensors']['5Hole']['Beta2']['Pressure_Pa']
            FSTRUCT['pStatic_5H_pa'] = h5dat['Sensors']['5Hole']['Static']['Pressure_Pa']
            FSTRUCT['pTip_5H_pa'] = h5dat['Sensors']['5Hole']['Tip']['Pressure_Pa']
            
    # Mjolnir Pitot-Static
        if 'PresAlpha' in h5dat['Sensors']['5Hole']: # For Mjolnir
            FSTRUCT['pAlpha_5H_pa'] = h5dat['Sensors']['5Hole']['Alpha1']['Pressure_Pa']
    
    #%% Airdata
    FSTRUCT['vIasAD_mps'] = h5dat['Sensor-Processing']['vIAS_ms']
    FSTRUCT['altAD_m'] = h5dat['Sensor-Processing']['hBaro_m']
    
    #%% Controllers
    FSTRUCT['refPhi_rad'] = h5dat['Control']['refPhi_rad']
    FSTRUCT['refTheta_rad'] = h5dat['Control']['refTheta_rad']
    if 'refPsi_rad' in h5dat['Control']:     
        FSTRUCT['refPsi_rad'] = h5dat['Control']['refPsi_rad']
    FSTRUCT['refV_ms'] = h5dat['Control']['refV_ms']
    FSTRUCT['refH_m'] = h5dat['Control']['refH_m']
    
    FSTRUCT['cmdEff'] = np.array([h5dat['Control']['cmdMotor_nd'],h5dat['Control']['cmdElev_rad'],h5dat['Control']['cmdRud_rad'],h5dat['Control']['cmdAilL_rad'],h5dat['Control']['cmdFlapL_rad'],h5dat['Control']['cmdFlapR_rad'],h5dat['Control']['cmdAilR_rad']])
    
    #%% GPS
    FSTRUCT['rGPS_BE_G_ddm'] = np.array([h5dat['Sensors']['uBlox']['Latitude_rad'] * r2d, h5dat['Sensors']['uBlox']['Longitude_rad'] * r2d, h5dat['Sensors']['uBlox']['Altitude_m']])
    FSTRUCT['vGPS_BE_L_mps'] = np.array([h5dat['Sensors']['uBlox']['EastVelocity_ms'], h5dat['Sensors']['uBlox']['NorthVelocity_ms'], h5dat['Sensors']['uBlox']['DownVelocity_ms']])
    
    #%% EKF
    FSTRUCT['rEKF_BE_G_ddm'] = np.array([h5dat['Sensor-Processing']['Latitude_rad'] * r2d, h5dat['Sensor-Processing']['Longitude_rad'] * r2d, h5dat['Sensor-Processing']['Altitude_m']])
    FSTRUCT['vEKF_BE_L_mps'] = np.array([h5dat['Sensor-Processing']['EastVelocity_ms'], h5dat['Sensor-Processing']['NorthVelocity_ms'], h5dat['Sensor-Processing']['DownVelocity_ms']])
    
    FSTRUCT['aEKF_mps2'] = np.array([h5dat['Sensor-Processing']['AccelX_mss'], h5dat['Sensor-Processing']['AccelY_mss'], h5dat['Sensor-Processing']['AccelZ_mss']])
    FSTRUCT['wEKF_rps'] = np.array([h5dat['Sensor-Processing']['GyroX_rads'], h5dat['Sensor-Processing']['GyroY_rads'], h5dat['Sensor-Processing']['GyroZ_rads']])
    FSTRUCT['sEKF_rad'] = np.array([h5dat['Sensor-Processing']['Roll_rad'], h5dat['Sensor-Processing']['Pitch_rad'], h5dat['Sensor-Processing']['Heading_rad']])
    
    # %%Mission
    FSTRUCT['socEngage'] = h5dat['Mission']['socEngage']
    FSTRUCT['ctrlSel'] = h5dat['Mission']['ctrlSel']
    FSTRUCT['testID'] = h5dat['Mission']['testID']
    FSTRUCT['exciteEngage'] = h5dat['Mission']['excitEngage']
    
    #%% Power
    FSTRUCT['MinCellVolt_V'] = h5dat['Sensor-Processing']['MinCellVolt_V']

    return(FSTRUCT, h5dat)

            
            











