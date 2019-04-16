#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Louis Mueller
University of Minnesota UAV Lab

Description:
Read and rename flight data .h5 file.  

"""
import h5py
import numpy as np
import math

r2d = 180/math.pi;

def h5read(filename):
            
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

    fltStruct = {}
    
    # TIME
    fltStruct['Time_us'] = h5dat['Sensors']['Fmu']['Time_us']
    
    # IMU (a, w, mag)
    
    fltStruct['aIMU_FMU_mps2'] = np.array([h5dat['Sensors']['Fmu']['Mpu9250']['AccelX_mss'],h5dat['Sensors']['Fmu']['Mpu9250']['AccelY_mss'],h5dat['Sensors']['Fmu']['Mpu9250']['AccelZ_mss']])
    fltStruct['wIMU_FMU_rads'] = np.array([h5dat['Sensors']['Fmu']['Mpu9250']['GyroX_rads'],h5dat['Sensors']['Fmu']['Mpu9250']['GyroY_rads'],h5dat['Sensors']['Fmu']['Mpu9250']['GyroZ_rads']])
    fltStruct['magIMU_FMU_uT'] = np.array([h5dat['Sensors']['Fmu']['Mpu9250']['MagX_uT'],h5dat['Sensors']['Fmu']['Mpu9250']['MagY_uT'],h5dat['Sensors']['Fmu']['Mpu9250']['MagZ_uT']])
    
    
    if 'Imu' in h5dat['Sensors']:
        MuninIMU = ['CenterAft', 'CenterFwd', 'LeftAft', 'LeftFwd', 'LeftMid', 'RightAft', 'RightFwd', 'RightMid']
        for location in MuninIMU:
            if location in h5dat['Sensors']['Imu']:
                fltStruct['a'+location+'IMU_IMU_mps2'] = np.array([h5dat['Sensors']['Imu'][location]['AccelX_mss'],h5dat['Sensors']['Fmu']['Mpu9250']['AccelY_mss'],h5dat['Sensors']['Fmu']['Mpu9250']['AccelZ_mss']])
                fltStruct['w'+location+'IMU_IMU_rads'] = np.array([h5dat['Sensors']['Imu'][location]['GyroX_rads'],h5dat['Sensors']['Fmu']['Mpu9250']['GyroY_rads'],h5dat['Sensors']['Fmu']['Mpu9250']['GyroZ_rads']])
                fltStruct['mag'+location+'IMU_IMU_uT'] = np.array([h5dat['Sensors']['Imu'][location]['MagX_uT'],h5dat['Sensors']['Fmu']['Mpu9250']['MagY_uT'],h5dat['Sensors']['Fmu']['Mpu9250']['MagZ_uT']])
                        
    # Thor Pitot-Static
    if 'Swift' in h5dat['Sensors']:
        fltStruct['pDiff_Pa'] = h5dat['Sensors']['Swift']['Differential']['Pressure_Pa']
        fltStruct['pStatic_Pa'] = h5dat['Sensors']['Swift']['Static']['Pressure_Pa']
        fltStruct['pitotDiffTemp_C'] = h5dat['Sensors']['Swift']['Differential']['Temperature_C']
        fltStruct['pitotStaticTemp_C'] = h5dat['Sensors']['Swift']['Static']['Temperature_C']
        
    # Hugin Pitot-Static
    if '5Hole' in h5dat['Sensors']:
        if 'Alpha1' in h5dat['Sensors']['5Hole']: # For Huginn
            fltStruct['pAlpha1_5H_pa'] = h5dat['Sensors']['5Hole']['Alpha1']['Pressure_Pa']
            fltStruct['pAlpha2_5H_pa'] = h5dat['Sensors']['5Hole']['Alpha2']['Pressure_Pa']
            fltStruct['pBeta1_5H_pa'] = h5dat['Sensors']['5Hole']['Beta1']['Pressure_Pa']
            fltStruct['pBeta2_5H_pa'] = h5dat['Sensors']['5Hole']['Beta2']['Pressure_Pa']
            fltStruct['pStatic_5H_pa'] = h5dat['Sensors']['5Hole']['Static']['Pressure_Pa']
            fltStruct['pTip_5H_pa'] = h5dat['Sensors']['5Hole']['Tip']['Pressure_Pa']
            
    # Mjolnir Pitot-Static
        if 'PresAlpha' in h5dat['Sensors']['5Hole']: # For Mjolnir
            fltStruct['pAlpha_5H_pa'] = h5dat['Sensors']['5Hole']['Alpha1']['Pressure_Pa']
    
    # Airdata
    fltStruct['vIasAD_mps'] = h5dat['Sensor-Processing']['vIAS_ms']
    fltStruct['altAD_m'] = h5dat['Sensor-Processing']['hBaro_m']
    
    # Controllers
    fltStruct['refPhi_rad'] = h5dat['Control']['refPhi_rad']
    fltStruct['refTheta_rad'] = h5dat['Control']['refTheta_rad']
    if 'refPsi_rad' in h5dat['Control']:     
        fltStruct['refPsi_rad'] = h5dat['Control']['refPsi_rad']
    fltStruct['refV_ms'] = h5dat['Control']['refV_ms']
    fltStruct['refH_m'] = h5dat['Control']['refH_m']
    
    fltStruct['cmdEff'] = np.array([h5dat['Control']['cmdMotor_nd'],h5dat['Control']['cmdElev_rad'],h5dat['Control']['cmdRud_rad'],h5dat['Control']['cmdAilL_rad'],h5dat['Control']['cmdFlapL_rad'],h5dat['Control']['cmdFlapR_rad'],h5dat['Control']['cmdAilR_rad']])
    
    # GPS
    fltStruct['rGPS_BE_G_ddm'] = np.array([h5dat['Sensors']['uBlox']['Latitude_rad'] * r2d, h5dat['Sensors']['uBlox']['Longitude_rad'] * r2d, h5dat['Sensors']['uBlox']['Altitude_m']])
    fltStruct['vGPS_BE_L_mps'] = np.array([h5dat['Sensors']['uBlox']['EastVelocity_ms'], h5dat['Sensors']['uBlox']['NorthVelocity_ms'], h5dat['Sensors']['uBlox']['DownVelocity_ms']])
    
    # EKF
    fltStruct['rEKF_BE_G_ddm'] = np.array([h5dat['Sensor-Processing']['Latitude_rad'] * r2d, h5dat['Sensor-Processing']['Longitude_rad'] * r2d, h5dat['Sensor-Processing']['Altitude_m']])
    fltStruct['vEKF_BE_L_mps'] = np.array([h5dat['Sensor-Processing']['EastVelocity_ms'], h5dat['Sensor-Processing']['NorthVelocity_ms'], h5dat['Sensor-Processing']['DownVelocity_ms']])
    
    fltStruct['aEKF_mps2'] = np.array([h5dat['Sensor-Processing']['AccelX_mss'], h5dat['Sensor-Processing']['AccelY_mss'], h5dat['Sensor-Processing']['AccelZ_mss']])
    fltStruct['wEKF_rps'] = np.array([h5dat['Sensor-Processing']['GyroX_rads'], h5dat['Sensor-Processing']['GyroY_rads'], h5dat['Sensor-Processing']['GyroZ_rads']])
    fltStruct['sEKF_rad'] = np.array([h5dat['Sensor-Processing']['Roll_rad'], h5dat['Sensor-Processing']['Pitch_rad'], h5dat['Sensor-Processing']['Heading_rad']])
    
    # Mission
    fltStruct['socEngage'] = h5dat['Mission']['socEngage']
    fltStruct['ctrlSel'] = h5dat['Mission']['ctrlSel']
    fltStruct['testID'] = h5dat['Mission']['testID']
    fltStruct['exciteEngage'] = h5dat['Mission']['excitEngage']
    
    # Power
    fltStruct['MinCellVolt_V'] = h5dat['Sensor-Processing']['MinCellVolt_V']

    return(fltStruct, h5dat)

            
            











