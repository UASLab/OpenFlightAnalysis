#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb  6 14:19:58 2019

@author: louismueller
"""

# JSBSim csv parser

import csv
import numpy as np

def CSVread(filename):

    #csvFilename = '/Users/louismueller/Documents/UAV_LAB/FlightAnalysis/FlightData/Thor/jsbsim_log.csv'
    
    simFile = {}
    
    with open(filename, 'r') as csvFile:
        reader = csv.reader(csvFile)
        count = 0
        for row in reader:
            if count == 0:
                names = row
                for i in range(0, len(row)):
                    simFile[names[i]] = np.array([])
            else:
                for i in range(0, len(row)):
                    simFile[names[i]] = np.append(simFile[names[i]], float(row[i]))
    
            count += 1
    
    csvFile.close()
    
    #%% Renaming
    
    simStruct = {}
    
    # Time
    simStruct['time_s'] = simFile['Time'] # !!Unsure on units!!
    
    # GPS
    simStruct['alt_true_m'] = simFile['/fdm/jsbsim/sensor/gps/alt_true_m']
    simStruct['lat_true_rad'] = simFile['/fdm/jsbsim/sensor/gps/lat_true_rad']
    simStruct['long_true_rad'] = simFile['/fdm/jsbsim/sensor/gps/long_true_rad']
    simStruct['v_true_mps'] = np.array([simFile['/fdm/jsbsim/sensor/gps/vEast_true_mps'], simFile['/fdm/jsbsim/sensor/gps/vNorth_true_mps'], simFile['/fdm/jsbsim/sensor/gps/vDown_true_mps']])
    
    # IMU
    simStruct['accel_true_fps2'] = np.array([simFile['/fdm/jsbsim/sensor/imu/accelX_true_fps2'], simFile['/fdm/jsbsim/sensor/imu/accelY_true_fps2'], simFile['/fdm/jsbsim/sensor/imu/accelZ_true_fps2']])
    simStruct['gyro_true_rps'] = np.array([simFile['/fdm/jsbsim/sensor/imu/gyroX_true_rps'], simFile['/fdm/jsbsim/sensor/imu/gyroY_true_rps'], simFile['/fdm/jsbsim/sensor/imu/gyroZ_true_rps']])
    
    # Pitot
    simStruct['presStatic_true_pa'] = simFile['/fdm/jsbsim/sensor/pitot/presStatic_true_pa']
    simStruct['presTip_true_pa'] = simFile['/fdm/jsbsim/sensor/pitot/presTip_true_pa']
    simStruct['temp_true_C'] = simFile['/fdm/jsbsim/sensor/pitot/temp_true_C']
    
    # Altitude
    simStruct['alt_AGL_ft'] = simFile['Altitude AGL (ft)']
    simStruct['alt_ASL_ft'] = simFile['Altitude ASL (ft)']
    
    # Alpha and Beta
    simStruct['alpha_deg'] = simFile['Alpha (deg)']
    simStruct['beta_deg'] = simFile['Beta (deg)']
    
    # Body Acceleration
    simStruct['accel_body_units'] = np.array([simFile['BodyAccel_X'], simFile['BodyAccel_Y'], simFile['BodyAccel_Z']]) # !!Unsure on units!!
    
    # Moments of Inertia
    simStruct['I_xx_units'] = simFile['I_{xx}'] # !!Unsure on units!!
    simStruct['I_xy_units'] = simFile['I_{xy}'] # !!Unsure on units!!
    simStruct['I_xz_units'] = simFile['I_{xz}'] # !!Unsure on units!!
    simStruct['I_yx_units'] = simFile['I_{yx}'] # !!Unsure on units!!
    simStruct['I_yy_units'] = simFile['I_{yy}'] # !!Unsure on units!!
    simStruct['I_yz_units'] = simFile['I_{yz}'] # !!Unsure on units!!
    simStruct['I_zx_units'] = simFile['I_{zx}'] # !!Unsure on units!!
    simStruct['I_zy_units'] = simFile['I_{zy}'] # !!Unsure on units!!
    simStruct['I_zz_units'] = simFile['I_{zz}'] # !!Unsure on units!!
    
    # P, Q, R
    simStruct['p_deg/s'] = simFile['P (deg/s)']
    simStruct['q_deg/s'] = simFile['Q (deg/s)']
    simStruct['r_deg/s'] = simFile['R (deg/s)']
    
    # Pdot, Qdot, Rdot
    simStruct['pdot_deg/s2'] = simFile['P dot (deg/s^2)']
    simStruct['qdot_deg/s2'] = simFile['Q dot (deg/s^2)']
    simStruct['rdot_deg/s2'] = simFile['R dot (deg/s^2)']
    
    # Qbar
    simStruct['qbar_psf'] = simFile['q bar (psf)']
    
    # Wind
    simStruct['wind_fps'] = np.array([simFile['Wind V_{East} (ft/s)'],simFile['Wind V_{North} (ft/s)'],simFile['Wind V_{Down} (ft/s)']])
    
    return(simStruct)