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
import pandas as pd

import numpy as np

from os import path
 
#%% Load HDF5 into Dictionary
def Load_h5(filename, toPandas = False, flatten = False):
    if toPandas: # Conversion to Pandas needs a flat dictionary
        flatten = True
    
    with h5py.File(filename, 'r') as f:
        data = LoadRecursive_h5(f, '/')
    
    if flatten:
        data = FlattenDict(data, separator ='/')
        
    if toPandas:
        data = pd.DataFrame(data).add_prefix('/')
    
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


def FlattenDict(data, separator = '/', prefix = ''): 
    
    if isinstance(data, dict): # FIXIT- this is an obnoxious comprehension
        flat = { prefix + separator + k if prefix else k : v 
             for kk, vv in data.items() 
             for k, v in FlattenDict(vv, separator, kk).items() 
             }
    else:
        flat = { prefix : data }
    
    return flat


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

    h5Data = Load_h5(filename, toPandas = True, flatten = True) # RAPTRS log data as hdf5
    sysConfig = JsonRead(fileConfig)
    oData = OpenData_RAPTRS(h5Data, sysConfig)

    return oData, h5Data


def MapColumns_h5_to_OData(h5Data, convertDef):
    
    newName = convertDef['newName'][0]
    newNameSub = convertDef['newName'][1]
    
    if isinstance(convertDef['oldName'], list):
        oldName = [convertDef['oldPath'] + name for name in convertDef['oldName']]
        assert(len(newNameSub) == len(oldName))
        numColumns = len(oldName)
    else:
        oldName = convertDef['oldPath'] + convertDef['oldName']
        newNameSub = ['']
        numColumns = 1
        
    
    multiIndx = pd.MultiIndex(levels=[[newName], newNameSub], labels=[[0]*numColumns, list(range(numColumns))])

    oData = h5Data[oldName]
    oData.columns = multiIndx

    return oData



def AllKeys(pathList, pathKey):
    keyList = [key for key in pathList if(pathKey in key)]
    return keyList

def CommonPath(pathList, pathKey):
    keyList = AllKeys(pathList, pathKey)
    
    sep = '/' # path.commonpath will work since the seperator is '/'
    pathCommon = path.commonpath(keyList) + sep # append with '/'
    return pathCommon


# Add a key:value to a new dictionary base on the the old DataFrame
def Build_OpenData_fromH5(oldDf, oldKeys, newKey, newSubs = [''], newDict = {}, required = True):
    
    keyCheck = []
    for key in oldKeys:
        keyCheck.append(key in oldDf.columns.values.tolist())

    if (required is True):
        assert(all(keyCheck) is True)
    
    else:
        if keyCheck is True:
            newDict[newKey] = oldDf[oldKeys] # Add the DataFrame to the new dictionary
            newDict[newKey].columns = newSubs # rename the columns
    
    return newDict

#%% Convert to OpenData Format
def OpenData_RAPTRS(h5Data, sysConfig, oData = {}):
    h5Keys = list(h5Data.keys())

    #%% Time
    fmuPath = CommonPath(h5Keys, 'Fmu') 
    timeDF = pd.Series(h5Data[fmuPath + 'Time_us'] * 10e-6, name = 'time_s')
    
    #%% Sensors
    oData['Sensor'] = {}
    
    #%% IMU
    imuKeys = AllKeys(h5Keys, 'Mpu9250')
    imuPath = CommonPath(h5Keys, 'Mpu9250') 
    
    # Create a new DF to seperate out the Sensor data
    imuDict = {}
    imuDF = h5Data[imuKeys]
    imuDF.set_index(timeDF, inplace = True)
    
    # Accel
    nameList = [imuPath + name for name in ['AccelX_mss', 'AccelY_mss', 'AccelZ_mss']]
    Build_OpenData_fromH5(imuDF, nameList, 'aImu_B_mps2', ['X', 'Y', 'Z'], newDict = imuDict)
    # Gyro
    nameList = [imuPath + name for name in ['GyroX_rads', 'GyroY_rads', 'GyroZ_rads']]
    Build_OpenData_fromH5(imuDF, nameList, 'wImu_B_rps', ['X', 'Y', 'Z'], newDict = imuDict)
    # Magnetometer
    nameList = [imuPath + name for name in ['MagX_uT', 'MagY_uT', 'MagZ_uT']]
    Build_OpenData_fromH5(imuDF, nameList, 'magImu_L_uT', ['X', 'Y', 'Z'], newDict = imuDict)
    
    # Combine the oData Dictionary into the oData DataFrame
    oData['Sensor']['IMU'] = pd.concat(imuDict, axis = 'columns')

    
    #%% GPS
    gpsKeys = AllKeys(h5Keys, 'uBlox')
    gpsPath = CommonPath(h5Keys, 'uBlox')
    
    # Create a new DF to seperate out the Sensor data
    gpsDict = {}
    gpsDF = h5Data[gpsKeys]
    gpsDF.set_index(timeDF, inplace = True)
    gpsDF = gpsDF.drop_duplicates(keep = 'first', inplace = False) # Remove all duplicates, Need to copy

    # Position
    nameList = [gpsPath + name for name in ['Latitude_rad', 'Longitude_rad', 'Altitude_m']]
    Build_OpenData_fromH5(gpsDF, nameList, 'rGps_D_rrm', ['Lat', 'Lon', 'Alt'], newDict = gpsDict)
    # Velocity
    nameList = [gpsPath + name for name in ['NorthVelocity_ms', 'EastVelocity_ms', 'DownVelocity_ms']]
    Build_OpenData_fromH5(gpsDF, nameList, 'vGps_L_mps', ['N', 'E', 'D'], newDict = gpsDict)
    # Time
    nameList = [gpsPath + name for name in ['TOW', 'Year', 'Month', 'Day', 'Hour', 'Minute', 'Second']]
    Build_OpenData_fromH5(gpsDF, nameList, 'Time', ['TOW', 'Year', 'Month', 'Day', 'Hour', 'Minute', 'Second'], newDict = gpsDict)
    # Fix Flag
    nameList = [gpsPath + name for name in ['Fix']]
    Build_OpenData_fromH5(gpsDF, nameList, 'Fix', [''], newDict = gpsDict)
    
    # Combine the oData Dictionary into the oData DataFrame
    oData['Sensor']['GPS'] = pd.concat(gpsDict, axis = 'columns')
    
    
    #%% Pitot
    pitotName = []
    if AllKeys(h5Keys, 'Pitot'):
        pitotName = 'Pitot'
    elif AllKeys(h5Keys, 'Swift'):# Swift - Some Pitot data was logged with "Swift"
        pitotName = 'Swift'

    if pitotName:
        pitotKeys = AllKeys(h5Keys, pitotName)
        pitotPath = CommonPath(h5Keys, pitotName)
        
        # Create a new DF to seperate out the Sensor data
        pitotDict = {}
        pitotDF = h5Data[pitotKeys]
        pitotDF.set_index(timeDF, inplace = True)
        
        # Signals
        old = 'Static'; new = old;
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Pressure_Pa'], 'p' + new + '_Pa', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Temperature_C'], 'temp' + new + '_C', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Status'], 'status' + new, newDict = pitotDict, required = False)
        
        old = 'Differential'; new = 'Tip';
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Pressure_Pa'], 'p' + new + '_Pa', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Temperature_C'], 'temp' + new + '_C', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Status'], 'status' + new, newDict = pitotDict, required = False)
        
        old = 'Tip'; new = old;
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Pressure_Pa'], 'p' + new + '_Pa', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Temperature_C'], 'temp' + new + '_C', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Status'], 'status' + new, newDict = pitotDict, required = False)

        # Combine the oData Dictionary into the oData DataFrame
        oData['Sensor']['Pitot'] = pd.concat(pitotDict, axis = 'columns')


    #%% 5Hole
    pitotName = []
    if AllKeys(h5Keys, '5Hole'):
        pitotName = 'Pitot'


    if pitotName:
        pitotKeys = AllKeys(h5Keys, pitotName)
        pitotPath = CommonPath(h5Keys, pitotName)
        
        # Create a new DF to seperate out the Sensor data
        pitotDict = {}
        pitotDF = h5Data[pitotKeys]
        pitotDF.set_index(timeDF, inplace = True)
        
        # Signals
        old = 'Static'; new = old
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Pressure_Pa'], 'p' + new + '_Pa', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Temperature_C'], 'temp' + new + '_C', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Status'], 'status' + new, newDict = pitotDict, required = False)
        
        old = 'Differential'; new = 'Tip'
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Pressure_Pa'], 'p' + new + '_Pa', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Temperature_C'], 'temp' + new + '_C', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Status'], 'status' + new, newDict = pitotDict, required = False)
        
        old = 'Tip'; new = old
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Pressure_Pa'], 'p' + new + '_Pa', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Temperature_C'], 'temp' + new + '_C', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Status'], 'status' + new, newDict = pitotDict, required = False)
        
        old = 'Alpha1'; new = old
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Pressure_Pa'], 'p' + new + '_Pa', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Temperature_C'], 'temp' + new + '_C', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Status'], 'status' + new, newDict = pitotDict, required = False)
        
        old = 'Alpha2'; new = old
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Pressure_Pa'], 'p' + new + '_Pa', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Temperature_C'], 'temp' + new + '_C', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Status'], 'status' + new, newDict = pitotDict, required = False)
        
        old = 'Beta1'; new = old
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Pressure_Pa'], 'p' + new + '_Pa', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Temperature_C'], 'temp' + new + '_C', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Status'], 'status' + new, newDict = pitotDict, required = False)
        
        old = 'Beta2'; new = old
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Pressure_Pa'], 'p' + new + '_Pa', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Temperature_C'], 'temp' + new + '_C', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Status'], 'status' + new, newDict = pitotDict, required = False)
        
        old = 'PresAlpha'; new = 'Alpha'
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Pressure_Pa'], 'p' + new + '_Pa', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Temperature_C'], 'temp' + new + '_C', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Status'], 'status' + new, newDict = pitotDict, required = False)
        
        old = 'PresBeta'; new = 'Beta'
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Pressure_Pa'], 'p' + new + '_Pa', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Temperature_C'], 'temp' + new + '_C', newDict = pitotDict, required = False)
        Build_OpenData_fromH5(pitotDF, [pitotPath + old + '/Status'], 'status' + new, newDict = pitotDict, required = False)

        # Combine the oData Dictionary into the oData DataFrame
        oData['Sensor']['5Hole'] = pd.concat(pitotDict, axis = 'columns')


    ## Structural IMUs
    imuKeys = AllKeys(h5Keys, 'Sensors/Imu')
    if imuKeys:
        # Create a new DF to seperate out the Sensor data
        imuDF = h5Data[imuKeys]
        imuDF.set_index(timeDF, inplace = True)
        
        for imuName in imuKeys:
            imuPath = CommonPath(h5Keys, 'Imu') + imuName
            imuDict = {}
        
            # Accel
            nameList = [imuPath + name for name in ['AccelX_mss', 'AccelY_mss', 'AccelZ_mss']]
            Build_OpenData_fromH5(imuDF, nameList, 'a'+imuName+'_mps2', ['X', 'Y', 'Z'], newDict = imuDict)
            # Gyro
            nameList = [imuPath + name for name in ['GyroX_rads', 'GyroY_rads', 'GyroZ_rads']]
            Build_OpenData_fromH5(imuDF, nameList, 'w'+imuName+'_rps', ['X', 'Y', 'Z'], newDict = imuDict)
            # Magnetometer
            nameList = [imuPath + name for name in ['MagX_uT', 'MagY_uT', 'MagZ_uT']]
            Build_OpenData_fromH5(imuDF, nameList, 'mag'+imuName+'uT', ['X', 'Y', 'Z'], newDict = imuDict, required = False)
            
            # Combine the oData Dictionary into the oData DataFrame
            oData['Sensor']['StructIMU'] = pd.concat(imuDict, axis = 'columns')

    
    ## Power
    # Create a new DF to seperate out the Sensor data
    powerKeys = AllKeys(h5Keys, 'Sensors/Fmu/Voltage') + AllKeys(h5Keys, 'Power')
    powerDF = h5Data[powerKeys]
    powerDF.set_index(timeDF, inplace = True)
    
    powerDF.rename(columns = {'/Sensors/Fmu/Voltage/Input_V': 'pwrFmu_V'}, inplace = True, copy = False)
    powerDF.rename(columns = {'/Sensors/Fmu/Voltage/Regulated_V': 'pwrFmuReg_V'}, inplace = True, copy = False)
    
    powerDF.rename(columns = {'/Sensors/Power/FuselageServoVoltage_V': 'pwrFusServo_V'}, inplace = True, copy = False)
    powerDF.rename(columns = {'/Sensors/Power/WingServoVoltage_V': 'pwrWingServo_V'}, inplace = True, copy = False)

    powerDF.rename(columns = {'/Sensors/Power/voltAvionics/CalibratedValue': 'pwrAvionics_V'}, inplace = True, copy = False)
    powerDF.rename(columns = {'/Sensors/Power/currAvionics/CalibratedValue': 'pwrAvionics_A'}, inplace = True, copy = False)

    powerDF.rename(columns = {'/Sensors/Power/voltProp/CalibratedValue': 'pwrProp_V'}, inplace = True, copy = False)
    powerDF.rename(columns = {'/Sensors/Power/currProp/CalibratedValue': 'pwrProp_A'}, inplace = True, copy = False)

    powerDF.rename(columns = {'/Sensors/Power/voltPropLeft/CalibratedValue': 'pwrPropLeft_V'}, inplace = True, copy = False)
    powerDF.rename(columns = {'/Sensors/Power/currPropLeft/CalibratedValue': 'pwrPropLeft_A'}, inplace = True, copy = False)

    powerDF.rename(columns = {'/Sensors/Power/voltPropRight/CalibratedValue': 'pwrPropRight_V'}, inplace = True, copy = False)
    powerDF.rename(columns = {'/Sensors/Power/currPropRight/CalibratedValue': 'pwrPropRight_A'}, inplace = True, copy = False)
    
    # Combine the oData Dictionary into the oData DataFrame
    oData['Sensor']['Power'] = powerDF

    
    ### Sensor Processing, onboard estimates
    oData['SenProc'] = {}
    senProcKeys = AllKeys(h5Keys, 'Sensor-Processing')
    senProcPath = CommonPath(h5Keys, 'Sensor-Processing') 
    
    # Create a new DF to seperate out the Sensor data
    senProcDict = {}
    senProcDF = h5Data[senProcKeys]
    senProcDF.set_index(timeDF, inplace = True)
    
    ## EKF
    # Accel
    nameList = [senProcPath + name for name in ['AccelX_mss', 'AccelY_mss', 'AccelZ_mss']]
    Build_OpenData_fromH5(senProcDF, nameList, 'aEst_B_mps2', ['X', 'Y', 'Z'], newDict = senProcDict)
    # Gyro
    nameList = [senProcPath + name for name in ['GyroX_rads', 'GyroY_rads', 'GyroZ_rads']]
    Build_OpenData_fromH5(senProcDF, nameList, 'wEst_B_rps', ['X', 'Y', 'Z'], newDict = senProcDict)
    # Orientation
    nameList = [senProcPath + name for name in ['Roll_rad', 'Pitch_rad', 'Heading_rad']]
    Build_OpenData_fromH5(senProcDF, nameList, 'sEst_L_rad', ['X', 'Y', 'Z'], newDict = senProcDict)
    # Position
    nameList = [senProcPath + name for name in ['Latitude_rad', 'Longitude_rad', 'Altitude_m']]
    Build_OpenData_fromH5(gpsDF, nameList, 'rEst_D_rrm', ['Lat', 'Lon', 'Alt'], newDict = senProcDict)
    # Velocity
    nameList = [senProcPath + name for name in ['NorthVelocity_ms', 'EastVelocity_ms', 'DownVelocity_ms']]
    Build_OpenData_fromH5(gpsDF, nameList, 'vEst_L_mps', ['N', 'E', 'D'], newDict = senProcDict)


    ## AirData
    Build_OpenData_fromH5(gpsDF, [senProcPath + 'vIAS_ms'], 'vIas_mps', newDict = senProcDict, required = False)
    Build_OpenData_fromH5(gpsDF, [senProcPath + 'hBaro_m'], 'hBaro_m', newDict = senProcDict, required = False)
    Build_OpenData_fromH5(gpsDF, [senProcPath + 'alpha_rad'], 'alpha_rad', newDict = senProcDict, required = False)
    Build_OpenData_fromH5(gpsDF, [senProcPath + 'beta_rad'], 'beta_rad', newDict = senProcDict, required = False)

    
    ### Control
    # Mission
    oData['Mission'] = {}
#    oData['Mission']['socEngage'] = h5Data['/Mission/socEngage']
#    oData['Mission']['ctrlSel'] = h5Data['/Mission/ctrlSel']
#    oData['Mission']['testID'] = h5Data['/Mission/testID']
#    oData['Mission']['exciteEngage'] = h5Data['/Mission/excitEngage']
    
    # Control
    oData['Control'] = {}
#    # Controllers
#    oData['refPhi_rad'] = h5Data['Control']['refPhi_rad']
#    oData['refTheta_rad'] = h5Data['Control']['refTheta_rad']
#    if 'refPsi_rad' in h5Data['Control']:
#        oData['refPsi_rad'] = h5Data['Control']['refPsi_rad']
#    oData['refV_mps'] = h5Data['Control']['refV_ms']
#    oData['refH_m'] = h5Data['Control']['refH_m']
    
    # Effectors
    oData['Control']['Effectors'] = {}
#    # sysConfig['Effectors']
#    effList = ['cmdMotor_nd', 'cmdElev_rad', 'cmdRud_rad', 'cmdAilL_rad', 'cmdAilR_rad', 'cmdFlapL_rad', 'cmdFlapR_rad',
#               'cmdTE1L_rad', 'cmdTE1R_rad', 'cmdTE2L_rad', 'cmdTE2R_rad', 'cmdTE3L_rad', 'cmdTE3R_rad', 'cmdTE4L_rad', 'cmdTE4R_rad', 'cmdTE5L_rad', 'cmdTE5R_rad', 'cmdLEL_rad', 'cmdLER_rad']
#
#    oData['Effectors'] = {}
#    for eff in effList:
#        if eff in h5Data['Control']:
#             oData['Effectors'][eff] = h5Data['Control'][eff]
    
    # Excitation
    oData['Control']['Excitation'] = {}
#    for k, v in h5Data['Excitation'].items():
#        for sigName, sigVal in v.items():
#            if sigName in oData['Excitation']:
#                oData['Excitation'][sigName] += sigVal
#            else:
#                oData['Excitation'][sigName] = sigVal
    
#
#    # Make sure the base values are available from the excitation
#    oData['Control'] = {}
#    for sigName in oData['Excitation'].keys():
#        if sigName not in oData['Control']:
#            oData['Control'][sigName] = h5Data['Control'][sigName]


    
    
    return oData


#%% HDF5 Read
def Log_JSBSim(filename):

    simData = Load_CSV(filename)
    oData = OpenData_RAPTRS(simData)

    return oData, simData


#%% Read JSBSim CSV log
def Load_CSV(filename):

    simData = pd.read_csv(filename)

    return simData


#%%
def OpenData_JSBSim(simData, oData = {}):


    # Time
    oData['time_s'] = simData['Time'] # !!Unsure on units!!

    # GPS
    oData['alt_m'] = simData['/fdm/jsbsim/sensor/gps/alt_true_m']
    oData['lat_rad'] = simData['/fdm/jsbsim/sensor/gps/lat_true_rad']
    oData['lon_rad'] = simData['/fdm/jsbsim/sensor/gps/long_true_rad']
    oData['v_true_mps'] = np.array([simData['/fdm/jsbsim/sensor/gps/vEast_true_mps'], simData['/fdm/jsbsim/sensor/gps/vNorth_true_mps'], simData['/fdm/jsbsim/sensor/gps/vDown_true_mps']])

    # IMU
    oData['accel_fps2'] = np.array([simData['/fdm/jsbsim/sensor/imu/accelX_true_fps2'], simData['/fdm/jsbsim/sensor/imu/accelY_true_fps2'], simData['/fdm/jsbsim/sensor/imu/accelZ_true_fps2']])
    #oData['gyro_rps'] = np.array([simData['/fdm/jsbsim/sensor/imu/gyroX_true_rps'], simData['/fdm/jsbsim/sensor/imu/gyroY_true_rps'], simData['/fdm/jsbsim/sensor/imu/gyroZ_true_rps']])

    # Pitot
    oData['presStatic_Pa'] = simData['/fdm/jsbsim/sensor/pitot/presStatic_true_Pa']
    oData['presTip_Pa'] = simData['/fdm/jsbsim/sensor/pitot/presTip_true_Pa']
    oData['temp_C'] = simData['/fdm/jsbsim/sensor/pitot/temp_true_C']

    # Altitude
    oData['alt_AGL_ft'] = simData['Altitude AGL (ft)']
    oData['alt_ASL_ft'] = simData['Altitude ASL (ft)']

    # Alpha and Beta
    oData['alpha_rad'] = simData['Alpha (deg)'] * d2r
    oData['beta_rad'] = simData['Beta (deg)'] * d2r

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
