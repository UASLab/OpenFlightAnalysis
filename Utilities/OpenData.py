#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 28 15:05:36 2018

@author: louismueller
"""

# import
import numpy as np

#%% 
def PlotOverview(oData):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # Overview Plots
    # Find interesting stuff: takeoff, landing, excitations etc.
    time_s = oData['time_s']
    
    # Airspeed
    vIas_mps = oData['vIas_mps']
    vGps_mps = np.linalg.norm(oData['vGps_L_mps'], 2, axis=0)
    vB_mps = np.linalg.norm(oData['vB_L_mps'], 2, axis=0)
    fig0, ax0 = plt.subplots()
    ax0.plot(time_s, oData['refV_mps'], label='ref')
    ax0.plot(time_s, vIas_mps, label='airspeed')
    ax0.plot(time_s, vGps_mps, '.', label='Gps')
    ax0.plot(time_s, vB_mps, label='Ekf')
    ax0.set_xlabel('Time (s)')
    ax0.set_ylabel('Airspeed (m/s)')
    ax0.set_title('Air Data Airspeed')
    ax0.legend()
    
    # Altitude
    altBaro_m = oData['altBaro_m']
    altGps_m = oData['rGps_D_ddm'][2]
    altB_m = oData['rB_D_ddm'][2]
    fig1, ax1 = plt.subplots()
    ax1.plot(time_s, oData['refH_m'], label='ref')
    ax1.plot(time_s, altBaro_m, label='Baro')
    ax1.plot(time_s, altGps_m - altGps_m[1000], '.', label='GPS')
    ax1.plot(time_s[1000:], altB_m[1000:] - altB_m[1000], label='Ekf')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Altitude (m)')
    ax1.set_title('Altitude')
    ax1.legend()
    
    # X and Y Position
    latGps_deg = oData['rGps_D_ddm'][0]
    lonGps_deg = oData['rGps_D_ddm'][1]
    latB_deg = oData['rB_D_ddm'][0]
    lonB_deg = oData['rB_D_ddm'][1]
    fig2, ax2 = plt.subplots()
    ax2.plot(lonGps_deg, latGps_deg, '.', label='GPS')
    ax2.plot(lonB_deg[1000:], latB_deg[1000:], label='Ekf')
    ax2.axis('equal')
    ax2.set_xlabel('Longitude (deg)')
    ax2.set_ylabel('Latitude (deg)')
    ax2.set_title('Latitude and Longitude')
    ax1.legend()
    
    # Voltage
    pwrFmu_V = oData['pwrFmu_V']
    fig3, ax3 = plt.subplots()
    ax3.plot(time_s, pwrFmu_V)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Avionics Voltage (V)')
    ax3.set_title('Power')
    
    # 3D Position
    fig4 = plt.figure()
    ax4 = fig4.gca(projection='3d')
    ax4.plot(lonGps_deg, latGps_deg, altGps_m, '.', label='GPS')
    ax4.plot(lonB_deg[1000:], latB_deg[1000:], altB_m[1000:], label='Ekf')
    ax4.axis('equal')
    ax4.set_xlabel('Longitude (deg)')
    ax4.set_ylabel('Latitude (deg)')
    ax4.set_title('Flight Path')
    ax1.legend()
    
    plt.show()
    
    return 1

#%% Find Excitation Times based on 'exciteEngage'
def FindExcite(oData):
    # returns list of tuples: 
    #(testID, [timeMin_us, timeMax_us])
       
    # Create array that is the testID value when exciteEngage is True and -1 everywhere else
    iTestExcite = np.where(oData['exciteEngage'], oData['testID'], -1 * np.ones_like(oData['testID']))
    
    # Find where the index changes
    iRange = np.where(iTestExcite[:-1] != iTestExcite[1:])[0]
    
    iStartList = iRange[0::2]
    iEndList = iRange[1::2]
    
    excList = []
    for iExc in range(0,len(iStartList)):
        iStart = iStartList[iExc]
        iEnd = iEndList[iExc]
        timeRange_us = [oData['time_us'][iStart], oData['time_us'][iEnd]]
                
        testID = oData['testID'][iStart]
        
        exc = (testID, timeRange_us)
        excList.append(exc)
    
    return excList

#%% 
def TestPointOut(excList, testPointList):

    testList = []
    for iExc, exc in enumerate(excList):
        iTestID = exc[0]
        testPoint = testPointList[iTestID]
        testPoint['time_us'] = str(excList[iExc][1])
        testList.append(testPoint)

    return testList

#%% Segment oData by condition
def Segment(oData, cond):
    # cond = (condName, [min, max])
    # if cond is a list, will return a list of oData segments
    # Example: cond = ('time_s', [60, 61])
    
    # Recursive call if cond is a list of conditions
    if type(cond) is list:
        oDataSeg = []
        for c in cond:
            oDataSeg.append(Segment(oData, c))
        return oDataSeg
            
    # Slice into Segments
    condName = cond[0]
    condRange = cond[1]
    
    # Bool that matches the condition
    iCond = (oData[condName] >= condRange[0]) & (oData[condName] <= condRange[1])
    
    oDataSeg = {}
    for k,v in oData.items(): # Loop through oData
        oDataSeg[k] = np.copy(v[...,iCond])
        
    return oDataSeg