#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  7 13:43:19 2019

@author: rega0051
"""

#%%
# Import Libraries
import numpy as np
import json

import Loader
import OpenData

import matplotlib.pyplot as plt


fileLog = '/home/rega0051/FlightArchive/Huginn/HuginnFLT02/HuginnFLT02.h5'
fileTestDef = '/home/rega0051/FlightArchive/Huginn/HuginnFLT02/huginn_def.json'
fileSysConfig = '/home/rega0051/FlightArchive/Huginn/HuginnFLT02/huginn.json'

#%%
# Read in raw h5 data into dictionary and data
oData, h5Data = Loader.Log_RAPTRS(fileLog)

# Plot Overview of flight
oData = OpenData.Segment(oData, ('time_s', [445, 695]))
OpenData.PlotOverview(oData)

#%% Find Excitation Times
excList = OpenData.FindExcite(oData)

print('\n\nFlight Excitation Times:\n')
for iExc in range(0, len(excList)):
    print('Excitiation: ', excList[iExc][0], ', Time: [', excList[iExc][1][0], ',', excList[iExc][1][1], ']')

segList = [('time_us', excList[0][1])]

oDataExc = OpenData.Segment(oData, segList)
OpenData.PlotOverview(oDataExc[0])


#%%
time_s = oData['time_s']
pwrPropLeft_W = oData['pwrPropLeft_A'] * oData['pwrPropLeft_V']
pwrPropRight_W = oData['pwrPropRight_A'] * oData['pwrPropRight_V']

# Voltage
fig0, ax0 = plt.subplots()
ax0.plot(time_s, pwrPropLeft_W, time_s, pwrPropRight_W)
ax0.set_xlabel('Time (s)')
ax0.set_ylabel('Propulsion Power (W)')
ax0.set_title('Power')


pwrPropLeft_mAh = 1000.0 / 3600.0 * np.trapz(oData['pwrPropLeft_A'], time_s)
pwrPropRight_mAh = 1000.0 / 3600.0 * np.trapz(oData['pwrPropRight_A'], time_s)

pwrPropActual_mAh = 1393.0 + 1362.0

scale = pwrPropActual_mAh / (pwrPropLeft_mAh + pwrPropRight_mAh)

scale * 54.6448

#
fig, ax1 = plt.subplots()

color = 'tab:red'
ax1.set_xlabel('time (s)')
ax1.set_ylabel('Airspeed (m/s)', color=color)
ax1.plot(time_s, oData['vIas_mps'], color=color)
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
ax2.set_ylabel('Altitude (m)', color=color)  # we already handled the x-label with ax1
ax2.plot(time_s, oData['altBaro_m'], color=color)
ax2.tick_params(axis='y', labelcolor=color)

fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.show()

#%% Analyze Glide
glideSeg = ('time_s', [676, 686])
oDataGlide = OpenData.Segment(oData, glideSeg)

from scipy import signal
b, a = signal.butter(2, 0.25/50)
altBaro_m = signal.filtfilt(b, a, oDataGlide['altBaro_m'])
vIas_mps = signal.filtfilt(b, a, oDataGlide['vIas_mps'])

time_s = oDataGlide['time_s'][50:-75]
altBaro_m = altBaro_m[50:-75]
vIas_mps = vIas_mps[50:-75]

plt.figure()
plt.plot(oDataGlide['time_s'], oDataGlide['altBaro_m'], time_s, altBaro_m)
plt.plot(oDataGlide['time_s'], oDataGlide['vIas_mps'], time_s, vIas_mps)

dIas_m = np.trapz(vIas_mps, time_s)

temp_deg = np.arcsin(altBaro_m / dIas_m) * 180.0 / np.pi

LD = 1/np.tan(temp_deg * np.pi / 180.0)


plt.figure()
plt.plot(vIas_mps, LD, '.')
plt.xlabel('Airspeed (m/s)')
plt.ylabel('Lift / Drag')
plt.grid()

#%% Save _init.json file
# Open init json file
json_init = {}
#json_init = JsonRead(fileTestDef)

# Open flight json file
flightjson = Loader.JsonRead(fileSysConfig)
testPointList = flightjson['Mission-Manager']['Test-Points']

json_init['Test-Points'] = OpenData.TestPointOut(excList, testPointList)

if False:
    Loader.JsonWrite(fileTestDef, json_init)
    print('Init File Updated:\n', fileTestDef)
else:
    json.dumps(json_init, indent = 4, ensure_ascii=False)
    print('\nInit file NOT updated\n\n')
