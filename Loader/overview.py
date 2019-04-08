#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Louis Mueller
University of Minnesota UAV Lab

Description:
Overview is used for general analysis of .h5 flight data. General flight overview
plots and excitation numbers and timestamps will be displayed. The [--save] option
will update a file in the given flight folder named <flight>_init.json with all
excitation details and times given in the <flight>.json given in the same flight folder.
Timestamps and h5 data can then be used using analysis.py.

General Steps:
1. Load .h5 and .json from flightFolder
2. Plot general figures
3. Save to _init.json

Example:
python overview.py [flightFolder] [--save]
python overview.py ThorFLT123 --save

Future Improvements:
- Overview of sim csv
- Save figures
"""

# Import Libraries
import os
import json
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from h5read import h5read

# Include arguments for load
parser = argparse.ArgumentParser(description="Load h5 file and load overview plots")
parser.add_argument("flightFolder")
parser.add_argument("--save", action="store_true")
args = parser.parse_args()

basepath = '/Users/louismueller/Documents/UAV_LAB/Analysis/FlightData'

flight = str(args.flightFolder)

# Locate files
h5filename = os.path.join(basepath, 'Thor', flight, flight + '.h5')
init_jsonfilename = os.path.join(basepath, 'Thor',flight,flight + '_init.json')
flight_jsonfilename = os.path.join(basepath, 'Thor', flight, flight + '.json')

# Read in raw h5 data into dictionary and data
h5dict, h5data = h5read(h5filename)

# Overview Plots
# Find interesting stuff: takeoff, landing, excitations etc.

time = h5dict['Time_us']

# Altitude
altAD_m = h5dict['altAD_m']
fig1, ax1 = plt.subplots()
ax1.plot(time, altAD_m)
ax1.set_xlabel('Time (us)')
ax1.set_ylabel('Altitude (m)')
ax1.set_title('Air Data Altitude')

# X and Y Position
rx_deg = h5dict['rGPS_BE_G_ddm'][0]
ry_deg = h5dict['rGPS_BE_G_ddm'][1]
fig2, ax2 = plt.subplots()
ax2.plot(rx_deg, ry_deg)
ax2.axis('equal')
ax2.set_xlabel('Latitude (deg)')
ax2.set_ylabel('Longitude (deg)')
ax2.set_title('Latitude and Longitude')

# Voltage
MinCellVolt_V = h5dict['MinCellVolt_V']
fig3, ax3 = plt.subplots()
ax3.plot(time, MinCellVolt_V)
ax3.set_xlabel('Time (us)')
ax3.set_ylabel('Min Cell Voltage (V)')
ax3.set_title('Min Cell Voltage')

# 3D Position
rz_m = h5dict['rGPS_BE_G_ddm'][2]
fig4 = plt.figure()
ax4 = fig4.gca(projection='3d')
ax4.plot(rx_deg, ry_deg, rz_m)
ax4.axis('equal')
ax4.set_xlabel('Latitude (deg)')
ax4.set_ylabel('Longitude (deg)')
ax4.set_title('Flight Path')


#%% Find Excitation Times

exciteID = np.array([])
exciteIndex = np.array([])

for ID in range(0,len(h5dict['exciteEngage'])):

    # Flags
    currentTest = h5dict['exciteEngage'][ID]
    previousTest = h5dict['exciteEngage'][ID - 1]

    # Beginning Excitation
    if  (previousTest != 1 and currentTest == 1):
        exciteID = np.append(exciteID, h5dict['testID'][ID - 1]) #Note Test ID Name
        exciteIndex = np.append(exciteIndex, ID) #Note Start of Excitation

    # Ending Excitation
    if  (previousTest == 1 and currentTest == 0):
        exciteID = np.append(exciteID, h5dict['testID'][ID]) #Note Test ID Name
        exciteIndex = np.append(exciteIndex, ID) #Note End of Excitation

exciteID = exciteID.astype(int)
exciteIndex = exciteIndex.astype(int)

print('\n\nFlight Excitation Times:\n')

for i in range(0, len(exciteID)):
    if i%2 == 0:
        print('Excitiation', exciteID[i], 'Time: (', exciteIndex[i], ',', exciteIndex[i + 1], ')')

#%% Save _init.json file

if args.save:
    # find split times, start and stop.
    # add this to json init

    def jsonRead(filename):
        with open(filename, 'r') as f:
            data = json.load(f)
            f.close()
        return data

    # Open init json file
    json_init = jsonRead(init_jsonfilename)

    # Open flight json file
    flightjson = jsonRead(flight_jsonfilename)
    testPointList = flightjson['Mission-Manager']['Test-Points']

    json_init['Test-Points'] = []

    for i in range(0, len(exciteID)):
        if i%2 == 0:
            for testPoint in testPointList:
                if testPoint['Test-ID'] == str(exciteID[i]):
                    testPoint['Start_us'] = str(exciteIndex[i])
                    testPoint['Stop_us'] = str(exciteIndex[i + 1])
                    json_init['Test-Points'].append(testPoint)

    with open(init_jsonfilename, 'w') as f:
        json.dump(json_init, f, indent = 4, ensure_ascii=False)
        f.close()

    print('Init File Updated:\n', init_jsonfilename)

else:
    print('\nInit file NOT updated\n\n')

#%% Show Plot

plt.show()
