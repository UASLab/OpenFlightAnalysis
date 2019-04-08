#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 28 15:04:17 2018

@author: louismueller
"""
import os
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from h5read import h5read
from jsonRead import jsonRead
from exciteSplit import exciteSplit

basepath = '/Users/louismueller/Documents/UAV_LAB/Analysis/FlightData'

h5filename = os.path.join(basepath, 'Thor', 'ThorFlt121.h5')
init_jsonfilename = os.path.join(basepath, 'Thor', 'thor_init.json')
flight_jsonfilename = os.path.join(basepath, 'Thor', 'thor.json')



#%% Overview

## python3 overview ThorFlt121.h5 --save thor_init.json --config thor.json
# Load flight (and sim data)

FSTRUCT, h5data = h5read(h5filename)

# Find interesting stuff: takeoff, landing, excitations etc.

# Overview plots

time = FSTRUCT['Time_us']

# altitude 
altAD_m = FSTRUCT['altAD_m']
plt.plot(time, altAD_m)
plt.xlabel('Time (us)')
plt.ylabel('Altitude (m)')
plt.title('Air Data Altitude')
plt.show()

# xy position
rx_deg = FSTRUCT['rGPS_BE_G_ddm'][0]
ry_deg = FSTRUCT['rGPS_BE_G_ddm'][1]
plt.axis('equal')
plt.xlabel('Latitude (deg)')
plt.ylabel('Longitude (deg)')
plt.title('Latitude and Longitude')
plt.plot(rx_deg, ry_deg)
plt.show()

# Voltage
MinCellVolt_V = FSTRUCT['MinCellVolt_V']
plt.plot(time, MinCellVolt_V)
plt.xlabel('Time (us)')
plt.ylabel('Min Cell Voltage (V)')
plt.title('Min Cell Voltage')
plt.show()

# 3D position plot
# I need to set x and y equal, maybe use sharex, sharey?
rz_m = FSTRUCT['rGPS_BE_G_ddm'][2]
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(rx_deg, ry_deg, rz_m)
plt.axis('equal')
plt.xlabel('Latitude (deg)')
plt.ylabel('Longitude (deg)')
plt.title('Flight Path')
plt.show()

# Save _init.json file




#%% Analysis Runs
# Specify flights and segments to load/slice

# From Flt121 load test 1, 2, 3
# From Flt122 load test 3, 5, 'Landing'

# Load flight (and sim) and _init (propogate meta data)

# Slice

# Analysis



#seg[0]['sim'] <- from sim
#seg[0]['flt'] <- from FLT
#seg[0]['def'] <- from _init



#%% Joining JSON and Excitations



excitations = exciteSplit(FSTRUCT)

json_init = jsonRead(init_jsonfilename)

for k,v in excitations.items():
    excitations[k].update(json_init)
    
    
#%% Updating init json with excitation log from flight json

flightjson = jsonRead(flight_jsonfilename)
json_init['Test-Points'] = flightjson['Mission-Manager']['Test-Points']

with open(init_jsonfilename, 'w') as f:
    json.dump(json_init, f, indent = 4, ensure_ascii=False)
    f.close()
