#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Louis Mueller
University of Minnesota UAV Lab

Description:
Analysis is used for in depth analysis of Goldy and JSBSim flight data
in .h5 and .csv formats, respectively. Given the flightFolder and desired excitations
to analyze, analysis.py will split and compare flight data from each flight.

General Steps:
1. Load .h5 (and csv) data from flightFolder
2. Load requested excitation times from _init.json
3. Split flight data from using times and store desired excitations
4. ?Interpolate? and compare flight and sim data

"""

import os
import json
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from h5read import h5read
from JSBcsvRead import JSBcsvRead

parser = argparse.ArgumentParser(description="Load h5, sim, and init files and analyze")
parser.add_argument("flightFolder", help="folder that includes .h5, _init.json, and aircraft config .json")
parser.add_argument("simFolder", help="folder that includes .csv from sim")
parser.add_argument("--savefigs", action="store_true")
parser.add_argument("--excitations", nargs='+', help="list of generated excitations to analyze ( --excitations 1 2 3 4 )")
parser.add_argument("--manual", nargs='+', help="list of manually entered tests to analyze ( --manual 1 2 3 4 )")
args = parser.parse_args()

basepath = '/Users/louismueller/Documents/UAV_LAB/Analysis/FlightData'

flight = str(args.flightFolder)
sim = str(args.simFolder)
h5filename = os.path.join(basepath, 'Thor', flight, flight + '.h5')
init_jsonfilename = os.path.join(basepath, 'Thor',flight,flight + '_init.json')
flight_jsonfilename = os.path.join(basepath, 'Thor', flight, flight + '.json')
simfilename = os.path.join(basepath, 'Thor', sim, sim + '.csv')

#%% Analysis Runs
# Specify flights and segments to load/slice

# inputs are filename, init file with times we want (manual and auto gen?)

# From Flt121 load test 1, 2, 3
# From Flt122 load test 3, 5, 'Landing'

# Load flight (and sim) and _init (propogate meta data)

# Read h5
h5dict, h5data = h5read(h5filename)

# Read CSV
simdict = JSBcsvRead(simfilename)

# Read _init json
def jsonRead(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
        f.close()
    return data

json_init = jsonRead(init_jsonfilename)
autoPoints = json_init['Auto-Points']
manualPoints = json_init['Manual-Points']

# Find start and stop times for requested events
startTime = []
stopTime = []

if args.excitations:
    for exIdx in args.excitations:
        for init in autoPoints:
            if init['Test-ID'] == exIdx:

                istart = init['Start_us']
                startTime.append(istart)
                print(istart)

                istop = init['Stop_us']
                stopTime.append(istop)
                print(istop)

if args.manual:
    for manIdx in args.manual:
        for init in manualPoints:
            if init['Test-ID'] == manIdx:
                
                istart = init['Start_us']
                startTime.append(istart)
                print(istart)

                istop = init['Stop_us']
                stopTime.append(istop)
                print(istop)              

# Slice         
EXCITATIONSSPLIT = {}

for excitation in args.excitations:
    # Initialize excitation dictionary and excitation names, store definitions
    EXCITATIONSSPLIT['excitation'+str(excitation)] = {}
    EXCITATIONSSPLIT['excitation'+str(excitation)]['flt'] = {}
    EXCITATIONSSPLIT['excitation'+str(excitation)]['sim'] = {}
    EXCITATIONSSPLIT['excitation'+str(excitation)]['def'] = json_init

# Store Split h5
for k,v in h5dict.items(): 
    for i in range(len(args.excitations)):
        segment = np.hsplit(v, np.array([int(startTime[i]), int(stopTime[i])]))
        EXCITATIONSSPLIT['excitation'+str(excitation)]['flt'][k] = segment[1]   
        #print(k)

# Assume there is sim data? 
for k,v in simdict.items(): 
    for i in range(len(args.excitations)):
        segment = np.hsplit(v, np.array([int(startTime[i]), int(stopTime[i])]))
        EXCITATIONSSPLIT['excitation'+str(excitation)]['sim'][k] = segment[1]   
        #print(k)
        


# Test Plot

f, ax1 = plt.subplots()
flightTime = EXCITATIONSSPLIT['excitation1']['flt']['Time_us']
flightAlt = EXCITATIONSSPLIT['excitation1']['flt']['altAD_m']
ax1.plot(flightTime, flightAlt)

f, ax2 = plt.subplots()
simTime = EXCITATIONSSPLIT['excitation1']['sim']['time_s']
simAlt = EXCITATIONSSPLIT['excitation1']['sim']['alt_AGL_ft']
ax2.plot(simTime, simAlt)


plt.show()     


# Analysis

#seg[0]['sim'] <- from sim
#seg[0]['flt'] <- from FLT
#seg[0]['def'] <- from _init
