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
1. Load .h5 data and


"""

import os
import json
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from h5read import h5read

parser = argparse.ArgumentParser(description="Load h5, sim, and init files and analyze")
parser.add_argument("flightFolder", help="folder that includes .h5, _init.json, and aircraft config .json")
parser.add_argument("--savefigs", action="store_true")
parser.add_argument("--excitations", nargs='+', help="list of generated excitations to analyze ( --excitations 1 2 3 4 )")
parser.add_argument("--manual", nargs='+', help="list of manually entered tests to analyze ( --manual 1 2 3 4 )")
args = parser.parse_args()

basepath = '/Users/louismueller/Documents/UAV_LAB/Analysis/FlightData'

flight = str(args.flightFolder)
h5filename = os.path.join(basepath, 'Thor', flight, flight + '.h5')
init_jsonfilename = os.path.join(basepath, 'Thor',flight,'thor_init.json')
flight_jsonfilename = os.path.join(basepath, 'Thor', flight,'thor.json')


#%% Analysis Runs
# Specify flights and segments to load/slice

# inputs are filename, init file with times we want (manual and auto gen?)

# From Flt121 load test 1, 2, 3
# From Flt122 load test 3, 5, 'Landing'

# Load flight (and sim) and _init (propogate meta data)

h5dict, h5data = h5read(h5filename)

def jsonRead(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
        f.close()
    return data

json_init = jsonRead(init_jsonfilename)
testPoints = json_init['Test-Points']

# Find start and stop times for requested events
startTime = []
stopTime = []

if args.excitations:
    for exIdx in args.excitations:
        for init in testPoints:
            if init['Test-ID'] == exIdx:

                istart = init['Start_us']
                startTime.append(istart)
                print(istart)

                istop = init['Stop_us']
                stopTime.append(istop)
                print(istop)

#if args.manual:










for k,v in excitations.items():
    excitations[k].update(json_init)





# Slice

EXCITATIONSSPLIT = {}

for excitation in range(len(exciteID)):
    EXCITATIONSSPLIT['excitation'+str(exciteID[excitation])] = {}


for k,v in FSTRUCT.items(): # going through everything in fstruct
    segments = np.hsplit(v,exciteIndex[0:None]) # splitting fom excitations
    excitationVals = []
    for seg in range(len(segments)): # Go through each segment
        if seg%2 == 1: # get every other
            excitationVals.append(segments[seg])

        for vals in range(len(excitationVals)):
            EXCITATIONSSPLIT['excitation'+str(exciteID[vals])][k] = excitationVals[vals]






# Analysis

#seg[0]['sim'] <- from sim
#seg[0]['flt'] <- from FLT
#seg[0]['def'] <- from _init
