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
python Overview.py [flightFolder] [--save]
python Overview.py ThorFLT123 --save

Future Improvements:
- Overview of sim csv
- Save figures
"""

# Import Libraries
import os
import json
import argparse

import Loader
import OpenData

#%% Parse and Load
# Include arguments for load
parser = argparse.ArgumentParser(description = "Load h5 file and load overview plots")
parser.add_argument("path")
parser.add_argument("fileLogName")
parser.add_argument("fileDefName")
parser.add_argument("fileConfigName")
parser.add_argument("--save", action = "store_true")
args = parser.parse_args()

pathData = str(args.path)
fileLogName = str(args.fileLogName)
fileDefName = str(args.fileDefName)
fileConfigName = str(args.fileConfigName)

# Locate files
fileLog = os.path.join(pathData, fileLogName)
fileTestDef = os.path.join(pathData, fileDefName)
fileSysConfig = os.path.join(pathData, fileConfigName)


#%%
# Read in raw h5 data into dictionary and data
oData, h5Data = Loader.Log_RAPTRS(fileLog)

# Plot Overview of flight
OpenData.PlotOverview(oData)

#%% Find Excitation Times
exciteID, exciteIndex = OpenData.FindExcite(oData)

print('\n\nFlight Excitation Times:\n')
for i in range(0, len(exciteID)):
    if i%2 == 0:
        print('Excitiation', exciteID[i], 'Time: (', exciteIndex[i], ',', exciteIndex[i + 1], ')')

#%% Save _init.json file
# Open init json file
json_init = {}
#json_init = JsonRead(fileTestDef)

# Open flight json file
flightjson = Loader.JsonRead(fileSysConfig)

json_init['Test-Points'] = OpenData.TestPointOut(exciteID, exciteIndex, flightjson)

if args.save:
    Loader.JsonWrite(fileTestDef, json_init)
    print('Init File Updated:\n', fileTestDef)
else:
    json.dumps(json_init, indent = 4, ensure_ascii=False)
    print('\nInit file NOT updated\n\n')
