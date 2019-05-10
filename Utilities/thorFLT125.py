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


fileLog = '/home/rega0051/FlightArchive/Thor/ThorFLT125/ThorFLT125.h5'
fileTestDef = '/home/rega0051/FlightArchive/Thor/ThorFLT125/FLT_Def.json'
fileSysConfig = '/home/rega0051/FlightArchive/Thor/ThorFLT125/thor.json'

#%%
# Read in raw h5 data into dictionary and data
oData, h5Data = Loader.Log_RAPTRS(fileLog)

# Plot Overview of flight
OpenData.PlotOverview(oData)

#%% Find Excitation Times
excList = OpenData.FindExcite(oData)

print('\n\nFlight Excitation Times:\n')
for iExc in range(0, len(excList)):
    print('Excitiation: ', excList[iExc][0], ', Time: [', excList[iExc][1][0], ',', excList[iExc][1][1], ']')


#%% Save _init.json file
# Open init json file
testDef = {}
#json_init = JsonRead(fileTestDef)

# Open flight json file
flightjson = Loader.JsonRead(fileSysConfig)
testPointList = flightjson['Mission-Manager']['Test-Points']

testDef['Test-Points'] = OpenData.TestPointOut(excList, testPointList)


if False:
    Loader.JsonWrite(fileTestDef, testDef)
    print('Init File Updated:\n', fileTestDef)
else:
    json.dumps(testDef, indent = 4, ensure_ascii=False)
    print('\nInit file NOT updated\n\n')


#%% RTSM
    
segList = [('time_us', excList[0][1]), 
           ('time_us', excList[1][1]), 
           ('time_us', excList[2][1]), 
           ('time_us', excList[4][1]), 
           ('time_us', excList[5][1]), 
           ('time_us', excList[6][1]), 
           ('time_us', excList[8][1]), 
           ('time_us', excList[9][1]), 
           ('time_us', excList[10][1])]

oDataSegs = OpenData.Segment(oData, segList)

iSeg = 7
plt.plot(oDataSegs[iSeg]['time_s'], oDataSegs[iSeg]['cmdEff'][1])

