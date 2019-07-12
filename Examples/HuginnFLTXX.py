"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Analysis for Huginn (mAEWing2) FLT XX
"""

#%%
# Import Libraries
import os
import numpy as np
import matplotlib.pyplot as plt

# Hack to allow loading the Core package
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), "..")))
    path.insert(0, abspath(join(dirname(argv[0]), "..", 'Core')))
    
    del path, argv, dirname, abspath, join

from Core import Loader
from Core import OpenData


# Constants
hz2rps = 2 * np.pi
rps2hz = 1 / hz2rps

pathBase = os.path.join('/home', 'rega0051', 'FlightArchive')
ac = 'Huginn'
flt = 'FLT06'

fileLog = os.path.join(pathBase, ac, ac + flt, ac + flt + '.h5')
fileTestDef = os.path.join(pathBase, ac, ac + flt, ac.lower() + '_def.json')
fileSysConfig = os.path.join(pathBase, ac, ac + flt, ac.lower() + '.json')


#%%
# Read in raw h5 data into dictionary and data
oData, h5Data = Loader.Log_RAPTRS(fileLog, fileSysConfig)

# Plot Overview of flight
#oData = OpenData.Segment(oData, ('time_s', [380, 1050]))
OpenData.PlotOverview(oData)


#%% Find Excitation Times
excList = OpenData.FindExcite(oData)

print('\n\nFlight Excitation Times:\n')
for iExc in range(0, len(excList)):
    print('Excitiation: ', excList[iExc][0], ', Time: [', excList[iExc][1][0], ',', excList[iExc][1][1], ']')

segList = [('time_us', excList[0][1])]

oDataExc = OpenData.Segment(oData, segList)
#OpenData.PlotOverview(oDataExc[0])


#%% Save _init.json file
# Open init json file
json_init = {}
#json_init = JsonRead(fileTestDef)

# Open flight json file
flightjson = Loader.JsonRead(fileSysConfig)
testPointList = flightjson['Mission-Manager']['Test-Points']

json_init['Test-Points'] = OpenData.TestPointOut(excList, testPointList)

if True:
    Loader.JsonWrite(fileTestDef, json_init)
    print('Init File Updated:\n', fileTestDef)
else:
    import json
    json.dumps(json_init, indent = 4, ensure_ascii=False)
    print('\nInit file NOT updated\n\n')

