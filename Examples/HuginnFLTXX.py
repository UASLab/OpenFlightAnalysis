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
flt = 'FLT03'

fileLog = os.path.join(pathBase, ac, ac + flt, ac + flt + '.h5')
fileTestDef = os.path.join(pathBase, ac, ac + flt, ac.lower() + '_def.json')
fileSysConfig = os.path.join(pathBase, ac, ac + flt, ac.lower() + '.json')


#%%
# Read in raw h5 data into dictionary and data
oData, h5Data = Loader.Log_RAPTRS(fileLog, fileSysConfig)

# Plot Overview of flight
#oData = OpenData.Segment(oData, ('time_s', [950, 970]))
OpenData.PlotOverview(oData)


#%% Find Excitation Times
excList = OpenData.FindExcite(oData)
segList = []
print('\n\nFlight Excitation Times:\n')
for iExc in range(0, len(excList)):
    print('Excitiation: ', excList[iExc][0], ', Time: [', excList[iExc][1][0], ',', excList[iExc][1][1], ']')

    segList.append( ('time_us', excList[iExc][1]))

oDataExc = OpenData.Segment(oData, segList)
#OpenData.PlotOverview(oDataExc[0])


#%% Save _init.json file
# Open init json file
fltDef = {}
#fltDef = JsonRead(fileTestDef)

# Open flight json file
fltConfig = Loader.JsonRead(fileSysConfig)
testPointList = fltConfig['Mission-Manager']['Test-Points']

fltDef['Test-Points'] = OpenData.TestPointOut(excList, testPointList)

if True:
    Loader.JsonWrite(fileTestDef, fltDef)
    print('Init File Updated:\n', fileTestDef)
else:
    import json
    json.dumps(fltDef, indent = 4, ensure_ascii=False)
    print('\nInit file NOT updated\n\n')

