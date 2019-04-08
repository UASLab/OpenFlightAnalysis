#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 28 15:05:36 2018

@author: louismueller
"""

# import
import numpy as np


def exciteSplit(FSTRUCT):

    
    #%% Splitting based on testID
    exciteID = np.array([])
    exciteIndex = np.array([])
    
    for ID in range(0,len(FSTRUCT['exciteEngage'])):
        
        # Flags
        currentTest = FSTRUCT['exciteEngage'][ID]
        previousTest = FSTRUCT['exciteEngage'][ID - 1]
    
        # Beginning Excitation
        if  (previousTest != 1 and currentTest == 1):
            exciteID = np.append(exciteID, FSTRUCT['testID'][ID - 1]) #Note Test ID Name
            exciteIndex = np.append(exciteIndex, ID) #Note Start of Excitation
        
        # Ending Excitation
        if  (previousTest == 1 and currentTest == 0):
            exciteIndex = np.append(exciteIndex, ID) #Note End of Excitation
    
    exciteID = exciteID.astype(int)
    exciteIndex = exciteIndex.astype(int)
    
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
                
 #%%           
    return(EXCITATIONSSPLIT)