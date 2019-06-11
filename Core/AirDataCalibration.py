"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np

import AirData

# Constants

#%%
# Define the Cost Function for the Optimization
def CostFunc(xOpt, optInfo, oDataList, param):
    
    # Unpack the optimization 'x' vector; vMean is the first 3 vals, the rest are param entries
    vWind = []
    iEnd = 0
    for iSeg, seg in enumerate(optInfo['wind']):
        lenX = len(seg['val'])
        iStart = iEnd
        iEnd = iStart + lenX
        iSlice = slice(iStart, iEnd)
        vWind.append(xOpt[iSlice])
    
    # Apply global free variables
    param['pTip']['K'] = xOpt[iEnd]
    param['pTip']['bias'] = xOpt[iEnd+1]
    
    costList = []
    for iSeg, oData in enumerate(oDataList):
        
        # Apply segment-wise free variables
        oData['vMean_AE_L_mps'] = vWind[iSeg]
        
        # Compute the Airspeed solution with error model parameters
        calib = AirData.ApplyCalibration(oData, param)
        v_BA_B_mps, v_BA_L_mps = AirData.Airspeed2NED(calib['v_PA_P_mps'], oData['sB_L_rad'], param)
    
        # Compute the Ground Speeds
        # Wind Estimate, assume constant at mean value
        numSamp = v_BA_B_mps.shape[-1]
        v_AE_L_mps = np.repeat([oData['vMean_AE_L_mps']], numSamp, axis=0).T
        
        # Compute the Groudspeeds from the Corrected Airspeeds using the Wind Estimate
        vEst_BE_L_mps = v_AE_L_mps + v_BA_L_mps
    
        # Cost for each segment
        cost = np.linalg.norm(oData['vB_L_mps'] - vEst_BE_L_mps, 2) / numSamp
        costList.append(cost)
    
    # Combined Total Cost
    costTotal = np.mean(costList)
    
    # Display progress
    optInfo['Nfeval'].append(optInfo['Nfeval'][-1] + 1)
    optInfo['cost'].append(costTotal)
    if optInfo['Nfeval'][-1]%10 == 0:
#        plt.plot(optInfo['Nfeval'], optInfo['cost'])
        print('Nfeval: ', optInfo['Nfeval'][-1], 'Cost: ', optInfo['cost'][-1])

    return costTotal


#%% Air Calibration Wrapper
def EstCalib(opt, oDataList, param):
    
    from scipy.optimize import minimize
    
    # Pack segment-wise free variables
    xOpt = []
    xBnds = []
    for iSeg, seg in enumerate(opt['wind']):
        for i in range(0, len(seg['val'])):
            xOpt.append(seg['val'][i])
            xBnds.append([seg['lb'][i], seg['ub'][i]])

    for p in opt['param']:
        xOpt.append(p['val'])
        xBnds.append([p['lb'], p['ub']])
    
    
    # Initial Guess, based on assuming perfect calibration, constant wind.
    for iSeg, oData in enumerate(oDataList):
        calib = AirData.ApplyCalibration(oData, param)
        v_BA_B_mps, v_BA_L_mps = AirData.Airspeed2NED(calib['v_PA_P_mps'], oData['sB_L_rad'], param)
        
        # Compute the Mean of the Wind Estimate, in NED
        # Subtract the Estimated Body Airspeed from the Inertial Velocity
        #v_AE_L = v_BE_L - v_BA_L
        oData['vMean_AE_L_mps'] = np.mean(oData['vB_L_mps'] - v_BA_L_mps, axis=1)
               

    # Optimization Info dict, pass to function
    optInfo = {}
    optInfo['wind'] = opt['wind']
    optInfo['param'] = opt['param']
    optInfo['Nfeval'] = [0]
    optInfo['cost'] = [0]
    opt['args'] = (optInfo, oDataList, param)
    
    # Run Optimization
    optResult = minimize(CostFunc, xOpt, bounds = xBnds, args = opt['args'], method = opt['Method'], options = opt['Options'])
    
    # Copy Results to xOpt
    xOpt = np.copy(optResult['x'])
    
    return optResult 