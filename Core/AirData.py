"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np

from SensorModel import SensorErrorModel
import KinematicTransforms

# Constants
vSoundSL_mps = 340.29 # standard sea level speed of sound, m/s
pStdSL_Pa = 101325.0  # standard sea level pressure, Pa
tempStdSL_K = 288.15  # standard sea level temperature, K

L = 0.0065            # standard lapse rate, K/m
R = 8.314             # gas constant, J/kg-mol
M = 0.02895           # molecular mass dry air, kg/mol
g = 9.807             # acceleration due to gravity at sea level, m/s/s

#%% Compute Indicated airspeed
def IAS(pTip_Pa):
    
    temp = 5.0 * (((pTip_Pa / pStdSL_Pa + 1.0) ** (2.0/7.0)) - 1.0)
#    vIas_mps = vSoundSL_mps * np.sign(temp) * np.sqrt(np.sign(temp) * np.abs(temp))
    vIas_mps = vSoundSL_mps * np.sign(temp) * np.sqrt(np.abs(temp))
    
    return vIas_mps

#%% Compute Equivalent airspeed
def EAS(pTip_Pa, pStatic_Pa):
    
    temp = 5.0 * pStatic_Pa / pStdSL_Pa * (((pTip_Pa / pStatic_Pa + 1.0) ** (2.0/7.0)) - 1.0)
#    vEas_mps = vSoundSL_mps * np.sign(temp) * np.sqrt(np.sign(temp) * np.abs(temp))
    vEas_mps = vSoundSL_mps * np.sign(temp) * np.sqrt(np.abs(temp))
    
    return vEas_mps

#%% Compute True airspeed
def TAS(pTip_Pa, pStatic_Pa, temp_C):
    
    vEas_mps = EAS(pTip_Pa, pStatic_Pa)
    vTas_mps = vEas_mps * np.sqrt((temp_C + 273.15) / tempStdSL_K)
    
    return vTas_mps

#%% Compute pressure altitude
def PressAltitude(pStatic_Pa):
    
    altPress_m = (tempStdSL_K / L) * (1.0 - ((pStatic_Pa / pStdSL_Pa) ** ((L * R)/(M * g))))
    
    return altPress_m

#%% Computes density altitude
def DensAltitude(pStatic_Pa, temp_C):
    
    altDens_m = (tempStdSL_K / L) * (1.0 - (((pStatic_Pa / pStdSL_Pa) * (tempStdSL_K/(temp_C+273.15))) ** ((L*R)/(M*g - L*R))));
    
    return altDens_m

#%% Computes atmospheric density
def AtmDensity(pStatic_Pa, temp_C):

    rho_kgpm3 = (M * pStatic_Pa) / (R * (temp_C + 273.15))
    
    return rho_kgpm3


#%% Angle of Attack or Sidslip estimate when all ports are measuring pressure difference from static ring
# For alpha: Angle ports are alpha, Side ports are beta
# For beta: Angle ports are beta, Side ports are alpha
def AeroAngle1(pTip, pAngle1, pAngle2, pSide1, pSide2, kCal):

    pNorm = pTip - 0.5 * (pSide1 + pSide2)

    angle = (pAngle1 - pAngle2) / (kCal * pNorm)

    return angle

#%% Angle of Attack estimate when pAlpha is measuring pAlpha1-pAlpha2 directly
def AeroAngle2(pTip, pAngle, kCal):

    angle = pAngle / (kCal * pTip);

    return angle


#%%
def Airspeed2NED(v_PA_P_mps, s_BL_rad, param):

    from scipy.spatial.transform import Rotation as R
        
    # Assume the rotation rate of the atmosphere is negligible
    w_AL_L_rps = np.zeros_like(v_PA_P_mps)
    
    # Compute the Rotation rate of the Probe wrt the Atm
    # w_BA_B_rps = w_BL_B_rps + T_L2B * w_AL_L_rps
    # w_BA_B_rps = w_BL_B_rps + w_AL_L_rps # FIXIT - should have transformation from L to B
    w_BA_B_rps = np.zeros_like(v_PA_P_mps)
    
    # Translate and Rotate the Pitot measurement to the Body frame
    v_BA_B_mps = KinematicTransforms.TransPitot(v_PA_P_mps, w_BA_B_rps, param['s_B_rad'], param['r_B_m']);
    
    # Transform Coordinates from B to L
    v_BA_L_mps = np.zeros_like(v_BA_B_mps)
    numSamp = v_PA_P_mps.shape[-1]
    for iSamp in range(0, numSamp):
#        T_B2L = R.from_euler('XYZ', -s_BL_rad[:,iSamp], degrees = False).as_dcm().T
        T_B2L = R.from_euler('ZYX', s_BL_rad[[2,1,0], iSamp], degrees = False).as_dcm()
    
        # Compute the NED velocity
        v_BA_L_mps[:,iSamp] = T_B2L @ v_BA_B_mps[:,iSamp]

    return v_BA_B_mps, v_BA_L_mps


#%% Airdata
def ApplyCalibration(meas, param):
    ''' Compute the Airdata Parameters with a given sensor error/calibration model.
    
    Inputs:
      meas           - Dictionary of measured airdata sensor values
        (pTip_Pa)    - Magnitude of the Differential Pressure of the Probe wrt Atm [P/A] (Pa)
        (pStatic_Pa) - Magnitude of the Static Pressure of the Probe wrt Atm [P/A] (Pa)
        (tempProbe_C)- Temperature of the Airdata transducers (C)
        (pAlpha_Pa)  - Optional, Magnitude of the Alpha1 Pressure of the Probe wrt Atm [P/A] (Pa)
        (pAlpha1_Pa) - Optional, Magnitude of the Alpha1 Pressure of the Probe wrt Atm [P/A] (Pa)
        (pAlpha2_Pa) - Optional, Magnitude of the Alpha1 Pressure of the Probe wrt Atm [P/A] (Pa)
        (pBeta_Pa)   - Optional, Magnitude of the Alpha1 Pressure of the Probe wrt Atm [P/A] (Pa)
        (pBeta1_Pa)  - Optional, Magnitude of the Alpha1 Pressure of the Probe wrt Atm [P/A] (Pa)
        (pBeta2_Pa)  - Optional, Magnitude of the Alpha1 Pressure of the Probe wrt Atm [P/A] (Pa)
        
      param          - Dictionary of airdata calibration parameters
        (r_B_m)      - Position of the Probe Frame wrt Body Frame in Body Coordinates [P/B]B (m)
        (s_B_rad)    - Orientation ('ZYX') of the Probe Frame wrt Body Frame [P/B] (rad)
        (pTip)       - Dictionary of individual sensor error model parameters
        (pStatic)    - Dictionary of individual sensor error model parameters
        (pXX)        - Dictionary of individual sensor error model parameters, matches keys in 'meas'

        (airpseed)   - Dictionary of parameters to compute airspeed from pressures
        (alt)        - Dictionary of parameters to compute altitude from pressures
        (alpha)      - Dictionary of parameters to compute alpha from pressures
        (beta)       - Dictionary of parameters to compute alpha from pressures
                
    Outputs:
      calib          - Dictionary of airdata parameters and calibrated sensor values
          (vIas_mps)
          (VEas_mps)
          (VTas_mps)
          (altPres_m)
          (alpha_rad)
          (beta_rad)
          (v_PA_P_mps) - Velocity of the Probe wrt Atm in Probe Coordinates (aka: u,v,w) [P/A]P (m/s2)
    
    '''
    
    # Constants
    r2d = 180.0 / np.pi
    
    ## Apply Error Models, Estimate the "true" measures from the Error Models
    # Apply Pitot-Static Error Models
    calib = {}
    for key, val in meas.items():
        if key in ['pTip_Pa', 'pStatic_Pa', 'tempProbe_C', 'pAlpha_Pa', 'pAlpha1_Pa', 'pAlpha2_Pa', 'pBeta_Pa', 'pBeta1_Pa', 'pBeta2_Pa']:    
            pKey = key.split('_')[0] # The param key should be just the units stripped off
            if pKey in param.keys():
                calib[key] = SensorErrorModel(val, param[pKey])
            else: # Just copy the data
                calib[key] = val
                
    
    # Determine Probe type
    typeProbe = None
    if 'pAlpha_Pa' in calib.keys():
        typeProbe = '5Hole2'
    elif 'pAlpha1_Pa' in calib.keys():
        typeProbe = '5Hole1'
    else:
        typeProbe = 'Pitot'
    
    #
    numSamp = calib['pTip_Pa'].shape[-1]
    calib['v_PA_P_mps'] = np.zeros((3, numSamp))
    
    if typeProbe in ['Pitot']:
        # Airspeeds
        calib['vIas_mps'] = IAS(calib['pTip_Pa'])
        calib['vEas_mps'] = EAS(calib['pTip_Pa'], calib['pStatic_Pa'])
        calib['vTas_mps'] = TAS(calib['pTip_Pa'], calib['pStatic_Pa'], calib['tempProbe_C'])
        
        # Pressure Altitude
        calib['altPress_m'] = PressAltitude(calib['pStatic_Pa'])
        
        # Airspeed Vector ['u', 'v', 'w']
        calib['v_PA_P_mps'][0] = calib['vIas_mps']
        
    if typeProbe in ['5Hole1']:
        # Airspeeds
        calib['vIas_mps'] = IAS(calib['pTip_Pa'])
        calib['vEas_mps'] = EAS(calib['pTip_Pa'], calib['pStatic_Pa'])
        calib['vTas_mps'] = TAS(calib['pTip_Pa'], calib['pStatic_Pa'], calib['tempProbe_C'])
        
        # Pressure Altitude
        calib['altPress_m'] = PressAltitude(calib['pStatic_Pa'])
        
        # Inflow Angles: Angle of Attack and Sideslip
        calib['alpha_rad'] = AeroAngle1(calib['pTip_Pa'], calib['pAlpha1_Pa'], calib['pAlpha2_Pa'], calib['pBeta1_Pa'], calib['pBeta2_Pa'], param['alphaCal'])
        calib['alpha_deg'] = calib['alpha_rad'] * r2d
        
        calib['beta_rad'] = AeroAngle1(calib['pTip_Pa'], calib['pBeta1_Pa'], calib['pBeta2_Pa'], calib['pAlpha1_Pa'], calib['pAlpha2_Pa'], param['betaCal'])
        calib['beta_deg'] = calib['beta_rad'] * r2d
        
        # Airspeed Vector ['u', 'v', 'w']
        calib['v_PA_P_mps'][0] = calib['vIas_mps']
#        calib['v_PA_P_mps'][0] = calib['vIas_mps'] / (np.cos(calib['alpha_rad']) * np.cos(calib['beta_rad']))
        calib['v_PA_P_mps'][1] = calib['vIas_mps'] * np.sin(calib['beta_rad'])
        calib['v_PA_P_mps'][2] = calib['v_PA_P_mps'][0] * np.tan(calib['alpha_rad'])

    if typeProbe in ['5Hole2']:
        # Airspeeds
        calib['vIas_mps'] = IAS(calib['pTip_Pa'])
        calib['vEas_mps'] = EAS(calib['pTip_Pa'], calib['pStatic_Pa'])
        calib['vTas_mps'] = TAS(calib['pTip_Pa'], calib['pStatic_Pa'], calib['tempProbe_C'])
        
        # Pressure Altitude
        calib['altPress_m'] = PressAltitude(calib['pStatic_Pa'])
        
        # Inflow Angles: Angle of Attack and Sideslip
        calib['alpha_rad'] = AeroAngle2(calib['pTip_Pa'], calib['pAlpha_Pa'], param['alphaCal'])
        calib['alpha_deg'] = calib['alpha_rad'] * r2d
        
        calib['beta_rad'] = AeroAngle2(calib['pTip_Pa'], calib['pBeta_Pa'], param['betaCal'])
        calib['beta_deg'] = calib['beta_rad'] * r2d
        
        # Airspeed Vector ['u', 'v', 'w']
        calib['v_PA_P_mps'][0] = calib['vIas_mps']
#        calib['v_PA_P_mps'][0] = calib['vIas_mps'] / (np.cos(calib['alpha_rad']) * np.cos(calib['beta_rad']))
        calib['v_PA_P_mps'][1] = calib['vIas_mps'] * np.sin(calib['beta_rad'])
        calib['v_PA_P_mps'][2] = calib['v_PA_P_mps'][0] * np.tan(calib['alpha_rad'])
        
        
    return calib
