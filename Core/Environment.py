"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np
#import scipy.signal as signal

# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps

ft2m = 0.3048
m2ft = 1 / ft2m



#%% Generate Turbulence
def TurbSpectDryden(sigma, L_ft, freq_rps, V_fps, b_ft):
    '''
    sigma - Turbulence Intensity [u,v,w]
    L_ft - Turbulence length scale [u,v,w]
    freq_rps - frequency rad/sec (np.array)
    V_fps - linear velocity
    
    Source: MIL-HDBK-1797B
    MIL-HDBK-1797B is all imperial units, we want all SI units
    '''
    
    sigma_u = sigma[0]
    sigma_v = sigma[1]
    sigma_w = sigma[2]
    
    Lu_ft = L_ft[0]
    Lv_ft = L_ft[1]
    Lw_ft = L_ft[2]

    # u - direction
    scale = sigma_u**2 * (2 * Lu_ft) / (np.pi * V_fps)
    LO2 = (Lu_ft * freq_rps/V_fps)**2
    
    Pu = scale * (1 / (1 + LO2))
    
    # v - direction
    scale = sigma_v**2 * (2 * Lv_ft) / (np.pi * V_fps)
    LO2 = (Lv_ft * freq_rps/V_fps)**2
    
    Pv = scale * ((1 + 12 * LO2) / (1 + 4 * LO2)**2)

    # w - direction
    scale = sigma_w**2 * (2 * Lw_ft) / (np.pi * V_fps)
    LO2 = (Lw_ft * freq_rps/V_fps)**2
    
    Pw = scale * ((1 + 12 * LO2) / (1 + 4 * LO2)**2)
        
    # p - direction
    scale = sigma_w**2 / (2 * V_fps * Lw_ft)
    Pp = scale * 0.8 * (2 * np.pi * Lw_ft / (4*b_ft))**(1/3) / (1 + (4*b_ft*freq_rps/(np.pi*V_fps))**2)
 
    # q - direction
    Pq = (freq_rps/V_fps)**2 / (1 + (4*b_ft*freq_rps / (np.pi * V_fps))**2) * Pw
            
    # r - direction
    Pr = -(freq_rps/V_fps)**2 / (1 + (3*b_ft*freq_rps / (np.pi * V_fps))**2) * Pv
    
    return [Pu, Pv, Pw], [Pp, Pq, Pr]


def TurbIntensityLow(h_ft, W20_fps = None, level = None):
    kts2fps = 1.68781
    if (W20_fps is None):
        if level.lower() == 'light':
            W20_fps = 15 * kts2fps
        if level.lower() == 'moderate':
            W20_fps = 30 * kts2fps
        if level.lower() == 'severe':
            W20_fps = 45 * kts2fps
            
    sigma_u = (0.1 * W20_fps) / (0.177 + 0.000823 * h_ft)**1.2
    sigma_v = sigma_u
    sigma_w = 0.1 * W20_fps
    
    return [sigma_u, sigma_v, sigma_w]

def TurbLengthScaleLow(h_ft):
    # h_ft < 1000
    # u,v,w aligned with mean wind
    
    # Ensure the height AGL is below 1000 ft
    h_ft = min(h_ft, 1000)
    
    Lu_ft = h_ft / (0.177 + 0.000823 * h_ft)**1.2
    Lv_ft = 0.5 * Lu_ft
    Lw_ft = 0.5 * h_ft

    return [Lu_ft , Lv_ft, Lw_ft]

def TurbLengthScaleHigh():
    # h_ft >= 2000
    # u,v,w aligned with body 

    Lu_ft = 1750
    Lv_ft = 0.5 * 1750
    Lw_ft = 0.5 * 1750

    
    return [Lu_ft , Lv_ft, Lw_ft] 

