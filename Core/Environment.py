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
def TurbSpectVonKarman(sigma, L_ft, omega):
    '''
    sigma - Turbulence Intensity [u,v,w]
    L_ft - Turbulence length scale [u,v,w]
    omega - spatial frequency (np.array)

    # Units of omega, dictate the units of Puvw
    #
    # omega             : (rad/ft)    :: Puvw : sigma_w^2 * L/pi ((ft/sec)^2 * rad/ft)
    # omega/(2*pi)      : (cycle/ft)  :: Puvw : sigma_w^2 * 2*L ((ft/sec)^2 * cycle/ft)
    # omega*L           : (rad)       :: Puvw : sigma_w^2 * 1/pi ((ft/sec)^2 * rad)
    # omega/V           : (rad/s)     :: Puvw : sigma_w^2 * L/(V*pi) ((ft/sec)^2 * rad/s)
    # omega*(2*pi)/V    : (cycle/s)   :: Puvw : sigma_w^2 * 2*L/V ((ft/sec)^2 * cycle/s)

    Source: MIL-HDBK-1797B
    MIL-HDBK-1797B is all imperial units, we want all SI units
    '''
    # Common scale equation
    scale = np.asarray(sigma)**2 * (2 * np.asarray(L_ft)) / np.pi

    # u - direction
    LO2 = (1.339 * L_ft[0] * omega)**2
    Pu = scale[0] / (1 + LO2)**(5/6)

    # v - direction
    LO2 = (2 * 1.339 * L_ft[1] * omega)**2
    Pv = scale[1] * (1 + 8/3 * LO2) / (1 + LO2)**(11/6)

    # w - direction
    LO2 = (2 * 1.339 * L_ft[2] * omega)**2
    Pw = scale[2] * (1 + 8/3 * LO2) / (1 + LO2)**(11/6)

    return np.array([Pu, Pv, Pw])


def TurbSpectDryden(sigma, L_ft, omega):
    '''
    sigma - Turbulence Intensity [u,v,w]
    L_ft - Turbulence length scale [u,v,w]
    omega - frequency (np.array)

    # Units of omega, dictate the units of Puvw
    #
    # omega             : (rad/ft)    :: Puvw : sigma_w^2 * L/pi ((ft/sec)^2 * rad/ft)
    # omega/(2*pi)      : (cycle/ft)  :: Puvw : sigma_w^2 * 2*L ((ft/sec)^2 * cycle/ft)
    # omega*L           : (rad)       :: Puvw : sigma_w^2 * 1/pi ((ft/sec)^2 * rad)
    # omega/V           : (rad/s)     :: Puvw : sigma_w^2 * L/(V*pi) ((ft/sec)^2 * rad/s)
    # omega*(2*pi)/V    : (cycle/s)   :: Puvw : sigma_w^2 * 2*L/V ((ft/sec)^2 * cycle/s)

    Source: MIL-HDBK-1797B
    MIL-HDBK-1797B is all imperial units, we want all SI units
    '''
    # Common scale equation
    scale = np.asarray(sigma)**2 * (2 * np.asarray(L_ft)) / np.pi

    # u - direction
    LO2 = (L_ft[0] * omega)**2
    Pu = scale[0] / (1 + LO2)

    # v - direction
    LO2 = (L_ft[1] * omega)**2
    Pv = scale[1] * (1 + 12 * LO2) / (1 + 4 * LO2)**2

    # w - direction
    LO2 = (L_ft[2] * omega)**2
    Pw = scale[2] * (1 + 12 * LO2) / (1 + 4 * LO2)**2

    return np.array([Pu, Pv, Pw])


# Rotation Rates
def TurbSpectRate(Puvw, sigma, L_ft, freq_rps, V_fps, b_ft):
    '''
    Puvw - Turbulence Spectrum [u,v,w] ((ft/sec)^2 * rad/s)
    sigma - Turbulence variation [u,w,w] (ft/sec)^2
    L_ft - Turbulence length scale [u,v,w] (ft)
    freq_rps - frequency (rad/s)
    V_fps - Velocity (ft/s)
    b_ft - Wing Span (ft)
    '''

    omega = freq_rps / V_fps

    # p - direction
    scale = sigma[2]**2 / (2 * V_fps * L_ft[2])
    Pp = scale * 0.8 * (2 * np.pi * L_ft[2] / (4 * b_ft))**(1/3) / (1 + (4 * b_ft * omega / np.pi)**2)

    # q - direction
    Pq = omega**2 / (1 + (4 * b_ft * omega / np.pi)**2) * Puvw[2]

    # r - direction
    Pr = -omega**2 / (1 + (3 * b_ft * omega / np.pi)**2) * Puvw[1]


    return np.array([Pp, Pq, Pr])


# Turbulence Intesity
def TurbIntensity(h_ft = None, U20_fps = None, level = 0):
    from scipy.interpolate import interp1d
    from scipy.interpolate import interp2d

    # level can be either 'light'/'common', 'moderate'/'uncommon', 'severe'/'extrordinary', or the desired probability of exceedance
    # Table of Probability of Exceedance is from MIL-DTL-9490E
    # 'severe' follows the 1e-5 curve
    # 'moderate' follows the 1e-3 curve
    # 'light' follows the

    kts2fps = 1.68781

    hLow_ft = 1000
    hHi_ft = 2000

    sigma_u = None
    sigma_v = None
    sigma_w = None

    if type(level) is str:
        if level.lower() in 'light' or 'common':
            probExceed = 1e-1
        elif level.lower() in 'moderate' or 'uncommon':
            probExceed = 1e-3
        elif level.lower() in 'severe' or 'extrordinary':
            probExceed = 1e-5
    else:
        probExceed = level

#        # 'Light'
#        hBrk_ft = np.array([1e3, 9e3, 18e3, 80e3])
#        probExceedBrk = np.array([5, 5, 3, 3])
#
#        # 'Moderate'
#        hBrk_ft = np.array([1e3, 14e3, 45e3, 80e3])
#        probExceedBrk = np.array([10, 10, 3, 3])
#
#        # 'Severe'
#        hBrk_ft = np.array([1e3, 4e3, 25e3, 80e3])
#        probExceedBrk = np.array([15, 21.5, 21.5, 3])

    if h_ft >= hHi_ft:

        hBrk_ft = np.array([500, 1750, 3750, 7500, 15e3, 25e3, 35e3, 45e3, 55e3, 65e3, 75e3, 80e3, 100e3])
        probExceedBrk = np.array([2e-1, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6])

        sigmaTable_fps = np.array([
            [ 3.2,  2.2, 1.5,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0],
            [ 4.2,  3.6, 3.3,  1.6,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0],
            [ 6.6,  6.9, 7.4,  6.7,  4.6,  2.7,  0.4,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0],
            [ 8.6,  9.6, 10.6, 10.1, 8.0,  6.6,  5.0,  4.2,  2.7,  0.0,  0.0,  0.0,  0.0],
            [11.8, 13.0, 16.0, 15.1, 11.6, 9.7,  8.1,  8.2,  7.9,  4.9,  3.2,  2.1,  2.1],
            [15.6, 17.6, 23.0, 23.6, 22.1, 20.0, 16.0, 15.1, 12.1, 7.9,  6.2,  5.1,  5.1],
            [18.7, 21.5, 28.4, 30.2, 30.7, 31.0, 25.2, 23.1, 17.5, 10.7, 8.4,  7.2,  7.2]
        ])

        interpInt = interp2d(hBrk_ft, probExceedBrk, sigmaTable_fps, kind = 'linear')

        sigma_w = interpInt(h_ft, probExceed)

        if type(level) is str:
            sigma_w = max(sigma_w, 3)

        sigma_u = sigma_w
        sigma_v = sigma_w

    elif h_ft <= hLow_ft:

        if (U20_fps is None):
            if type(level) is str:
                if level.lower() == 'light':
                    U20_fps = 15 * kts2fps
                if level.lower() == 'moderate':
                    U20_fps = 30 * kts2fps
                if level.lower() == 'severe':
                    U20_fps = 45 * kts2fps
            else:
                U20Brk = np.array([2e-1, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6])
                probExceedBrk = np.array([2e-1, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6])

                U20_fps = level

        sigma_w = 0.1 * U20_fps
        sigma_u = sigma_w / (0.177 + 0.000823 * h_ft)**0.4
        sigma_v = sigma_u

        if h_ft is 'terrain':

            probExceedBrk = np.array([2e-1, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6])
            sigmaTableV_fps = np.array([4.0, 5.1, 8.0, 10.2, 12.1, 14.0, 23.1])
            sigmaTableW_fps = np.array([3.5, 4.4, 7.0, 8.9, 10.5, 12.1, 17.5])

            interpIntV = interp1d(probExceedBrk, sigmaTableV_fps, kind = 'linear')
            interpIntW = interp1d(probExceedBrk, sigmaTableW_fps, kind = 'linear')

            sigma_v = interpIntV(probExceed)
            sigma_w = interpIntW(probExceed)
            sigma_u = sigma_v


    return np.array([sigma_u, sigma_v, sigma_w])


def TurbLengthScale(h_ft = None, turbType = 'VonKarman'):

    hLow_ft = 1000
    hHi_ft = 2000

    if h_ft >= hHi_ft:
        # h_ft >= 2000
        # u,v,w aligned with body
        if turbType.lower() == 'dryden':
            Lu_ft = 1750
            Lv_ft = 0.5 * Lu_ft
            Lw_ft = 0.5 * Lu_ft
        if turbType.lower() == 'vonkarman':
            Lu_ft = 2500
            Lv_ft = 0.5 * Lu_ft
            Lw_ft = 0.5 * Lu_ft

    elif h_ft <= hLow_ft:
        # h_ft <= 1000
        # u,v,w aligned with mean wind

        # Ensure the height AGL is above 0 ft
        h_ft = max(h_ft, 0)

        Lu_ft = h_ft / (0.177 + 0.000823 * h_ft)**1.2
        Lv_ft = 0.5 * Lu_ft
        Lw_ft = 0.5 * h_ft

    else:
        # h_ft between 1000 and 2000 ft, Linear Interpolation
        LuLow_ft , LvLow_ft, LwLow_ft = TurbLengthScale(hLow_ft, turbType = turbType)
        LuHi_ft , LvHi_ft, LwHi_ft = TurbLengthScale(hHi_ft, turbType = turbType)

        hInt_nd = h_ft / (hHi_ft - hLow_ft)

        Lu_ft = (LuHi_ft - LuLow_ft) * hInt_nd + LuLow_ft
        Lv_ft = (LvHi_ft - LvLow_ft) * hInt_nd + LvLow_ft

    return np.array([Lu_ft , Lv_ft, Lw_ft])


# Wind Shear (MIL-DTL-9490E, 3.1.3.7.3.2)
def WindShear(h_ft, u20_fps):

    u_fps = u20_fps * (0.46 * np.log10(h_ft) + 0.4)

    return u_fps
