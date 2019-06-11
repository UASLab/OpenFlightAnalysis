"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np


def RelVel(vel_AO, omega_AO, pos_BA, vel_BA = np.asarray([0, 0, 0])):
    '''Compute the relative velocity.
    
    Inputs:
      vel_AO   - Velocity of frame A wrt frame O
      omega_AO - Rotation rate of frame A wrt O
      pos_BA   - Position of frame B wrt A
      vel_BA   - Velocity of frame B wrt A [0, 0, 0]
    
    Outputs:
      vel_BO   - Velocity of frame B wrt frame O
    
    Notes:
      There are no internal unit conversions.
      The vectors must all be in the same coordinate system
    '''
    
    # Compute the Relative Velocity
    vel_BO = vel_AO + vel_BA + np.cross(omega_AO, pos_BA)
    
    return vel_BO

def RelAccel(accel_AO, omega_AO, omegaDot_AO, pos_BA, accel_BA = np.asarray([0, 0, 0]), vel_BA = np.asarray([0, 0, 0])):
    '''Compute the relative acceleration.
    
    Inputs:
      accel_AO    - Acceleration of frame A wrt frame O
      omega_AO    - Rotation rate of frame A wrt O
      omegaDot_AO - Rotation rate of frame A wrt O
      pos_BA      - Position of frame B wrt A
      accel_BA    - Acceleration of frame B wrt A [0, 0, 0]
      vel_BA      - Velocity of frame B wrt A [0, 0, 0]
    
    Outputs:
      accel_BO   - Acceleration of frame B wrt frame O
    
    Notes:
      There are no internal unit conversions.
      The vectors must all be in the same coordinate system
    '''
    
    # Compute the Relative Acceleration
    accel_BO = accel_AO + accel_BA + np.cross(omegaDot_AO, pos_BA) + np.cross(omega_AO, np.cross(omega_AO, pos_BA)) + 2 * np.cross(omega_AO, vel_BA)
    
    return accel_BO


def D2E(r_D_, degrees = False):
    ''' Convert Geodetic to ECEF Coordinates
    
    Inputs:
      r_D_ - Position in Geodetic Coordinates (-, -, m)
      degrees - r_D_ is input in degrees [True]
    
    Outputs:
      r_E_m - Position in ECEF Coordinates (m, m, m)
    '''
    
    # Change units
    if degrees:
        d2r = np.pi / 180.0
        r_D_ = (r_D_.T * np.asarray([d2r, d2r, 1.0])).T
    
    # Parameters for WGS84
    R_m = 6378137.0 # Earth semimajor axis (m)
    f_nd = 1/298.257223563 # reciprocal flattening (nd)
    
    # Derived parameters
    eSquared_nd = 2*f_nd - f_nd**2 # eccentricity squared
    Rew = R_m / np.sqrt(1 - eSquared_nd * np.sin(r_D_[0])**2) # Radius East-West at Latitude
    
    ## Convert
    r_E_m = np.nan * np.ones_like(r_D_)
    r_E_m[0] = (Rew + r_D_[2]) * np.cos(r_D_[0]) * np.cos(r_D_[1])
    r_E_m[1] = (Rew + r_D_[2]) * np.cos(r_D_[0]) * np.sin(r_D_[1])
    r_E_m[2] = (Rew * (1 - eSquared_nd) + r_D_[2]) * np.sin(r_D_[0])
    
    return r_E_m


def D2L(rRef_LD_D_, r_PD_D_, degrees = False):
    ''' Convert ECEF Coordinates to Local Level
    
    Inputs:
      rRef_LD_D_ - Reference Position in Geodetic Coordinates [-, -, m]
      r_PD_D_   - Position in Geodetic Coordinates
    
    Outputs:
      r_PL_L_m - Position in Local Level Coordinates (m)
    
    Notes:
      Uses D2E and E2L
    '''
    
    # Change units to Radians
    if degrees:
        d2r = np.pi / 180.0
        rRef_LD_D_ = (rRef_LD_D_.T * np.asarray([d2r, d2r, 1.0])).T
        r_PD_D_ = (r_PD_D_.T * np.asarray([d2r, d2r, 1.0])).T
    
    # Reference location of L wrt D in ECEF
    r_LD_E_m = D2E(rRef_LD_D_, degrees = False)
    
    # Position of P wrt E in ECEF
    r_PE_E_m = D2E(r_PD_D_, degrees = False)
        
    lenVec = r_PD_D_.shape[-1]
    r_PL_E_m = np.zeros_like(r_PD_D_)
    for indx in range(0, lenVec):
        r_PL_E_m[:, indx] = r_PE_E_m[:, indx] - r_LD_E_m # Distance from Ref in ECEF
        
    T_E2L, r_PL_L_m = E2L(rRef_LD_D_, r_PL_E_m, degrees = False)
    
    return r_PL_L_m


def E2L(rRef_LD_D_, r_PL_E_m, degrees = False):
    ''' Convert ECEF Coordinates to Local Level
    
    Inputs:
      rRef_LD_D_ - Reference Position in Geodetic Coordinates (ddm)
      r_PE_E_m   - Position in ECEF Coordinates (m)
    
    Outputs:
      T_E2L - Transformation Matrix from ECEF to Local Level Coordinates
      r_L_m - Position in Local Level Coordinates (m)
    
    Notes:
      T_E2L = R2(270-lat)*R3(long)
    '''
    from scipy.spatial.transform import Rotation as R
    
    # Transfor the Coordinates
    # Compute the Transformation Matrix at the Reference Location
    
    # Change units
    d2r = np.pi / 180.0
    if degrees:
        rRef_LD_D_ = (rRef_LD_D_.T * np.asarray([d2r, d2r, 1.0])).T

    latRef = 270.0 * d2r
    
    sRef_ = np.asarray([rRef_LD_D_[1], (latRef - rRef_LD_D_[0])])
    T_E2L = R.from_euler('ZY', sRef_, degrees = False).as_dcm() # Intrinsic rotation about Z then Y
    
    # Transform Coordinates from ECEF to NED
    r_PL_L_m = T_E2L.T @ r_PL_E_m
    
    return T_E2L, r_PL_L_m


def TransPitot(v_PA_P_mps, w_BA_B_rps, s_PB_rad, r_PB_B_m):
    ''' Transform Pitot Measurements from Probe location to Body.
    
    Inputs:
      v_PA_P_mps - Velocity of the Probe wrt Atm in Probe Coordinates [P/A]P (m/s2)
      w_BA_B_rps - Rotation rate of the Body Frame wrt Atm in Body Coordinates [B/A]B (rad/s)
      s_PB_rad   - Orientation (321) of the Probe Frame wrt Body Frame [P/B] (rad)
      r_PB_B_m   - Position of the Probe Frame wrt Body Frame in Body Coordinates [P/B]B (m)
    
    Outputs:
      v_BA_B_mps - Velocity of the Body wrt Atm in Body Coordinates [B/A]B (m/s2)
      v_BA_L_mps - Velocity of the Body wrt Atm in Local Level Coordinates [B/A]L (m/s2)
    '''
    from scipy.spatial.transform import Rotation as R
    
    # Parameterize transformation from P to B
    T_B2P = R.from_euler('zyx', s_PB_rad[[2,1,0]], degrees = False).as_dcm()
    T_P2B = T_B2P.T
    
    v_PB_B_mps = np.asarray([0, 0, 0]) # Velocity of the Probe wrt the Body Frame [P/B]B (m/s)
    v_BP_B_mps = -v_PB_B_mps
    
    r_BP_B_m = -r_PB_B_m
    
    #
    v_BA_B_mps = np.nan * np.ones_like(v_PA_P_mps)
    numSamp = v_PA_P_mps.shape[-1]
    for indx in range(0, numSamp):
        # Transform from P to B
        v_PA_B_mps = T_P2B @ v_PA_P_mps[:,indx]
        v_BA_B_mps[:,indx] = RelVel(v_PA_B_mps, w_BA_B_rps[:,indx], r_BP_B_m, v_BP_B_mps)
    
    return v_BA_B_mps
