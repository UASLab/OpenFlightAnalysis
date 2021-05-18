"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np

#%% Servo Model
# Limit
def Limit(x, xLim):
    if abs(x) > xLim:
        x = np.sign(x) * xLim 
    return x

# Freeplay
def Freeplay(x, xOut, freeplay):
    if (x > xOut):
      xOut = max(xOut, x - freeplay/2);
    elif (x < xOut):
      xOut = min(xOut, x + freeplay/2);
    return xOut

class Servo:
    def __init__(self, dt, freqNat_rps, damp, pwrLim = np.Inf, aLim = np.Inf, vLim = np.Inf, pLim = np.Inf, freeplay = 0.0):
        self.dt = dt
        self.freqNat_rps = freqNat_rps
        self.damp = damp
        self.pwrLim = pwrLim # Specific Power Pwr/J = v * a = A**2 * freqNat_rps**3
        self.aLim = aLim
        self.vLim = vLim
        self.pLim = pLim
        self.freeplay = freeplay
        self.a = 0.0
        self.v = 0.0
        self.p = 0.0
        self.pOut = 0.0
    
    
    def Update(self, u):
        pErr = self.p - u
        
        # Compute Acceleration
        self.a = -(2*self.damp*self.freqNat_rps * self.v + self.freqNat_rps**2 * pErr)
    
        # Acceleration Limit
        self.a = Limit(self.a, self.aLim)
        
        # Apply limit to Specific Power (pwr = a * v)
        # pwr = self.a * self.v
        pwr = self.a * (self.v + self.a * self.dt)
        if abs(pwr) > self.pwrLim:
            self.a = np.sign(self.a) * abs(self.pwrLim / (self.v + self.a * self.dt))
            
        # Integrate Acceleration to update Velocity
        self.v += self.a * self.dt
            
        # Rate Limit
        self.v = Limit(self.v, self.vLim)
        
        # Integrate Velocity to update Position
        self.p += self.v * self.dt
        
        # Freeplay
        self.pOut = Freeplay(self.p, self.pOut, self.freeplay)
          
        # Position Limit
        # self.pOut = Limit(self.p, self.pLim)
        self.pOut = Limit(self.pOut, self.pLim)
                
        return self.pOut
