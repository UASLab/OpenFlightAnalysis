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
    def __init__(self, dt, freqNat_rps, damp, timeDelay_s = 0, cmdLim = np.Inf, pwrLim = np.Inf, aLim = np.Inf, vLim = np.Inf, pLim = np.Inf, freeplay = 0.0):
        self.dt = dt
        self.timeDelay_s = timeDelay_s
        self.freqNat_rps = freqNat_rps
        self.damp = damp

        self.cmdLim = cmdLim
        self.pwrLim = pwrLim # Specific Power Pwr/J = v * a = A**2 * freqNat_rps**3
        self.aLim = aLim
        self.vLim = vLim
        self.pLim = pLim

        self.freeplay = freeplay


    def Start(self):
        self.cmd = 0.0
        cmdQueue = np.zeros(max(int(self.timeDelay_s/self.dt), 1))
        self.cmdQueue = cmdQueue
        self.vState = 0.0
        self.pState = 0.0
        self.pwr = 0.0
        self.a = 0.0
        self.v = 0.0
        self.p = 0.0
        self.pOut = 0.0


    def Update(self, cmd):
        # Command Limit

        # Apply the time delay as a queue
        self.cmdQueue[:-1] = self.cmdQueue[1:]
        self.cmdQueue[-1] = cmd

        self.cmd = Limit(self.cmdQueue[0], self.cmdLim)

        # Tracking Error
        e = self.p - self.cmd

        # Compute Acceleration
        self.a = -(2*self.damp*self.freqNat_rps * self.vState + self.freqNat_rps**2 * e)

        # Acceleration Limit
        self.a = Limit(self.a, self.aLim)

        # Specific Power Limit (pwr/J = a * v)
        # pwr = self.a * self.vState
        self.pwr = self.a * (self.vState + self.a * self.dt)
        if abs(self.pwr) > self.pwrLim:
            self.a = np.sign(self.a) * abs(self.pwrLim / self.vState)

        # Integrate Acceleration to update Velocity
        self.v = self.vState + self.a * self.dt

        # Rate Limit
        self.v = Limit(self.v, self.vLim)
        self.a = (self.v - self.vState) / self.dt

        # Compute specific power again
        self.pwr = self.a * self.v

        # Integrate Velocity to update Position
        self.p = self.pState + self.v * self.dt

        # Freeplay
        self.pOut = Freeplay(self.p, self.pOut, self.freeplay)

        # Position Limit
        self.pOut = Limit(self.pOut, self.pLim)
        self.v = (self.p - self.pState) / self.dt


        self.pState = self.pOut
        self.vState = self.v


        return self.pOut

