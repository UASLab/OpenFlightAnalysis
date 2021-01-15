"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np
import control

#%% Connect by strings
def ConnectName(sysList, connectNames, inKeep, outKeep):

    sys = []
    for s in sysList:
        if sys == []:
            sys = s
        else:
            sys = control.append(sys, control.ss(s))

    inNames = []
    outNames = []
    for s in sysList:
        inNames += s.InputName
        outNames += s.OutputName

    Q = [[inNames.index(s)+1, outNames.index(s)+1]  for s in connectNames]

    inputv = [inNames.index(s)+1 for s in inKeep]
    outputv = [outNames.index(s)+1 for s in outKeep]

    sysOut = control.connect(sys, Q, inputv, outputv)

    sysOut.InputName = inKeep
    sysOut.OutputName = outKeep

    return sysOut


#%% Controller Models
# 2-DOF PID with excitation input
def PID2(Kp = 1, Ki = 0, Kd = 0, b = 1, c = 1, Tf = 0):
    # Inputs: ['ref', 'sens', 'exc']
    # Outputs: ['cmd', 'ff', 'fb', 'exc']

    sysR = control.tf2ss(control.tf([Kp*b*Tf + Kd*c, Kp*b + Ki*Tf, Ki], [Tf, 1, 0]))
    sysY = control.tf2ss(control.tf([Kp*Tf + Kd, Kp + Ki*Tf, Ki], [Tf, 1, 0]))
    sysX = control.tf2ss(control.tf(1,1)) # Excitation Input

    sys = control.append(sysR, sysY, sysX)

    sys.C = np.concatenate((sys.C[0,:] - sys.C[1,:] + sys.C[2,:], sys.C))
    sys.D = np.concatenate((sys.D[0,:] - sys.D[1,:] + sys.D[2,:], sys.D))

    sys.outputs = 4

    return sys


#%% Effector Models
def ActuatorModel(bw, delay = (0, 1)):
    # Inputs: ['cmd']
    # Outputs: ['pos']

    sysNom = control.tf2ss(control.tf(1, [1/bw, 1]))

    delayPade = control.pade(delay[0], n=delay[1])
    sysDelay = control.tf2ss(control.tf(delayPade[0], delayPade[1]))

    sys = sysDelay * sysNom

    return sys


#%% Sensor models
def SensorModel(bw, delay = (0, 1)):
    # Inputs: ['meas', 'dist']
    # Outputs: ['sens']

    sysNom = control.tf2ss(control.tf(1, [1/bw, 1]))

    delayPade = control.pade(delay[0], n=delay[1])
    sysDelay = control.tf2ss(control.tf(delayPade[0], delayPade[1]))

    sysDist = control.tf2ss(control.tf(1, 1))

    sys = control.append(sysDelay * sysNom, sysDist)
    sys.C = sys.C[0,:] + sys.C[1,:]
    sys.D = sys.D[0,:] + sys.D[1,:]

    sys.outputs = 1

    return sys
