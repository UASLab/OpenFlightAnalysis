"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import Systems
import numpy as np
import control

import matplotlib.pyplot as plt


# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps
d2r = np.pi/180
r2d = 1/d2r

#%% US25e @17 m/s
A = np.array([[-1.5075141732773e-08, 3.52548256695526e-09, 0, 1, -9.4614084373833e-05, 0.0551110840032007, 0, 0, 0, 0, 0, 0, 0],[-3.51480725402154e-09, 0, 0, 0, 0.999998526321596, 0.00171678613575457, 0, 0, 0, 0, 0, 0, 0],[-2.73955699015303e-07, 1.93999064761037e-10, 0, 0, -0.00171939130534064, 1.00151599299531, 0, 0, 0, 0, 0, 0, 0],[0, 0, 0, -16.093281964627, -0.139331847922205, 3.36733290152386, 0.0641598396789031, -2.82336072789796, 0.00341285522191952, 2.69406616153886e-09, -2.45351955099935e-09, -0.000305350541915879, -0.0128378125604161],[0, 0, 0, -1.65172003270966e-08, -15.8073729938731, 1.24355004881079, 1.0483672161376, 2.57438180638635e-09, -7.40469649475076, 6.76976259264397e-18, -6.16530696751281e-18, -7.67297665445943e-13, -0.0131461492171692],[0, 0, 0, 0.514001561383309, -0.711687216963793, -2.77502233193406, 0.00584617234727531, 1.70151703540306, 0.000314102055369676, 2.45489133138549e-10, -2.23570005930838e-10, -2.78242015390957e-05, -0.00116980825507311],[0, -9.79069403403114, 0, 1.43545992664233e-10, -0.8912294479237, 6.21652178060893e-08, -0.59452928348666, -0.000987234275265025, 0.808056371377502, -9.06225232545553e-10, 4.17164804178681e-10, 5.17480844884583e-05, 0.0125985024168313],[9.79067960570885, 0.000926337551399568, 0, 0.896756079983008, -9.16020039877139e-11, -16.8189425372007, 0.00197447678969674, -0.872591324595545, 0.000108887948499417, -2.82301420835357e-11, 1.29952571254347e-11, 1.61202397722137e-06, 0],[0.0168085278032739, -0.53957576134847, 0, -6.27260486611496e-08, 15.7185902206702, -5.34124623866913e-11, -0.73691437649521, -5.44780694701113e-05, -7.55980389894898, 1.64435725872259e-08, -7.56951313505922e-09, -0.000938976162094652, 0],[0.395312240136752, -9.78128998000427e-05, -7.1830544974382, 0, 0, 0, -0.904934576962061, -0.422532019375643, -0.0505974706279831, 0, 0, 0, 0],[0.847958979377808, 4.56109378542124e-05, -15.4079107971907, 0, 0, 0, 0.421977923367986, -0.906346376507767, 0.0216997241337062, 0, 0, 0, 0],[0.00160381869688203, -16.9999995467147, 0, 0, 0, 0, -0.0550276624532794, -0.00171418491415924, 0.9984833588687, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 135.454006056826, 5.00658966111091e-07, 7.46590165357177, 7.28257345721243e-07, -6.63233019810226e-07, -0.0825420616408248, -5.90289647474733]])
B = np.array([[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],[9.26047920965562, -1.27364548088261e-09, -5.00837350014868, 78.2349393764052, -78.2349393824847, 33.0131903630656, -33.0131903725348],[0, -133.690540958441, -0.00471785949745694, 3.00071183120475, 2.99600954220068, 2.62378754041215, 2.62378754041215],[0.843834346102439, -1.83787683968599e-10, -82.0410621087846, -5.75214205021287, 5.7521420497139, -1.79412039039175, 1.79412038961457],[0, 0.469793876112028, -0.838533246029698, 0.650333818652782, -0.185431971381835, 0.36205784780099, 0.36205784780099],[0, 1.1822284325028e-09, 5.30214133241287, 1.06445249410525e-09, -2.02467818136338e-09, -7.47807838950765e-10, -7.47807838950765e-10],[0, -2.70276046705527, -0.0462179527408058, -6.55817192749153, -6.60423734391721, -10.2506660290912, -10.2506660290912],[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],[2503.28376351617, 0, 0, 0, 0, 0, 0]])
C = np.array([[1.00000000000003, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],[0, 0.99999999999679, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],[0, 0, 1.00000000000016, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 1.43545992664233e-10, 0.0443499288060191, -5.74183970656932e-10, -0.59452928348666, -0.000987237320456371, 0.808056097831262, -4.58064245815772e-10, 4.17164804178681e-10, 5.19178658968939e-05, 0.0125985024168313],[0, 0, 0, -0.0388232967467113, -9.16020039877139e-11, 0.155293186986845, 0.00197447983488809, -0.872591324595545, 0.000108828072535648, -1.42693309567761e-11, 1.29952571254347e-11, 1.61731289403071e-06, 0],[0, 0, 0, 1.33531155966728e-11, -1.25564550351733, -5.34124623866913e-11, -0.73691410294897, -5.44181935063429e-05, -7.55980389894898, 8.31163916675568e-09, -7.56951313505922e-09, -0.000942056865570217, 0],[0, 0, 0, 0, 0, 0, 0.998484476506202, 3.69055312750108e-09, 0.0550340822041545, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, -2.16762358190449e-10, 0.0588235307163837, -1.19474240412444e-11, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, -0.00323729902531156, 0, 0.0587343822720376, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0, 0, 0, 8.82286353462966e-06, -8.03509152338271e-06, -1.00000000000377, 0],[-9.43422783798579e-05, 0.999999995518051, 5.29564998623389e-18, 0, 0, 0, 0.00323729426829648, 0.00010083440895332, -0.0587342959783891, 0, 0, 0, 0]])
D = np.array([[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],[0, 0.469793876112028, -0.838533246029698, 0.650333818652782, -0.185431971381835, 0.36205784780099, 0.36205784780099],[0, 1.1822284325028e-09, 5.30214133241287, 1.06445249410525e-09, -2.02467818136338e-09, -7.47807838950765e-10, -7.47807838950765e-10],[0, -2.70276046705527, -0.0462179527408058, -6.55817192749153, -6.60423734391721, -10.2506660290912, -10.2506660290912],[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0]])

sysLin = control.ss(A, B, C, D)

sysLin_InputNames = ['posThrot', 'posElev', 'posRud', 'posAilL', 'posAilR', 'posFlapL', 'posFlapR']
sysLin_OutputNames = ['phi', 'theta', 'psi', 'p', 'q', 'r', 'ax', 'ay', 'az', 'V', 'beta', 'alpha', 'h', 'gamma']
sysLin_StateNames = ['phi', 'theta', 'psi', 'p', 'q', 'r', 'u', 'v', 'w', 'Xe', 'Ye', 'Ze', 'velMotor']



#%% Effector Models
# dt = SampleTime;
dt = 1/50
ordPade = 1

## Plant Model
# Effector models
motorBW_rps = 2.0 * hz2rps
motorDelay_s = 0.050
# motorDelay_s = motorDelay_s + 4*dt; # Artificial added Delay
motorDelay = (motorDelay_s, ordPade)

servoBW_rps = 6.0 * hz2rps # Hitec HS255BB
servoDelay_s = 0.050
# servoDelay_s = servoDelay_s + 4*dt; # Artificial added Delay
servoDelay = (servoDelay_s, ordPade)


sysThrot = Systems.ActuatorModel(motorBW_rps, motorDelay); sysThrot_InputNames = ['cmdThrot']; sysThrot_OutputNames = ['posThrot']; sysThrot_StateNames = ['posThrot', 'velThrot']
sysElev = Systems.ActuatorModel(servoBW_rps, servoDelay); sysElev_InputNames = ['cmdElev']; sysElev_OutputNames = ['posElev']; sysElev_StateNames = ['posElev', 'velElev']
sysRud = Systems.ActuatorModel(servoBW_rps, servoDelay); sysRud_InputNames = ['cmdRud']; sysRud_OutputNames = ['posRud']; sysRud_StateNames = ['posRud', 'velRud']
sysAilL = Systems.ActuatorModel(servoBW_rps, servoDelay); sysAilL_InputNames = ['cmdAilL']; sysAilL_OutputNames = ['posAilL']; sysAilL_StateNames = ['posAilL', 'velAilL']
sysAilR = Systems.ActuatorModel(servoBW_rps, servoDelay); sysAilR_InputNames = ['cmdAilR']; sysAilR_OutputNames = ['posAilR']; sysAilR_StateNames = ['posAilR', 'velAilR']
sysFlapL = Systems.ActuatorModel(servoBW_rps, servoDelay); sysFlapL_InputNames = ['cmdFlapL']; sysFlapL_OutputNames = ['posFlapL']; sysFlapL_StateNames = ['posFlapL', 'velFlapL']
sysFlapR = Systems.ActuatorModel(servoBW_rps, servoDelay); sysFlapR_InputNames = ['cmdFlapR']; sysFlapR_OutputNames = ['posFlapR']; sysFlapR_StateNames = ['posFlapR', 'velFlapR']

# Combine Actuators
sysAct = control.append(sysThrot, sysElev, sysRud, sysAilL, sysAilR, sysFlapL, sysFlapR)

sysAct_InputNames = sysThrot_InputNames + sysElev_InputNames + sysRud_InputNames + sysAilL_InputNames + sysAilR_InputNames + sysFlapL_InputNames + sysFlapR_InputNames
sysAct_OutputNames = sysThrot_OutputNames + sysElev_OutputNames + sysRud_OutputNames + sysAilL_OutputNames + sysAilR_OutputNames + sysFlapL_OutputNames + sysFlapR_OutputNames
sysAct_StateNames = sysThrot_StateNames + sysElev_StateNames + sysRud_StateNames + sysAilL_StateNames + sysAilR_StateNames + sysFlapL_StateNames + sysFlapR_StateNames


#%% Sensor models
sensorBW_rps = 1/dt * hz2rps
sensorAirBW_rps = 2 * hz2rps

sensorDelay_s = 1*dt
# sensorDelay_s = sensorDelay_s + 4*dt; # Artificial added Delay
sensorDelay = (sensorDelay_s, ordPade)

sysSensPhi = Systems.SensorModel(sensorBW_rps, sensorDelay); sysSensPhi_InputNames = ['phi', 'phiDist']; sysSensPhi_OutputNames = ['sensPhi']; sysSensPhi_StateNames = ['posPhi', 'velPhi']
sysSensTheta = Systems.SensorModel(sensorBW_rps, sensorDelay); sysSensTheta_InputNames = ['theta', 'thetaDist']; sysSensTheta_OutputNames = ['sensTheta']; sysSensTheta_StateNames = ['posTheta', 'velTheta']

sysSensP = Systems.SensorModel(sensorBW_rps, sensorDelay); sysSensP_InputNames = ['p', 'pDist']; sysSensP_OutputNames = ['sensP']; sysSensP_StateNames = ['posP', 'velP']
sysSensQ = Systems.SensorModel(sensorBW_rps, sensorDelay); sysSensQ_InputNames = ['q', 'qDist']; sysSensQ_OutputNames = ['sensQ']; sysSensQ_StateNames = ['posQ', 'velQ']
sysSensR = Systems.SensorModel(sensorBW_rps, sensorDelay); sysSensR_InputNames = ['r', 'rDist']; sysSensR_OutputNames = ['sensR']; sysSensR_StateNames = ['posR', 'velR']

sysSensSpeed = Systems.SensorModel(sensorAirBW_rps, sensorDelay); sysSensSpeed_InputNames = ['V', 'VDist']; sysSensSpeed_OutputNames = ['sensV']; sysSensSpeed_StateNames = ['posV', 'velV']
sysSensHeight = Systems.SensorModel(sensorAirBW_rps, sensorDelay); sysSensHeight_InputNames = ['h', 'hDist']; sysSensHeight_OutputNames = ['sensH']; sysSensHeight_StateNames = ['posH', 'velH']

# Combine Sensors
sysSens = control.append(
         sysSensPhi,
         sysSensTheta,
         sysSensP,
         sysSensQ,
         sysSensR,
         sysSensSpeed,
         sysSensHeight)

sysSens_InputNames = sysSensPhi_InputNames + sysSensTheta_InputNames + sysSensP_InputNames + sysSensQ_InputNames + sysSensR_InputNames + sysSensSpeed_InputNames + sysSensHeight_InputNames
sysSens_OutputNames = sysSensPhi_OutputNames + sysSensTheta_OutputNames + sysSensP_OutputNames + sysSensQ_OutputNames + sysSensR_OutputNames + sysSensSpeed_OutputNames + sysSensHeight_OutputNames
sysSens_StateNames = sysSensPhi_StateNames + sysSensTheta_StateNames + sysSensP_StateNames + sysSensQ_StateNames + sysSensR_StateNames + sysSensSpeed_StateNames + sysSensHeight_StateNames


#%% Assemble Plant model
inNames = sysAct_InputNames + sysLin_InputNames + sysSens_InputNames
outNames = sysAct_OutputNames + sysLin_OutputNames + sysSens_OutputNames

sysPlant_ConnectNames = sysAct_OutputNames + sysSens_InputNames[0::2]
sysPlant_InputNames = sysAct_InputNames + sysSens_InputNames[1::2]
sysPlant_OutputNames = sysSens_OutputNames
sysPlant_StateNames = sysAct_StateNames + sysLin_StateNames + sysSens_StateNames

sysPlant = Systems.ConnectName(control.append(sysAct, sysLin, sysSens), 
                      inNames, outNames, 
                      sysPlant_ConnectNames, sysPlant_InputNames, sysPlant_OutputNames)


#%% Controller Models
sysPhi = Systems.PID2(0.64, Ki = 0.20, Kd = 0.07, b = 1, c = 0, Tf = dt)
sysPhi_InputNames = ['refPhi', 'sensPhi', 'excP']
sysPhi_OutputNames = ['cmdP', 'ffP', 'fbP', 'excP']

sysTheta = Systems.PID2(0.90, Ki = 0.30, Kd = 0.08, b = 1, c = 0, Tf = dt)
sysTheta_InputNames = ['refTheta', 'sensTheta', 'excQ']
sysTheta_OutputNames = ['cmdQ', 'ffQ', 'fbQ', 'excQ']

tauYaw = 5.72
sysYaw = control.append(control.tf2ss(control.tf([0.5],[1.0])), control.tf2ss(control.tf([0.03, 0.0],[1.0, tauYaw])), control.tf2ss(control.tf(1,1)))

sysYaw.C = np.concatenate((sysYaw.C[0,:] - sysYaw.C[1,:] + sysYaw.C[2,:], sysYaw.C))
sysYaw.D = np.concatenate((sysYaw.D[0,:] - sysYaw.D[1,:] + sysYaw.D[2,:], sysYaw.D))
    
sysYaw.outputs = 4
sysYaw_InputNames = ['refYaw', 'sensR', 'excR']
sysYaw_OutputNames = ['cmdR', 'ffR', 'fbR', 'excR']


# Append SCAS Feedback systems
sysScas = control.append(sysPhi, sysTheta, sysYaw)
sysScas_InputNames = sysPhi_InputNames + sysTheta_InputNames + sysYaw_InputNames
sysScas_OutputNames = sysPhi_OutputNames + sysTheta_OutputNames + sysYaw_OutputNames


#%% Mixer
# Surface Mix
outNames = ['p', 'q', 'r']
surfNames = ['posElev', 'posRud', 'posAilL', 'posAilR', 'posFlapL', 'posFlapR']

indxOut = [sysLin_OutputNames.index(s) for s in outNames]
indxSurf = [sysLin_InputNames.index(s) for s in surfNames]
sysLinD = control.c2d(sysLin, dt, 'zoh')
ctrlEff = (sysLinD.C @ sysLinD.B)[indxOut,][:,indxSurf] + sysLinD.D[indxOut,][:,indxSurf]
ctrlEff [abs(ctrlEff) / np.max(abs(ctrlEff)) < 0.05] = 0.0

mixSurf = np.linalg.pinv(ctrlEff)
mixSurf [abs(mixSurf) / np.max(abs(mixSurf)) < 0.05] = 0.0
nSurf, nCntrl = mixSurf.shape
sysMixerSurf = control.ss(np.zeros((1,1)), np.zeros((1,nCntrl)), np.zeros((nSurf,1)), mixSurf)
sysMixerSurf_InputNames = ['cmd' + s.upper() for s in outNames]
sysMixerSurf_OutputNames = [s.replace('pos', 'cmd') for s in surfNames]

# Throttle Mix
sysMixerSpeed = control.tf2ss(control.tf(1,1))
sysMixerSpeed_InputNames = ['cmdSpeed']
sysMixerSpeed_OutputNames = ['cmdThrot']

sysMixer = control.append(sysMixerSpeed, sysMixerSurf)
sysMixer_InputNames = sysMixerSpeed_InputNames + sysMixerSurf_InputNames
sysMixer_OutputNames = sysMixerSpeed_OutputNames + sysMixerSurf_OutputNames

#%% Combine Controller and Mixer
inNames = sysScas_InputNames + sysMixer_InputNames
outNames = sysScas_OutputNames + sysMixer_OutputNames

sysCtrl_ConnectNames = sysMixer_InputNames[1:]
sysCtrl_InputNames = [inNames[i-1] for i in [1, 4, 7, 2, 5, 8, 3, 6, 9]]
sysCtrl_OutputNames = [outNames[i-1] for i in [13, 14, 15, 16, 17, 18, 19, 3, 7, 11, 1, 5, 9]]

sysCtrl = Systems.ConnectName(control.append(sysScas, sysMixer), 
                      inNames, outNames, 
                      sysCtrl_ConnectNames, sysCtrl_InputNames, sysCtrl_OutputNames)


#%% Create the Open-Loop System
inNames = sysCtrl_InputNames + sysPlant_InputNames
outNames = sysCtrl_OutputNames + sysPlant_OutputNames

sysOL_ConnectNames = sysPlant_InputNames[:7]
sysOL_InputNames = sysCtrl_InputNames + sysPlant_InputNames[-7:]
sysOL_OutputNames = sysSens_OutputNames + sysCtrl_OutputNames[-6:]

sysOL = Systems.ConnectName(control.append(sysCtrl, sysPlant), inNames, outNames, sysOL_ConnectNames, sysOL_InputNames, sysOL_OutputNames)


# Bode Plots
plt.figure(1)
_ = control.bode_plot(sysOL[3,0], omega_limits = [0.01, 500], Hz = True, dB = True)
_ = control.bode_plot(sysOL[4,1], omega_limits = [0.01, 500], Hz = True, dB = True)
_ = control.bode_plot(sysOL[5,2], omega_limits = [0.01, 500], Hz = True, dB = True)


#%% Closed-Loop System
inNames = sysCtrl_InputNames + sysPlant_InputNames
outNames = sysCtrl_OutputNames + sysPlant_OutputNames

sysCL_ConnectNames = ['cmdThrot', 'cmdElev', 'cmdRud', 'cmdAilL', 'cmdAilR', 'cmdFlapL', 'cmdFlapR', 'sensPhi', 'sensTheta', 'sensR']
sysCL_InputNames = [inNames[i-1] for i in [1, 2, 3, 7, 8, 9, 17, 18, 19, 20, 21, 22, 23]]
sysCL_OutputNames = [outNames[i-1] for i in [8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]]

sysCL = Systems.ConnectName(control.append(sysCtrl, sysPlant), inNames, outNames, sysCL_ConnectNames, sysCL_InputNames, sysCL_OutputNames)


# Steps
timeStep_s = 5.0
timeRate_s = 1.0/50.0
time_s = np.linspace(0.0, timeStep_s, int(timeStep_s/timeRate_s)+1)

plt.figure(2)
_, stepPhiOut = control.step_response(sysCL, input=0, T=time_s)
plt.subplot(4,1,1); plt.plot(time_s, stepPhiOut[0])
plt.subplot(4,1,2); plt.plot(time_s, stepPhiOut[3])
plt.subplot(4,1,3); plt.plot(time_s, stepPhiOut[6])
plt.subplot(4,1,4); plt.plot(time_s, stepPhiOut[8])

plt.figure(3)
_, stepThetaOut = control.step_response(sysCL, input=1, T=time_s)
plt.subplot(4,1,1); plt.plot(time_s, stepThetaOut[1])
plt.subplot(4,1,2); plt.plot(time_s, stepThetaOut[4])
plt.subplot(4,1,3); plt.plot(time_s, stepThetaOut[7])
plt.subplot(4,1,4); plt.plot(time_s, stepThetaOut[9])

plt.figure(4)
_, stepYawOut = control.step_response(sysCL, input=2, T=time_s)
plt.subplot(3,1,1); plt.plot(time_s, stepYawOut[2])
plt.subplot(3,1,2); plt.plot(time_s, stepYawOut[5])
plt.subplot(3,1,3); plt.plot(time_s, stepYawOut[10])


plt.figure(5)
_, stepExcP = control.step_response(sysCL, input=3, T=time_s)
plt.subplot(4,1,1); plt.plot(time_s, stepExcP[0])
plt.subplot(4,1,2); plt.plot(time_s, stepExcP[3])
plt.subplot(4,1,3); plt.plot(time_s, stepExcP[6])
plt.subplot(4,1,4); plt.plot(time_s, stepExcP[8])

plt.figure(6)
_, stepExcQ = control.step_response(sysCL, input=4, T=time_s)
plt.subplot(4,1,1); plt.plot(time_s, stepExcQ[1])
plt.subplot(4,1,2); plt.plot(time_s, stepExcQ[4])
plt.subplot(4,1,3); plt.plot(time_s, stepExcQ[7])
plt.subplot(4,1,4); plt.plot(time_s, stepExcQ[9])

plt.figure(7)
_, stepExcR = control.step_response(sysCL, input=5, T=time_s)
plt.subplot(3,1,1); plt.plot(time_s, stepExcR[2])
plt.subplot(3,1,2); plt.plot(time_s, stepExcR[5])
plt.subplot(3,1,3); plt.plot(time_s, stepExcR[10])

