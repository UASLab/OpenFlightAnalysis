load('UltraStick25e_Lin17.mat')

% dt = SampleTime;
dt = 1/50;

% Constants
d2r = pi/180;
r2d = 180/pi;

%% Plant Model
% Effector models
timeDelayMotor_s = 0.050;
% timeDelayMotor_s = timeDelayMotor_s + 4*dt; % Artificial added Delay
motorBW_rps = 2.0 * 2*pi;

timeDelayServo_s = 0.050;
% timeDelayServo_s = timeDelayServo_s + 4*dt; % Artificial added Delay
servoBW_rps = 6.0 * 2*pi; % Hitec HS255BB

sysThrot = tf(1, [1/motorBW_rps, 1],'inputn', 'cmdThrot', 'outputn', '\delta_t', 'InputDelay', timeDelayMotor_s);
sysElev = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdElev', 'outputn', '\delta_e', 'InputDelay', timeDelayServo_s);
sysRud = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdRud', 'outputn', '\delta_r', 'InputDelay', timeDelayServo_s);
sysAilR = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdAilR', 'outputn', '\delta_aR', 'InputDelay', timeDelayServo_s);
sysFlapR = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdFlapR', 'outputn', '\delta_fR', 'InputDelay', timeDelayServo_s);
sysFlapL = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdFlapL', 'outputn', '\delta_fL', 'InputDelay', timeDelayServo_s);
sysAilL = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdAilL', 'outputn', '\delta_aL', 'InputDelay', timeDelayServo_s);

sysAct = ss(append(sysThrot, sysElev, sysRud, sysAilR, sysFlapR, sysFlapL, sysAilL));

% Sensor models
timeDelaySensor_s = 1*dt;
% timeDelaySensor_s = timeDelaySensor_s + 4*dt; % Artificial added Delay
sensorBW_rps = 1/dt * 2*pi;
sensorAirspeedBW_rps = 2 * 2*pi;
sensorHeightBW_rps = 2 * 2*pi;

sysSensP = tf(1, [1/sensorBW_rps, 1], 'inputn', 'p', 'outputn', 'sensP', 'OutputDelay', timeDelaySensor_s);
sysSensQ = tf(1, [1/sensorBW_rps, 1], 'inputn', 'q', 'outputn', 'sensQ', 'OutputDelay', timeDelaySensor_s);
sysSensR = tf(1, [1/sensorBW_rps, 1], 'inputn', 'r', 'outputn', 'sensR', 'OutputDelay', timeDelaySensor_s);

sysSensAx = tf(1, [1/sensorBW_rps, 1], 'inputn', 'ax', 'outputn', 'sensAx', 'OutputDelay', timeDelaySensor_s);
sysSensAy = tf(1, [1/sensorBW_rps, 1], 'inputn', 'ay', 'outputn', 'sensAy', 'OutputDelay', timeDelaySensor_s);
sysSensAz = tf(1, [1/sensorBW_rps, 1], 'inputn', 'az', 'outputn', 'sensAz', 'OutputDelay', timeDelaySensor_s);

sysSensPhi = tf(1, [1/sensorBW_rps, 1], 'inputn', 'phi', 'outputn', 'sensPhi', 'OutputDelay', timeDelaySensor_s);
sysSensTheta = tf(1, [1/sensorBW_rps, 1], 'inputn', 'theta', 'outputn', 'sensTheta', 'OutputDelay', timeDelaySensor_s);
sysSensPsi = tf(1, [1/sensorBW_rps, 1], 'inputn', 'psi', 'outputn', 'sensPsi', 'OutputDelay', timeDelaySensor_s);

sysSensSpeed = tf(1, [1/sensorAirspeedBW_rps, 1], 'inputn', 'V', 'outputn', 'sensV', 'OutputDelay', timeDelaySensor_s);
sysSensHeight = tf(1, [1/sensorHeightBW_rps, 1], 'inputn', 'h', 'outputn', 'sensH', 'OutputDelay', timeDelaySensor_s);

sysSens = ss(append(sysSensAx, sysSensAy, sysSensAz, sysSensP, sysSensQ, sysSensR, sysSensPhi, sysSensTheta, sysSensPsi, sysSensSpeed, sysSensHeight));

% Pade approximation
% ordPade = 2;
% sysAct = pade(sysAct, ordPade);
% sysSens = pade(sysSens, ordPade);


sysLin = linmodel;
[numOut, numIn] = size(sysLin);
numState = length(sysLin.StateName);
iStateDist = 4:9; % index of the disturbance states

Baug = zeros(numState, 6); Baug(iStateDist, :) = eye(6);
Daug = zeros(numOut, 6);
sysLinDist = ss(sysLin.A, [sysLin.B, Baug], sysLin.C, [sysLin.D, Daug]);
distInputName = {'pDist', 'qDist', 'rDist', 'uDist', 'vDist', 'wDist'}';
sysLinDist.InputName = [sysLin.InputName; distInputName];
sysLinDist.OutputName = sysLin.OutputName;
sysLinDist.StateName = sysLin.StateName;


% Assemble Plant model
sysPlant = connect(sysAct, sysLinDist, sysSens, [sysAct.InputName; distInputName], sysSens.OutputName);

%% Controller Models
ctrlScale = 1.0;

% Phi Controller
sysScasPhi = pid2();
sysScasPhi.InputName = {'refPhi', 'sensPhi'};
sysScasPhi.OutputName = 'cmdRoll';
sysScasPhi.Kp = ctrlScale * 0.64;
sysScasPhi.Ki = ctrlScale * 0.20;
sysScasPhi.Kd = ctrlScale * 0.07;
sysScasPhi.Tf = dt; 
sysScasPhi.b = 1;
sysScasPhi.c = 0;

% Theta Controller
sysScasTheta = pid2();
sysScasTheta.InputName = {'refTheta', 'sensTheta'};
sysScasTheta.OutputName = 'cmdPitch';
sysScasTheta.Kp = ctrlScale * 0.90;
sysScasTheta.Ki = ctrlScale * 0.30;
sysScasTheta.Kd = ctrlScale * 0.08;
sysScasTheta.Tf = dt;
sysScasTheta.b = 1;
sysScasTheta.c = 0;

% Yaw Controller
sysCtrlYawPass = tf(1);
sysCtrlYawPass.InputName = 'refYaw';
sysCtrlYawPass.OutputName = 'cmdYawPass';

kYaw = 0.03;
tauYaw = 5.72;
sysCtrlYawDamp = tf(-kYaw * tf([1, 0.0],[1.0, tauYaw]));
sysCtrlYawDamp.InputName = 'sensR';
sysCtrlYawDamp.OutputName = 'cmdYawDamp';

sysCtrlYawSum = sumblk('cmdYaw = cmdYawPass + cmdYawDamp');
sysCtrlYaw = connect(sysCtrlYawPass, sysCtrlYawDamp, sysCtrlYawSum, [sysCtrlYawPass.InputName, sysCtrlYawDamp.InputName], sysCtrlYawSum.OutputName);


% Append SCAS Controllers 
sysScas = append(sysScasPhi, sysScasTheta, sysCtrlYaw);

%% Mixer Definition
sysMixerSpeed = ss(1, 'inputn', {'cmdSpeed'}, 'outputn', {'cmdThrotMix'});

indxOut = 4:6;
indxSurf = [2,3,5,7,6,4];
sysLinD = c2d(sysLin, dt, 'zoh');
ctrlEff = (sysLinD(indxOut, indxSurf).C * sysLinD(indxOut, indxSurf).B + sysLinD(indxOut, indxSurf).D);

ctrlEff (abs(ctrlEff) < 0.01) = 0.0;

sysMixerSurf = ss(pinv(ctrlEff));
sysMixerSurf.InputName = {'cmdRollMix', 'cmdPitchMix', 'cmdYawMix'};
sysMixerSurf.OutputName = {'cmdElevMix', 'cmdRudMix', 'cmdAilRMix', 'cmdFlapRMix', 'cmdFlapLMix', 'cmdAilLMix'};

sysMixer = append(sysMixerSpeed, sysMixerSurf);

%% Excitation
sysExcRoll = sumblk('cmdRollMix = excRoll + cmdRoll');
sysExcPitch = sumblk('cmdPitchMix = excPitch + cmdPitch');
sysExcYaw = sumblk('cmdYawMix = excYaw + cmdYaw');

sysCtrlExc = append(sysExcRoll, sysExcPitch, sysExcYaw);

sysSurfExc = append(...
    sumblk('cmdThrot = excThrot + cmdThrotMix'), ...
    sumblk('cmdElev = excElev + cmdElevMix'), ...
    sumblk('cmdRud = excRud + cmdRudMix'), ...
    sumblk('cmdAilR = excAilR + cmdAilRMix'), ...
    sumblk('cmdFlapR = excFlapR + cmdFlapRMix'), ...
    sumblk('cmdFlapL = excFlapL + cmdFlapLMix'), ...
    sumblk('cmdAilL = excAilL + cmdAilLMix'));


%% Combine Controller and Mixer
% Systems
sysCtrl = connect(sysScas, sysCtrlExc, sysMixer, sysSurfExc, ...
    [sysScas.InputName(1:2:6); sysCtrlExc.InputName(1:2:end); sysSurfExc.InputName(1:2:end)], [sysSurfExc.OutputName]);

sysOL = connect(sysCtrl, sysPlant, [sysCtrl.InputName; sysPlant.InputName(8:end)], [sysPlant.OutputName]);

% Combine Controllers, with analysis point at excitation injection
sysCtrlCL = connect(sysScas, sysCtrlExc, sysMixer, sysSurfExc, sysPlant, ...
    [sysCtrl.InputName; distInputName], ...
    [sysPlant.OutputName; sysCtrlExc.InputName(2:2:end); sysCtrl.OutputName], ...
    [sysCtrlExc.InputName(2:2:end); sysCtrl.OutputName]);

sysL_Cntrl = getLoopTransfer(sysCtrlCL, sysCtrlExc.InputName(2:2:end), 1);
sysL_Out = getLoopTransfer(sysCtrlCL, sysCtrl.OutputName, 1);

return;
%% Steps
timeStep_s = 5;
opt = stepDataOptions;

opt.StepAmplitude = 10 *d2r;
figure(1); step(sysCtrlCL([7,4,12,18], 1), timeStep_s, opt);

opt.StepAmplitude = 2 *d2r;
figure(2); step(sysCtrlCL([8,5,13,16], 2), timeStep_s, opt);

opt.StepAmplitude = 5 *d2r;
figure(3); step(sysCtrlCL([9,6,14,17], 3), timeStep_s, opt);


%%
% Bode
figure(4); margin(sysL_Cntrl(1,1)); grid on; xlim([0.1,100]);
figure(5); margin(sysL_Cntrl(2,2)); grid on; xlim([0.1,100]);
figure(6); margin(sysL_Cntrl(3,3)); grid on; xlim([0.1,100]);

%%
figure(7); hSig = sigmaplot(sysL_Cntrl, {0.1*hz2rps, 100*hz2rps}, 2); grid on;
ylim([0, 2])
setoptions(hSig, 'FreqUnits', 'Hz');
setoptions(hSig, 'MagUnits', 'abs');



