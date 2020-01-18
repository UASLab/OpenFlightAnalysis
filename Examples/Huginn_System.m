load('mW2_FD_FLEX_FEM2p1_1218.mat');

% dt = SampleTime;
dt = 1/50;

% Constants
d2r = pi/180;
r2d = 180/pi;

ft2m = 0.3048;
m2ft = 1/ft2m;

%% Plant Model
% Assemble Plant model
V_mps = 23;
switch V_mps
    case 18
        sysLin = Ndof6_mW2_FEM2p1_18;
        maxThrust_lbf = 24;
    case 23
        sysLin = Ndof6_mW2_FEM2p1_23;
        maxThrust_lbf = 18.5; % Max thrust at 23 m/s
    case 28
        sysLin = Ndof6_mW2_FEM2p1_28;
        maxThrust_lbf = 13;
    case 33
        sysLin = Ndof6_mW2_FEM2p1_33;
        maxThrust_lbf = 8.7;
end

augStateOutName = 'eta1';

sysLinAug = ss(sysLin.A, sysLin.B, [sysLin.C; double(strcmp(sysLin.StateName, augStateOutName))'], [sysLin.D; zeros(size(sysLin.InputName))']);
sysLinAug.InputName = sysLin.InputName;
sysLinAug.StateName = sysLin.StateName;
sysLinAug.OutputName = [sysLin.OutputName; augStateOutName];

% Effector models
timeDelayMotor_s = 0.004;
motorBW_rps = 1.0 * 2*pi;

timeDelayServo_s = 0.004;
servoBW_rps = 18.5 * 2*pi; % Futaba BLS471SV, cmd @ 50Hz

sysLTE1 = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdLTE1 [rad]', 'outputn', 'posLTE1', 'InputDelay', timeDelayServo_s);
sysRTE1 = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdRTE1 [rad]', 'outputn', 'posRTE1', 'InputDelay', timeDelayServo_s);
sysLTE2 = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdLTE2 [rad]', 'outputn', 'posLTE2', 'InputDelay', timeDelayServo_s);
sysRTE2 = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdRTE2 [rad]', 'outputn', 'posRTE2', 'InputDelay', timeDelayServo_s);
sysLTE3 = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdLTE3 [rad]', 'outputn', 'posLTE3', 'InputDelay', timeDelayServo_s);
sysRTE3 = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdRTE3 [rad]', 'outputn', 'posRTE3', 'InputDelay', timeDelayServo_s);
sysLTE4 = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdLTE4 [rad]', 'outputn', 'posLTE4', 'InputDelay', timeDelayServo_s);
sysRTE4 = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdRTE4 [rad]', 'outputn', 'posRTE4', 'InputDelay', timeDelayServo_s);
sysLTE5 = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdLTE5 [rad]', 'outputn', 'posLTE5', 'InputDelay', timeDelayServo_s);
sysRTE5 = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdRTE5 [rad]', 'outputn', 'posRTE5', 'InputDelay', timeDelayServo_s);
sysLLE = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdLLE [rad]', 'outputn', 'posLLE', 'InputDelay', timeDelayServo_s);
sysRLE = tf(1, [1/servoBW_rps, 1], 'inputn', 'cmdRLE [rad]', 'outputn', 'posRLE', 'InputDelay', timeDelayServo_s);
sysMotor = tf(maxThrust_lbf, [1/motorBW_rps, 1],'inputn', 'cmdMotor []', 'outputn', 'Thrust lb', 'InputDelay', timeDelayMotor_s);

sysAct = ss(append(sysLTE1, sysRTE1, sysLTE2, sysRTE2, sysLTE3, sysRTE3, sysLTE4, sysRTE4, sysLTE5, sysRTE5, sysLLE, sysRLE, sysMotor));

% Convert Left and Right into Symetric and Asym
indxEffLeft = 1:2:12;
indxEffRight = 2:2:12;

indxEffSym = 1:6;
indxEffAsym = 8:13;
indxEffMotor = 7;

sysSym = 0.5 * sumblk('%S = %L + %R', sysLinAug(:, indxEffSym).InputName, sysAct(indxEffLeft, :).OutputName, sysAct(indxEffRight, :).OutputName);
sysAsym = 0.5 * sumblk('%S = %L - %R', sysLinAug(:, indxEffAsym).InputName, sysAct(indxEffLeft, :).OutputName, sysAct(indxEffRight, :).OutputName);

sysLinSurf = connect(sysSym, sysAsym, sysLinAug, [sysSym.InputName; sysLinAug.InputName(indxEffMotor)], [sysLinAug.OutputName]);

% Sensor models
timeDelaySensor_s = 0.9*dt + 0.002; % 90% of frame is computation, 2 ms for sensor response.
sensorBW_rps = 20 * 2*pi;
sensorAirspeedBW_rps = 1.0 * 2*pi;
sensorHeightBW_rps = 1.0 * 2*pi;

sysSensPhi = tf(1, [1/sensorBW_rps, 1], 'inputn', 'phi [rad]', 'outputn', 'sensPhi [rad]', 'OutputDelay', timeDelaySensor_s);
sysSensTheta = tf(1, [1/sensorBW_rps, 1], 'inputn', 'Theta CB [rad]', 'outputn', 'sensTheta [rad]', 'OutputDelay', timeDelaySensor_s);

sysSensP = tf(1, [1/sensorBW_rps, 1], 'inputn', 'pFMU [rad/s]', 'outputn', 'sensP [rad]', 'OutputDelay', timeDelaySensor_s);
sysSensQ = tf(1, [1/sensorBW_rps, 1], 'inputn', 'qFMU [rad/s]', 'outputn', 'sensQ [rad]', 'OutputDelay', timeDelaySensor_s);
sysSensR = tf(1, [1/sensorBW_rps, 1], 'inputn', 'r [rad/s]', 'outputn', 'sensR [rad/s]', 'OutputDelay', timeDelaySensor_s);

sysSensSpeed = tf(ft2m, [1/sensorAirspeedBW_rps, 1], 'inputn', 'V [fps]', 'outputn', 'sensV [m/s]', 'OutputDelay', timeDelaySensor_s);
sysSensHeight = tf(ft2m, [1/sensorHeightBW_rps, 1], 'inputn', 'h [ft]', 'outputn', 'sensH [m]', 'OutputDelay', timeDelaySensor_s);

sysSensBend = tf(1, [1/sensorBW_rps, 1], 'inputn', 'eta1', 'outputn', 'sensBend [m/s^2]', 'OutputDelay', timeDelaySensor_s);

sysSens = ss(append(sysSensPhi, sysSensTheta, sysSensP, sysSensQ, sysSensR, sysSensSpeed, sysSensHeight));

% Pade approximation
% ordPade = 2;
% sysAct = pade(sysAct, ordPade);
% sysSens = pade(sysSens, ordPade);

% Combine Plant
sysPlant = connect(sysAct, sysLinSurf, sysSens, sysAct.InputName, [sysSens.OutputName; augStateOutName; 'Thrust lb']);


%% Controller Model
% SCAS-Phi Controller
sysScasPhi = pid2();
sysScasPhi.InputName = {'refPhi [rad]', 'sensPhi [rad]'};
sysScasPhi.OutputName = 'cmdRoll [rad/s]';
sysScasPhi.Kp =  0.2;
sysScasPhi.Ki = 0.05 * sysScasPhi.Kp;
sysScasPhi.Kd = 0.004;
sysScasPhi.Tf = dt;
sysScasPhi.b = 1;
sysScasPhi.c = 0;

% SCAS-Theta Controller
sysScasTheta = pid2();
sysScasTheta.InputName = {'refTheta [rad]', 'sensTheta [rad]'};
sysScasTheta.OutputName = 'cmdPitch_PID [rad/s]';
sysScasTheta.Kp = 0.286;
sysScasTheta.Ki = 0.5 * sysScasTheta.Kp;
sysScasTheta.Kd = 0.0 * sysScasTheta.Kp;
sysScasTheta.Tf = dt;
sysScasTheta.b = 1;
sysScasTheta.c = 0;

% Bending Controller
sysScasBend = pid2();
sysScasBend.InputName = {'refBend [m/s^2]', 'sensBend [m/s^2]'};
sysScasBend.OutputName = 'cmdBend []';
sysScasBend.Kp = 0.0;


% Thrust to Pitch Cross-Feed:
qBar_Pa = 0.5 * (V_mps)^2;
kCF = 1.96 * d2r * maxThrust_lbf / (0.002373 * (m2ft)^2);

sysScasCrossTf = tf((kCF / qBar_Pa) * tf(4, [1, 4]));
sysScasCrossTf.InputName = 'cmdThrust []';
sysScasCrossTf.OutputName = 'cmdPitch_cross [rad/s]';

sysScasCrossSum = sumblk('cmdPitch [rad/s] = cmdPitch_PID [rad/s] + cmdPitch_cross [rad/s]');
sysScasCross = connect(sysScasCrossTf, sysScasCrossSum, sysScasCrossTf.InputName, sysScasCrossSum.OutputName);


% Append SCAS
sysScas = append(sysScasPhi, sysScasTheta, sysScasBend, sysScasCrossTf, sysScasCrossSum);


%% Mixer
sysLinSurfD = c2d(sysLinSurf, dt, 'zoh');

indxAllocObjNames =   {'p', 'q', 'eta1dt'};
indxAllocObj = [];
for indxObj = 1:length(indxAllocObjNames)
    indxAllocObj(indxObj) = find(strcmp(sysLinSurfD.StateName, indxAllocObjNames{indxObj}));
end

indxAllocEffNames = {'posLTE1', 'posRTE1', 'posLTE2', 'posRTE2', 'posLTE3', 'posRTE3', 'posLTE4', 'posRTE4', 'posLTE5', 'posRTE5', 'posLLE', 'posRLE'};
indxAllocEff = [];
for indxEff = 1:length(indxAllocEffNames)
    indxAllocEff(indxEff) = find(strcmp(sysLinSurfD.InputName, indxAllocEffNames{indxEff}));
end

ctrlEff = ss(sysLinSurfD.B(indxAllocObj, indxAllocEff));
ctrlEff.D (abs(ctrlEff.D) < 0.1) = 0.0;

ctrlEff.OutputName = {'cmdRollMix [rad/s]', 'cmdPitchMix [rad/s]', 'cmdBendMix []'};
ctrlEff.InputName = sysPlant.InputName(1:12);

sysMixerSurf = ss(pinv(ctrlEff.D));
sysMixerSurf.InputName = ctrlEff.OutputName;
sysMixerSurf.OutputName = ctrlEff.InputName;

sysMixerSpeed = ss(1, 'inputn', {'cmdThrust []'}, 'outputn', {'cmdMotor []'});

sysMixer = append(sysMixerSurf, sysMixerSpeed);

%% Excitation
sysExcRoll = sumblk('cmdRollMix [rad/s] = excRoll + cmdRoll [rad/s]');
sysExcPitch = sumblk('cmdPitchMix [rad/s] = excPitch + cmdPitch [rad/s]');
sysExcBend = sumblk('cmdBendMix [rad/s] = excBend + cmdBend []');

sysExc = append(sysExcRoll, sysExcPitch, sysExcBend);

%% Combine Controller and Mixer
% Systems
sysCtrl = connect(sysScas, sysExc, sysMixer, [sysScas.InputName(1:2:5); sysExc.InputName(1:2:end)], sysMixer.OutputName);
sysOL = series(sysCtrl, sysPlant);

% Combine Controllers, with analysis pointat excitation injection
sysCtrlCL = connect(sysScas, sysExc, sysMixer, sysPlant, ...
    [sysCtrl.InputName], [sysPlant.OutputName; sysExc.InputName(2:2:end); sysCtrl.OutputName], sysExc.InputName(2:2:end));

sysCtrlL = getLoopTransfer(sysCtrlCL, sysExc.InputName(2:2:end), 1);

return;
%% Plots
% Steps
timeStep_s = 5;
opt = stepDataOptions;

opt.StepAmplitude = 10 *d2r;
figure(1); step(sysCtrlCL([1, 16], 1), timeStep_s, opt);

opt.StepAmplitude = 2 *d2r;
figure(2); step(sysCtrlCL([2, 16, 6, 8], 2), timeStep_s, opt);

%%
% Bode
figure(3); margin(sysCtrlL(1,1)); grid on; xlim([0.1,100]);
figure(4); margin(sysCtrlL(2,2)); grid on; xlim([0.1,100]);

