function PlotOverviewGoldy1(sig)

%% Constants
r2d = 180/pi;
d2r = 1/r2d;


% Plot Mode Changes
figure;
subplot(4,1,1)
plot(sig.time_s, sig.Mission.socEngage); grid on;
legend('socEngage');
subplot(4,1,2)
plot(sig.time_s, sig.Mission.ctrlSel); grid on;
legend('ctrlSel');
subplot(4,1,3)
plot(sig.time_s, sig.Mission.testID); grid on;
legend('testID');
subplot(4,1,4)
plot(sig.time_s, sig.Mission.excitEngage); grid on;
legend('testEngage');
linkaxes(findobj(gcf, 'Type', 'axes'), 'x');

% Controller Performance
optPlotCell.xNames = {[], [], [], 'Time(s)'};
optPlotCell.yNames = {'Phi (deg)', 'Theta (deg)', 'Airspeed (m/s)', 'Altitude (m)'};
optPlotCell.legNames = {{'Ref', 'Sens'}, {'Ref', 'Sens'}, {'Ref', 'Sens'}, {'Ref', 'Sens'}};
optPlotCell.titleStr = 'Controller Performance';

figure;
PlotCell({sig.time_s}, ...
    {{sig.Control.refPhi_rad * r2d, sig.NavFilter.s_rad(1,:) * r2d};  
    {sig.Control.refTheta_rad * r2d, sig.NavFilter.s_rad(2,:) * r2d};   
    {sig.Control.refV_ms, sig.Sensor_Processing.vIAS_ms};
    {sig.Control.refH_m, sig.Sensor_Processing.hBaro_m}}, ...
    optPlotCell);

% Plot Surfaces
optPlotCell.xNames = {[], [], [], [], 'Time(s)'};
optPlotCell.yNames = {[], [], [], [], []};
optPlotCell.legNames = {{},{},{},{},{}};
optPlotCell.titleStr = 'Control Commands';

PlotCell({sig.time_s}, ...
    {{sig.Control.cmdEff(1,:)};  
    {sig.Control.cmdEff(2,:) * r2d};
    {sig.Control.cmdEff(3,:) * r2d}; 
    {sig.Control.cmdEff(4,:) * r2d, sig.Control.cmdEff(7,:) * r2d};  
    {sig.Control.cmdEff(5,:) * r2d, sig.Control.cmdEff(6,:) * r2d}}, ...
    optPlotCell);

% Pilot
optPlotCell.xNames = {[], [], [], [], 'Time(s)'};
optPlotCell.yNames = {'Roll Incp (nd)', 'Pitch Incp (nd)', 'Yaw Incp (nd)', 'Flap Incp (nd)', 'Throt Incp (nd)'};
optPlotCell.legNames = {{},{},{},{},{}};
optPlotCell.titleStr = 'Inceptor Commands';

PlotCell({sig.time_s}, ...
    {{sig.Pilot.Channels.x2};  
    {sig.Pilot.Channels.x3};
    {sig.Pilot.Channels.x4};
    {sig.Pilot.Channels.x5};
    {sig.Pilot.Channels.x6}}, ...
    optPlotCell);

% Velocity
optPlotCell.xNames = {[], [], 'Time(s)'};
optPlotCell.yNames = {'vNorth (m/s)', 'vEast (m/s)', 'vDown (m/s)'};
optPlotCell.legNames = {{'GPS', 'NavFilter'}, {}, {}, {}};
optPlotCell.titleStr = 'GPS and NavFilter Velocity';
PlotCell({sig.time_s}, ...
    {{sig.Gps.v_BE_L_mps(1,:), sig.NavFilter.v_BE_L_mps(1,:)};  
    {sig.Gps.v_BE_L_mps(2,:), sig.NavFilter.v_BE_L_mps(2,:)};  
    {sig.Gps.v_BE_L_mps(3,:), sig.NavFilter.v_BE_L_mps(3,:)}}, ...
    optPlotCell);

% Position
optPlotCell.yNames = {'Lat (deg)', 'Long (deg)', 'Alt (m)'};
optPlotCell.titleStr = 'GPS and NavFilter Position';

PlotCell({sig.time_s}, ...
    {{sig.Gps.r_BE_G_ddm(1,:), sig.NavFilter.r_BE_G_ddm(1,:)};  
    {sig.Gps.r_BE_G_ddm(2,:), sig.NavFilter.r_BE_G_ddm(2,:)};  
    {sig.Gps.r_BE_G_ddm(3,:), sig.NavFilter.r_BE_G_ddm(3,:)}}, ...
    optPlotCell);
