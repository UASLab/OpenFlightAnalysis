function PlotOverviewGoldy1(sig)

%% Constants
r2d = 180/pi;
d2r = 1/r2d;


% Plot Mode Changes
figure;
subplot(4,1,1)
plot(sig.time_s, sig.miss.mode); grid on;
legend('mode');
subplot(4,1,2)
plot(sig.time_s, sig.miss.clawMode); grid on;
legend('clawMode');
subplot(4,1,3)
plot(sig.time_s, sig.miss.clawSelect); grid on;
legend('clawSelect');
subplot(4,1,4)
plot(sig.time_s, sig.sysid.sysidSelect); grid on;
legend('sysidSelect');
linkaxes(findobj(gcf, 'Type', 'axes'), 'x');

% Controller Performance
optPlotCell.xNames = {[], [], [], 'Time(s)'};
optPlotCell.yNames = {'Phi (deg)', 'Theta (deg)', 'Airspeed (m/s)', 'Altitude (m)'};
optPlotCell.legNames = {{'Cmd', 'Sens'}, {}, {}, {}};
optPlotCell.titleStr = 'Controller Performance';

PlotCell({sig.time_s}, ...
    {{sig.cntrl.phiCmd_rad * r2d, sig.imu.phi_rad * r2d};  
    {sig.cntrl.thetaCmd_rad * r2d + 4.0, sig.imu.theta_rad * r2d};
    {sig.cntrl.iasCmd_mps, sig.aird.ias_mps};
    {sig.cntrl.altCmd_m, sig.aird.h_m}}, ...
    optPlotCell);

% Plot Surfaces
optPlotCell.yNames = {[], [], [], []};
optPlotCell.legNames = {{'L1 (deg)', 'R1 (deg)'}, {'L2 (deg)', 'R2 (deg)'}, {'L3 (deg)', 'R3 (deg)'}, {'L4 (deg)', 'R4 (deg)'}};
optPlotCell.titleStr = 'Surface Commands';

PlotCell({sig.time_s}, ...
    {{sig.act.l1_rad * r2d, sig.act.r1_rad * r2d};  
    {sig.act.l2_rad * r2d, sig.act.r2_rad * r2d};  
    {sig.act.l3_rad * r2d, sig.act.r3_rad * r2d};  
    {sig.act.l4_rad * r2d, sig.act.r4_rad * r2d}}, ...
    optPlotCell);

% Pilot
optPlotCell.xNames = {[], 'Time(s)'};
optPlotCell.yNames = {'Roll Incp (nd)', 'Pitch Incp (nd)'};
optPlotCell.legNames = {[], []};
optPlotCell.titleStr = 'Inceptor Commands';

PlotCell({sig.time_s}, ...
    {{sig.pilot.roll_nd};  
    {sig.pilot.pitch_nd}}, ...
    optPlotCell);

% Velocity
optPlotCell.xNames = {[], [], 'Time(s)'};
optPlotCell.yNames = {'vNorth (m/s)', 'vEast (m/s)', 'vDown (m/s)'};
optPlotCell.legNames = {{'GPS', 'EKF'}, {}, {}, {}};
optPlotCell.titleStr = 'GPS and EKF Velocity';
PlotCell({sig.time_s}, ...
    {{sig.gps.velN_mps, sig.nav.velN_mps};  
    {sig.gps.velE_mps, sig.nav.velE_mps};  
    {sig.gps.velD_mps, sig.nav.velD_mps}}, ...
    optPlotCell);

% Position
optPlotCell.yNames = {'Lat (deg)', 'Long (deg)', 'Alt (m)'};
optPlotCell.titleStr = 'GPS and EKF Position';

PlotCell({sig.time_s}, ...
    {{sig.gps.lat_deg, sig.nav.lat_deg};  
    {sig.gps.lon_deg, sig.nav.lon_deg};  
    {sig.gps.alt_m, sig.nav.alt_m}}, ...
    optPlotCell);
