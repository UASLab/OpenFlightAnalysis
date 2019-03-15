function [data] = StructFlightGoldy1(raw, fileVers)
% Create the FDAT structure from the Goldy 1 Raw structure
%
%Inputs:
% raw   - Raw flight structure
% fileVers    - Version of flight system []
%
%Outputs:
% data - Structure of file variables
%
%Notes:
% Any Special Notes.
%

%Version History: Version 1.4
% 09/23/2016  C. Regan     Initial Release (v1.0)
% 10/03/2016  C. Regan     Nav filter output was in radians (v1.1)
% 10/03/2016  C. Regan     Fixed Accel Bias Names (v1.2)
% 10/17/2016  C. Regan     Changed to simple and faster input parsing (v1.3)
% 10/17/2016  C. Regan     Added a file version for Goldy1 (v1.4)
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(1, 2);
if nargin < 2
    fileVers = [];
end

% Default Values

% Check the number of outputs
nargoutchk(0, 1);


%% Constants
r2d = 180/pi;


%% Load the appropriate function for the type of flight computer
data.raw = raw;

% Time
data.time_s = raw.time;

% Actuation
switch lower(fileVers)
    case {10, '10'}
        data.act.l1_rad = raw.l1;
        data.act.l2_rad = raw.da;
        data.act.l3_rad = raw.de;
        data.act.l4_rad = raw.l4;
        data.act.r1_rad = raw.r1;
        data.act.r2_rad = -raw.da;
        data.act.r3_rad = raw.de;
        data.act.r4_rad = raw.r4;
        data.act.thrt_nd = raw.dthr;
        
    case {20, '20', 30, '30', 40, '40'}
        data.act.l1_rad = raw.l1;
        data.act.l2_rad = raw.l2;
        data.act.l3_rad = raw.l3;
        data.act.l4_rad = raw.l4;
        data.act.r1_rad = raw.r1;
        data.act.r2_rad = raw.r2;
        data.act.r3_rad = raw.r3;
        data.act.r4_rad = raw.r4;
        data.act.thrt_nd = raw.dthr;
        
    otherwise
end

% Air Data
data.aird.h_m = raw.h;
data.aird.hFilt_m = raw.h_filt;
data.aird.ias_mps = raw.ias;
data.aird.iasFilt_mps = raw.ias_filt;
data.aird.presDyn_Pa = raw.Pd;
data.aird.presStatic_Pa = raw.Ps;

% Controllers
switch lower(fileVers)
    case {40, '40'}
        data.cntrl.altCmd_m = raw.alt_cmd;
    otherwise
        data.cntrl.altCmd_m = NaN(size(data.time_s));
end
data.cntrl.iasCmd_mps = raw.ias_cmd;
data.cntrl.phiCmd_rad = raw.phi_cmd;
data.cntrl.pitchCmd_rad = raw.pitch_cmd_pilot;
data.cntrl.rollCmd_rad = raw.roll_cmd_pilot;
data.cntrl.thetaCmd_rad = raw.theta_cmd;
data.cntrl.zDotCmd_mps = raw.zdot_cmd;

% GPS
data.gps.alt_m = raw.alt;
switch lower(fileVers)
    case {30, '30', 40, '40'}
        data.gps.hAcc_m = raw.gps_hAcc;
        data.gps.velN_mps = raw.gps_vn;
        data.gps.velE_mps = raw.gps_ve;
        data.gps.velD_mps = raw.gps_vd;
        data.gps.updateFlag_nd = raw.gps_update;
    case {20, '20'}
        data.gps.hAcc_m = NaN(size(data.time_s));
        data.gps.velN_mps = NaN(size(data.time_s));
        data.gps.velE_mps = NaN(size(data.time_s));
        data.gps.velD_mps = NaN(size(data.time_s));
        data.gps.updateFlag_nd = raw.gps_update;
    otherwise
        data.gps.hAcc_m = NaN(size(data.time_s));
        data.gps.updateFlag_nd = NaN(size(data.time_s));
        data.gps.velN_mps = NaN(size(data.time_s));
        data.gps.velE_mps = NaN(size(data.time_s));
        data.gps.velD_mps = NaN(size(data.time_s));
end
data.gps.lat_deg = raw.lat;
data.gps.lon_deg = raw.lon;
data.gps.satVisible_int = raw.satVisible;

% Health and Status
data.health.adStatus = raw.adStatus;
data.health.cpuLoad_nd = raw.cpuLoad;
switch lower(fileVers)
    case {40, '40'}
        data.health.etimeAct_ms = raw.etime_actuators;
        data.health.etimeCntrl_ms = raw.etime_control;
        data.health.etimeDaq_ms = raw.etime_daq;
        data.health.etimeGuid_ms = raw.etime_guidance;
        data.health.etimeLog_ms = raw.etime_datalog;
        data.health.etimeMiss_ms = raw.etime_mission;
        data.health.etimeNav_ms = raw.etime_nav;
        data.health.etimeSysid_ms = raw.etime_sysid;
        data.health.etimeTelem_ms = raw.etime_telemetry;
        data.health.loopCounter = raw.loopCounter;
        
    otherwise
        data.health.etimeAct_ms = NaN(size(data.time_s));
        data.health.etimeCntrl_ms = NaN(size(data.time_s));
        data.health.etimeDaq_ms = NaN(size(data.time_s));
        data.health.etimeGuid_ms = NaN(size(data.time_s));
        data.health.etimeLog_ms = NaN(size(data.time_s));
        data.health.etimeMiss_ms = NaN(size(data.time_s));
        data.health.etimeNav_ms = NaN(size(data.time_s));
        data.health.etimeSysid_ms = NaN(size(data.time_s));
        data.health.etimeTelem_ms = NaN(size(data.time_s));
        data.health.loopCounter = NaN(size(data.time_s));
end

% IMU
data.imu.accelX_mps2 = raw.ax;
data.imu.accelY_mps2 = raw.ay;
data.imu.accelZ_mps2 = raw.az;
data.imu.hx_gauss = raw.hx;
data.imu.hy_gauss = raw.hy;
data.imu.hz_gauss = raw.hz;
data.imu.p_rps = raw.p;
data.imu.phi_rad = raw.phi;
data.imu.psi_rad = raw.psi;
data.imu.q_rps = raw.q;
data.imu.r_rps = raw.r;
data.imu.status = raw.imuStatus;
data.imu.theta_rad = raw.theta;

% Instrumentation
data.inst.accelCF_mps2 = raw.accel_cf;
data.inst.accelCR_mps2 = raw.accel_cr;
data.inst.accelLF_mps2 = raw.accel_lf;
data.inst.accelLR_mps2 = raw.accel_lr;
data.inst.accelRF_mps2 = raw.accel_rf;
data.inst.accelRR_mps2 = raw.accel_rr;

% Mission
data.miss.mode = raw.mode;
data.miss.clawMode = raw.claw_mode;
data.miss.clawSelect = raw.claw_select;
data.miss.runNum_nd = raw.run_num;
data.miss.altInit_m = raw.init_alt;

% Navigation Estimation (EKF)
data.nav.accelXBias_mps2 = raw.ax_bias;
data.nav.accelYBias_mps2 = raw.ay_bias;
data.nav.accelZBias_mps2 = raw.az_bias;
data.nav.alt_m = raw.navalt;
data.nav.lat_deg = raw.navlat * r2d;
data.nav.lon_deg = raw.navlon * r2d;
data.nav.pBias_rps = raw.p_bias;
data.nav.qBias_rps = raw.q_bias;
data.nav.rBias_rps = raw.r_bias;
data.nav.validFlag_nd = raw.navValid;
data.nav.velD_mps = raw.navvd;
data.nav.velE_mps = raw.navve;
data.nav.velN_mps = raw.navvn;

% Pilot Commands
switch lower(fileVers)
    case {20, '20', 30, '30', 40, '40'}
        data.pilot.selectSwitch_nd = raw.select_incp;
        data.pilot.modeSwitch_nd = raw.mode_incp;
    otherwise
      data.pilot.selectSwitch_nd = NaN(size(data.time_s));
        data.pilot.modeSwitch_nd = NaN(size(data.time_s));
end
data.pilot.pitch_nd = raw.pitch_incp;
data.pilot.roll_nd = raw.roll_incp;

% System Identification and Excitation
data.sysid.pitchExcite_rad = raw.pitch_cmd_excite;
data.sysid.rollExcite_rad = raw.roll_cmd_excite;
data.sysid.runExcitation = raw.run_excitation;
data.sysid.sysidSelect = raw.sysid_select;


%% Outputs


