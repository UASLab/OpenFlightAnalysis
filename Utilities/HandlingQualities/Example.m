% Handling Qualities example.
%
%Notes:
% 
%
%Dependency:
% ExampleDir
% ExampleLat
% ExampleLong
%

%Version History: Verison 1.0
% 03/09/2006  C. Regan     Initial Release (v1.0)
%


%% FIXME: Script contains links to data not included in AVFTT
% close all

%% Pitch sweep analysis
if 1
%     clear all

    % Load file with pitch sweep data
    loadFile = 'freq_sweep_A_M8_H37.mat'; % C17 data

    load(loadFile);

    % Setup variables
    stkQ_in = PSTICK_1_;
    qBody_dps = QB;
    nz_g = NZPS;
    theta_deg = THETA;
    alpha_deg = ALPHAD;

    vTrue_fps = VTRUE(1); % just the trim value is used

    % Specify the flight category
    category = 'A';

    % Provide a meaningful descriptor for the plot titles
    plotTitleDesc = loadFile;

    % Plot some of the data for verification
    plot(time, stkQ_in, time, qBody_dps, time, alpha_deg);

    % Run the Longitudinal Example
    ExampleLong(stkQ_in, qBody_dps, nz_g, theta_deg, alpha_deg, vTrue_fps, category, plotTitleDesc);
end

%% Roll sweep analysis
if 0
    clear all

    % Load file with roll sweep data
    % TODO: data files aren't included
    %loadFile = 'R:\F15_IFCS\postflight\flt0184\data\fdas_c28d.mat';
    loadFile = '/export/home/reganc/F15_IFCS/postflight/flt0184/data/fdas_c28d.mat';

    load(loadFile);

    stkP_in = dap;
    pBody_dps = pbody;
    phi_deg = phi;
    ny_g = nycs;

    category = 'A';
    plotTitleDesc = loadFile;

    plot(time, stkP_in, time, pBody_dps);
    ExampleLat(stkP_in, pBody_dps, phi_deg, ny_g, category, plotTitleDesc);
end

%% Yaw sweep analysis
if 0
    clear all

    % Load file with yaw sweep data
    % TODO: data files aren't included
    %loadFile = 'R:\F15_IFCS\postflight\flt0181\data\fdas_c18e.mat';
    loadFile = '/export/home/reganc/F15_IFCS/postflight/flt0181/data/fdas_c18e.mat';

    load(loadFile);

    stkR_in = drp;
    rBody_dps = rbody;
    beta_deg = beta_cn03;

    category = 'A';
    plotTitleDesc = loadFile;

    plot(time, stkR_in, time, rBody_dps);
    ExampleDir(stkR_in, beta_deg, category, plotTitleDesc);
end
