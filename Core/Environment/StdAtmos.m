function [pres_Pa, dens_kgpm3, temp_C, grav_mps2, dynVisc_Pas] = StdAtmos(alt_m)
% OpenFlightAnalysis - StdAtmos
% Compute standard atmosphere parameters (1976 model).
%
% Inputs:
%  alt_m - geometric altitude (m)
%
% Outputs:
%  pres_Pa - ambient static pressure (N/m^2 = Pa)
%  dens_kgpm3  - ambient air density (Kg/m^3)
%  temp_C      - ambient air temperature (deg C)
%  grav_mps2   - acceleration due to gravity (m/s^2)
%  dynVisc_Pas - Dynamic viscosity (N s / m^2 = Pa s)
%
% Notes:
%




%% Check I/O Arguments
narginchk(1, 1);
nargoutchk(0, 5);

%% Return lookup data for 1976 US Standard Atmosphere
% altTbl_ft = [-5000	0	5000	10000	15000	20000	25000	30000	35000	40000	45000	50000	60000	70000	80000	90000	100000	150000	200000	250000];
% tempTbl_F = [76.84	59	41.17	23.36	5.55	-12.26	-30.05	-47.83	-65.61	-69.7	-69.7	-69.7	-69.7	-67.42	-61.98	-56.54	-51.1	19.4	-19.78	-88.77];
% gravTbl_fps2 = [32.189	32.174	32.159	32.143	32.128	32.112	32.097	32.082	32.066	32.051	32.036	32.02	31.99	31.959	31.929	31.897	31.868	31.717	31.566	31.415];
% presTbl_psi = [17.554	14.696	12.228	10.108	8.297	6.759	5.461	4.373	3.468	2.73	2.149	1.692	1.049	0.651	0.406	0.255	0.162	0.02	0.003	0];
% densTbl_slugpft3 = [27.45	23.77	20.48	17.56	14.96	12.67	10.66	8.91	7.38	5.87	4.62	3.64	2.26	1.39	0.86	0.56	0.33	0.037	0.0053	0.00065];
% dynViscTbl_lbspft3 = [3.836	3.737	3.637	3.534	3.43	3.324	3.217	3.107	2.995	2.969	2.969	2.969	2.969	2.984	3.018	3.052	3.087	3.511	3.279	2.846];

altTbl_m = [-1000	0	1000	2000	3000	4000	5000	6000	7000	8000	9000	10000	15000	20000	25000	30000	40000	50000	60000	70000	80000];
tempTbl_C = [21.5	15	8.5	2	-4.49	-10.98	-17.47	-23.96	-30.45	-36.94	-43.42	-49.9	-56.5	-56.5	-51.6	-46.64	-22.8	-2.5	-26.13	-53.57	-74.51];
gravTbl_mps2 = [9.81	9.807	9.804	9.801	9.797	9.794	9.791	9.788	9.785	9.782	9.779	9.776	9.761	9.745	9.73	9.715	9.684	9.654	9.624	9.594	9.564];
presTbl_Pa = [11.39	10.13	8.988	7.95	7.012	6.166	5.405	4.722	4.111	3.565	3.08	2.65	1.211	0.5529	0.2549	0.1197	0.0287	0.007978	0.002196	0.00052	0.00011];
densTbl_kgpm3 = [1.347	1.225	1.112	1.007	0.9093	0.8194	0.7364	0.6601	0.59	0.5258	0.4671	0.4135	0.1948	0.08891	0.04008	0.01841	0.003996	0.001027	0.0003097	0.00008283	0.00001846];
dynViscTbl_Pas = [1.821	1.789	1.758	1.726	1.694	1.661	1.628	1.595	1.561	1.527	1.493	1.458	1.422	1.422	1.448	1.475	1.601	1.704	1.584	1.438	1.321];


%% Lookup data at specified altitude
temp_C = interp1(altTbl_m, tempTbl_C, alt_m, 'linear');
pres_Pa = interp1(altTbl_m, presTbl_Pa, alt_m, 'linear');
dens_kgpm3 = interp1(altTbl_m, densTbl_kgpm3, alt_m, 'linear');
grav_mps2 = interp1(altTbl_m, gravTbl_mps2, alt_m, 'linear');
dynVisc_Pas = interp1(altTbl_m, dynViscTbl_Pas, alt_m, 'linear');


