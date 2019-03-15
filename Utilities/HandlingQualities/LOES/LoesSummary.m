function [] = LoesSummary(outFile, typeTF, tfParamsValue, cost)
% Create a summary file for a LOES fit.
%
%Usage:  [] = LoesSummary(outFile, typeTF, tfParamsValue, cost);
%
%Inputs:
% outFile       - output file
% typeTF        - transfer function type; see: 'LoesTFparams'
% tfParamsValue - transfer function parameters
% cost          - weighted cost of the LOES fit
%
%Outputs:
% 
%
%Notes:
% 
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(4, 4, nargin, 'struct'))
error(nargoutchk(0, 0, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs


%% Open the output file
fid = fopen(outFile, 'a');


%% Write summary parameters to output file
switch typeTF
    case 1
        fprintf(fid,'Q/dep transfer function coefficients:\n');
        fprintf(fid,'Numerator:\n');
        fprintf(fid,'Kq = %6.3f\n',tfParamsValue(3));
        fprintf(fid,'L_alpha = %6.3f\n',tfParamsValue(5));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(4));
        fprintf(fid,'Denominator:\n');
        fprintf(fid,'Frequency = %6.3f\n',tfParamsValue(2));
        fprintf(fid,'Damping = %6.3f\n',tfParamsValue(1));
    case 2
        fprintf(fid,'Nz/dep transfer function coefficients:\n');
        fprintf(fid,'Numerator:\n');
        fprintf(fid,'Kn = %6.3f\n',tfParamsValue(3));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(4));
        fprintf(fid,'Denominator:\n');
        fprintf(fid,'Frequency = %6.3f\n',tfParamsValue(2));
        fprintf(fid,'Damping = %6.3f\n',tfParamsValue(1));
    case 3
        fprintf(fid,'Simultaneous fit of q/dep and nz/dep transfer functions\n');
        fprintf(fid,'MIL-STD-1797 format\n');
        fprintf(fid,'Q/dep transfer function Numerator:\n');
        fprintf(fid,'Kq = %6.3f\n',tfParamsValue(1));
        fprintf(fid,'1/T-theta1 = %6.3f\n',tfParamsValue(2));
        fprintf(fid,'1/T-theta2 = %6.3f\n',tfParamsValue(3));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(4));
        fprintf(fid,'Nz/dep transfer function numerator:\n');
        fprintf(fid,'Kn = %6.3f\n',tfParamsValue(5));
        fprintf(fid,'1/T-h1 = %6.3f\n',tfParamsValue(6));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(7));
        fprintf(fid,'Denominator:\n');
        fprintf(fid,'Phugoid Frequency = %6.3f\n',tfParamsValue(9));
        fprintf(fid,'Phugoid Damping = %6.3f\n',tfParamsValue(8));
        fprintf(fid,'Short Period Frequency = %6.3f\n',tfParamsValue(11));
        fprintf(fid,'Short Period Damping = %6.3f\n',tfParamsValue(10));
    case 4
        fprintf(fid,'Simultaneous fit of q/dep and nz/dep transfer functions\n');
        fprintf(fid,'Low order format\n');
        fprintf(fid,'Q/dep transfer function Numerator:\n');
        fprintf(fid,'Kq = %6.3f\n',tfParamsValue(1));
        fprintf(fid,'L_alpha = %6.3f\n',tfParamsValue(2));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(3));
        fprintf(fid,'Nz/dep transfer function numerator:\n');
        fprintf(fid,'Kn = %6.3f\n',tfParamsValue(4));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(5));
        fprintf(fid,'Denominator:\n');
        fprintf(fid,'Short Period Frequency = %6.3f\n',tfParamsValue(7));
        fprintf(fid,'Short Period Damping = %6.3f\n',tfParamsValue(6));
    case 5
        fprintf(fid,'Simultaneous fit of q/dep and aoa/dep transfer functions\n');
        fprintf(fid,'Low order format\n');
        fprintf(fid,'Q/dep transfer function Numerator:\n');
        fprintf(fid,'Kq = %6.3f\n',tfParamsValue(1));
        fprintf(fid,'L_alpha = %6.3f\n',tfParamsValue(2));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(3));
        fprintf(fid,'aoa/dep transfer function numerator:\n');
        fprintf(fid,'Ka = %6.3f\n',tfParamsValue(4));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(5));
        fprintf(fid,'Denominator:\n');
        fprintf(fid,'Short Period Frequency = %6.3f\n',tfParamsValue(7));
        fprintf(fid,'Short Period Damping = %6.3f\n',tfParamsValue(6));
    case 6
        fprintf(fid,'AOA/dep transfer function coefficients:\n');
        fprintf(fid,'Numerator:\n');
        fprintf(fid,'Ka = %6.3f\n',tfParamsValue(3));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(4));
        fprintf(fid,'Denominator:\n');
        fprintf(fid,'Frequency = %6.3f\n',tfParamsValue(2));
        fprintf(fid,'Damping = %6.3f\n',tfParamsValue(1));
    case 7
        fprintf(fid,'p/dap transfer function coefficients:\n');
        fprintf(fid,'Numerator:\n');
        fprintf(fid,'Kp = %6.3f\n',tfParamsValue(2));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(3));
        fprintf(fid,'Denominator:\n');
        fprintf(fid,'1/Tr = %6.3f\n',tfParamsValue(1));
    case 8
        fprintf(fid,'p/dap high order transfer function coefficients:\n');
        fprintf(fid,'Numerator:\n');
        fprintf(fid,'Kp = %6.3f\n',tfParamsValue(6));
        fprintf(fid,'Omega_phi = %6.3f\n',tfParamsValue(7));
        fprintf(fid,'Zeta_phi = %6.3f\n',tfParamsValue(8));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(5));
        fprintf(fid,'Denominator:\n');
        fprintf(fid,'1/Tr = %6.3f\n',tfParamsValue(1));
        fprintf(fid,'1/Ts = %6.3f\n',tfParamsValue(2));
        fprintf(fid,'Omega_d = %6.3f\n',tfParamsValue(4));
        fprintf(fid,'Zeta_d = %6.3f\n',tfParamsValue(3));
    case 9
        fprintf(fid,'b/drp transfer function coefficients:\n');
        fprintf(fid,'Numerator:\n');
        fprintf(fid,'Kb = %6.3f\n',tfParamsValue(3));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(4));
        fprintf(fid,'Denominator:\n');
        fprintf(fid,'Omega_d = %6.3f\n',tfParamsValue(2));
        fprintf(fid,'Zeta_d = %6.3f\n',tfParamsValue(1));
    case 10
        fprintf(fid,'b/drp high order transfer function coefficients:\n');
        fprintf(fid,'Numerator:\n');
        fprintf(fid,'Kb = %6.3f\n',tfParamsValue(5));
        fprintf(fid,'1/Tb1= %6.3f\n',tfParamsValue(6));
        fprintf(fid,'1/Tb2= %6.3f\n',tfParamsValue(7));
        fprintf(fid,'1/Tb3= %6.3f\n',tfParamsValue(8));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(9));
        fprintf(fid,'Denominator:\n');
        fprintf(fid,'1/Tr = %6.3f\n',tfParamsValue(3));
        fprintf(fid,'1/Ts = %6.3f\n',tfParamsValue(4));
        fprintf(fid,'Omega_d = %6.3f\n',tfParamsValue(2));
        fprintf(fid,'Zeta_d = %6.3f\n',tfParamsValue(1));
    case 11
        fprintf(fid,'phi/dap high order transfer function coefficients:\n');
        fprintf(fid,'Numerator:\n');
        fprintf(fid,'Kp = %6.3f\n',tfParamsValue(3));
        fprintf(fid,'Omega_phi = %6.3f\n',tfParamsValue(2));
        fprintf(fid,'Zeta_phi = %6.3f\n',tfParamsValue(1));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(4));
        fprintf(fid,'Denominator:\n');
        fprintf(fid,'1/Tr = %6.3f\n',tfParamsValue(8));
        fprintf(fid,'1/Ts = %6.3f\n',tfParamsValue(7));
        fprintf(fid,'Omega_d = %6.3f\n',tfParamsValue(6));
        fprintf(fid,'Zeta_d = %6.3f\n',tfParamsValue(5));
    case 12
        fprintf(fid,'phi/dap transfer function coefficients:\n');
        fprintf(fid,'Numerator:\n');
        fprintf(fid,'Km = %6.3f\n',tfParamsValue(2));
        fprintf(fid,'time delay = %7.4f\n',tfParamsValue(3));
        fprintf(fid,'Denominator:\n');
        fprintf(fid,'1/Tr = %6.3f\n',tfParamsValue(1));
end

fprintf(fid,'Cost function = %6.3f\n', cost);


%% Close the output file
fclose(fid);


%% Check Outputs
