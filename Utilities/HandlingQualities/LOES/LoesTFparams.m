function [tfParamsInit, tfParamsLB, tfParamsUB, tfParamsParams, tfParamsInfo, tfParamsName, tfParamsTF, tfParamsName2, tfParamsTF2] = LoesTFparams(typeTF)
% Returns the transfer function parameters for fitting LOES.
%
%Usage:  [tfParamsInit, tfParamsLB, tfParamsUB, tfParamsParams, tfParamsInfo, tfParamsName, tfParamsTF, tfParamsName2, tfParamsTF2] = LoesTFparams(typeTF);
%
%Inputs:
% typeTF - transfer function type
%
%Outputs:
% tfParamsInit   - initial value of the transfer function parameters
% tfParamsLB     - lower bound of the transfer function parameters
% tfParamsUB     - upper bound of the transfer function parameters
% tfParamsParams - parameters of the transfer function
% tfParamsInfo   - info about the transfer function
% tfParamsName   - name of the transfer function
% tfParamsTF     - the transfer function form
% tfParamsName2  - name of the 2nd transfer function
% tfParamsTF2    - the 2nd transfer function form
%
%Notes:
% Options:  typeTF   transfer function
%             1           'q/dep'
%             2           'nz/dep'
%             3      'q/dep' & 'nz/dep' (MIL-STD-1797)
%             4      'q/dep' & 'nz/dep' (LOES)
%             5      'q/dep' & 'aoa/dep' (LOES)
%             6           'aoa/dep'
%             7           'p/dap'
%             8           'p/dap' (high-order)
%             9           'beta/drp'
%             10           'beta/drp' (high-order)
%             11           'phi/dap' (high-order)
%             12           'phi/dap'
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(1, 1, nargin, 'struct'))
error(nargoutchk(0, 9, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs


%% Look-up TF type
switch typeTF
    case 1
        tfParamsName = 'q/dep';
        tfParamsInfo = ['LOES fit of ''' tfParamsName ''''];
        tfParamsTF = {...
            'Kq*(s + Lalpha)*exp(-t*s)';...
            '------------------------';...
            's*s + 2*z*w*s + w*w'};

        tfParamsParams = {'z', 'w', 'Kq', 't', 'Lalpha'};
        tfParamsInit = [0.7, 4.0, 50, 0.01, 1.5];
        tfParamsLB = [0, 0, 0, 0, 0];
        tfParamsUB = [2.5, 15, 300, 0.3, 15];
    case 2
        tfParamsName = 'nz/dep';
        tfParamsInfo = ['LOES fit of ''' tfParamsName ''''];
        tfParamsTF = {...
            'Kn*exp(-t*s)';...
            '-----------';...
            's*s + 2*z*w*s + w*w'};

        tfParamsParams = {'z', 'w', 'Kn', 't'};
        tfParamsInit = [0.7, 4, 10, 0.01];
        tfParamsLB = [0, 1, 0, 0];
        tfParamsUB = [2.5, 15, 50, 0.1];
    case 3
        % Simultaneous fit of q/dep and nz/dep from MIL-STD-1797
        tfParamsName = 'q/dep';
        tfParamsTF = {...
            'K1*s*(s + 1/Ttheta1)*(s + 1/Ttheta2)*exp(-t1*s)';...
            '--------------------------------------------';...
            '(s*s + 2*zp*wp + wp*wp)*(s*s + 2*zsp*wsp + wsp*wsp)'};

        tfParamsName2 = 'nz/dep';
        tfParamsTF2 = {...
            'Kn*(s + 1/Th1)*exp(-t2*s)';...
            '-------------------------------------------';...
            '(s*s + 2*zp*wp + wp*wp)*(s*s + 2*zsp*wsp + wsp*wsp)'};
        
        tfParamsInfo = ['Simultaneous LOES fit of ''' tfParamsName ''' and ''' tfParamsName2 ''' (MIL-STD-1797)'];
        
        tfParamsParams = {'K1', 'Ttheta1', 'Ttheta2', 't1', 'Kn', 'Th1', 't2', 'zp', 'wp', 'zsp', 'wsp'};
        tfParamsInit = [10, 10, 0.2, 0.01, 10, 10, 0.01, 0.7, 0.5, 0.7, 5];
        tfParamsLB = [0, 0.2, 0.01, 0, 0, 0.2, 0, 0, 0, 0, 0];
        tfParamsUB = [200, Inf, Inf, 0.1, 200, Inf, 0.1, 1.5, 5, 1.5, 15];
    case 4
        % Simultaneous fit of q/dep and nz/dep from Low-order approximations
        tfParamsName = 'q/dep';
        tfParamsTF = {...
            'Kq*(s + Lalpha)*exp(-t1*s)';...
            '----------------------------';...
            's*s + 2*zsp*wsp*s + wsp*wsp'};

        tfParamsName2 = 'nz/dep';
        tfParamsTF2 = {...
            'Kn*exp(-t2*s)';...
            '-------------------------';...
            's*s + 2*zsp*wsp*s + wsp*wsp'};
        
        tfParamsInfo = ['Simultaneous LOES fit of ''' tfParamsName ''' and ''' tfParamsName2 ''' (low order)'];

        tfParamsParams = {'Kq', 'Lalpha', 't1', 'Kn', 't2', 'zsp', 'wsp'};
        tfParamsInit = [10, 1.5, 0.01, 10, 0.01, 0.7, 5];
        tfParamsLB = [0, 0, 0, 0, 0, 0, 0];
        tfParamsUB = [300, 15, 0.1, 50, 0.1, 1.5, 15];
    case 5
        % Simultaneous fit of q/dep and aoa/dep from Low-order approximations
        tfParamsName = 'q/dep';
        tfParamsTF = {...
            'Kq*(s + Lalpha)exp(-t1*s)';...
            '-------------------------';...
            's*s + 2*zsp*wsp*s + wsp*wsp'};

        tfParamsName2 = 'aoa/dep';
        tfParamsTF2 = {...
            'Ka*exp(-t2*s)';...
            '---------------------';...
            's*s + 2*zsp*wsp*s + wsp*wsp'};

        tfParamsInfo = ['Simultaneous LOES fit of ''' tfParamsName ''' and ''' tfParamsName2 ''' (low order)'];
        
        tfParamsParams = {'Kq', 'Lalpha', 't1', 'Ka', 't2', 'zsp', 'wsp'};
        tfParamsInit = [10, 1.5, 0.01, 10, 0.01, 0.7, 5];
        tfParamsLB = [0, 0, 0, 0, 0, 0, 0];
        tfParamsUB = [300, 15, 0.1, 50, 0.1, 1.5, 15];
    case 6
        tfParamsName = 'aoa/dep';
        tfParamsInfo = ['LOES fit of ''' tfParamsName ''''];
        tfParamsTF = {...
            'Ka*exp(-t*s)';...
            '----------------';...
            's*s + 2*z*w*s + w*w'};

        tfParamsParams = {'z', 'w', 'Ka', 't'};
        tfParamsInit = [0.7, 4, 10, 0.01];
        tfParamsLB = [0, 0, 0, 0];
        tfParamsUB = [2.5, 15, 50, 0.1];
    case 7
        tfParamsName = 'p/dap';
        tfParamsInfo = ['LOES fit of ''' tfParamsName ''''];
        tfParamsTF = {...
            'Kp*exp(-t*s)';...
            '------------';...
            's + 1/Tr'};

        tfParamsParams = {'Tr', 'Kp', 't'};
        tfParamsInit = [0.25, 100, 0.01];
        tfParamsLB = [0.05, 0, 0];
        tfParamsUB = [Inf, 500, 1];
    case 8
        tfParamsName = 'p/dap';
        tfParamsInfo = ['LOES fit of ''' tfParamsName ''' (high order)'];
        tfParamsTF = {...
            'Kp*s*(s*s + 2*zp*wp*s + wp*wp)*exp(-t*s)';...
            '------------------------------------------';...
            '(s + 1/Tr)*(s + 1/Ts)*(s*s + 2*zd*wd*s + wd*wd)'};

        tfParamsParams = {'Tr', 'Ts', 'zd', 'wd', 't', 'Kp', 'wp', 'zp'};
        tfParamsInit = [0.25, 10, 0.7, 5, 0.1, 100, 4, 0.7];
        tfParamsLB = [0.05, -5, 0.3, 0, 0, 0, 0, 0];
        tfParamsUB = [Inf, 5, 1.5, 15, 1, 500, 25, 1.5];
    case 9
        tfParamsName = 'beta/drp';
        tfParamsInfo = ['LOES fit of ''' tfParamsName ''''];
        tfParamsTF = {...
            'Kb*exp(-t*s)';...
            '-------------';...
            's*s + 2*z*w + w*w'};

        tfParamsParams = {'z', 'w', 'Kb', 't'};
        tfParamsInit = [0.75, 3.7, 50, 0.1];
        tfParamsLB = [0, 1, 0, 0];
        tfParamsUB = [2.5, 15, 100, 0.5];
    case 10
        tfParamsName = 'beta/drp';
        tfParamsInfo = ['LOES fit of ''' tfParamsName ''' (high order)'];
        tfParamsTF = {...
            'Kb*(s + 1/Tb1)*(s + 1/Tb2)*(s + 1/Tb3)*exp(-t*s)';...
            '--------------------------------------------';...
            '(s + 1/Tr)*(s + 1/Ts)*(s*s + 2*z*w*s + w*w)'};

        tfParamsParams = {'z', 'w', 'Tr', 'Ts', 'Kb', 'Tb1', 'Tb2', 'Tb3', 't'};
        tfParamsInit = [0.7, 4, 0.25, 10, 10, 10, 1, 0.1, 0.1];
        tfParamsLB = [0, 0, 0.0667, -2, 0, -5, 0.2, 1/300, 0];
        tfParamsUB = [1.5, 15, Inf, 2, 100, 5, 5, 0.2, 0.2];
    case 11
        tfParamsName = 'phi/dap';
        tfParamsInfo = ['LOES fit of ''' tfParamsName ''' (high order)'];
        tfParamsTF = {...
            'Kphi*(s*s + 2*zp*wp*s + wp*wp)*exp(-t*s)';...
            '-------------------------------------------';...
            '(s + 1/Ts)*(s + 1/Tr)*(s*s + 2*zd*wd*s + wd*wd)'};

        tfParamsParams = {'zp', 'wp', 'Kphi', 't', 'zd', 'wd', 'Ts', 'Tr'};
        tfParamsInit = [0.7, 4, 10, 0.1, 0.7, 4, Inf, 10];
        tfParamsLB = [0, 0, 0, 0, 0, 0, -5, 0];
        tfParamsUB = [1, 15, 300, 0.2, 1, 10, 5, Inf];
    case 12
        tfParamsName = 'phi/dap';
        tfParamsInfo = ['LOES fit of ''' tfParamsName ''''];
        tfParamsTF = {...
            'Km*exp(-t*s)';...
            '-------------';...
            's*s + s*1/Tr'};

        tfParamsParams = {'Tr', 'Km', 't'};
        tfParamsInit = [4, 10, 0.01];
        tfParamsLB = [0, 0, 0];
        tfParamsUB = [1, 500, 0.1];
    otherwise
        warning('Unkown transfer function type');
        tfParams = [];
end


%% Check Outputs
