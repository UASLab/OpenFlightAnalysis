% Example 8.9 Produces fig. 8.11
% 
% Copyright 1996-2003 Sigurd Skogestad & Ian Postlethwaite
% $Id: Expl8_9.m,v 1.1 2004/01/21 15:21:24 vidaral Exp $
% clear all; close all;
omega = logspace(-3,2,101);

tau = 75; 
G = tf([0 1],[tau 1])*[-87.8 1.4;-108.2 -1.4]; % Plant
K = tf([tau 1],[1 1e-6])*[-0.0015 0; 0 -0.075]; % Controller
wI = 0.2*tf([5 1],[0.5 1]); % Input uncertainty

Li = K * G;
Si = inv(eye(2) + Li);
Ti = Si * Li;

Tif = frd(Ti, omega);
Wif = frd(wI, omega);

svTi = sigma(Ti, omega);

blk=[1 1;1 1]; % all 1-by-1 complex blocks
[muTi_bnds, muTi_info] = mussv(Tif, blk);

figure(1); % 'Figure 8.11'
loglog(omega, squeeze(1/frdata(Wif)), 'r--'); hold on; grid on;
loglog(omega, squeeze(min(frdata(muTi_bnds))), 'b-');
loglog(omega, svTi(1,:), 'g-');
loglog(omega, svTi(2,:), 'g--');
legend({'1/|W_i|', '\mu_{\Delta_I}(T_i)', 'max \sigma(T_i)', 'min \sigma(T_i)'});

hold off
axis auto

%%
M = Tif * Wif;
svM = sigma(M, omega);
svM_max = max(svM);

[muMbnds, muMinfo] = mussv(M, blk);

muM = squeeze(min(frdata(muMbnds)));
km = 1./muM;

figure(2);
semilogx(omega, mag2db(1./svM_max), 'g-'); hold on; grid on;
semilogx(omega, mag2db(km), 'b-');
semilogx(omega, mag2db(1) * ones(size(omega)), 'r-');
semilogx(omega, mag2db(1/0.4) * ones(size(omega)), 'r--');
% legend({'max \sigma(M)', '\mu_{\Delta_I}(M)', 'Crit = 0', 'Crit = 0.4'});
legend({'max \sigma(M)', '\mu_{\Delta_I}(M)'});
xlabel('Frequency [rad/sec]')
ylabel('Margin [dB]')


hold off
axis auto