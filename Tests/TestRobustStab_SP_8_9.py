"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan

Example script for testing Frequency Response Estimation - MIMO.
"""

import numpy as np
import matplotlib.pyplot as plt
import control

# Hack to allow loading the Core package
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), "..")))
    path.insert(0, abspath(join(dirname(argv[0]), "..", 'Core')))

    del path, argv, dirname, abspath, join

from Core import FreqTrans


# Constants
hz2rps = 2*np.pi
rps2hz = 1/hz2rps

rad2deg = 180/np.pi
deg2rad = 1/rad2deg


#%% Define a linear plant systems
omega = np.logspace(-3,2,101)
tau = 75

G = control.tf([[[0, -87.8], [0, 1.4]], [[0, -108.2], [0, -1.4]]], [[[tau, 1], [tau, 1]], [[tau, 1], [tau, 1]]])
K = control.tf([[[-0.0015*tau, -0.0015*1], [0]], [[0], [-0.075*tau, -0.075*1]]], [[[1, 1e-6], [1]], [[1], [1, 1e-6]]])
wI = 0.2 * control.tf([5, 1],[0.5, 1])

G_frd = control.frd(G, omega)
K_frd = control.frd(K, omega)
Wi_frd = control.frd(wI, omega)

Wi_fresp = Wi_frd.fresp[0,0]
WiAbs_fresp = np.abs(Wi_fresp)

Li = K * G
Li_frd = control.frd(Li, omega)

Si_frd = control.FRD(Li_frd.fresp, omega)
for iFreq in range(len(omega)):
  Si_frd.fresp[..., iFreq] = np.linalg.inv(np.eye(2) + Li_frd.fresp[..., iFreq])

Ti_frd = Si_frd * Li_frd

svTi = FreqTrans.Sigma(Ti_frd.fresp)


# Compute strucuted singular value
Delta = np.ones((2,2), dtype=float)
muTi_bnds, muTi_info = FreqTrans.SigmaStruct(Ti_frd.fresp, Delta, bound = 'upper')


#%%
plt.figure('S&P Example 8.9 (Figure 8.11)')
plt.plot(omega, 1/WiAbs_fresp, '--r', label = '1/|Wi|}')
plt.plot(omega, muTi_bnds.T, 'b', label = 'mu(Ti)')
plt.plot(omega, svTi[0], 'g', label = 'max sv(Ti)')
plt.plot(omega, svTi[1], ':g', label = 'min sv(Ti)')

plt.xlabel('Frequency [rad/sec]')
plt.ylabel('Magnitude [-]')
plt.xscale('log')
plt.yscale('log')
plt.grid(True)
plt.legend()

#%%
M_fresp = Ti_frd.fresp * Wi_frd.fresp

svM = FreqTrans.Sigma(M_fresp)
svM_max = np.max(svM, axis = 0)

muM_bnds, muM_info = FreqTrans.SigmaStruct(M_fresp, Delta, bound = 'both')
muM_min = np.min(muM_bnds, axis = 0)

km = 1/muM_min

mag2db = control.mag2db

plt.figure('S&P Example 8.9 (Stability Margins)');
plt.semilogx(omega, mag2db(1/svM_max), 'g-', label = '1 / svM_max')
plt.semilogx(omega, mag2db(1/muM_min), 'b-', label = '1 / muM_min')
plt.semilogx(omega, mag2db(1) * np.ones_like(omega), 'r-', label = 'Crit = 0')
plt.semilogx(omega, mag2db(1/0.4) * np.ones_like(omega), 'r--', label = 'Crit = 0.4')

plt.xlabel('Frequency [rad/sec]')
plt.ylabel('Stability Margin [dB]')
plt.grid(True)
plt.legend()
