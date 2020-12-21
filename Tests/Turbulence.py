# -*- coding: utf-8 -*-
"""
Created on Fri Jul 26 11:47:02 2019

@author: rega0051
"""
import numpy as np
import matplotlib.pyplot as plt

# Hack to allow loading the Core package
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), "..")))
    path.insert(0, abspath(join(dirname(argv[0]), "..", 'Core')))

    del path, argv, dirname, abspath, join

import Environment

hz2rps = 2 * np.pi
rps2hz = 1/hz2rps

#%%
V_fps = 75
h_ft = 300
b_ft = 5
level = 'Light'

# Spatial Frequency
omega = np.logspace(-4, 1, 300)

sigma = Environment.TurbIntensity(h_ft, level = level)
L_ft = Environment.TurbLengthScale(h_ft)

# Linear Velocity
Puvw_Dryden =  Environment.TurbSpectDryden(sigma, L_ft, omega)
Puvw_VonKarman =  Environment.TurbSpectVonKarman(sigma, L_ft, omega)

fig, ax = plt.subplots(nrows=3, sharex='all', sharey='all')
ax[0].loglog(omega*rps2hz, Puvw_Dryden[0], label = "Dryden - Level: " + level)
ax[0].loglog(omega*rps2hz, Puvw_VonKarman[0], label = "VonKarman - Level: " + level)
ax[0].set_ylabel("Disturbance u"); ax[0].legend(); ax[0].grid(True)

ax[1].loglog(omega*rps2hz, Puvw_Dryden[1], label = "Dryden - Level: " + level)
ax[1].loglog(omega*rps2hz, Puvw_VonKarman[1], label = "VonKarman - Level: " + level)
ax[1].set_ylabel("Disturbance v"); ax[1].legend(); ax[1].grid(True)

ax[2].loglog(omega*rps2hz, Puvw_Dryden[2], label = "Dryden - Level: " + level)
ax[2].loglog(omega*rps2hz, Puvw_VonKarman[2], label = "VonKarman - Level: " + level)
ax[2].set_xlabel("Frequency (cycle/ft)"); ax[2].set_ylabel("Disturbance w"); ax[2].legend(); ax[2].grid(True)
#ax[2].set_xlim([0.1, 50])
fig.suptitle("Turbulence Spectrum")


#%% Rotation Rate
freq_rps = omega * V_fps
freq_hz = freq_rps / hz2rps

Ppqr_Dryden =  Environment.TurbSpectRate(Puvw_Dryden / V_fps, sigma, L_ft, freq_rps, V_fps, b_ft)
Ppqr_VonKarman =  Environment.TurbSpectRate(Puvw_VonKarman / V_fps, sigma, L_ft, freq_rps, V_fps, b_ft)

fig, ax = plt.subplots(nrows=3, sharex='all', sharey='all')
ax[0].loglog(freq_hz, Ppqr_Dryden[0], label = "Dryden - Level: " + level)
ax[0].loglog(freq_hz, Ppqr_VonKarman[0], label = "VonKarman - Level: " + level)
ax[0].set_ylabel("Disturbance p (rad/s)"); ax[0].legend(); ax[0].grid(True)

ax[1].loglog(freq_hz, Ppqr_Dryden[1], label = "Dryden - Level: " + level)
ax[1].loglog(freq_hz, Ppqr_VonKarman[1], label = "VonKarman - Level: " + level)
ax[1].set_ylabel("Disturbance q (rad/s)"); ax[1].legend(); ax[1].grid(True)

ax[2].loglog(freq_hz, -Ppqr_Dryden[2], label = "Dryden - Level: " + level)
ax[2].loglog(freq_hz, -Ppqr_VonKarman[2], label = "VonKarman - Level: " + level)
ax[2].set_xlabel("Frequency (Hz)"); ax[2].set_ylabel("Disturbance r (rad/s)"); ax[2].legend(); ax[2].grid(True)
ax[2].set_xlim([0.1, 50])
fig.suptitle("Turbulence Spectrum")
