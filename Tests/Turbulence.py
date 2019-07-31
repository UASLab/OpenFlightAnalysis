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

#%%
V_fps = 55
h_ft = 300
b_ft = 4
level = 'Light'

freq_hz = np.logspace(-2, 1, 500)
freq_rps = freq_hz * hz2rps

sigma = Environment.TurbIntensityLow(h_ft, level = level)
L_ft = Environment.TurbLengthScaleLow(h_ft)

Pv, Pw =  Environment.TurbSpectDryden(sigma, L_ft, freq_rps, V_fps, b_ft)
plt.figure(1)
#plt.semilogx(freq_hz, Pv[0], label = "Level: " + level + ", Direction: u")
#plt.semilogx(freq_hz, Pv[1], label = "Level: " + level + ", Direction: v")
#plt.semilogx(freq_hz, Pv[2], label = "Level: " + level + ", Direction: w")
plt.semilogx(freq_hz, Pw[0], label = "Level: " + level + ", Direction: p")
plt.semilogx(freq_hz, Pw[1], label = "Level: " + level + ", Direction: q")
plt.semilogx(freq_hz, -Pw[2], label = "Level: " + level + ", Direction: r")

gyroNoise_rps = 0.00175
gyroNoise_rps = np.repeat(gyroNoise_rps, freq_hz.shape[0])

#plt.semilogx(freq_hz, (gyroNoise_rps),  label = "Gyro Noise Specification")

plt.xlabel("Frequency (Hz)")
plt.ylabel("Disturbance (rad/s)")
plt.suptitle("Dryden Turbulence and Gyro Noise - h = 300 ft, b = 4 ft, V=55 ft/s")

plt.grid(True)
plt.legend()