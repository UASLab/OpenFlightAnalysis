#%%
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches
# Hack to allow loading the Core package
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), "..")))
    path.insert(0, abspath(join(dirname(argv[0]), "..", 'Core')))
    
    del path, argv, dirname, abspath, join
    
import FreqTrans

#%%
pCrit = -1+0j
T = np.array([-0.5 - 0.5j])
TUnc = np.array([0.5 + 0.25j])

rCritNom, rCritUnc, rCrit, pCont = FreqTrans.DistCritEllipse(T, TUnc, pCrit = pCrit, magUnit = 'mag')

#rCritNomCirc, rCritUncCirc, rCritCirc = FreqTrans.DistCritCirc(T, TUnc, pCrit = pCrit, magUnit = 'mag', typeNorm = 'RMS')
rCirc = np.sqrt(0.5) * np.abs(TUnc) # RMS
#rCirc = np.max([TUnc.real, TUnc.imag]) # Max
#rCirc = np.mean([TUnc.real, TUnc.imag]) # Mean
#rCirc = np.abs(TUnc) # RSS

TUncCirc = np.array([rCirc+1j*rCirc])
rCritNomCirc, rCritUncCirc, rCritCirc, pContCirc = FreqTrans.DistCritEllipse(T, TUncCirc, pCrit = pCrit, magUnit = 'mag')



#%
fig, ax = plt.subplots(nrows=1, ncols=1)

ax.plot(T.real, T.imag, 'b*-')
ax.plot([pCrit.real, T.real], [pCrit.imag, T.imag], 'r*:')

ellipse = matplotlib.patches.Ellipse(xy = [T.real, T.imag], width=2*TUnc.real, height=2*TUnc.imag, color='b', alpha = 0.5)
ax.add_patch(ellipse)
ax.plot([pCrit.real, pCont.real], [pCrit.imag, pCont.imag], 'b*--')


circ = matplotlib.patches.Ellipse(xy = [T.real, T.imag], width=2*TUncCirc.real, height=2*TUncCirc.imag, color='g', alpha = 0.5)
ax.add_patch(circ)
ax.plot([pCrit.real, pContCirc.real], [pCrit.imag, pContCirc.imag], 'g*--')


ax.axis('equal')
fig.suptitle(['Nom: ' + str(rCritNom[0]) + ' Ellipse: ' + str(rCrit[0]) + ', Circle: ' + str(rCritCirc[0])])


