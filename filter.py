import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as sig
from numpy.random import randn
from numpy.fft import fft, fftfreq, fftshift


plt.interactive(True)

f0 = 200e3

b0 = np.genfromtxt("./1stStage.txt")
b0p = np.genfromtxt("./1stStage2.txt")
b1 = np.genfromtxt("./2ndStage.txt")

ns = np.arange(256)
xs = randn(ns.size)
xs[16:] += 10.
ys = sig.lfilter(b0, [1.], xs)
ysp = sig.lfilter(b0p, [1.], xs)
zs = sig.correlate(ys, ys, mode='same')

w, gd = sig.group_delay((b0, [1.]))

w0, h0 = sig.freqz(b0, np.ones(b0.size))
w1, h1 = sig.freqz(b0)

plt.clf(); plt.loglog(0.5*f0*w0/np.pi, np.abs(h0), 0.5*f1*w1/np.pi, np.abs(h1))

ns = np.arange(2*b0.size)
xi = np.zeros(ns.size)
xi[0] = 1.
yi = sig.lfilter(b0, [1.], xi)
yip = sig.lfilter(b0p, [1.], xi)
ys = yi.cumsum()
ysp = yip.cumsum()


f0 = 200e3
f1 = 10e3
f2 = 400.
fnyq = f0/2.

b0, a0 = sig.butter(4, f1/fnyq)
flt = sig.dlti(b0, a0, dt=1./f0)
w, H = flt.freqresp()
t, y = flt.step(n=4096)

b1, a1 = sig.butter(2, f2/fnyq)

ns = np.arange(1024)
xs = randn(ns.size)
xs[16:] += 10.
y0s = sig.lfilter(b0, a0, xs)
y1s = sig.lfilter(b1, a1, y0s)
