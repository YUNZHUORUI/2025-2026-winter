#!/usr/bin/env python
# coding: utf-8


def flattpn(n):
    # returns the N-point normalized Flattop window in an array
    from numpy import ones, cos, sin, pi, arange, hstack, array
    nm1 = n - 1
    nr1 = arange(1,n)
    w =  (0.21170*ones(nm1)       - 0.40565*cos(2.0*pi*nr1/n)
         + 0.27808*cos(4*pi*nr1/n) - 0.09435*cos(6*pi*nr1/n)
         + 0.01022*cos(8*pi*nr1/n))
    w = hstack((array([0.0]),w)) 
    return w/w.mean()



def plotspec(npoints,delt,chref,*argv):
    from numpy.fft import fft
    from numpy import arange, sqrt, angle, rad2deg
    from matplotlib.pyplot import (figure, plot, grid, xlabel, ylabel, show,
      legend, title)
    
    npo2   = int(npoints/2)

    timwin = npoints*delt
    delf = 1/timwin
    fs   = npoints*delf
    fnq  = fs/2
    
    f = arange(0,fnq,delf)

    win = flattpn(npoints)
    
    Chref  = fft(chref[:npoints]*win)/npoints
    Chrefm = (Chref*Chref.conj()).real
    Chrefm = 2*sqrt(Chrefm)

    if len(argv)>0:
        ch2  = argv[0].copy()
        Ch2  = fft(ch2[:npoints]*win)/npoints
        Ch2m = (Ch2*Ch2.conj()).real
        Ch2m = 2*sqrt(Ch2m)
        
        phase = rad2deg(angle(Ch2/Chref))
    
    figure()
    plot(f,Chrefm[0:npo2],label = 'Ref')
    if len(argv)>0: plot(f,Ch2m[0:npo2],label = 'Ch 2')
    legend(loc = 'upper right')
    title('Spectrum')
    xlabel('frequency (Hz)')
    ylabel('Absolute value')
    grid(True)
    show()

    if len(argv)>0:
        figure()
        plot(f,phase[0:npo2])
        title('Phase of Ch 2 relative to Ref')
        xlabel('frequency (Hz)')
        ylabel('Angle in degrees')
        grid(True)
        show()
    
    return

