import numpy as np
from numpy import pi

f = 50e3 #Hz
C1 = 2e-8 /pi   #pF = e-12
C2 = 1e-8 /pi
omega = 2*np.pi*f
Zc1 = 1/(1j*omega*C1)
Zc2 = 1/(1j*omega*C2)
R = 1e3
U_up = R/(Zc1+R)*3
U_down = R/(Zc2+R)*3
U_out = abs(U_up - U_down)
print(U_out)
# effective voltage
U_rms = U_out/np.sqrt(2)
print(U_rms)