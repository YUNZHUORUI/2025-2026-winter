#!/usr/bin/env python
# coding: utf-8

# In[1]:


from pandas import read_excel
from matplotlib.pyplot import (figure, plot, grid, xlabel, ylabel, show,
  legend, title)
file1 = 'expdata.xlsx'

data1 = read_excel(file1, sheet_name='Sheet1')
print(data1)


# In[2]:


t1  = data1['t1'].values
yg1 = data1['yg1'].values


# In[3]:


# %matplotlib widget
# %matplotlib qt5
# %matplotlib notebook

figure(1)
plot(t1,yg1)
xlabel('time (s)')
ylabel('y acceleration (g)')
grid(True)
title('Free vibration 1DOF system')
show()

