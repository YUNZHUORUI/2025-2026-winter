#!/usr/bin/env python
# coding: utf-8

# In[1]:


from pandas import read_excel
from printarray import printarray


# In[2]:


KDF = read_excel('stiffness_matrix.xlsx')
MDF = read_excel('mass_matrix.xlsx')
CDF = read_excel('damping_matrix.xlsx')


# In[3]:


K = KDF.values
M = MDF.values
C = CDF.values


# In[4]:


print('Upper left sub-half-matrix of K')
for ii in range(4):
    for jj in range(4):print('{:.15f}  '.format(K[ii,jj]),end="")

    print('')

print('\nLower right sub-half-matrix of K')
for ii in range(4,8):
    for jj in range(4,8):print('{:.15f}  '.format(K[ii,jj]),end="")

    print('')

print('\nUpper right sub-half-matrix of K')
for ii in range(4):
    for jj in range(4,8):print('{:.15f}  '.format(K[ii,jj]),end="")

    print('')


# In[5]:


shouldbezero8times8 = abs(K - K.T)
print('shouldbezero8times8:')
for ii in range(8):
    printarray(shouldbezero8times8[ii,:],'.10f')


# In[6]:


print('\nUpper left sub-half-matrix of M')
for ii in range(4):
    for jj in range(4):print('{:.15f}  '.format(M[ii,jj]),end="")

    print('')

print('\nLower right sub-half-matrix of M')
for ii in range(4,8):
    for jj in range(4,8):print('{:.15f}  '.format(M[ii,jj]),end="")

    print('')


print('\nUpper right sub-half-matrix of M')
for ii in range(4):
    for jj in range(4,8):print('{:.15f}  '.format(M[ii,jj]),end="")

    print('')


# In[7]:


shouldbezero8times8 = abs(M - M.T)
print('shouldbezero8times8:')
for ii in range(8):
    printarray(shouldbezero8times8[ii,:],'.10f')


# In[8]:


print('\nUpper left sub-half-matrix of C')
for ii in range(4):
    for jj in range(4):print('{:.15f}  '.format(C[ii,jj]),end="")

    print('')

print('\nLower right sub-half-matrix of C')
for ii in range(4,8):
    for jj in range(4,8):print('{:.15f}  '.format(C[ii,jj]),end="")

    print('')


print('\nUpper right sub-half-matrix of C')
for ii in range(4):
    for jj in range(4,8):print('{:.15f}  '.format(C[ii,jj]),end="")

    print('')


# In[9]:


shouldbezero8times8 = abs(C - C.T)
print('shouldbezero8times8:')
for ii in range(8):
    printarray(shouldbezero8times8[ii,:],'.10f')

