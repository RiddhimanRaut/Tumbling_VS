import numpy as np 
# from math import *
from scipy.special import *
from scipy.misc import *
x=np.arange(4,8).reshape(2,2)
# k = np.arange(2,6).reshape(2,2,1,1)

# ident = np.identity(2)
# l = np.tile(ident,(2,2,1,1))
# # print l.shape
# mult = np.multiply(k,l)
# mult[mult==0] = 1
# print mult
# deter = np.linalg.det(mult)
# # print 1/np.sqrt(deter)
# print mult.sum(axis=(2,3))
print gamma(x)