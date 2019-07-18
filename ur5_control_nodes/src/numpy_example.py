import numpy as np
import random
import matplotlib.pyplot as plt
import datetime
from itertools import *
rows = cols = 3
mat_index = np.array(list(product(range(rows), range(cols)))).reshape(rows,cols,1,2)
new_mat = mat_index.reshape(rows,cols,1,1,1,2)
print new_mat
# print new_mat[1][0]
# l = np.tile(mat_index,(rows,cols,1,1,1,1))
# print l.shape




# x_minus_mu = np.subtract(l,new_mat)
# # print x_minus_mu.shape
# transpose = x_minus_mu.reshape(rows,rows,rows,rows,2,1)

# # print x_minus_mu
# prod = np.matmul(x_minus_mu,transpose)
# x = np.arange(9).reshape(3,3,1,1,1,1)
# # print prod
# power = np.power(prod,x)
# # print power

# # deter = np.linalg.det(x_minus_mu)

