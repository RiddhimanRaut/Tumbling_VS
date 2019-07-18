import numpy as np 
import random
from itertools import *
rows = cols = 100
eye = np.identity(2)
tile_eye = np.tile(eye,(rows,cols,1,1))
greyscale = np.random.random_integers(0,255,(rows,cols)).reshape(rows,cols)
greyscale[greyscale<2] = 2
#X_MINUS_MU
mat_index = np.array(list(product(range(rows), range(cols)))).reshape(rows,cols,1,2)
tiled_mat_index = np.tile(mat_index,(rows,cols,1,1,1,1))
mat_index = mat_index.reshape(rows,cols,1,1,1,2)
x_minus_mu = np.subtract(tiled_mat_index,mat_index)
transpose = x_minus_mu.reshape(rows,cols,rows,cols,2,1)

def sigma(frame):
    global rows,cols,tile_eye
    frame = frame.reshape(rows,cols,1,1)
    cov_matrix = np.multiply(frame,tile_eye)
    cov_matrix[cov_matrix==0] = 1
    return cov_matrix

def delta(frame):
    global x_minus_mu,transpose
    sig = sigma(frame)
    delta = np.matmul(x_minus_mu,sig)
    delta = np.matmul(delta,transpose)
    delta = delta.reshape(rows*rows,cols*cols)
    delta = np.ndarray.transpose(delta)
    print delta.reshape(rows,cols,rows,cols)

delta(greyscale)






