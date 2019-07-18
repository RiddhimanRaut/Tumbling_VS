import numpy as np
from math import *
def t_dist(mat):
    SMM_matrix=np.zeros((mat.shape[0],mat.shape[1]))
    for i in range(len(mat)):
        for j in range(len(mat[i])):
            #x is 2d
            d = 2
            mu = np.array([i,j])
            cov = np.ones((2,2))
            intensity = mat[i][j]
            
            if intensity!=0:
                cov[0,0] = cov[1,1] = intensity
                nu = 2*intensity/(intensity-1)
                
                # print "mu",mu
                sum = 0
                for k in range(len(mat)):
                    for l in range(len(mat[k])):
                        
                        x = np.array([k,l])
                        x_mu_mat = np.matrix(x-mu)
                        # print x_mu_mat
                        delta = x_mu_mat*cov*np.ndarray.transpose(x_mu_mat)
                        cov_det = np.linalg.det(cov)
                        delta = np.asscalar(delta)
                        t_denom = ((pi*nu)**(d/2))*gamma(nu/2)*(1+delta/nu)**((nu+d)/2)
                        t_num = gamma((nu+d)/2)/sqrt(cov_det)
                        sum += t_num/t_denom
                SMM_matrix[i][j] = sum
    return SMM_matrix

            
#main
mat1=np.array([[100,150,180],[70,200,110],[100,150,140]])
t_dist(mat1)
