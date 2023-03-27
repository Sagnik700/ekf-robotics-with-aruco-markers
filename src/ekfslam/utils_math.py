import numpy as np

# Untility math functions used in slam algorithm

def assemble_matrix(A, B, C, D):
    Z0 = np.append(A, B, axis = 1)
    Z1 = np.append(C, D, axis = 1)
    Z = np.append(Z0, Z1, axis = 0)
    
    return Z
