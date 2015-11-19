import numpy as np

# def computeNullspaceMap(Mat, condition_threash, prevN=None):
#     sings,V=np.linalg.svd(Mat)[1:]
#     ndim = 0
#     while ndim < sings.size and sings[ndim] != 0. and sings[0]/sings[ndim] <= condition_threash: 
#         ndim += 1
#     return V[ndim:,:].T

def computeNullspaceMap(Mat, condition_threash):
    U,sings,V=np.linalg.svd(Mat)
    ndim = 0
    while ndim < sings.size and sings[ndim] != 0. and sings[0]/sings[ndim] <= condition_threash: 
        ndim += 1
    V_inv = np.matrix(V[:ndim,:]).T
    Sig_inv = np.diag(sings[:ndim]**-1)
    U_inv = np.matrix(U[:,:ndim]).T
    Pinv = V_inv * Sig_inv * U_inv
    Null = np.identity(Mat.shape[1]) - Pinv*Mat
    return Null

def orderNullspaceMap(Map, prevMap):
    if Map.shape[0] != prevMap.shape[0] and Map.shape[1] != prevMap.shape[1]:
        return
    for i in range(Map.shape[1]):
        if Map[:,i].dot(prevMap[:,i]) < 0.0:
            Map[:,i] *= -1.0
    