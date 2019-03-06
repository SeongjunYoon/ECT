import numpy as np
from scipy.linalg import norm, inv, pinv
from scipy.sparse import identity


class InverseSolver():
    def __int__(self):
        pass

    def LBP(self, sensitivity_map, C_measured):
        return np.matmul(sensitivity_map.T, C_measured)

    def SVD(self, sensitivity_map, C_measured):
        S_pseudo_inv = pinv(sensitivity_map, rcond=1e-21)
        return np.matmul(S_pseudo_inv, C_measured)

    def Tikhonov(self, sensitivity_map, C_measured, u):
        (m,n) = sensitivity_map.shape
        I = identity(n, dtype='float64')
        S_t = sensitivity_map.T
        inv_target = np.matmul(S_t, sensitivity_map) + u*I
        inv_mat = inv(inv_target, overwrite_a=True)
        St_dot_C = np.matmul(S_t, C_measured)
        return np.matmul(inv_mat, St_dot_C)

    def Landweber(self, sensitivity_map, C_measured, iter_num):
        # Initial guess: LBP
        S_t = sensitivity_map.T
        g = np.matmul(S_t, C_measured)

        # Calculate initial error
        ek = np.matmul(sensitivity_map, g) - C_measured
        error_init = 0.5 * (norm(ek) ** 2)

        # Iterate
        for k in range(iter_num):
            ek = np.matmul(sensitivity_map, g) - C_measured
            gk = np.matmul(S_t, ek)
            ak = (norm(gk) ** 2) / (norm(np.matmul((np.matmul(sensitivity_map, S_t)), ek)) ** 2)
            g += -1 * ak * gk

        # Calculate final error
        error_final = 0.5 * (norm(ek) ** 2)
        print("Initial error = {}".format(error_init))
        print("Final error = {} after {} iterations".format(error_final, iter_num))
        return g





