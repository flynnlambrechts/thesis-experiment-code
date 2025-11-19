from pprint import pprint

import numpy as np
import quad_global_variables as qgv
from Formation2D import Formation2D, get_formation
from FormationController import FormationController


def stack_cols(array):
    return array.T.reshape(-1, 1)


def sig(x, alpha):
    return np.sign(x) * (np.abs(x) ** alpha)


def disturbances(n, t):
    """return a np array that is guaranteed to have a norm less than 1"""
    
    a = 0.99 / (np.sqrt(n))
    possible_disturbances = np.array([
        a, a * np.sin(t + np.pi / 10),
        a * np.cos(t + 2 * np.pi / 10), a,
        a * np.sin(t + 3 * np.pi / 10), a * np.sin(t + 3 * np.pi / 10) ,
        a, a * np.sin(t + 4 * np.pi / 10),
        a * np.cos(t + 5 * np.pi / 10), a,
        a * np.sin(t + 6 * np.pi / 10), a * np.sin(t + 6 * np.pi / 10) ,
        a, a * np.sin(t + 7 * np.pi / 10),
        a * np.cos(t + 8 * np.pi / 10), a,
        a * np.sin(t + 9 * np.pi / 10), a * np.sin(t + 9 * np.pi / 10) ,
    ])
    disturbances = possible_disturbances[:n]
    assert np.norm(disturbances) < 1, "Hardcoded Disturbances too large, exceed bound"
    
    # disturbance_functions = [
    #     lambda _i, _t: a,
    #     lambda i, t: a * np.sin(t + i * np.pi / 10),
    #     lambda i, t: a * np.cos(t + i * np.pi / 10)
    # ]

    # n_disturbance_functions = len(disturbance_functions)

    # disturbances = []
    # for i in range(n):
    #     x_i = i % n_disturbance_functions
    #     y_i = (x_i + 1) % n_disturbance_functions
    #     disturbances.append(disturbance_functions[x_i](i, t))
    #     disturbances.append(disturbance_functions[y_i](i, t))
    
    return disturbances
    

class FixedTimeController(FormationController):
    PSI = 0.01
    ALPHA = 0.5
    BETA = 1.5
    K1 = 0.10
    K2 = 0.10

    def __init__(self, formation: Formation2D):
        if qgv.swarm != None:
            super().__init__(qgv.swarm)

        self.formation = formation

        I = np.eye(formation.d, dtype="int")

        print("H=", formation.H)

        self.H_bar = np.kron(formation.H, I)

        print("H_bar=", self.H_bar)
        self.p_des = stack_cols(formation.p_des)

        e_des = self.H_bar @ self.p_des
        e_des = e_des.reshape((-1, 2)).T

        self.f_des = 0.5 * np.sum(e_des**2, axis=0).T
        self.d_des = np.linalg.norm(e_des, axis=0).T


    def control_law(self, p, t):
        p = stack_cols(p).round(8)

        e = self.H_bar @ p

        f = 0.5 * np.sum(e.reshape((-1, 2)).T ** 2, axis=0)

        M = self.formation.M
        R = np.zeros((M, 2 * M))
        for j in range(M):  # j from 0 to M-1
            R[j, 2 * j : 2 * j + 2] = e[2 * j : 2 * j + 2].T

        Rd = R @ self.H_bar

        delta = (f - self.f_des).T

        Rd_delta_prod = Rd.T @ delta

        u = (
            - self.K1 * sig(Rd_delta_prod, self.ALPHA)
            - self.K2 * sig(Rd_delta_prod, self.BETA)
            - self.PSI * np.sign(Rd_delta_prod)
        )
        d = self.PSI * disturbances(self.formation.N, t)
        p_dot = u + d
        # pprint(p_dot)
        p_dot = p_dot.reshape(-1, 2).T
        # pprint(p_dot)
        return p_dot


