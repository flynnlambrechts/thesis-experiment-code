import numpy as np
from pprint import pprint
from cflib.crazyflie.swarm import Swarm
from Formation2D import Formation2D, get_formation
import quad_global_variables as qgv
from FormationController import FormationController


class DistrubedController(FormationController):
    def __init__(self, swarm: Swarm | None):
        super().__init__(swarm)

    def control_law(self, p):

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
            - self.GAMMA * np.sign(Rd_delta_prod)
        )

        p_dot = u
        # pprint(p_dot)
        p_dot = p_dot.reshape(-1, 2).T
        # pprint(p_dot)
        return p_dot


