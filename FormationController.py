# --------------- Python Libraries --------------------------------------------
import numpy as np
import numpy.typing as npt
from abc import ABC, abstractmethod
import time
from pprint import pprint

# --------------- Crazyflie Library -----------------------------------------
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig

# --------------- Customary Modules -------------------------------------------
import quad_global_variables as qgv
from Formation2D import Formation2D

import threading

import csv
from datetime import datetime

class FormationController(ABC):
    """
    An Abstract Base Class for a single integrator formation controller
    """

    def __init__(self, swarm: Swarm | None):
        if swarm == None:
            raise RuntimeError(
                "Please initialize the swarm before initializing the controller"
            )

        self.agent_positions_2D = np.zeros((2, len(qgv.AGENTS)))
        self.u = np.zeros((2, len(qgv.AGENTS)))
        self.u_lock = threading.Lock()
        self.start = None
        self.p_memo = []

    @abstractmethod
    def control_law(self, p: npt.NDArray, t) -> npt.NDArray:
        pass

    def update(self, agent_id, position_2D):
        """Should update self.u"""
        if not self.start:
            self.start = datetime.now()
        t = (datetime.now() - self.start).total_seconds()
        # print(agent_id, position_2D)
        agent_index = qgv.AGENTS.index(agent_id)

        self.agent_positions_2D[:, agent_index] = np.transpose(np.array(position_2D))
        self.p_memo.append([t] + self.agent_positions_2D.T.flatten().tolist())
        
        u = self.control_law(self.agent_positions_2D, t)
        
        self._set_u(u)
    
    def save_data(self):
        filename = datetime.now().strftime("./results-data/%Y-%m-%d_%H-%M_positions.csv")
        print(f"Saving to {filename}...")
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerows(self.p_memo)

    def _set_u(self, u):
        with self.u_lock:
            self.u = u

    def get_u(self):
        with self.u_lock:
            u = self.u
        return u
