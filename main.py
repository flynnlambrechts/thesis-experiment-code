"""
This is the main file that can be used to control Crazyflie drones in
combination with the OptiTrack-Motive motion capture system.

The main function also sets up the NatNetClient to receive the streamed
rigid body data and pass the positions to each Crazyflie.

The Motive software must be kept openned and streaming the Crazyflies'
(rigid bodies) positions while running this python file.

System variables are mostly stored in quad_global_variables.py

The control law should be implemented in a separate file, for example,
'quad_controller_someFancyControlLaw.py', and then called in this file.

The provided sample controller, 'quad_controller_PID.py', controls a
single crazyflie quadcopter to hover at a prescribed height while not
drifting on a horizontal plane. However, the provided Python files
can be easily modified to accommdate multiple Crazyflies and serve as
a experimental platform for multi-agent-system-related projects.

Authors:    originally written by Juri Hemmi,
            and heavily adopted by Donglin Sui
"""

# --------------- Required Modules --------------------------------------------
# Do not modify the following modules!
# Do not remove any unused module!
import csv
import datetime
import logging
import math
import os
import sys
import time
from threading import Event

import keyboard
import matplotlib.pyplot as plt
import numpy as np
import quad_global_variables as qgv
import quad_pilot as qp
import quad_utilities as qu

# ----- Swarm -------
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from optirack.NatNetClient import NatNetClient

# ----- Logging -----
logging.basicConfig(level=logging.ERROR)

# ----- Control -----
from FixedTimeController import FixedTimeController
from Formation2D import get_formation


def run_swarm():

    try:

        print("=========== Setup OptiTrack ===========", end="\n")
        qu.setup_pos_stream()  # Obtain position reading from OptiTrack

        qu.swarm_initializer()
        assert qgv.swarm != None

        agent_id_by_uri = {uri: [qgv.AGENTS[i]] for i, uri in enumerate(qgv.uris)}
        print(agent_id_by_uri)

        formation = get_formation(len(qgv.AGENTS))

        if formation == None:
            raise RuntimeError("Could not get Formation")

        qgv.formation_controller = FixedTimeController(formation)
        time.sleep(1)
        # qgv.swarm.parallel_safe(qu.light_check)
        # qgv.swarm.parallel_safe(qp.hover_sequence)
        qgv.swarm.parallel_safe(
            lambda scf, agent_index: qp.formation_control_sequence(scf, agent_index),
            args_dict=agent_id_by_uri,
        )
        
        user_input = ""
        while (user_input != "y" and user_input != "n"):
            user_input = input("Save data? (y/n) ")
        
        if (user_input == "y"):
            qgv.formation_controller.save_data()
            
        qu.quad_shutDown()
    except Exception as e:
        # qgv.formation_controller.stop_controller()
        qu.quad_shutDown()
        raise e


def main():
    run_swarm()


if __name__ == "__main__":
    main()
