"""
This file contains user defined variables as well as variables that the system
uses at runtime.
"""

from math import sqrt

import numpy as np
from cflib.crazyflie.swarm import Swarm
from cflib.utils import uri_helper

pos_stream = []

# Enter Agent Ids in Ascending Order
AGENTS = [3, 4, 5, 6, 7, 8]  # store agent IDs

FLY_HEIGHT = 0.5  # default fly height for the drones
MAX_SPEED = 0.8
RUN_TIME = 15  # Doesn't include takeoff and landing time

USE_FULL_POSE = False

# TODO document best practices for choosing these
URIs_by_agent = {
    0: uri_helper.uri_from_env(default=("radio://1/100/2M/E7E7E7E7E0")),
    1: uri_helper.uri_from_env(default=("radio://1/100/2M/E7E7E7E7E1")),
    2: uri_helper.uri_from_env(default=("radio://1/100/2M/E7E7E7E7E2")),
    3: uri_helper.uri_from_env(default=("radio://1/100/2M/E7E7E7E7E3")),
    4: uri_helper.uri_from_env(default=("radio://1/100/2M/E7E7E7E7E4")),
    5: uri_helper.uri_from_env(default=("radio://1/100/2M/E7E7E7E7E5")),
    6: uri_helper.uri_from_env(default=("radio://0/80/2M/E7E7E7E7E6")),
    7: uri_helper.uri_from_env(default=("radio://0/80/2M/E7E7E7E7E7")),
    8: uri_helper.uri_from_env(default=("radio://0/80/2M/E7E7E7E7E8")),
    9: uri_helper.uri_from_env(default=("radio://0/80/2M/E7E7E7E7E9")),
}

uris = [URIs_by_agent[agent] for agent in AGENTS]

swarm: Swarm | None = None

formation_controller = None
CONTROL_PERIOD = 0.03
