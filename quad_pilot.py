"""
This file contains wrappers for the crazyflie library commands.

Author: Flynn Lambrechts
"""

import time

import numpy as np
import numpy.typing as npt
import quad_global_variables as qgv
from cflib.crazyflie.commander import Commander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from FormationController import FormationController


def _get_commander(scf: SyncCrazyflie) -> Commander:
    return scf.cf.commander


def take_off(scf):
    scf.cf.high_level_commander.takeoff(qgv.FLY_HEIGHT, 3)
    time.sleep(3)
    # position = qgv.FLY_HEIGHT
    # take_off_time = 3.0
    # sleep_time = 0.1
    # steps = int(take_off_time / sleep_time)
    # vz = position / take_off_time

    # print(vz)

    # for i in range(steps):
    #     scf.cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
    #     time.sleep(sleep_time)


def land(scf):
    cf = scf.cf
    position = qgv.FLY_HEIGHT
    landing_time = 2.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = - position / landing_time

    print(vz)

    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


def saturate_vec(v: npt.NDArray, max_magnitude) -> npt.NDArray:
    norm = np.linalg.norm(v)
    if norm > max_magnitude:
        print("Warning: Had to saturate vector")
        return v / norm * max_magnitude
    return v


def set_2D_velocity(scf, vx, vy):
    # prints(vx, vy)
    """This command will set the attitude controller setpoint for the next 500ms. After 500ms without next setpoint,
    the Crazyflie will apply a setpoint with the same thrust but with roll/pitch/yawrate = 0, this will make the Crazyflie stop accelerating.
    After 2 seconds without new setpoint the Crazyflie will cut power from the motors.
    """
    speed = np.sqrt(vx**2 + vy**2)
    if speed > qgv.MAX_SPEED:
        print("Warning: Speed exceeded max allowed - saturating")
        scale = float(qgv.MAX_SPEED) / speed
        vx *= scale
        vy *= scale

    _get_commander(scf).send_hover_setpoint(vx, vy, 0, qgv.FLY_HEIGHT)
    # _get_commander(scf).send_velocity_world_setpoint(vx, vy, 0, 0)


def hover(scf: SyncCrazyflie):
    set_2D_velocity(scf, 0, 0)


def hover_sequence(scf: SyncCrazyflie, duration=qgv.RUN_TIME):
    take_off(scf)

    start = time.time()
    while time.time() - start < duration:
        hover(scf)
        time.sleep(0.1)

    land(scf)


def formation_control_sequence(scf: SyncCrazyflie, agent_id):
    if qgv.formation_controller == None:
        raise RuntimeError("Formation Controller Was None")

    take_off(scf)
    hover(scf)

    print(f"Agent {agent_id} hovering and will start listening to controller")

    start = time.time()
    agent_index = qgv.AGENTS.index(agent_id)

    while time.time() - start < qgv.RUN_TIME:
        u = list(qgv.formation_controller.get_u()[:, agent_index])
        # print(agent_id, *u)
        set_2D_velocity(scf, *u)
        # set_2D_velocity(scf, 0, 0)
        time.sleep(qgv.CONTROL_PERIOD)

    print(f"Agent {agent_id} landing")

    hover(scf)
    land(scf)
