"""
This file contains all utility functions that are related to
 - establishing and shutting down connections
 - streaming positions, velocities, etc
 - initializing various components
This file does not contain any function that directly commands
the motion of the quadcopter. For functions that pilot the
drone, go to quad_pilot.py file.
"""

# //////////////////////////////////////////////////////////////////////////////
# //                           Required Modules                               //
# //////////////////////////////////////////////////////////////////////////////
# Do not modify the following modules!
# Do not remove any unused module!

# --------------- Python Libraries --------------------------------------------
import logging
import time
import datetime
import sys
from threading import Event
import csv
import os
import numpy as np
import matplotlib.pyplot as plt
import math
import keyboard

# --------------- Crazyflie Libraries -----------------------------------------
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie import commander
from cflib.crazyflie.swarm import Swarm

# --------------- Customary Modules -------------------------------------------
import quad_global_variables as qgv

# --------------- OptiTrack Modules -------------------------------------------
from optirack.NatNetClient import NatNetClient


# //////////////////////////////////////////////////////////////////////////////
# //                            Magic Numbers                                 //
# //////////////////////////////////////////////////////////////////////////////

TIME_BUFFER = 0.1  # in seconds
# //////////////////////////////////////////////////////////////////////////////
# //                      Set up position streaming                           //
# //////////////////////////////////////////////////////////////////////////////
# The following functions are originally written by Juri Hemmi and adapted to
# the swarm implementation by Flynn Lambrechts


# This function sends the pose to a given crazyflie
# The rotation is in quaternions
def send_pose_to_cf(cf, pos, rot=None):
    # print(".", end='')
    if rot != None:
        cf.extpos.send_extpose(pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], rot[3])
    else:
        cf.extpos.send_extpos(pos[0], pos[1], pos[2])


# This function gets passed to the NatNetClient and is called every
# time that a rigid bodies position is received.
# The pose is then sent to the crazyflie.
def receive_rigid_body_frame(new_id, position, rotation):
    # print(position)
    if qgv.swarm != None and qgv.swarm._is_open:
        radio = qgv.URIs_by_agent[new_id]
        if not radio in qgv.swarm._cfs:
            print(
                f"Warning: Got frame for unexpected crazyflie, streaming id: {new_id}"
            )
            return
        cf = qgv.swarm._cfs[radio].cf

        if qgv.USE_FULL_POSE:
            send_pose_to_cf(cf, position, rotation)
        else:
            send_pose_to_cf(cf, position)

        if qgv.formation_controller != None:
            # print(position)
            qgv.formation_controller.update(new_id, position[0:2])


def receive_new_frame(data_dict):
    pass


# This function sets up the position streaming
def setup_pos_stream():
    qgv.pos_stream = NatNetClient()
    # These functions will be called in the background whenever the relevant
    # frame is recieved from motive.
    qgv.pos_stream.new_frame_listener = receive_new_frame
    qgv.pos_stream.rigid_body_listener = receive_rigid_body_frame

    if not qgv.pos_stream.run():
        print("Could not get Rigid Body positions")
        exit(1)


def set_params(scf):
    scf.cf.param.set_value("stabilizer.estimator", "2")
    scf.cf.param.set_value("locSrv.extQuatStdDev", 0.6)
    scf.cf.param.set_value("commander.enHighLevel", "1")


def arm(scf):
    scf.cf.platform.send_arming_request(True)
    time.sleep(1.0)


def activate_led_bit_mask(scf):
    scf.cf.param.set_value("led.bitmask", 255)


def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value("led.bitmask", 0)


def attach_link_statistic_printer(scf):
    scf.cf.link_statistics.latency.latency_updated.add_callback(
        lambda latency: print(f"Latency {latency:.3f} ms")
    )
    scf.cf.link_statistics.link_quality_updated.add_callback(
        lambda link_quality: print(f"Link Quality {link_quality}")
    )


def position_callback(timestamp, data, logconf):
    x = data["kalman.stateX"]
    y = data["kalman.stateY"]
    z = data["kalman.stateZ"]
    print("pos: ({}, {}, {})".format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name="Position", period_in_ms=500)
    log_conf.add_variable("kalman.stateX", "float")
    log_conf.add_variable("kalman.stateY", "float")
    log_conf.add_variable("kalman.stateZ", "float")

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


def light_check(scf):
    for _ in range(2):
        activate_led_bit_mask(scf)
        time.sleep(0.5)
        deactivate_led_bit_mask(scf)
        time.sleep(0.5)


def swarm_initializer():
    cflib.crtp.init_drivers()

    print("=========== Initializing Swarm ===========")

    step = 0
    total_steps = 6

    step = print_ProgressBar(
        step, total_steps, prefix="Swarm ", suffix="[Initializing...]"
    )

    qgv.swarm = Swarm(qgv.uris)
    qgv.swarm.close_links()

    step = print_ProgressBar(
        step, total_steps, prefix="Swarm ", suffix="[Connecting Radios...]"
    )
    qgv.swarm.open_links()

    step = print_ProgressBar(
        step, total_steps, prefix="Swarm ", suffix="[Checking Params...]"
    )
    qgv.swarm.parallel_safe(lambda scf: scf.wait_for_params())
    qgv.swarm.sequential(set_params)

    step = print_ProgressBar(step, total_steps, prefix="Swarm ", suffix="[Arming...]")
    qgv.swarm.parallel_safe(arm)

    step = print_ProgressBar(
        step,
        total_steps,
        prefix="Swarm ",
        suffix="[Reset Estimators... - if this takes too long check Motive...]",
    )
    qgv.swarm.reset_estimators()

    step = print_ProgressBar(
        step,
        total_steps,
        prefix="Swarm ",
        suffix="[Light Check (check for latency)...]",
    )
    qgv.swarm.parallel_safe(light_check)
    print("============ Initialization Completed! ============", end="\n")


# //////////////////////////////////////////////////////////////////////////////
# //                       Quadcopter Shutting Down                           //
# //////////////////////////////////////////////////////////////////////////////


# This function closes the link between the Crazyradio PA and the crazyflie
def quad_shutDown():
    print("Closing link and shutting position stream.")
    if qgv.swarm is not None:
        qgv.swarm.close_links()

    qgv.pos_stream.shutdown()


# //////////////////////////////////////////////////////////////////////////////
# //                       Other Utility Functions                            //
# //////////////////////////////////////////////////////////////////////////////


# This function prints and updates an iteration progress bar in the terminal
# Taken from stackoverflow.com/questions/3173320/...
# .../text-progress-bar-in-terminal-with-block-characters/13685020
def print_ProgressBar(
    iteration,
    total,
    prefix="",
    suffix="",
    decimals=1,
    length=100,
    fill="\u2588",
    printEnd="\r",
):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration       - Required  :   current iteration (Int)
        total           - Required  :   total iteration (Int)
        prefix          - Optional  :   prefix string (Str)
        suffix          - Optional  :   suffix string (Str)
        decimals        - Optional  :   positive number of decimals in percent complete (Int)
        length          - Optional  :   bar fill character (Str)
        printEnd        - Optional  :   end character (e.g. "\r", "\r\n)"(Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + "-" * (length - filledLength)
    print(f"\r{prefix} |{bar}| {percent}% {suffix}", end=printEnd)
    # Print New Line on Complete
    if iteration == total:
        print()
    return iteration + 1
