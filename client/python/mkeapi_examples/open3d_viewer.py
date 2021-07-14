#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""This example code processes streamed 3D frames from a MagikEye sensor
   and displays it using the Open3D library
"""
__author__ = "Akhilraaj M"
__copyright__ = "Copyright (c) 2017-2020, Magik-Eye Inc."

# ------------------------------------------------------------------------------

import sys
import time

import pymkeapi
import pymkeapi.open3d

import open3d

from functools import partial

# ------------------------------------------------------------------------------
# Global variable to signal the mke client to stop pushing frames
# and exit from depth sensing mode
StopClientSignal = False

# define call back to exit visualisation

def exit_visu(vis):
    global StopClientSignal
    StopClientSignal = True
    print("Exiting visualization after user interrupt")
    return False


# check arguments
if len(sys.argv) < 2 or len(sys.argv) > 3:
    print("Bad number arguments.\n Usage %s IP [port=8888]" % sys.argv[0])
    exit(1)

host = sys.argv[1]                                      # hostname / IP of device
port = 8888 if len(sys.argv) < 3 else int(sys.argv[2])  # port - by default 8888

# ------------------------------------------------------------------------------

client = None
frame = None

try:
    bus = pymkeapi.TcpBus(host, port)
    client = pymkeapi.SyncClient(bus)

    # go to state depth sensing
    state = client.get_state()
    if state != pymkeapi.MKE_STATE_DEPTH_SENSOR:
        # firstly set state to IDLE
        if state != pymkeapi.MKE_STATE_IDLE:
            client.set_state(pymkeapi.MKE_STATE_IDLE)

        # then change state to DEPTH_SENSOR
        client.set_state(pymkeapi.MKE_STATE_DEPTH_SENSOR)

    # main body ------------------------------------------------------------------

    # prepare variables
    start_time = time.time()          # start of capturing
    timeout = 100                      # when should be stop_frame_push called
    num = 0                           # number of received images
    frame_type = 1                    # used frame type

    # start pushing frames
    start_seq_id = client.start_frame_push(frame_type)
    stop_seq_id = None

    # prepare Open3D Window
    vis = open3d.visualization.VisualizerWithKeyCallback()  # Open3D Visu Window
    vis.create_window()
    fptr = partial(exit_visu)
    vis.register_key_callback(ord('Q'), partial(exit_visu))

    # Get frame from MKE Client
    frame = client.get_pushed_frame(start_seq_id, stop_seq_id)

    # Add Open3D geometry
    pc = open3d.geometry.PointCloud()
    pymkeapi.open3d.update_point_cloud(pc, frame)
    vis.add_geometry(pc)  # Add the PCD to visu
    StopClientSignal = False

    while not StopClientSignal:
        print("Frame: ", num)

        # get next frame from the bus
        frame = client.get_pushed_frame(start_seq_id, stop_seq_id)

        if frame is None:
            print("Correctly finished loop")
            vis.destroy_window()
            # all replies should be processed
            break

        pymkeapi.open3d.update_point_cloud(pc, frame)
        vis.update_geometry(pc)
        vis.poll_events()
        vis.update_renderer()

        num += 1  # Counter for FPS estimation

    print("Average displayed FPS: %.2f" % (num/(time.time()-start_time)))
    print("Correct termination")

except Exception as e:
    print("An error occured: ", str(e))
finally:
    if client:
        print("Cleaning up buffer")
        stop_seq_id = client.stop_frame_push()

        while frame is not None:
            # Get frames until the buffer is empty
            frame = client.get_pushed_frame(start_seq_id, stop_seq_id)

            client.set_state(pymkeapi.MKE_STATE_IDLE)

# ------------------------------------------------------------------------------

