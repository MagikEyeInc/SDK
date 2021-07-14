#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""This example code grabs one frame from a MagikEye sensor, 
   converts into a Open3D point cloud and displays the result
"""

__author__ = "Akhilraaj M"
__copyright__ = "Copyright (c) 2017-2020, Magik-Eye Inc."

# ------------------------------------------------------------------------------

import sys

import open3d

import pymkeapi
import pymkeapi.open3d

# ------------------------------------------------------------------------------

# check arguments

if len(sys.argv) < 2 or len(sys.argv) > 3:
    print("Bad number arguments.\n Usage %s IP [port=8888]" % sys.argv[0])
    exit(1)

host = sys.argv[1]                                      # hostname / IP of device
port = 8888 if len(sys.argv) < 3 else int(sys.argv[2])  # port - by default 8888

# ------------------------------------------------------------------------------

client = None

try:
    bus = pymkeapi.TcpBus(host, port)
    client = pymkeapi.SyncClient(bus)

    # go to state depth sensing

    state = client.get_state()
    print("Current state: %d" % state)

    if state != pymkeapi.MKE_STATE_DEPTH_SENSOR:
        print("Invalid state, let's change it")

        # firstly set state to IDLE
        if state != pymkeapi.MKE_STATE_IDLE:
            client.set_state(pymkeapi.MKE_STATE_IDLE)

        # then change state to DEPTH_SENSOR
        client.set_state(pymkeapi.MKE_STATE_DEPTH_SENSOR)

        # now the device is in DEPTH_SENSOR state
        print("Current state: %d" % client.get_state())
    else:
        print("Already in the correct state.")

    # Get one frame
    frame = client.get_frame(pymkeapi.MKE_FRAME_TYPE_1)

    # Convert frame to Open3D point cloud
    pc = open3d.geometry.PointCloud()
    pymkeapi.open3d.update_point_cloud(pc, frame)

    # Visualize the resulting point cloud using Open3D
    open3d.visualization.draw_geometries([pc])

    print("Correct termination")

except Exception as e:
    print("An error occurred: %s" % str(e))
finally:
    if client:
        client.set_state(pymkeapi.MKE_STATE_IDLE)

# ------------------------------------------------------------------------------
