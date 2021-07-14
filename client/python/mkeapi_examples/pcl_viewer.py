#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""This example code processes streamed 3D frames from sensor in PCL format
"""
__author__ = "Akhilraaj M"
__copyright__ = "Copyright (c) 2017-2020, Magik-Eye Inc."

# ------------------------------------------------------------------------------
# imports

import sys
import time

import pcl.pcl_visualization
from functools import partial

import pymkeapi
import pymkeapi.pcl

# ------------------------------------------------------------------------------

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
    pc = pcl.PointCloud()
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

    # Prepare variables
    start_time = time.time()          # start of capturing
    timeout = 120                     # when should be stop_frame_push called
    num = 0                           # number of received images
    frame_type = 1                    # used frame type

    # Start pushing frames
    start_seq_id = client.start_frame_push(frame_type)
    stop_seq_id = None

    viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')
    v = not (viewer.WasStopped())

    # Get frame from MKE Client
    frame = client.get_pushed_frame(start_seq_id, stop_seq_id)
    pymkeapi.pcl.update_point_cloud(pc, frame)

    # Add the point cloud to the visualizer
    viewer.AddPointCloud(pc, b'scene_cloud', 0)
    viewer.SpinOnce()
    viewer.RemovePointCloud(b'scene_cloud', 0)

    while True:
        print("Frame: ", num)

        # send stop push after num frames
        if (stop_seq_id is None and (time.time() - start_time) >= timeout):
            break

        # get next frame from the bus
        frame = client.get_pushed_frame(start_seq_id, stop_seq_id)

        if frame is None:
            break

        pymkeapi.pcl.update_point_cloud(pc, frame)
        viewer.AddPointCloud(pc, b'scene_cloud', 0)
        viewer.SpinOnce()
        viewer.RemovePointCloud(b'scene_cloud', 0)

        num += 1  # Counter for FPS estimation

    print("Average displayed FPS: %.2f" % (num/(time.time()-start_time)))
    print("Correct termination")

except Exception as e:
    print("An error occured: ", str(e))
finally:
    viewer.Close()

    if client:
        print("Cleaning up buffer")
        stop_seq_id = client.stop_frame_push()

        while frame is not None:
            # Get frames until the buffer is empty
            frame = client.get_pushed_frame(start_seq_id, stop_seq_id)

            client.set_state(pymkeapi.MKE_STATE_IDLE)

# ------------------------------------------------------------------------------
