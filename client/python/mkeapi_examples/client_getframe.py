#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""This example code grabs one frame from sensor
"""
__author__ = "Ondra Fisar"
__copyright__ = "Copyright (c) 2017-2020, Magik-Eye Inc."

# ------------------------------------------------------------------------------

import sys

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import pymkeapi

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

    # get one frame
    frame = client.get_frame(pymkeapi.MKE_FRAME_TYPE_1)

    # show the frame
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(frame.pts3d[:,0], frame.pts3d[:,1], frame.pts3d[:,2], 'b.')
    plt.title("This is an example of getting a frame from the MagikEye sensor.")
    plt.show()

    print("Correct termination");

except Exception as e:
    print("An error occurred: %s" % str(e))
finally:
    if client:
        client.set_state(pymkeapi.MKE_STATE_IDLE)

# ------------------------------------------------------------------------------
