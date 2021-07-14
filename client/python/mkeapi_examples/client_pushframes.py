#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""This example code processes streamed 3D frames from sensor
"""
__author__ = "Ondra Fisar"
__copyright__ = "Copyright (c) 2017-2020, Magik-Eye Inc."

# ------------------------------------------------------------------------------

# imports

import sys
import time

#matplotlib.use("Qt5Agg")    # you can use diferrent backend for rendering
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

    if state != pymkeapi.MKE_STATE_DEPTH_SENSOR:

        print("Invalid state, let's change it to DEPTH_SENSOR")
        
        # firstly set state to IDLE
        
        if state != pymkeapi.MKE_STATE_IDLE:
            client.set_state(pymkeapi.MKE_STATE_IDLE)
          
        # then change state to DEPTH_SENSOR
        
        client.set_state(pymkeapi.MKE_STATE_DEPTH_SENSOR)

    else:
    
        print("Already in correct state.")

    # main body ------------------------------------------------------------------

    # prepare figure
  
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.ioff()
    fig.show()

    # prepare variables

    start_time = time.time()          # start of capturing
    timeout = 10                      # when should be stop_frame_push called
    num = 0                           # number of received images
    frame_type = 1                    # used frame type
  
    # start pushing frames
  
    start_seq_id = client.start_frame_push(frame_type)
    stop_seq_id = None

    while True:
    
        # send stop push after num frames
        
        if stop_seq_id is None and (time.time() - start_time) >= timeout:  
            stop_seq_id = client.stop_frame_push()
    
        # get next frame from the bus
        
        frame = client.get_pushed_frame(start_seq_id, stop_seq_id)
        
        # None denotes no more data
        
        if frame is None:
            print("Correctly finished loop")
          
            # all replies should be processed
            break
        
        # convert data and plot them
        
        ax.cla()
        ax.plot(frame.pts3d[:, 0], frame.pts3d[:, 1], frame.pts3d[:, 2], 'b.')
        plt.pause(0.001)
        
        num += 1
  
  
    print("Average displayed FPS: %.2f" % (num/(time.time()-start_time)))
    print("Correct termination")

except Exception as e:
    print("An error occured: ", str(e))
finally:
    if client:
        client.set_state(pymkeapi.MKE_STATE_IDLE)

# ------------------------------------------------------------------------------
