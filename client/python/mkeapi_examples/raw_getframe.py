#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""RAW example (without using pymkecli) how to get frame from sensor
"""
__author__ = "Ondra Fisar"
__copyright__ = "Copyright (c) 2017, Magik-Eye s.r.o., Prague"

# ------------------------------------------------------------------------------

# imports

import sys
import struct
import socket

# import matplotlib to show 3D data

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ------------------------------------------------------------------------------

# send method
#   cmd - command
#   seq_id - sequence id
#   params - command parameters


def mke_send(conn, cmd, seq_id, params = bytes(8)):
    
    assert(len(params) == 8)                        # params must have 8 bytes
    
    # assemble request

    req_header  = b'MKERQ100'                       # ASCII header of request
    cmd         = bytes('%04d' % cmd, 'ascii')      # ASCII REQUEST_GET_STATE
    seq_id      = struct.pack('<I', seq_id)         # binary sequence id
    request     = req_header + cmd + seq_id + params
                                                    # assembled whole request

    print("SENT:     %s %s %s %s" % (request[0:8], request[8:12], request[12:16], request[16:]))
                                            # nice print of whole request by parts

    # send datagram to device

    if conn.send(request) == 0:
        raise RuntimeError("Socket broken")

# ------------------------------------------------------------------------------

# receive method 
#   cmd - expected command
#   seq_id - expected sequence id
#   exp_status - expected status code
#
#   return tuple with:
#     0 - params
#     1 - payload if any

def mke_recv(conn, cmd, seq_id, exp_status):
    
    # wait for 48 bytes long reply

    reply = bytes()

    while len(reply) < 48:
        tmp = conn.recv(48)
        if len(tmp) == 0:
            raise RuntimeError("Socket closed");
        reply += tmp

    print("RECEIVED: %s %s %s %s %s %s" % (reply[0:8], reply[8:12], reply[12:16], reply[16:20], reply[20:24], reply[24:]))
                                            # nice print of whole reply by parts

    # check reply

    assert(reply[0:8] == b'MKERP100')       # reply should start with header MKERP100
    assert(int(reply[8:12]) == cmd)         # command should be same
    assert(reply[16:20] == struct.pack('<I', seq_id))
                                            # sequence id should be same

    payload_size = struct.unpack('<I', reply[20:24])[0];

    # if there is some payload - read it
    
    while len(reply) < (48 + payload_size):
        tmp = conn.recv(payload_size)
        if len(tmp) == 0:
            raise RuntimeError("Socket closed");
        reply += tmp
    
    if payload_size > 0:
        print("PAYLOAD[%d bytes]:  %s" % (payload_size,reply[48:64] if len(reply) > 64 else reply[48:]))
    
    # check expected status code
    
    ret_code = int(reply[12:16])
    if ret_code != exp_status:
        raise RuntimeError("Reply with unexpected status code %d" % ret_code)

    # return params and payload
    
    return ( reply[24:48], reply[48:] )
  
# ------------------------------------------------------------------------------

# check input arguments

if len(sys.argv) < 2 or len(sys.argv) > 4:
    print("Bad number arguments.\n Usage %s IP [port=8888]" % sys.argv[0])
    exit(1)

host = sys.argv[1]                                      # hostname / IP of device
port = 8888 if len(sys.argv) < 3 else int(sys.argv[2])  # port - by default 8888

# ------------------------------------------------------------------------------

# connect to socket

conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
conn.connect((host, port))

# ------------------------------------------------------------------------------

REQUEST_GET_STATE = 20
REQUEST_SET_STATE = 21
REQUEST_GET_FRAME = 26
REPLY_OK = 200
STATE_IDLE = 1
STATE_DEPTH_SENSOR = 2

STATES = {STATE_IDLE:           'STATE_IDLE',
          STATE_DEPTH_SENSOR:   'STATE_DEPTH_SENSOR'}

# ------------------------------------------------------------------------------

# switch to STATE_DEPTH_SENSOR -----

mke_send(conn, REQUEST_GET_STATE, 1)
(params, payload) = mke_recv(conn, REQUEST_GET_STATE, 1, REPLY_OK)

current_state = struct.unpack('<I', params[0:4])[0]

assert(params[4:] == bytes(20))                         # rest of params should be zero
assert(len(payload) == 0)                               # no payload expected

# ------------------------------------------------------------------------------

# if in IDLE go to DEPTH_SENSOR

if current_state == STATE_IDLE:
  
    print('Switching to ', STATES.get(STATE_DEPTH_SENSOR))
  
    mke_send(conn, REQUEST_SET_STATE, 2, struct.pack('<I', STATE_DEPTH_SENSOR) + bytes(4))
    (params, payload) = mke_recv(conn, REQUEST_SET_STATE, 2, REPLY_OK)
  
    assert(params == bytes(24))                           # no params expected
    assert(len(payload) == 0)                             # no payload expected

    print('Device is in STATE_DEPTH_SENSOR.')

else:
    print('Already in STATE_DEPTH_SENSOR');

# ------------------------------------------------------------------------------

# get one frame

mke_send(conn, REQUEST_GET_FRAME, 3, struct.pack('<H', 1) + bytes(6))   # use FRAME_TYPE_2
(params, payload) = mke_recv(conn, REQUEST_GET_FRAME, 3, REPLY_OK)

# read params

(timer, seqn, data_type, frame_type, num_data) = struct.unpack('<QQIHH', params)

# prepare lists

ids = []
xs = []
ys = []
zs = []

for i in range(num_data):
    sz = { 1: 8, 2: 12 }.get(frame_type)
    loc = payload[i*sz:(i+1)*sz]

    ids.append(struct.unpack('<H', loc[0:2])[0])
    xs.append(struct.unpack('<h', loc[2:4])[0])
    ys.append(struct.unpack('<h', loc[4:6])[0])
    zs.append(struct.unpack('<h', loc[6:8])[0])

# plot data

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(xs, ys, zs, 'b.')
plt.show()

# ------------------------------------------------------------------------------

# swithing back to IDLE

print('Switching to ', STATES.get(STATE_IDLE))

mke_send(conn, REQUEST_SET_STATE, 3, struct.pack('<I', STATE_IDLE) + bytes(4))
(params, payload) = mke_recv(conn, REQUEST_SET_STATE, 3, REPLY_OK)

print('Device is in STATE_IDLE.')

# ------------------------------------------------------------------------------
