#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""RAW example (without using pymkecli) how to get state of the sensor
"""
__author__ = "Ondra Fisar"
__copyright__ = "Copyright (c) 2017, Magik-Eye s.r.o., Prague"

# ------------------------------------------------------------------------------

# imports

import sys
import struct
import socket

# ------------------------------------------------------------------------------

# check input arguments

if len(sys.argv) < 2 or len(sys.argv) > 3:
    print("Bad number arguments.\n Usage %s IP [port=8888]" % sys.argv[0])
    exit(1)

host = sys.argv[1]                                      # hostname / IP of device
port = 8888 if len(sys.argv) < 3 else int(sys.argv[2])  # port - by default 8888

# ------------------------------------------------------------------------------

# connect to socket

conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
conn.connect((host, port))

# ------------------------------------------------------------------------------

# get current state -----

# assemble request

req_header  = b'MKERQ100'               # ASCII header of request
cmd         = b'0020'                   # ASCII REQUEST_GET_STATE
seq_id      = struct.pack('<I', 1)      # binary sequence id - could be anything
request     = req_header + cmd + seq_id + bytes(8)
                                        # assembled whole request - params = 8x b'\x00'

print("SENT:     %s %s %s %s" % (request[0:8], request[8:12], request[12:16], request[16:]))
                                        # nice print of whole request by parts

# send datagram to device

if conn.send(request) == 0:
    raise RuntimeError("Socket broken")

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
assert(reply[8:12] == cmd)              # command should be same
assert(reply[12:16] == b'0200')         # return code should be 0200 - means REPLY_OK
assert(reply[16:20] == seq_id)          # sequence id should be same

payload_size = struct.unpack('<I', reply[20:24])[0];

assert(payload_size == 0)               # REQUEST_GET_STATE should return reply without payload

current_state = struct.unpack('<I', reply[24:28])[0]

current_state_str = { 1: 'STATE_IDLE', 2: 'STATE_DEPTH_SENSOR' }.get(current_state)

print('Current state is: ', current_state_str)

assert(reply[28:48] == bytes(20))       # rest of reply should be filled by zeros

# ------------------------------------------------------------------------------
