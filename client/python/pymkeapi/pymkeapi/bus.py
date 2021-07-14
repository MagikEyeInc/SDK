#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""pymkeapi bus contains interface and implementation of synchronous buses
"""

__author__ = "Ondra Fisar"
__copyright__ = "Copyright (c) 2019-2020, Magik-Eye Inc."

# -----------------------------------------------------------------------------

# imports

import sys
import socket
import serial


# DefaultBus ------------------------------------------------------------------


class DefaultBus:
    """Interface for connecting with the sensor"""

    VERBOSE_NONE = 0
    VERBOSE_INFO = 1
    VERBOSE_DEBUG = 2

    def __init__(self, verbose_level=VERBOSE_NONE, verbose_stream=sys.stderr):
        """Constructor"""
        self.__seq_id = 0
        self.__verbose_level = verbose_level
        self.__verbose_stream = verbose_stream
        self.__buffer = bytearray(0)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __del__(self):
        self.close()

    def _send(self, data):
        """Send request data to bus."""
        raise NotImplementedError()

    def _recv(self, num_bytes):
        """Returns next `num_bytes` from bus."""
        raise NotImplementedError()

    def verbose_print(self, level, msg):
        """Print verbose message."""
        if level < self.__verbose_level:
            print(msg, file=self.__verbose_stream, flush=True)

    def send_request(self, api_request):
        """Send request to the device. Returns `sequence_id` of the request.

        Arguments:
        api_request: ApiRequest
        """
        if api_request.seq_id is None:
            api_request.seq_id = self.__seq_id
            self.__seq_id += 1
        msg = api_request.assemble()
        self._send(msg)
        self.verbose_print(DefaultBus.VERBOSE_DEBUG, "<<< %s" % msg)

        return api_request.seq_id

    def recv_reply(self, api_reply):
        """Fill next ApiReply from bus."""
        data = self.__buffer

        status, expected_len = api_reply.parse(data)
        while not status:
            while len(data) < expected_len:
                data += self._recv(expected_len - len(data))

            status, expected_len = api_reply.parse(data)

        self.__buffer = data[expected_len:]
        self.verbose_print(DefaultBus.VERBOSE_DEBUG, ">>> %s" % data[:expected_len])


# TcpBus ----------------------------------------------------------------------

class TcpBus(DefaultBus):

    def __init__(self, host, port=8888, **kwargs):
        """Connect to device by TCP/IP protocol.

        Arguments:
        host: host/IP of the device
        port: (default 8888) TCP port
        **kwargs: See DefaultBus class
        """

        self.__host = host
        self.__port = port
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__sock.connect((host, port))


        super().__init__(**kwargs)

    def close(self):
        self.__sock.close()

    def _send(self, data):
        """Send data to socket"""
        if self.__sock.send(data) == 0:
            raise RuntimeError("Socket broken")

    def _recv(self, num_bytes):
        """Receive `num_bytes` from socket"""
        tmp = self.__sock.recv(num_bytes)
        if len(tmp) == 0:
            raise RuntimeError("Socket closed")

        return tmp

# SerialBus --------------------------------------------------------------------

class SerialBus(DefaultBus):
  
    def __init__(self, dev_path, baudrate, parity=serial.PARITY_NONE, 
               stopbits=serial.STOPBITS_ONE, xonxoff=False, rtscts=False, 
               dsrdtr=False, **kwargs):
        """Connect to device by Serial port.

        Arguments:
        dev_path: path to serial port
        baudrate,parity,stopbits,xonxoff,rtscts,dsrdtr: See `serial` module documentation
        **kwargs: See DefaultBus class
        """

        self.__serial = serial.serial_for_url(dev_path, do_not_open=True)
        self.__serial.baudrate = baudrate
        self.__serial.parity = serial.PARITY_NONE
        self.__serial.stopbits = serial.STOPBITS_ONE
        self.__serial.xonxoff = False
        self.__serial.rtscts = False
        self.__serial.dsrdtr = False
        self.__serial.open()

        super().__init__(**kwargs)

    def close(self):
        self.__serial.close()

    def _send(self, data):
        return self.__serial.write(data)
    
    def _recv(self, num_bytes):
        return self.__serial.read(num_bytes)

# -----------------------------------------------------------------------------
