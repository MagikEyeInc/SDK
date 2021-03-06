#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MkE API definitions and classes
"""

__author__ = "Ondra Fisar"
__copyright__ = "Copyright (c) 2017-2010, Magik-Eye Inc."

import struct
import numpy as np
import collections

# -----------------------------------------------------------------------------

# API Replies

MKE_REPLY_DATA_WILL_START = 100
MKE_REPLY_DATA_WILL_CONTINUE = 101
MKE_REPLY_DATA_STOPPED = 102

MKE_REPLY_OK = 200

MKE_REPLY_CLIENT_ERROR = 400
MKE_REPLY_CLIENT_MALFORMED_REQUEST = 401
MKE_REPLY_CLIENT_ILLEGAL_REQUEST_TYPE = 402
MKE_REPLY_CLIENT_REQUEST_DOES_NOT_APPLY = 403
MKE_REPLY_CLIENT_REQUEST_PAYLOAD_TOO_LONG = 404

MKE_REPLY_SERVER_ERROR = 500
MKE_REPLY_SERVER_REQUEST_INTERRUPTED = 501
MKE_REPLY_SERVER_BUSY = 502
MKE_REPLY_SERVER_INSUFFICIENT_RESOURCES = 503
MKE_REPLY_SERVER_FATAL_ERROR = 504

# API Requests -----

MKE_REQUEST_TERMINATE = 10
MKE_REQUEST_GET_FIRMWARE_INFO = 11
MKE_REQUEST_GET_DEVICE_INFO = 12
MKE_REQUEST_GET_DEVICE_XML = 13
MKE_REQUEST_GET_STATE = 20
MKE_REQUEST_SET_STATE = 21
MKE_REQUEST_GET_POLICY = 22
MKE_REQUEST_SET_POLICY = 23
MKE_REQUEST_START_FRAME_PUSH = 24
MKE_REQUEST_STOP_FRAME_PUSH = 25
MKE_REQUEST_GET_FRAME = 26
MKE_REQUEST_LIST_POLICIES = 27
MKE_REQUEST_UPLOAD_PACKAGE = 2001

MKE_REQUEST_UNDEF = 0
MKE_REQUEST_DYNAMIC_OFFSET = 2000
MKE_REQUEST_DYNAMIC_LAST = 2999

# API State enums -----

MKE_STATE_IDLE = 1
MKE_STATE_DEPTH_SENSOR = 2

# API Terminate enums -----

MKE_TERMINATE_BY_REBOOT = 1
MKE_TERMINATE_BY_SHUTDOWN = 2

# API FrameType enum

MKE_FRAME_TYPE_1 = 1
MKE_FRAME_TYPE_2 = 2

# API Data3D enum

MKE_DATA3D_MM = 0
MKE_DATA3D_MM2 = 1
MKE_DATA3D_MM4 = 2
MKE_DATA3D_MM8 = 3
MKE_DATA3D_MM16 = 4


FwInfo = collections.namedtuple('FwInfo', ['posixtime', 'git_commit',
                                           'fw_version', 'sys_version'])
DeviceInfo = collections.namedtuple('DeviceInfo', ['unit_id', 'device_id'])


class Error(RuntimeError):

    def __init__(self, message, ret_code, seq_id):
        nw_msg = message + " (received ret_code: %d:%s, @%d)" % (ret_code,
                                                                 Error.ret_code_to_string(ret_code), seq_id)
        super(RuntimeError, self).__init__(nw_msg)

        self.__orig_msg = message
        self.ret_code = ret_code
        self.seq_id = seq_id

    def __reduce__(self):
        return Error, (self.__orig_msg, self.ret_code, self.seq_id)

    @staticmethod
    def ret_code_to_string(ret_code):
        """Translate ret_code into string enum."""
        vars = globals()
        for name, value in vars.items():
            if name.startswith('MKE_REPLY_') and value == ret_code:
                return name
        return "!INVALID_RET_CODE!"


# -----

class Request:
    """Class to cover API request"""
    __slots__ = 'cmd', 'seq_id', 'params', 'payload'

    _MAGIC_HEAD = b'MKERQ100'
    _PACKET_LEN = 24
    _PARAMS_LEN = 8

    def __init__(self, cmd=MKE_REQUEST_UNDEF, seq_id=None, params=None, payload=None):
        """Contruct new Request.

        Arguments:
        cmd: API command of request
        seq_id: `sequnce_id` of the request. Could be None, will be auto-generated by Bus
        params: request parameters
        """
        if cmd is None:
            return

        if params is not None:
            assert (len(params) == Request._PARAMS_LEN)
        else:
            params = bytearray(Request._PARAMS_LEN)
        
        if payload is not None and len(payload) > 0:
            assert ((MKE_REQUEST_DYNAMIC_OFFSET < cmd < MKE_REQUEST_DYNAMIC_LAST))
        else:
            payload = bytearray(0)

        self.cmd = cmd
        self.seq_id = seq_id
        self.params = params
        self.payload = payload

    def __str__(self):
        """Stringify object"""
        return "ApiRequest: (cmd: %d, seq_id: %d, params: %s" % (
            self.cmd, self.seq_id, str(self.params))

    def assemble(self):
        """Assemble request."""
        cmd_b = bytes("%04d" % self.cmd, 'ascii')
        seq_id_b = struct.pack('<I', self.seq_id)
        return Request._MAGIC_HEAD + cmd_b + seq_id_b + self.params + self.payload

# -----

class Reply:
    """Class to wrap API reply.

    Properties:
    cmd: API command reply answering to.
    ret_code: Return code of the reply.
    seq_id: `sequence_id` of request reply answering to.
    params: reply params
    payload: payload of reply
    """

    __slots__ = 'cmd', 'ret_code', 'seq_id', 'params', 'payload'

    _MAGIC_HEAD = b'MKERP100'
    _PACKET_LEN = 48
    _PARAMS_LEN = 24

    def __init__(self):
        self.cmd = None
        self.ret_code = None
        self.seq_id = None
        self.params = None
        self.payload = None

    def __str__(self):
        """Stringify object"""
        return "ApiReply: (cmd: %d, ret_code: %d, seq_id: %d, params: %s, len(payload): %d" % (
            self.cmd, self.ret_code, self.seq_id, str(self.params), len(self.payload))

    def parse(self, data):
        """Parse reply from given data.
        Returns tuple:
        [0]: status of parsing (True/False)
        [1]: required/used size of data.
        """
        if len(data) < Reply._PACKET_LEN:
            return False, Reply._PACKET_LEN

        if not data.startswith(Reply._MAGIC_HEAD):
            raise RuntimeError("Received BAD magic: " + str(data[0:len(Reply._MAGIC_HEAD)]))

        payload_size = struct.unpack('<I', data[20:24])[0]

        if len(data) < (Reply._PACKET_LEN + payload_size):
            return False, Reply._PACKET_LEN + payload_size

        self.cmd = int(data[8:12])
        self.ret_code = int(data[12:16])
        self.seq_id, = struct.unpack('<I', data[16:20])
        self.params = bytes(data[24:48])
        self.payload = bytes(data[48:48 + payload_size])

        return True, Reply._PACKET_LEN + payload_size


# ------------------------------------------------------------------------------

class Frame:
    """Frame object with detections"""
    __slots__ = 'timer', 'seqn', 'data_type', 'frame_type', 'uids', 'pts3d', 'lids', 'dids'

    __FRAME_ITEM_SIZES = {MKE_FRAME_TYPE_1: 8,
                          MKE_FRAME_TYPE_2: 12}
    __DATA_TYPE_FACTOR = {MKE_DATA3D_MM: 1.0,
                          MKE_DATA3D_MM2: 0.5,
                          MKE_DATA3D_MM4: 0.25,
                          MKE_DATA3D_MM8: 0.125,
                          MKE_DATA3D_MM16: 0.0625}

    def __init__(self):
        self.timer = None
        self.seqn = None
        self.data_type = None
        self.frame_type = None

    def parse(self, params, payload):
        """Parse frame from its binnary form."""
        (self.timer, self.seqn, self.data_type, self.frame_type, num_pts) = struct.unpack('<QQIHH', params)
        self.parse_payload_only(num_pts, payload)

    def parse_payload_only(self, num_pts, payload):
        """Parse payload only."""
        # prepare memory

        self._prepare(num_pts)

        # fill memory
        frame_item_size = self._frame_item_size(self.frame_type)

        for i in range(num_pts):

            item_data = payload[i * frame_item_size:(i + 1) * frame_item_size]
            self._parse_frame_item(i, item_data)

            # post-process detections

        self.pts3d *= Frame.__DATA_TYPE_FACTOR.get(self.data_type)

    def _prepare(self, num_pts):
        """Prepare structures before parsering"""
        self.uids = np.empty(num_pts)
        self.pts3d = np.empty((num_pts, 3))
        if self.frame_type >= MKE_FRAME_TYPE_2:
            self.lids = np.empty(num_pts)
            self.dids = np.empty(num_pts)
        else:
            self.lids = None
            self.dids = None

    def _parse_frame_item(self, idx, item_data):
        """Parse single frame item."""
        self.uids[idx] = struct.unpack('<H', item_data[0:2])[0]
        self.pts3d[idx, :] = struct.unpack('<hhh', item_data[2:8])

        if self.frame_type >= MKE_FRAME_TYPE_2:
            (self.lids[idx], self.dids[idx]) = struct.unpack('<HH', item_data[8:12])

    def _frame_item_size(self, frame_type):
        """Return frame item size"""
        return Frame.__FRAME_ITEM_SIZES[frame_type]
