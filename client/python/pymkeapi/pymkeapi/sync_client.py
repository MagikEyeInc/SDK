# -*- coding: utf-8 -*-

"""SyncClient

   synchronous client for communication with MkE sensor
"""

import struct
import binascii
from . import api
from . import bus
from . import __version__


__author__ = "Ondra Fisar"
__copyright__ = "Copyright (c) 2017-2020, Magik-Eye Inc."


# -----------------------------------------------------------------------------


class SyncClient:
    """Synchronous client"""
    __slots__ = '__bus', '_default_frame_type'

    def __init__(self, bus):
        """Connect to the sensor by bus"""
        self.__bus = bus
        self.__print_connected()
        self._default_frame_type = api.Frame

    # PUBLIC CALLS ===========================================

    @staticmethod
    def get_version():
        return __version__
    
    @staticmethod
    def get_supported_api_versions():
        return [(1,0,0)]

    def terminate(self, method):
        """Terminates the sensor (reboot or shutdown)."""
        # terminate does not send reply
        params = struct.pack('<I', method) + bytearray(4)
        return self._send_and_check(api.Request(cmd=api.MKE_REQUEST_TERMINATE,
                                                params=params),
                                    [api.MKE_REPLY_OK])

    # -------------------------------------------------------------------------

    def get_fw_info(self):
        """Get Firmware info. Returns tuple of 3 numbers (major, minor, patch)."""
        reply = self._send_and_check(api.Request(api.MKE_REQUEST_GET_FIRMWARE_INFO),
                                     [api.MKE_REPLY_OK])

        buildtime, commit_no = struct.unpack('<QI', reply.params[0:12])
        fw_ver = struct.unpack('BBB', reply.params[12:15])
        sys_ver = struct.unpack('BBB', reply.params[15:18])

        return api.FwInfo(posixtime=buildtime,
                          git_commit=commit_no,
                          fw_version=fw_ver,
                          sys_version=sys_ver)

    # -------------------------------------------------------------------------

    def get_device_info(self):
        """Returns description of the sensor"""
        reply = self._send_and_check(api.Request(api.MKE_REQUEST_GET_DEVICE_INFO),
                                     [api.MKE_REPLY_OK])
        return api.DeviceInfo(device_id=struct.unpack('<H', reply.params[0:2]),
                              unit_id=reply.params[2:10])

    # -------------------------------------------------------------------------

    def get_device_xml(self):
        """Returns description of the sensor"""
        reply = self._send_and_check(api.Request(api.MKE_REQUEST_GET_DEVICE_XML),
                                     [api.MKE_REPLY_OK])

        return reply.payload

    # -------------------------------------------------------------------------

    def get_state(self):
        """Returns current state of the sensor."""
        reply = self._send_and_check(api.Request(api.MKE_REQUEST_GET_STATE),
                                     [api.MKE_REPLY_OK])
        return struct.unpack('<I', reply.params[0:4])[0]

    # -------------------------------------------------------------------------

    def set_state(self, state):
        """Set state to `state`."""
        params = struct.pack('<I', state) + bytearray(4)
        self._send_and_check(api.Request(api.MKE_REQUEST_SET_STATE,
                                         params=params),
                             [api.MKE_REPLY_OK])

    # -------------------------------------------------------------------------

    def start_frame_push(self, frame_type):
        """Start pushing frames (with `frame_type`) from sensor. 
        Returns `sequence id` of the request.
        """
        params = struct.pack('<H', frame_type) + bytearray(6)
        reply = self._send_and_check(api.Request(api.MKE_REQUEST_START_FRAME_PUSH,
                                                 params=params),
                                     [api.MKE_REPLY_DATA_WILL_START])
        return reply.seq_id

    # -------------------------------------------------------------------------

    def stop_frame_push(self):
        """Stop pushing frames form sensor. 
        Returns `sequence id` of the request
        """
        return self.__bus.send_request(api.Request(api.MKE_REQUEST_STOP_FRAME_PUSH))

    # -------------------------------------------------------------------------

    def get_frame(self, frame_type):
        """Return current frame.

        Arguments:
        frame_type: Type of the desired frame
        """
        params = struct.pack('<H', frame_type) + bytearray(6)
        reply = self._send_and_check(api.Request(api.MKE_REQUEST_GET_FRAME, params=params),
                                     [api.MKE_REPLY_OK])
        frame = self._default_frame_type()
        frame.parse(reply.params, reply.payload)
        return frame

    # -------------------------------------------------------------------------

    def get_pushed_frame(self, start_seq_id, stop_seq_id=None):
        """Returns next frame pushed from the sensor by `start_frame_push` call.
        """
        reply = api.Reply()
        self.__bus.recv_reply(reply)

        if reply.seq_id == start_seq_id:  # received reply from start_frame_push

            if reply.ret_code == api.MKE_REPLY_DATA_STOPPED \
                    or reply.ret_code == api.MKE_REPLY_SERVER_REQUEST_INTERRUPTED:  # this is last reply
                return None

            # reply should have MKE_REPLY_DATA_WILL_CONTINUE status

            if reply.ret_code != api.MKE_REPLY_DATA_WILL_CONTINUE:
                raise api.Error("Unexpected reply from `start_frame_push`", reply.ret_code, reply.seq_id)

            frame = self._default_frame_type()
            frame.parse(reply.params, reply.payload)
            return frame

        elif reply.seq_id == stop_seq_id:  # received reply from stop_frame_push

            # reply should be MKE_REPLY_OK

            if reply.ret_code != api.MKE_REPLY_OK:
                raise api.Error("Unexpected reply from `stop_frame_push`", reply.ret_code, reply.seq_id)

            # ignore this message and read next until MKE_REPLY_DATA_STOPPED

            return self.get_pushed_frame(start_seq_id, stop_seq_id)

        else:
            raise api.Error("Unexpected reply with seq_id %d" % reply.seq_id, reply.ret_code, reply.seq_id)

    # -------------------------------------------------------------------------

    def get_policy(self):
        """Get current policy."""
        reply = self._send_and_check(api.Request(api.MKE_REQUEST_GET_POLICY),
                                     [api.MKE_REPLY_OK])

        return str(reply.params, 'ascii').rstrip('\0')

    # -------------------------------------------------------------------------

    def set_policy(self, policy_name):
        """Set profile to the sensor."""
        assert (type(policy_name) == str)

        params = bytes(policy_name, 'ascii')
        if len(params) > 8:
            raise RuntimeError("Policy name is too long")
        params += bytes(8 - len(params))

        self._send_and_check(api.Request(api.MKE_REQUEST_SET_POLICY,
                                         params=params),
                             [api.MKE_REPLY_OK])

    # -------------------------------------------------------------------------

    def list_policies(self):
        """List policies."""
        reply = self._send_and_check(api.Request(api.MKE_REQUEST_LIST_POLICIES),
                                     [api.MKE_REPLY_OK])

        return str(reply.payload, 'ascii').rstrip('\0').split('\0')

    # -------------------------------------------------------------------------

    def upload_package(self, payload=bytearray(0)):
        """Upload package specified either as a filename or as a bytearray."""

        if isinstance(payload, str):
            # Interpret payload as a filename
            with open(payload, mode='rb') as file:
                payload = file.read()

        crc32 = binascii.crc32(payload) ^ 0xFFFFFFFF
        params = struct.pack('<I', len(payload)) + struct.pack('<I', crc32)

        reply = self._send_and_check(api.Request(api.MKE_REQUEST_UPLOAD_PACKAGE,
                                                 params=params,
                                                 payload=payload),
                                     [api.MKE_REPLY_OK])

    # -------------------------------------------------------------------------
    # PRIVATE/PROTECTED METHODS -----------------------------------------------

    def __print_connected(self):
        """Print into verbose stream that client is connected"""
        fw_info = self.get_fw_info()
        dev_info = self.get_device_info()
        self.__bus.verbose_print(bus.DefaultBus.VERBOSE_INFO,
                                 "Connected to device with FW_VER: %s (SYS_VER: %s) on device %s:" %
                                 ('.'.join([str(v) for v in fw_info.fw_version]),
                                  '.'.join([str(v) for v in fw_info.sys_version]),
                                  dev_info.unit_id.decode('ascii')))

    def _send_and_check(self, api_request, expected_ret_codes=None):
        """Send api.Request and check and returns api.Reply.
        Checking ret code must be in list `expected_ret_codes`.
        """
        seq_id = self.__bus.send_request(api_request)
        reply = api.Reply()
        self.__bus.recv_reply(reply)

        assert (api_request.cmd == reply.cmd and seq_id == reply.seq_id)

        if expected_ret_codes is not None and not reply.ret_code in expected_ret_codes:
            raise api.Error("Unexpected reply status code", reply.ret_code, reply.seq_id)

        return reply

# -----------------------------------------------------------------------------
