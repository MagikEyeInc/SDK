'''
* publisher
*
* Copyright (c) 2020-2021, Magik-Eye Inc.
* author: Jigar Patel, jigar@magik-eye.com
'''

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import threading
import time
import pymkeapi
import numpy as np
from pymkeros.device_info import DeviceInfo

# ============================================================================
# MkEPointCloudPublisher - TODO :Check for thread safe and correct termination


class MkEPointCloudPublisher:

    def __init__(self, device_info):
        self.device_info_ = device_info
        self.publishing_flag_ = False
        self.pub_ = None
        rospy.init_node(self.device_info_.getNodeName(), anonymous=True)

        # Other variables
        self.tcp_bus_ = None
        self.client_ = None
        self.thread_ = None

    # =========================================================================

    # Check - Needed ? Calls the function again
    def __del__(self):
        self.stopPublishing()

    # =========================================================================

    def publish(self):
        self.publishing_flag_ = True

        try:
            while self.publishing_flag_:
                frame = self.client_.get_frame(pymkeapi.MKE_FRAME_TYPE_1)
                ftype = frame.frame_type
                num = frame.pts3d.shape[0]
                pt_size = {1: 8, 2: 12}.get(ftype)
                dim = {1: 4, 2: 6}.get(ftype)
                data = np.concatenate((frame.pts3d, frame.uids.reshape(frame.
                                                                       uids.shape[0], 1)), axis=1)
                msg = PointCloud2()
                msg.header.frame_id = 'map'
                msg.height = 1
                msg.width = num
                msg.fields = [
                    PointField(name='x', offset=0,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='uid', offset=12,
                               datatype=PointField.FLOAT32, count=1)
                ]
                msg.is_bigendian = False
                msg.point_step = 16
                msg.row_step = msg.point_step * num
                msg.is_dense = False
                msg.data = np.asarray(data, np.float32).tostring()
                self.pub_.publish(msg)

        except Exception as e:
            print(e)

    # =========================================================================

    def closeConnection(self):
        # TODO: Check if reset exists. (del vs assiging to None)
        self.client_ = None
        self.tcp_bus_ = None
        self.pub_ = None

    # =========================================================================

    def startPublishing(self):
        if self.publishing_flag_:
            return

        try:
            self.tcp_bus_ = pymkeapi.TcpBus(self.device_info_.getIpAddr(),
                                            8888)
            self.client_ = pymkeapi.SyncClient(self.tcp_bus_)

            state = self.client_.get_state()

            if state == pymkeapi.MKE_STATE_IDLE:
                self.client_.set_state(pymkeapi.MKE_STATE_DEPTH_SENSOR)
            elif state != pymkeapi.MKE_STATE_DEPTH_SENSOR:
                self.client_.set_state(pymkeapi.MKE_STATE_IDLE)
                self.client_.set_state(pymkeapi.MKE_STATE_DEPTH_SENSOR)

            # Create topic publisher
            topic_name = self.device_info_.getTopicName()
            self.pub_ = rospy.Publisher(topic_name, PointCloud2,
                                        queue_size=10)

        except Exception as e:
            self.closeConnection()
            print(e)

        self.thread_ = threading.Thread(target=self.publish)
        self.thread_.start()

    # =========================================================================

    def stopPublishing(self):
        if not self.publishing_flag_:
            return
        self.publishing_flag_ = False
        self.thread_.join()

        if not self.client_:
            self.closeConnection()
            return

        state = self.client_.get_state()
        if state != pymkeapi.MKE_STATE_IDLE:
            self.client_.set_state(pymkeapi.MKE_STATE_IDLE)
        self.closeConnection()

    # =========================================================================

    def getDeviceInfo(self):
        return self.device_info_

    # =========================================================================

    def setDeviceInfo(self, device_info):
        self.device_info_ = device_info

# =============================================================================
# =============================================================================


if __name__ == "__main__":
    device_info = DeviceInfo("34cff660", "192.168.0.117")

    # Test the constructor
    device_info1 = DeviceInfo()
    mkepub = MkEPointCloudPublisher(device_info1)
    print(mkepub)
    mkepub.setDeviceInfo(device_info)
    print(mkepub.getDeviceInfo())

    # Test start publishing
    mkepub.startPublishing()
    time.sleep(5)

    # Test stop publishing
    mkepub.stopPublishing()
