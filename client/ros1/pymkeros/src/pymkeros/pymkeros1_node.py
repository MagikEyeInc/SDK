#!/usr/bin/env python3
"""
* python_mkeros1_node
*
* Copyright (c) 2020-2021, Magik-Eye Inc.
* author: Jigar Patel, jigar@magik-eye.com
"""

# ============================================================================
# ROS

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest

# ============================================================================
# System

import threading
import time
import sys

# ============================================================================
# mkeros1_node

import argparse
import os
from pymkeros.device_info import DeviceInfo
from pymkeros.device_discovery import DeviceDiscovery
from pymkeros.publisher import MkEPointCloudPublisher

# ============================================================================
# Globals

publisher = None
device_discovery = DeviceDiscovery()
device_info = DeviceInfo()

# ============================================================================
# ============================================================================
# ROS callbacks

"""
    Start service callback.
    @param std_srvs/Trigger/Request req
    @return std_srvs/Trigger/Response res
    @brief
    To Start publishing on PCD Topic
"""


def start_service_callback(req):

    global publisher, device_info, device_discovery

    if not publisher:
        return True

    uid = device_info.getUnitId()
    start_service = device_info.getStartServiceName()

    try:
        if uid:
            rospy.loginfo("Looking for device: %s", uid)
            device_discovery.updateDeviceList()

            dinfo = DeviceInfo()
            if not device_discovery.validateDevice(uid, dinfo):
                raise RuntimeError(f"Cannot find device: {uid}")

            rospy.loginfo("Found device: %s: %s",
                          dinfo.getUnitId(), dinfo.getIpAddr())
            publisher.setDeviceInfo(dinfo)

        publisher.startPublishing()

        status_msg = "Started point cloud publishing at " + start_service
        res = TriggerResponse(success=True, message=status_msg)

        rospy.loginfo("%s", status_msg)

        return res

    except Exception as e:
        res = TriggerResponse(success=False, message=str(e))
        status_msg = "Failed to start point cloud publishing at "\
            + start_service + ": " + str(e)

        rospy.loginfo("%s", status_msg)

        return res

    return TriggerResponse(success=True, message="")


# ============================================================================
"""
    Stop service callback.
    @param std_srvs/Trigger/Request req
    @return std_srvs/Trigger/Response res - [Success/Failure and message]
    @brief
    To Stop publishing on PCD Topic
"""


def stop_service_callback(req):

    global publisher, device_info, device_discovery

    if (not publisher):
        return True

    stop_service = device_info.getStopServiceName()

    try:
        publisher.stopPublishing()
        status_msg = "Stopped point cloud publishing at " + stop_service
        res = TriggerResponse(success=True, message=status_msg)

        rospy.loginfo("%s", status_msg)

    except Exception as e:
        res = TriggerResponse(success=False, message=str(e))
        status_msg = "Failed to stop point cloud publishing at "\
            + stop_service + ": " + str(e)

        rospy.loginfo("%s", status_msg)

    return res

# ============================================================================
# ============================================================================
# ROS node initialization


def launch_node(dinfo):
    global publisher, device_discovery
    rospy.init_node(dinfo.getNodeName(), anonymous=True)
    rospy.loginfo("Launching node: %s", dinfo.getNodeName())

    # Start service server
    start_service = dinfo.getStartServiceName()
    rospy.loginfo("Starting service: %s", start_service)
    rospy.Service(start_service, Trigger, start_service_callback)

    # Stop service server
    stop_service = dinfo.getStopServiceName()
    rospy.loginfo("Starting service: %s", stop_service)
    rospy.Service(stop_service, Trigger, stop_service_callback)

    publisher = MkEPointCloudPublisher(dinfo)

    rospy.spin()

# ============================================================================


def start_publishing(dinfo):
    rospy.init_node(dinfo.getNodeName(), anonymous=True)

    start_service = dinfo.getStartServiceName()

    start_client = rospy.ServiceProxy(start_service, Trigger)
    srv = TriggerRequest()

    rospy.loginfo("Calling service:, %s", start_service)

    response = start_client(srv)

    if response.success:
        rospy.loginfo("Service called successfully: %s", start_service)
        return 0
    else:
        rospy.loginfo("Service called failed: " + start_service +
                      ": " + response.message)
        return 1

# ============================================================================


def stop_publishing(dinfo):
    rospy.init_node(dinfo.getNodeName(), anonymous=True)

    stop_service = dinfo.getStopServiceName()

    stop_client = rospy.ServiceProxy(stop_service, Trigger)
    srv = TriggerRequest()

    rospy.loginfo("Calling service: %s", stop_service)

    response = stop_client(srv)

    if response.success:
        rospy.loginfo("Service called successfully: %s", stop_service)
        return 0
    else:
        rospy.loginfo("Service called failed: " + stop_service +
                      ": " + response.message)
        return 1

# ============================================================================
# ============================================================================
# Argparse


# TODO: With correct implementation to take from CMakeLists.txt
PYMKEROS1_VERSION = "1.0.0"

CliParser = argparse.ArgumentParser(
    f"This is pymkeros1_node v {PYMKEROS1_VERSION} (c) MagikEye 2020-2021")

CliParser.add_argument("-v", "--version", help="Output version string",
                       action="store_true", default=False)

group = CliParser.add_mutually_exclusive_group()
group.add_argument("--start", help="Start node services (if applicable), "
                   "and point cloud publishing topic",
                   action="store_true", default=False)
group.add_argument("--stop", help="Stop point cloud publiblishing topic",
                   action="store_true", default=False)
group.add_argument("--launch", help="Launch node and start its services",
                   action="store_true", default=False)

CliParser.add_argument("--discover", help="Perform the local network sensor\
                discovery procedure and output a list of available sensor",
                       action="store_true", default=False)

CliParser.add_argument("--device", type=str, help="IP address or Unit ID of \
                an MkE device", default="")
CliParser.add_argument("--alias", type=str, help="Device name alias",
                       default="")

CliParser.add_argument("--timeout", type=int, help="Device discovery timeout\
                [s]", default=1)

# ============================================================================
# ============================================================================
# Main


def main():
    try:
        if rospy.has_param('device'):
            device_param = rospy.get_param('device')

            if DeviceDiscovery.isIpAddress(device_param):
                device_info.setIpAddr(device_param)
            else:
                device_info.setUnitId(device_param)

            if rospy.has_param('alias'):
                alias_param = rospy.get_param('alias')
                device_info.setAlias(alias_param)
            else:
                device_info.setAlias()

            launch_node(device_info)

            return 0

        # Launch - For roslaunch
        if(sys.argv[-1].startswith("__")):
            cli = CliParser.parse_args(sys.argv[1:-2])
        # Run - For rosrun
        else:
            cli = CliParser.parse_args(sys.argv[1:])

        if cli.version:
            print(PYMKEROS1_VERSION)

        device_discovery.setTimeout(cli.timeout)

        if cli.discover:
            if not cli.device:
                device_discovery.updateDeviceList()

                for key, value in device_discovery.getDeviceList().items():
                    print(key+":"+value)

                return 0
            else:
                # Search for devices on local network
                device_discovery.updateDeviceList()

                # Check the availability of the cli.device only
                if(device_discovery.validateDevice(cli.device)):
                    return 0
                else:
                    exit(1)

        if DeviceDiscovery.isIpAddress(cli.device):
            device_info.setIpAddr(cli.device)
        else:
            device_info.setUnitId(cli.device)

        device_info.setAlias(cli.alias)

        # Perform launch/start/stop action
        if cli.launch:
            if not cli.device:
                raise RuntimeError('Please set the --device option')

            launch_node(device_info)

        elif cli.start:
            if not device_info.getName():
                raise RuntimeError('Please set the --device or --alias option')

            return start_publishing(device_info)

        elif cli.stop:
            if not device_info.getName():
                raise RuntimeError('Please set the --device or --alias option')

            return stop_publishing(device_info)

    except Exception as e:
        print("mkeros1_node fatal error: ", e)
        return 1

    return 0


if __name__ == "__main__":
    main()
