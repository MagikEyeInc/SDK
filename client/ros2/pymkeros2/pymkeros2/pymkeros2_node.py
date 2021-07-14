#!/usr/bin/env python3
"""
* python_mkeros2_node
*
* Copyright (c) 2020-2021, Magik-Eye Inc.
* author: Jigar Patel, jigar@magik-eye.com
"""

# ============================================================================
# ROS2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from example_interfaces.srv import Trigger

# ============================================================================
# System

import threading
import time
import sys

# ============================================================================
# mkeros2_node

import argparse
from pymkeros2.device_info import DeviceInfo
from pymkeros2.device_discovery import DeviceDiscovery
from pymkeros2.publisher import MkEPointCloudPublisher

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


def start_service_callback(req, res):

    global publisher, device_info, device_discovery

    if not publisher:
        return True

    uid = device_info.getUnitId()
    start_service = device_info.getStartServiceName()

    try:
        if uid:
            publisher.get_logger().info(f"Looking for device: {uid}")
            device_discovery.updateDeviceList()

            dinfo = DeviceInfo()
            if not device_discovery.validateDevice(uid, dinfo):
                raise RuntimeError(f"Cannot find device: {uid}")

            publisher.get_logger().info(f"Found device: {dinfo.getUnitId()}:\
                                       {dinfo.getIpAddr()}")
            publisher.setDeviceInfo(dinfo)

        publisher.startPublishing()

        status_msg = f"Started point cloud publishing at {start_service}"

        res.success = True
        res.message = status_msg

        publisher.get_logger().info(status_msg)

    except Exception as e:
        res.success = False
        res.message = str(e)

        status_msg = f"Failed to start point cloud publishing at \
                       {start_service} : {str(e)}"

        publisher.get_logger().info(status_msg)

    return res


# ============================================================================
"""
    Stop service callback.
    @param std_srvs/Trigger/Request req
    @return std_srvs/Trigger/Response res - [Success/Failure and message]
    @brief
    To Stop publishing on PCD Topic
"""


def stop_service_callback(req, res):

    global publisher, device_info, device_discovery

    if (not publisher):
        return True

    stop_service = device_info.getStopServiceName()

    try:
        publisher.stopPublishing()
        status_msg = f"Stopped point cloud publishing at {stop_service}"

        res.success = True
        res.message = status_msg

        publisher.get_logger().info(status_msg)

    except Exception as e:
        res.success = False
        res.message = str(e)

        status_msg = f"Failed to stop point cloud publishing at \
                       {stop_service} : {str(e)}"

        publisher.get_logger().info(status_msg)

    return res

# ============================================================================
# ============================================================================
# ROS node initialization


def launch_node(dinfo):
    global publisher, device_discovery

    publisher = MkEPointCloudPublisher(dinfo)

    publisher.get_logger().info(f"Launching node: {dinfo.getNodeName()}")

    # Start service server
    start_service = dinfo.getStartServiceName()
    publisher.get_logger().info(f"Starting service: {start_service}")
    service_start = publisher.create_service(
        Trigger, start_service, start_service_callback)

    # Stop service server
    stop_service = dinfo.getStopServiceName()
    publisher.get_logger().info(f"Starting service: {stop_service}")
    service_stop = publisher.create_service(
        Trigger, stop_service, stop_service_callback)

    rclpy.spin(publisher)

# ============================================================================


def start_publishing(dinfo):
    n = Node(dinfo.getStartServiceNodeName())

    start_service = dinfo.getStartServiceName()

    start_client = n.create_client(Trigger, start_service)

    n.get_logger().info(f"Calling service:, {start_service}")

    while not start_client.wait_for_service(timeout_sec=1.0):
        n.get_logger().info('service not available, waiting again...')

    req = Trigger.Request()
    result = start_client.call_async(req)

    while rclpy.ok():
        rclpy.spin_once(n)
        if result.done():
            try:
                response = result.result()
            except Exception as e:
                n.get_logger().info(
                    f"Service call failed {str(e)}")
            else:
                n.get_logger().info(
                    f"Service called successfully: {start_service}")
            break

    n.destroy_node()
    rclpy.shutdown()
    return 0

# ============================================================================


def stop_publishing(dinfo):
    n = Node(dinfo.getStopServiceNodeName())

    stop_service = dinfo.getStopServiceName()

    stop_client = n.create_client(Trigger, stop_service)

    n.get_logger().info(f"Calling service:, {stop_service}")

    while not stop_client.wait_for_service(timeout_sec=1.0):
        n.get_logger().info('service not available, waiting again...')

    req = Trigger.Request()
    result = stop_client.call_async(req)

    while rclpy.ok():
        rclpy.spin_once(n)
        if result.done():
            try:
                response = result.result()
            except Exception as e:
                n.get_logger().info(
                    f"Service call failed {str(e)}")
            else:
                n.get_logger().info(
                    f"Service called successfully: {stop_service}")
            break

    n.destroy_node()
    rclpy.shutdown()
    return 0

# ============================================================================
# ============================================================================
# Argparse


# TODO: With correct implementation to take from CMakeLists.txt
PYMKEROS2_VERSION = "1.0.0"

CliParser = argparse.ArgumentParser(
    f"This is pymkeros2_node v {PYMKEROS2_VERSION}(c) MagikEye 2020-2021")

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


def main(args=None):
    try:
        rclpy.init(args=args)
        param_node = Node("MKEROS_NODE")

        param_node.declare_parameter("device", "")
        device_param = param_node.get_parameter(
            "device").get_parameter_value().string_value

        if device_param:
            if DeviceDiscovery.isIpAddress(device_param):
                device_info.setIpAddr(device_param)
            else:
                device_info.setUnitId(device_param)

            param_node.declare_parameter("alias", "")
            alias_param = param_node.get_parameter(
                "alias").get_parameter_value().string_value

            if alias_param:
                device_info.setAlias(alias_param)
            else:
                device_info.setAlias()

            launch_node(device_info)
            return 0

        cli = CliParser.parse_args()
        if cli.version:
            print(MKEROS2_VERSION)

        device_discovery.setTimeout(cli.timeout)

        if cli.discover:
            if not cli.device:
                device_discovery.updateDeviceList()
                for key, value in device_discovery.getDeviceList().items():
                    print(f"{key} : {value}")

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
        print(f"mkeros2_node fatal error: {str(e)}")
        return 1

    return 0


if __name__ == "__main__":
    main()
