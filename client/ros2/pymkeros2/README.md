Introduction
============

This document describes v1.0 of `pymkeros2` v1.0 ROS2 Package. This
package contains ROS2 [1] node `pymkeros2_node` for publishing 3D point
cloud data provided by Magik Eye sensors. Currently, `pymkeros2_node`
connects to Magik Eye devices that provide 3D data using the TCP/IP
protocol. The `pymkeros2_node` codebase depends on the MkE
API[\[mkeapi\]](#mkeapi) Python client implementation `pymkeapi`. The
following table lists the officialy supported platforms for
`pymkeros2_node`:

| Ubuntu Version     | ROS Distribution                                                                          |
|--------------------|-------------------------------------------------------------------------------------------|
| Ubuntu 18.04 64bit | [*ROS Crystal*](https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Binary) |
| Ubuntu 20.04 64bit | [*ROS Foxy*](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Binary)       |

> **Note**
>
> This document assumes that the reader has a working knowledge of ROS2
> and the ROS2 package compilation procedure. Documentation or
> explanation of any of these topics is out of the scope of this
> document.

Upon launch, the `pymkeros2_node` registers a new node
`pymkeros2_node_`*NAME*, where *NAME* is the node’s unique identifier
that depends on the command line parameters passed to the node
executable. It also publishes two services:
`pymkeros2_startpublish_`*NAME* and `pymkeros2_stoppublish_`*NAME*. Once
the `pymkeros2_startpublish_`*NAME* service is invoked, the node
connects to a Magik Eye sensor via TCP/IP network and starts publishing
the sensor’s 3D data stream under the `pymkeros2_node_pcd_`*NAME* topic.
The topic is unpublished and the connection to the sensor closed upon
invocation of the `pymkeros2_stoppublish_`*NAME* service.

Compilation
===========

Dependencies
------------

The `pymkeros2_node` codebase depends on the MkE
API[\[mkeapi\]](#mkeapi) Python client implementation `pymkeapi`. Root
path of the `pymkeapi` installation can be provided to the `PYTHONPATH`
environment variable in the following manner

    $ export PYTHONPATH=$PYTHONPATH:/path/to/pymkeapi/

Installation
------------

The `pymkeros2_node` ROS2 node installation is based on the CMake build
system and Colcon [2]. Let’s suppose that the ROS2 distribution has been
installed into the `${ROS2_ROOT}` directory and the `pymkeros2_node`
codebase resides in the `${PYMKEROS2_ROOT}` ROS2 Package. The
`${PYMKEROS2_ROOT}` ROS2 package resides in the `${ROS2_WS}` ROS2
workspace. In order to install the package, run the following command:

    $ mkdir "${ROS2_WS}/build"
    $ cd "${ROS2_WS}/build"
    $ source ${ROS2_ROOT}/setup.bash
    $ colcon build --symlink-install --packages-select pymkeros2 --base-paths ..

This will create the installation directory `install` in the
`${ROS2_WS}` ROS2 workspace. `--symlink-install` allows the installed
files to be changed by changing the files in the source space (e.g.
Python files or other not compiled resources) for faster iteration.

To test the `pymkeros2_node` execution, execute the following commands:

    $ source ${ROS2_WS}/build/install/setup.bash
    $ ros2 run pymkeros2 pymkeros2_node --help

Execution
=========

Once compiled, the `pymkeros2_node` can be invoked through the
`pymkeros2` package. The `help` parameter lists and describes the
available command line parameters:

    $ source ${ROS2_WS}/install/setup.bash
    $ ros2 run pymkeros2 pymkeros2_node --help

MkE Sensor Discovery
--------------------

In order to connect to a Magik Eye sensor, the `pymkeros2_node`
executable needs to be provided with the IP address of unit ID of the
sensor in question. Since all Magik Eye TCP/IP-enabled sensors implement
network discovery using the SSDP protocol, `pymkeros2_node` executable
provides the `discover` command line option that will list all MagikEye
sensors connected to the local TCP/IP network. In the following example,
the `pymkeros2_node` executable was able to discover two MagikEye
sensors:

    $ ros2 run pymkeros2 pymkeros2_node --discover
    MagikEyeOne-0242be55:192.168.0.100
    MagikEyeOne-0242ac2a:192.168.4.101

The list specifies the unit ID’s and respective IP addresses of the
discovered sensors.

The `discover` parameter can be also used in combination with the
`device` parameter to check the availability of a particular sensor. The
value of the `device` parameter can be an IP adress or a unit ID:

    $ ros2 run pymkeros2 pymkeros2_node --discover --device MagikEyeOne-0242be55
    $ echo $?
    0
    $ ros2 run pymkeros2 pymkeros2_node --discover --device 192.168.0.100
    $ echo $?
    0
    $ ros2 run pymkeros2 pymkeros2_node --discover --device 192.168.0.102
    $ echo $?
    1

Launching
---------

### Launching the node with Command line paramters (CLI).

The node `pymkeros2_node` is launched if the `launch` and `device`
parameters are provided:

    $ ros2 run pymkeros2 pymkeros2_node --launch \
                                    --device MagikEyeOne-0242be55
    [ INFO] [...]: Launching node: pymkeros2_node_MagikEyeOne_0242be55
    [ INFO] [...]: Starting service: pymkeros2_startpublish_MagikEyeOne_0242be55
    [ INFO] [...]: Starting service: pymkeros2_stoppublish_MagikEyeOne_0242be55

The above will launch a node called
`pymkeros2_node_MagikEyeOne-0242be55` and start two services called
`pymkeros2_startpublish_MagikEyeOne_0242be55` and
`pymkeros2_stoppublish_MagikEyeOne_0242be55` respectively. Again, the
`device` parameter can also contain the sensor’s IP address. The sensor
specific part of the node and services names can be overriden using the
`alias` parameter:

    $ ros2 run pymkeros2 pymkeros2_node --launch \
                                    --device MagikEyeOne-0242be55
                                    --alias s1
    [ INFO] [...]: Launching node: pymkeros2_node_s1
    [ INFO] [...]: Starting service: pymkeros2_startpublish_s1
    [ INFO] [...]: Starting service: pymkeros2_stoppublish_s1

> **Note**
>
> The node will *not* connect to the sensor upon launch, nor will it
> check the availability of the sensor. The connection will only be
> attempted upon invocation of the `pymkeros2_node_startpublish_`*NAME*
> service. For an immediate check of the sensor’s availability, use the
> `discover` parameter.

### Launching the node using Launch file.

The `launch` file present in
`${PYMKEROS2_ROOT}/launch/pymkeros2_launch.py` can be used to launch the
`pymkeros2_node` with default parameters described in
`${PYMKEROS2_ROOT}/config/pymkeros2_config.yaml`. The `device` parameter
is mandatory to use the `launch` file. The `device` parameter should be
provided with IP Address/Unit ID. The `alias` parameter is optional.

For example, the parameters described in `pymkeros2_config.yaml` are as
under:

    # Default configurations
    MKEROS_NODE:
    ros__parameters:
        device : "MagikEyeOne-34cff660"
        alias : "sensor1"

Launching the launch file with above parameters can be done in the
following manner:

    $ ros2 launch pymkeros2 pymkeros2_launch.py
    [INFO] [launch]: All log files can be found below /home/magikeye/.ros/log/2021-02-11-11-31-42-559158-magikeye-Yoga-Slim-7-14IIL05-16338
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [pymkeros2-1]: process started with pid [16340]
    [pymkeros2-1] [INFO] [1613023303.031078917] [pymkeros2_node_sensor1]: Launching node: pymkeros2_node_sensor1
    [pymkeros2-1] [INFO] [1613023303.031477020] [pymkeros2_node_sensor1]: Starting service: pymkeros2_startpublish_sensor1
    [pymkeros2-1] [INFO] [1613023303.033186781] [pymkeros2_node_sensor1]: Starting service: pymkeros2_stoppublish_sensor1

Services
========

Upon execution, the `pymkeros2_node` binary publishes two services:
`pymkeros2_startpublish_`*NAME* and `pymkeros2_stoppublish_`*NAME*.

Start Publishing
----------------

Once the `pymkeros2_startpublish_`*NAME* service is invoked, the node
connects to a Magik Eye sensor via TCP/IP network and starts publishing
the sensor’s 3D data stream under the `pymkeros2_node_pcd_`*NAME* topic.
If the sensor has been specified via its IP address, the node will try
to connect to the sensor directly. In the case the sensor has been
specified using its unit ID, the discovery procedure to recover its IP
address will be performed. Once the connection is established, the
`pymkeros2_node_pcd_`*NAME* topic is published.

The `pymkeros2_node` node provides a convenience parameter `start` to
call the start service. The device can be specified via the `device` or
`alias` options:

    $ ros2 run pymkeros2 pymkeros2_node --start --alias s1
    [INFO] [1612953661.365947423] [rclcpp]: Calling service: pymkeros2_startpublish_sensor1
    [INFO] [1612953662.439238763] [rclcpp]: Service called successfully: pymkeros2_startpublish_sensor1

Stop Publishing
---------------

The `pymkeros2_node_pcd_`*NAME* topic is unpublished and the connection
to the sensor closed upon invocation of the
`pymkeros2_stoppublish_`*NAME* service.

The `pymkeros2_node` binary provides a convenience parameter `stop` to
call the stop service. The device can be specified via the `device` or
`alias` options:

    $ ros2 run pymkeros2 pymkeros2_node --stop --alias s1
    [INFO] [1612953665.295884780] [rclcpp]: Calling service: pymkeros2_stoppublish_sensor1
    [INFO] [1612953665.409713437] [rclcpp]: Service called successfully: pymkeros2_stoppublish_sensor1

Accessing The Point Cloud Data
==============================

While publishing, the sensor data will be available on the topic called
`pymkeros2_node_pcd_`*NAME*. The message format of the data published on
this topic is `sensormsgs::PointCloud2`.

Bibliography
============

-   \[\] *MagikEye API v1.0*, 2020, Magik Eye Inc.

[1] [ros2](https://index.ros.org/doc/ros2/)

[2] <https://colcon.readthedocs.io/en/released/user/quick-start.html>
