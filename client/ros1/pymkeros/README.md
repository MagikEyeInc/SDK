Introduction
============

This document describes PYMKEROS1\_NODE (also spelled as
`pymkeros1_node.py` in this document) v1.0, a ROS1 [1] node from the
`pymkeros` package for publishing 3D point cloud data provided by Magik
Eye sensors. Currently, `pymkeros1_node.py` connects to Magik Eye
devices that provide 3D data using the TCP/IP protocol. The
`pymkeros1_node.py` codebase depends on the MkE API[\[mkeapi\]](#mkeapi)
Python client implementation `pymkeapi`. The following table lists the
officialy supported platforms for `pymkeros1_node.py`:

| Ubuntu Version     | ROS Distribution                                                 |
|--------------------|------------------------------------------------------------------|
| Ubuntu 18.04 64bit | [*ROS Melodic*](http://wiki.ros.org/melodic/Installation/Ubuntu) |
| Ubuntu 20.04 64bit | [*ROS Noetic*](http://wiki.ros.org/noetic/Installation/Ubuntu)   |

> **Note**
>
> This document assumes that the reader has a working knowledge of ROS
> and the ROS package compilation procedure. Documentation or
> explanation of any of these topics is out of the scope of this
> document.

Upon launch, the `pymkeros1_node.py` registers a new node
`pymkeros1_node_`*NAME*, where *NAME* is the node’s unique identifier
that depends on the command line parameters passed to the node
executable. It also publishes two services:
`pymkeros1_startpublish_`*NAME* and `pymkeros1_stoppublish_`*NAME*. Once
the `pymkeros1_startpublish_`*NAME* service is invoked, the node
connects to a Magik Eye sensor via TCP/IP network and starts publishing
the sensor’s 3D data stream under the `pymkeros1_node_pcd_`*NAME* topic.
The topic is unpublished and the connection to the sensor closed upon
invocation of the `pymkeros1_stoppublish_`*NAME* service.

Compilation
===========

The `pymkeros1_node.py` ROS node compilation is based on the CMake build
system and Catkin [2]. Let’s suppose that the ROS distribution has been
installed into the `${ROS_ROOT}` directory and the `pymkeros1_node.py`
codebase resides in the `${PYMKEROS_ROOT}` ROS package of `${ROS1_WS}`
ROS workspace. The following BASH commands will compile the
`pymkeros1_node.py` into the `${ROS1_WS}/build/` directory:

    $ mkdir "${ROS1_WS}/build"
    $ cd "${ROS1_WS}/build"
    $ source ${ROS_ROOT}/setup.bash
    $ catkin_make --pkg pymkeros --source ..

This creates a `${ROS1_WS_DEVEL}` space in the `${ROS1_WS}`/build path.
To test the `pymkeros1_node.py` compilation, execute the following
commands:

    $ source ${ROS1_WS_DEVEL}/setup.bash
    $ rosrun pymkeros pymkeros1_node.py --help

Dependencies
------------

The `pymkeros1_node.py` codebase depends on the MkE API Python client
implementation called `pymkeapi`. The `pymkeapi` package is available as
a pip-installable wheel archive. Alternatively, the root path of the
`pymkeapi` package can be provided via the `PYTHONPATH` environment
variable in the following manner:

    $ export PYTHONPATH=$PYTHONPATH:/path/to/pymkeapi/

Execution
=========

Once compiled, the `pymkeros1_node.py` can be invoked through the
`pymkeros` package. The `help` parameter lists and describes the
available command line parameters:

    $ source ${ROS1_WS_DEVEL}/setup.bash
    $ rosrun pymkeros pymkeros1_node.py --help

MkE Sensor Discovery
--------------------

In order to connect to a Magik Eye sensor, the `pymkeros1_node.py`
script needs to be provided with an IP address or a unit ID of the
sensor in question. Since all Magik Eye TCP/IP-enabled sensors implement
network discovery using the SSDP protocol, `pymkeros1_node.py` script
provides the `discover` command line option that will list all MagikEye
sensors connected to the local TCP/IP network. In the following example,
the `pymkeros1_node.py` executable was able to discover two MagikEye
sensors:

    $ rosrun pymkeros pymkeros1_node.py --discover
    MagikEyeOne-0242be55:192.168.0.100
    MagikEyeOne-0242ac2a:192.168.4.101

The list specifies the unit ID’s and respective IP addresses of the
discovered sensors.

The `discover` parameter can be also used in combination with the
`device` parameter to check the availability of a particular sensor. The
value of the `device` parameter can be an IP adress or a unit ID:

    $ rosrun pymkeros pymkeros1_node.py --discover \
                                      --device MagikEyeOne-0242be55
    $ echo $?
    0
    $ rosrun pymkeros pymkeros1_node.py --discover --device 192.168.0.100
    $ echo $?
    0
    $ rosrun pymkeros pymkeros1_node.py --discover --device 192.168.0.102
    $ echo $?
    1

Launching
---------

The `pymkeros1_node.py` node can be launched either by providing the
connection information through the command line parameters or through a
launch file.

> **Note**
>
> The `roscore` (rosmaster) process must already be running in order for
> the `roslaunch` or `rosrun` commands to work.

### Launching Through the Command Line

The node is launched when the `launch` and `device` parameters are
provided:

    $ rosrun pymkeros pymkeros1_node.py --launch \
                                 --device MagikEyeOne-0242be55
    [ INFO] [...]: Launching node: pymkeros1_node_MagikEyeOne_0242be55
    [ INFO] [...]: Starting service: pymkeros1_startpublish_MagikEyeOne_0242be55
    [ INFO] [...]: Starting service: pymkeros1_stoppublish_MagikEyeOne_0242be55

The above will launch a node called
`pymkeros1_node_MagikEyeOne-0242be55` and start two services called
`pymkeros1_startpublish_MagikEyeOne_0242be55` and
`pymkeros1_stoppublish_MagikEyeOne_0242be55` respectively. Again, the
`device` parameter can also contain the sensor’s IP address. The sensor
specific part of the node and services names can be overriden using the
`alias` parameter:

    $ rosrun pymkeros pymkeros1_node.py --launch \
                                 --device MagikEyeOne-0242be55
                                 --alias s1
    [ INFO] [...]: Launching node: pymkeros1_node_s1
    [ INFO] [...]: Starting service: pymkeros1_startpublish_s1
    [ INFO] [...]: Starting service: pymkeros1_stoppublish_s1

> **Note**
>
> The node will *not* connect to the sensor upon launch, nor will it
> check the availability of the sensor. The connection will only be
> attempted upon invocation of the `pymkeros1_startpublish_`*NAME*
> service. For an immediate check of the sensor’s availability, use the
> `discover` parameter.

### Launching Through a Launch File

The launch file `${PYMKEROS_ROOT}/launch/pymkeros1.launch` can be used
to launch the node with the default parameters described in
`${PYMKEROS_ROOT}/config/pymkeros1_config.yaml`. The `device` parameter
is a mandatory parameter of the launch file. The `device` parameter
should be provided as an IP Address or a unit ID. The `alias` parameter
is optional.

For example, the launch file `pymkeros1_config.yaml` can look as
follows:

    # Default configurations
    device : "192.168.0.117"
    # alias : "s1"

Launching the node using a launch file with the above parameters can be
done using the `roslaunch` command:

    $ roslaunch pymkeros pymkeros1.launch

> **Note**
>
> If roslaunch is used to launch the `pymkeros1_node.py` using the above
> method, then `rosrun` should not be invoked to launch start and stop
> services or for other CLI parameters.

Services
========

Upon execution, the `pymkeros1_node.py` node publishes two services:
`pymkeros1_startpublish_`*NAME* and `pymkeros1_stoppublish_`*NAME*.

> **Note**
>
> If `roslaunch` is used to launch the `pymkeros1_node.py`, then the
> services `pymkeros1_startpublish_`**NAME** and
> `pymkeros1_stoppublish_`**NAME** should be called using
> `rosservice call` command of ROS API.

Start Publishing
----------------

Once the `pymkeros1_startpublish_`*NAME* service is invoked, the node
connects to a Magik Eye sensor via TCP/IP network and starts publishing
the sensor’s 3D data stream under the `pymkeros1_node_pcd_`*NAME* topic.
If the sensor has been specified via its IP address, the node will try
to connect to the sensor directly. In the case the sensor has been
specified using its unit ID, the discovery procedure to recover its IP
address will be performed. Once the connection is established, the
`pymkeros1_node_pcd_`*NAME* topic is published.

The `pymkeros1_node.py` script provides a convenience parameter `start`
to call the start service. The device can be specified via the `device`
or `alias` options:

    $ rosrun pymkeros pymkeros1_node.py --start --alias s1
    [ INFO] [...]: Calling service: pymkeros1_startpublish_s1
    [ INFO] [...]: Service called successfully: pymkeros1_startpublish_s1

Stop Publishing
---------------

The `pymkeros1_node_pcd_`*NAME* topic is unpublished and the connection
to the sensor closed upon invocation of the
`pymkeros1_stoppublish_`*NAME* service.

The `pymkeros1_node.py` script provides a convenience parameter `stop`
to call the stop service. The device can be specified via the `device`
or `alias` options:

    $ rosrun pymkeros pymkeros1_node.py --stop --alias s1
    [ INFO] [...]: Calling service: pymkeros1_stoppublish_s1
    [ INFO] [...]: Service called successfully: pymkeros1_stoppublish_s1

Accessing The Point Cloud Data
==============================

While publishing, the sensor data will be available on the topic called
`pymkeros1_node_pcd_`*NAME*. The message format of the data published on
this topic is `sensormsgs::PointCloud2`.

Bibliography
============

-   \[\] *MagikEye API v1.0*, 2020, Magik Eye Inc.

[1] [ros.org](http://ros.org)

[2] <http://docs.ros.org/en/api/catkin/html>
