![](../../mke-banner.png "Magik Eye Banner")

# Python Client for MkE API #

The MkE SDK contains `pymkeapi`, a Python (version 3.x) package that implements an MkE API client. It also contains several examples of the MkE API client code implementation and communication.

The package is distributed in the form of a [Python wheel](pymkeapi-1.2.4-py3-none-any.whl) which can be easily installed--along with its dependencies--using the `pip3` tool:
```
$ pip3 install pymkeapi-1.2.4-py3-none-any.whl
```

The `pymkeapi` package contains the `SyncClient` class, which is a thin wrapper over the MkE API with an almost 1:1 correspondence between the API requests and the `SyncClient` methods. See the following HTML or PDF files for the documentation of the `pymkeapi.SyncClient` class:

* [client/python/doc/pysyncclient.html](doc/pysyncclient.html)
* [client/python/doc/pysyncclient.pdf](doc/pysyncclient.pdf)

## Example codes

The MkE API client examples in Python reside in the [client/python/mkeapi_examples](mkeapi_examples) directory. All of the examples require IP address or hostname of the sensor as the first parameter and optionally the port number as the second parameter. The default port number is 8888:

`python3 example_script.py <IP address or hostname> [<port number>]`

### Examples of the direct MkE API communication

These examples demostrate the direct MkE API communication with the sensor.
These provide the best way to understand the client/sensor communication protocol.

>  **Note:** All request and related reply datagrams are immediately printed in the RAW format.

--------------------------------------------------------------
Command               Function
-----------           ----------------------------------------
`raw_getstate.py`      • Checks the current sensor state <br>
                       • Switches to `MKE_STATE_DEPTH_SENSOR` <br>
                       • Waits for a while <br>
                       • Switches back to `MKE_STATE_IDLE`
                       
`raw_setstate.py`      • Checks the current sensor state. <br>
                       • Switches to `MKE_STATE_DEPTH_SENSOR` <br>
                       • Waits for a while <br>
                       • Switches back to `MKE_STATE_IDLE` 
                       
`raw_getframe.py`      • Checks the current sensor state <br>
                       • Switches to `MKE_STATE_DEPTH_SENSOR` <br>
                       • Captures one frame from the sensor <br>
                       • Switches back to `MKE_STATE_IDLE`

`sensor_discovery.py`  Shows how to use the SSDP protocol
                       to discover MagikEye sensors on the 
                       local network.
-------------------------------------------------------------


### Example usage of the `SyncClient` client 

These examples use Python MkE API implementation `SyncClient`.

--------------------------------------------------------------
Command               Function
-----------           ----------------------------------------
`client_getframe.py`   • Checks the current sensor state <br>
                       • Switches to `MKE_STATE_DEPTH_SENSOR` <br>
                       • Captures one frame from the sensor<br>
                       • Switches back to `MKE_STATE_IDLE`
                       
`client_pushframes.py` • Checks the current sensor state <br>
                       • Switches to `MKE_STATE_DEPTH_SENSOR` <br>
                       • Captures one frame from the sensor <br>
                       • Switches back to `MKE_STATE_IDLE` 
                       
`simple_viewer.py`     A simple PySide2/vispy based point cloud 
                       viewer
-------------------------------------------------------------

### Integration examples of the `SyncClient` client 

These examples show how to use `SyncClient` with common 
3rd party libraries:

* [PCL](http://pointclouds.org)
* [Open3D](http://open3d.org)

Information about required dependencies to run these examples
may be found in:

* [client/cpp/doc/mkeclient_integration_examples.html](../cpp/doc/mkeclient_integration_examples.html)
* [client/cpp/doc/mkeclient_integration_examples.pdf](../cpp/doc/mkeclient_integration_examples.pdf)


--------------------------------------------------------------
Command               Function
-----------           ----------------------------------------
`pcl_getframe.py`      Shows how to use `pymkeapi` in connection                    
                       with the [PCL](http://pointclouds.org)
                       library. 

`pcl_viewer.py`        A Simple point cloud viewer based on 
                       the [PCL](http://pointclouds.org)
                       library. 

`open3d_getframe.py`   Shows how to use `pymkeapi` in connection                   
                       with the [Open3D](http://open3d.org)
                       library. 

`open3d_viewer.py`     A simple point cloud viewer based on 
                       the [Open3D](http://open3d.org)
                       library. 




---

 © 2016-2021 MagikEye Inc.
