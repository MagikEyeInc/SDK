![](../../mke-banner.png "Magik Eye Banner")

# C++ Client for MkE API #

The MkE SDK provides C++ library implementation of the client part
of MkE API called `libmkeclient`:
 
* [client/cpp/doc/mkeclient.html](doc/mkeclient.html) ([README.md](doc/README.md))
* [client/cpp/doc/mkeclient.pdf](doc/mkeclient.pdf)

Source code documentation:

* [client/cpp/doc/html](doc/html/index.html)

## Example codes

Examples for the MkE API C++ Client reside in the [mkeclient/src/examples](mkeclient/src/examples) directory. 

All examples are console applications that report progresss and results to standard output.
All examples require IP address or hostname of the sensor as the first parameter.

------------------------------------------------------------------------------
Example                     Function
-----------                 ----------------------------------------                     
`demo_device_state.cpp`      Shows how to connect to sensor
                             and get:<br>
                             • current sensor state <br>
                             • device info <br>
                             • firmware info <br>
                             Also provides an example on 
                             how to switch sensor states.
                       
`demo_getframe_sync.cpp`     Shows how to get a point cloud data from sensor
                             using polling method (getting data on demand). <br>
                             It does these steps:<br>
                             • connects to the sensor <br>
                             • switches to `MKE_STATE_DEPTH_SENSOR` state<br>
                             • gets a frame (via synchronous MkE C++ Client call)<br>
                             • converts frame points to real-world coordinates<br>
                             • prints few data samples to standard output<br> 
                             • switches back to `MKE_STATE_IDLE` state

`demo_getframe_async.cpp`    Show the same as `demo_getframe_sync.cpp`, only 
                             it does so via asynchronous MkE C++ Client call.
 
`demo_pushframes.cpp`        Shows how to get point cloud data from sensor
                             using pushing method (sensor pushes data).<br>
                             It does these steps:<br>
                             • connects to the sensor <br>
                             • switches to `MKE_STATE_DEPTH_SENSOR` state<br>
                             • starts frame pushing<br>
                             • processes all incoming frames for a few seconds
                              (data samples are printed to standard output)<br>
                             • stops frame pushing<br>
                             • switches back to `MKE_STATE_IDLE` state

`demo_device_discovery.cpp`  Shows how to use the SSDP protocol
                             to discover MagikEye sensors on the 
                             local network. 

`test_latency.cpp`           Simple latency testing example.
------------------------------------------------------------------------------


## Integration example codes

Examples on how to integrate MkE API C++ Client to 3rd party libraries (PCL, Open3D and OpenCV) are separated 
in [client/cpp/mkeclient/src/integration_examples](mkeclient/src/integration_examples) directory, 
more information on those in:

* [client/cpp/doc/mkeclient_integration_examples.html](doc/mkeclient_integration_examples.html) ([mkeclient_integration_examples.md](doc/mkeclient_integration_examples.md))
* [client/cpp/doc/mkeclient_integration_examples.pdf](doc/mkeclient_integration_examples.pdf) 

## C++ Client tests

Tests for verification purposes of the MkE API C++ Client reside in the [client/cpp/mkeclient/src/tester](mkeclient/src/tester) directory. 
More information about these tests:

* [client/cpp/doc/mkeclient_tester.html](doc/mkeclient_tester.html) ([mkeclient_tester.md](doc/mkeclient_tester.md))
* [client/cpp/doc/mkeclient_tester.pdf](doc/mkeclient_tester.pdf) 

---

 © 2016-2022 MagikEye Inc.
