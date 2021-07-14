![](../../mke-banner.png "Magik Eye Banner")

# ROS1 MkE Point Cloud Nodes #

The MkE SDK provides C++ and Python implementations of ROS1 point cloud publisher nodes.

## C++ Publisher Node

The C++ implementation of ROS1 publisher node is called `mkeros1_node`. This node is capable of connecting to a MagikEye sensor via the MkE API and publishing its 3D data via a ROS topic. Note that the code depends on the C++ MkE API client library [libmkeclient](../cpp/README.html) which is also distributed as a part of this SDK.

* [client/ros1/doc/mkeros1_node.html](doc/mkeros1_node.html)
* [client/ros1/doc/mkeros1_node.pdf](doc/mkeros1_node.pdf)

## Python Publisher Node

The Python implementation of ROS1 publisher node is called `pymkeros1_node`. This node is capable of connecting to a MagikEye sensor via the MkE API and publishing its 3D data via a ROS topic. Note that the code depends on the Python MkE API package [pymkeapi](../python/README.html) which is also distributed as a part of this SDK.

* [client/ros1/doc/pymkeros1_node.html](doc/pymkeros1_node.html)
* [client/ros1/doc/pymkeros1_node.pdf](doc/pymkeros1_node.pdf)

---

 © 2016-2021 MagikEye Inc.
