![](../../mke-banner.png "Magik Eye Banner")

# ROS2 MkE Point Cloud Nodes #

The MkE SDK provides C++ and Python implementations of ROS2 point cloud publisher nodes.

## C++ Publisher Node

The C++ implementation of ROS2 publisher node is called `mkeros2_node`. This node is capable of connecting to a MagikEye sensor via the MkE API and publishing its 3D data via a ROS topic. Note that the code depends on the C++ MkE API client library [libmkeclient](../cpp/README.html) which is also distributed as a part of this SDK.

* [client/ros2/doc/mkeros2_node.html](doc/mkeros2_node.html) ([mkeros2_node.md](doc/mkeros2_node.md))
* [client/ros2/doc/mkeros2_node.pdf](doc/mkeros2_node.pdf)

## Python Publisher Node

The Python implementation of ROS2 publisher node is called `pymkeros2_node`. This node is capable of connecting to a MagikEye sensor via the MkE API and publishing its 3D data via a ROS topic. Note that the code depends on the Python MkE API package [pymkeapi](../python/README.html) which is also distributed as a part of this SDK.

* [client/ros2/doc/pymkeros2_node.html](doc/pymkeros2_node.html) ([pymkeros2_node.md](doc/pymkeros2_node.md))
* [client/ros2/doc/pymkeros2_node.pdf](doc/pymkeros2_node.pdf)

---

 © 2016-2022 MagikEye Inc.
