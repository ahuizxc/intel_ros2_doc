# Intel Ros2 Project Tutorial

## Overview

Intel Ros2 Project contains several ROS2 packages in object classification, detection, localization and tracking and SLAM.

## Package Lists

### [ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense)

* ros2_intel_realsense for using Intel RealSense cameras (D400 series) with ROS2.

### [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs)

* ros2_intel_movidius_ncs is a ROS2 wrapper for NC API of NCSDK, providing the following features:

    * A ROS2 service for object classification and detection of a static image file.

    * A ROS2 publisher for object classification and detection of a video stream from a RGB camera.

    * Demo applications to show the capabilities of ROS2 service and publisher.

    * Support multiple CNN models of Caffe and Tensorflow.

### [ros2_object_analytics](https://github.com/intel/ros2_object_analytics)

* ros2_object_analytics is a ROS2 wrapper for realtime object detection, localization and tracking. These packages aim to provide real-time object analyses over RGB-D camera inputs, enabling ROS developer to easily create amazing robotics advanced features, like intelligent collision avoidance and semantic SLAM.

### [ros2_object_map](https://github.com/intel/ros2_object_map)

* ros2_object_map is ROS2 package which designes to mark tag of objects on map when SLAM. It uses ros2_object_analytics for object detection.

### [ros2_moving_object](https://github.com/intel/ros2_moving_object)

* ros2_moving_object is addressing moving objects based on messages generated by Object Analytics ros2_object_analytics. ros2_moving_object delivers further analysis for the localized and tracked objects from Object Analytics by adding motion information, i.e., the velocity information about tracked objects. Such information can extend robot's ability of motion planing and collision avoidance.

## License

Copyright 2018 Intel Corporation

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this project except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

**Other names and brands may be claimed as the property of others*

###### Any security issue should be reported using process at https://01.org/security