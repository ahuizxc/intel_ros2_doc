# Package Instruction

## ros2_intel_realsense

### 1 Overview

These are packages for using Intel RealSense cameras (D400 series) with ROS2.

### 2 Running the demo

#### 2.1 Start the camera node

To start the camera node in ROS2, plug in the camera, then type the following command:

```bash
# To launch with "ros2 run"
$ ros2 run realsense_ros2_camera realsense_ros2_camera
# OR, to invoke the executable directly
$ realsense_ros2_camera
```

This will stream all camera sensors and publish on the appropriate ROS2 topics. PointCloud2 is enabled by default, till we provide ROS2 python launch options.

#### 2.2 View camera data

To start the camera node in ROS2 and view the depth pointcloud in rviz via [ros1_bridge](https://github.com/ros2/ros1_bridge/blob/master/README.md):

```bash
# firstly self-build ros1_bridge, than refer to section "Example 1b: ROS 2 talker and ROS 1 listener"

# in console #1 launch roscore
$ source /opt/ros/kinetic/setup.bash
$ roscore

# in console #2 launch ros1_bridge
$ source /opt/ros/kinetic/setup.bash
$ cd ~/ros2_ws
$ source ./install/local_setup.bash
$ export ROS_MASTER_URI=http://localhost:11311
$ ros2 run ros1_bridge dynamic_bridge

# in console #3 launch rviz
$ source /opt/ros/kinetic/setup.bash
$ rosrun rviz rviz -d ~/ros2_ws/src/ros2_intel_realsense/realsense_ros2_camera/rviz/ros2.rviz

# in console #4 launch realsense_ros2_camera
$ source ~/ros2_ws/install/local_setup.bash
$ realsense_ros2_camera
```

This will launch [RViz](http://wiki.ros.org/rviz) and display the five streams: color, depth, infra1, infra2, pointcloud.

**NOTE:** In case PointCloud2 stream is not observed, try stop the "realsense_ros2_camera" and re-launch this node from console #4. This's a known issue and workaround is made (right fixing in ros1_bridge, details discussed in [ROS discourse](https://discourse.ros.org/t/ros1-bridge-failed-to-pass-tf-static-message-when-subscribed-from-rviz/3863)).

**NOTE:** Visulization in ROS2 pending on [rviz2](https://github.com/ros2/rviz).

![realsense_ros2_camera visualization results](https://github.com/intel/ros2_intel_realsense/raw/master/realsense_ros2_camera/rviz/ros2_rviz.png "realsense_ros2_camera visualization results")

### 3 Interfaces

[/camera/depth/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[/camera/color/image_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[/camera/infra1/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[/camera/infra2/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[/camera/depth/color/points](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)

### 4 Known Issues

* This ROS2 node does not currently provide any dynamic reconfigure support for camera properties/presets.
* We support Ubuntu Linux Xenial Xerus 16.04 on 64-bit, but not support Mac OS X 10.12 (Sierra) and Windows 10 yet.

### 5 TODO

A few features to be ported from the latest realsense_ros_camera v2.0.2

* RGB-D point cloud (depth_registered)

* Preset/Controls

## ros2_message_filters

### 1 Overview

`ros2_message_filters` blends various messages based on the conditions that filter needs to met and derives from ROS2 porting of [ROS message_filters](http://wiki.ros.org/message_filters). It collects commonly used message "filtering" algorithms into a common space. A message filter is defined as something which a message arrives into and may or may not be spit back out of at a later point in time. 

### 2 Running the test

- Run unit tests
```
$ament test src/ros2/ros2_message_filters

1: Test timeout computed to be: 60
1: -- run_test.py: invoking following command in '/home/jwang-robot/ros2_ws/src/ros2/message_filters':
1:  - /home/jwang-robot/ros2_ws/build/message_filters/message_filters-test_simple --gtest_output=xml:/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-test_simple.gtest.xml
1: [==========] Running 2 tests from 1 test case.
1: [----------] Global test environment set-up.
1: [----------] 2 tests from SimpleFilter
1: [ RUN      ] SimpleFilter.callbackTypes
1: [       OK ] SimpleFilter.callbackTypes (0 ms)
1: [ RUN      ] SimpleFilter.oldRegisterWithNewFilter
1: [       OK ] SimpleFilter.oldRegisterWithNewFilter (0 ms)
1: [----------] 2 tests from SimpleFilter (0 ms total)
1: 
1: [----------] Global test environment tear-down
1: [==========] 2 tests from 1 test case ran. (0 ms total)
1: [  PASSED  ] 2 tests.
1: -- run_test.py: return code 0
1: -- run_test.py: inject classname prefix into gtest result file '/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-test_simple.gtest.xml'
1: -- run_test.py: verify result file '/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-test_simple.gtest.xml'
 1/10 Test  #1: message_filters-test_simple ....................   Passed    0.09 sec
test 2
      Start  2: message_filters-msg_cache_unittest

2: Test timeout computed to be: 60
2: -- run_test.py: invoking following command in '/home/jwang-robot/ros2_ws/src/ros2/message_filters':
2:  - /home/jwang-robot/ros2_ws/build/message_filters/message_filters-msg_cache_unittest --gtest_output=xml:/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-msg_cache_unittest.gtest.xml
2: [==========] Running 5 tests from 1 test case.
2: [----------] Global test environment set-up.
2: [----------] 5 tests from Cache
2: [ RUN      ] Cache.easyInterval
2: [       OK ] Cache.easyInterval (0 ms)
2: [ RUN      ] Cache.easySurroundingInterval
2: [       OK ] Cache.easySurroundingInterval (0 ms)
2: [ RUN      ] Cache.easyUnsorted
2: [       OK ] Cache.easyUnsorted (0 ms)
2: [ RUN      ] Cache.easyElemBeforeAfter
2: [       OK ] Cache.easyElemBeforeAfter (0 ms)
2: [ RUN      ] Cache.eventInEventOut
2: [       OK ] Cache.eventInEventOut (0 ms)
2: [----------] 5 tests from Cache (0 ms total)
2: 
2: [----------] Global test environment tear-down
2: [==========] 5 tests from 1 test case ran. (0 ms total)
2: [  PASSED  ] 5 tests.
2: -- run_test.py: return code 0
2: -- run_test.py: inject classname prefix into gtest result file '/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-msg_cache_unittest.gtest.xml'
2: -- run_test.py: verify result file '/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-msg_cache_unittest.gtest.xml'
 2/10 Test  #2: message_filters-msg_cache_unittest .............   Passed    0.09 sec
test 3
      Start  3: message_filters-test_chain

3: Test timeout computed to be: 60
3: -- run_test.py: invoking following command in '/home/jwang-robot/ros2_ws/src/ros2/message_filters':
3:  - /home/jwang-robot/ros2_ws/build/message_filters/message_filters-test_chain --gtest_output=xml:/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-test_chain.gtest.xml
3: [==========] Running 8 tests from 1 test case.
3: [----------] Global test environment set-up.
3: [----------] 8 tests from Chain
3: [ RUN      ] Chain.simple
3: [       OK ] Chain.simple (0 ms)
3: [ RUN      ] Chain.multipleFilters
3: [       OK ] Chain.multipleFilters (0 ms)
3: [ RUN      ] Chain.addingFilters
3: [       OK ] Chain.addingFilters (1 ms)
3: [ RUN      ] Chain.inputFilter
3: [       OK ] Chain.inputFilter (0 ms)
3: [ RUN      ] Chain.nonSharedPtrFilter
3: [       OK ] Chain.nonSharedPtrFilter (0 ms)
3: [ RUN      ] Chain.retrieveFilter
3: [       OK ] Chain.retrieveFilter (0 ms)
3: [ RUN      ] Chain.retrieveFilterThroughBaseClass
3: [       OK ] Chain.retrieveFilterThroughBaseClass (0 ms)
3: [ RUN      ] Chain.retrieveBaseClass
3: [       OK ] Chain.retrieveBaseClass (0 ms)
3: [----------] 8 tests from Chain (1 ms total)
3: 
3: [----------] Global test environment tear-down
3: [==========] 8 tests from 1 test case ran. (1 ms total)
3: [  PASSED  ] 8 tests.
3: -- run_test.py: return code 0
3: -- run_test.py: inject classname prefix into gtest result file '/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-test_chain.gtest.xml'
3: -- run_test.py: verify result file '/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-test_chain.gtest.xml'
 3/10 Test  #3: message_filters-test_chain .....................   Passed    0.09 sec
test 4
...
...
...
100% tests passed, 0 tests failed out of 10

Label Time Summary:
gtest     =  16.15 sec (9 tests)
pytest    =   0.54 sec (1 test)

Total Test time (real) =  16.69 sec

```

- Run fuzz tests (`disabled by default`)
```
$./build/message_filters/message_filters-test_fuzz

[==========] Running 3 tests from 3 test cases.
[----------] Global test environment set-up.
[----------] 1 test from TimeSequencer
[ RUN      ] TimeSequencer.fuzz_sequencer
[       OK ] TimeSequencer.fuzz_sequencer (5118 ms)
[----------] 1 test from TimeSequencer (5118 ms total)

[----------] 1 test from TimeSynchronizer
[ RUN      ] TimeSynchronizer.fuzz_synchronizer
[       OK ] TimeSynchronizer.fuzz_synchronizer (5013 ms)
[----------] 1 test from TimeSynchronizer (5013 ms total)

[----------] 1 test from Subscriber
[ RUN      ] Subscriber.fuzz_subscriber
[       OK ] Subscriber.fuzz_subscriber (5010 ms)
[----------] 1 test from Subscriber (5010 ms total)

[----------] Global test environment tear-down
[==========] 3 tests from 3 test cases ran. (15142 ms total)
[  PASSED  ] 3 tests.
```

### 3 Interfaces

- Filter Pattern
  - `registerCallback(cb)`
  - `connectInput(filter)`
- Subscriber
  - `subscribe(nodep, topic, 1, myCallback)`
- Time Synchronizer
  - Input : 
    - C++: Up to 9 separate filters, each of which is of the form `void callback(const std::shared_ptr<M const>&)`. The number of filters supported is determined by the number of template arguments the class was created with. 
    - Python: N separate filters, each of which has signature `callback(msg)`. 
  - Output : 
    - C++: For message types M0..M8, `void callback(const std::shared_ptr<M0 const>&`, ..., `const std::shared_ptr<M8 const>&)`. The number of parameters is determined by the number of template arguments the class was created with.
    - Python: `callback(msg0.. msgN)`. The number of parameters is determined by the number of template arguments the class was created with.

```
  #include <message_filters/subscriber.h>
  #include <message_filters/time_synchronizer.h>
  #include <sensor_msgs/Image.h>
  #include <sensor_msgs/CameraInfo.h>

  using namespace sensor_msgs;
  using namespace message_filters;
    
  void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
  {
     // Solve all of perception here...
  }
  
  int main(int argc, char** argv)
  {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("test_node");
    Subscriber<Image> image_sub(nh.get(), "image");
    Subscriber<CameraInfo> info_sub(nh.get(), "camera");
    TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);
    sync.registerCallback(std::bind(&callback, _1, _2));
    rclcpp::spin(nh);
    return 0;
   }
```


### 4 Known issues
* python not support headless message
* Not verify with Windows and OS X environment and there may be some errors

### 5 TODO

--

## ros2_intel_movidius_ncs

### 1 Overview

The Movidius™ Neural Compute Stick ([NCS](https://developer.movidius.com/)) is a tiny fanless deep learning device that you can use to learn AI programming at the edge. NCS is powered by the same low power high performance Movidius™ Vision Processing Unit ([VPU](https://www.movidius.com/solutions/vision-processing-unit)) that can be found in millions of smart security cameras, gesture controlled drones, industrial machine vision equipment, and more.  

This project is a ROS2 wrapper for NC API of [NCSDK](https://movidius.github.io/ncsdk/), providing the following features:

* A ROS2 service for object classification and detection of a static image file

* A ROS2 publisher for object classification and detection of a video stream from a RGB camera

* Demo applications to show the capabilities of ROS2 service and publisher

* Support multiple CNN models of Caffe and Tensorflow

### 2 Running the Demo

#### 2.1 Classification

##### 2.1.1 Supported CNN Models

####### *Table1*
|CNN Model|Framework|Usage|
|:-|:-|:-|
|AlexNet|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#alexnet)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#alexnet)|
|GoogLeNet|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#googlenet)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#googlenet)|
|SqueezeNet|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#squeezenet)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#squeezenet)|
|Inception_v1|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#inception_v1)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#inception_v1)|
|Inception_v2|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#inception_v2)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#inception_v2)|
|Inception_v3|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#inception_v3)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#inception_v3)|
|Inception_v4|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#inception_v4)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#inception_v4)|
|MobileNet|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#mobilenet)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#mobilenet)|

##### 2.1.2 Classification Result with GoogLeNet

![classification with googlenet](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/data/results/googlenet_dog.png "classification with googlenet")

##### 2.1.3 Running the Demo

* [Static Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md)
* [Video Streaming](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md)

#### 2.2 Detection

##### 2.2.1 Supported CNN Models

|CNN Model|Framework|Usage|
|:-|:-|:-|
|MobileNetSSD(Recommended)|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_detection.md#mobilenet_ssd)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_detection.md#mobilenet_ssd)|
|TinyYolo_v1|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_detection.md#tinyyolo_v1)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_detection.md#tinyyolo_v1)|

##### 2.2.2 Detection Result with MobileNetSSD
![detection with mobilenetssd](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/data/results/mobilenetssd_car_bicycle.png "detection with mobilenetssd")

##### 2.2.3 Running the Demo

* [Static Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_detection.md)
* [Video Streaming](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_detection.md)

### 3 Interfaces

#### 3.1 Topic

Classification: ```/movidius_ncs_nodelet/classified_objects```  
Detection: ```/movidius_ncs_nodelet/detected_objects```

#### 3.2 Service

Classification: ```/movidius_ncs_image/classify_object```  
Detection: ```/movidius_ncs_image/detect_object```

### 4 Known Issues

* Only absolute path of image file supported in image inference demo
* Only test on RealSense D400 series camera

### 5 TODO

* Keep synchronized with [ROS NCS Package](https://github.com/intel/ros_intel_movidius_ncs/tree/master)

## ros2_object_analytic

### 1 Overview

Object Analytics (OA) is ROS2 wrapper for realtime object detection, localization and tracking.
These packages aim to provide real-time object analyses over RGB-D camera inputs, enabling ROS developer to easily create amazing robotics advanced features, like intelligent collision avoidance and semantic SLAM. It consumes [sensor_msgs::PointClould2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) data delivered by RGB-D camera, publishing topics on [object detection](https://github.com/intel/ros2_object_msgs), [object tracking](https://github.com/intel/ros2_object_analytics/tree/master/object_analytics_msgs), and [object localization](https://github.com/intel/ros2_object_analytics/object_analytics_msgs) in 3D camera coordination system.

OA keeps integrating with various "state-of-the-art" algorithms.

* Object detection offload to VPU, Intel Movidius NCS, with MobileNet SSD model and Caffe framework.

### 2 Running the demo

#### Step 1. *[In terminal 1]* Launch Realsense camera node
  
```bash
# Terminal 1:
. <install-space-with-realsense-ros2-camera>/local_setup.bash
realsense_ros2_camera
```

#### Step 2. *[In terminal 1]* Launch NCS and OA node

```bash
# Terminal 2
. <install-space-with-object-analytics-launch>/local_setup.bash
echo -e "param_file: mobilenetssd.yaml\ninput_topic: /object_analytics/rgb" > `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/config/default.yaml
launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/analytics_movidius_ncs.py
```

#### Step 3. *[In terminal 1]* Launch OA Rviz

```bash
# Terminal 3
. <install-space-with-object-analytics-launch>/local_setup.bash
launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/object_rviz.py
```

### 3 Interfaces

#### 3.1 Subscribed topics
  /movidius_ncs_stream/detected_objects ([object_msgs::msg::ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))

#### 3.2 Published topics
  /object_analytics/rgb ([sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg))

  /object_analytics/pointcloud ([sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg))

  /object_analytics/localization ([object_analytics_msgs::msg::ObjectsInBoxes3D](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/ObjectsInBoxes3D.msg))

  /object_analytics/tracking ([object_analytics_msgs::msg::TrackedObjects](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/TrackedObjects.msg))


### 3.3 Customize launch
  By default, object analytics will launch both tracking and localization features, but either tracking or localization or both can be dropped. Detailed please refer comments embedded in launch file.

### 4 Known issues

--

### 5 TODO

--

## ros2_object_map

### 1 Introduction
ros2_object_map is ROS2 package which designes to mark tag of objects on map when SLAM. It uses [ros2_object_analytics](https://github.com/intel/ros2_object_analytics) for object detection.

![Architecture of Object Map](https://github.intel.com/otc-rse/ros2_object_map/blob/dev/object_map/object_map/ObjectMap.PNG "architecture of object map")

### 2 Running the demo

####  Step 1. *[In terminal 1]* Launch Realsense Camera node

```bash
# terminal 1 
source ~/ros2_ws/install/local_setup.bash
realsense_ros2_camera
```

#### Step 2. *[In terminal 2]* Launch object_analytics node

```bash
# terminal 2
source ~/ros2_ws/install/local_setup.bash
launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/analytics_movidius_ncs.py
```

#### Step 3. *[In terminal 3]* Launch ros2_object_map node

```bash
# terminal 3
source source ~/ros2_ws/install/local_setup.bash
ros2 run object_map object_map_node
```

#### Step 4. *[In terminal 4]* Launch ROS1 roscore

```bash
# terminal 4
source /opt/ros/kinetic/setup.bash
roscore
```

#### Step 5. *[In terminal 5]* Launch ROS1 bridge

```bash
# terminal 5
source /opt/ros/kinetic/setup.bash
source ros2_ws/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge

```

#### Step 6. *[In terminal 6]* Launch ROS1 Rviz

``` bash
# terminal 6
source /opt/ros/kinetic/setup.bash
roslaunch turtlebot_rviz_launchers view_robot.launch

within rviz gui, click "Add", and select "MarkerArray", then input "/object_map/Markers" into "Marker Topic"
```

### 3 Interfaces

#### 3.1 Topic

  * ```/object_map/Markers``` : Publish MarkerArray on RVIZ
  * ```/object_map/map_save``` : Subscribe map_save topic to save object maps
  * ```/movidius_ncs_stream/detected_objects```: Subscribe ObjectsInBoxes from object_analytics
  * ```/object_analytics/tracking```: Subscribe TrackedObjects from object_analytics
  * ```/object_analytics/localization```: Subscribe ObjectsInBoxes3D from object_analytics

#### 3.2 Save object map

```bash
ros2 topic pub --once /object_map/map_save std_msgs/Int32 -1

```

### 4 Known Issues

#### * Map tag cannot be correctly displayed in Rviz while robot is moving

reason: tf2 python api is not supported in ROS2 currrently

next step: will implement it while tf2-python api is ready in ROS2  

#### * Configure File is not supported 

reason: yaml configure file and dynamic configure file are not supported in ROS2 currently

next step: will implement it while it is ready in next release of ROS2

### 5 TODO

--

## ros2_moving_object

### 1. Overview

Moving Object component is addressing moving objects based on messages generated by
Object Analytics [ros2_object_analytics](https://github.com/intel/ros2_object_analytics).
ros2_moving_object delivers further analysis for the localized and tracked objects from Object Analytics by adding **motion information**, i.e., the **velocity** information about tracked objects. Such information can extend robot's ability of motion planing and collision avoidance.

Thanks to [ros2_object_analytics](https://github.com/intel/ros2_object_analytics) and [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs) to provide an AI solution for object detection, tracking and localization. See [the umbrella wiki page](http://wiki.ros.org/intelrosproject) to learn the hierarchical data flow and overview description for the related components.

This component involves 2 ROS2 packages:
- **moving_object**: the main package covering logic of moving object analysis and information generation.
- **moving_object_msgs**: the message package storing the motion information of moving objects and published into ROS2 system.

### 2. Running the demo

  #### Step 1. *[In terminal 1]* Launch realsense camera node.

```bash
source </ros2/install/dir>/local_setup.bash
source </my/overlay_ws/dir>/install/local_setup.bash
realsense_ros2_camera
```

#### Step 2. *[In terminal 2]* Launch object analysis node.

```bash
source </ros2/install/dir>/local_setup.bash
source </my/overlay_ws/dir>/install/local_setup.bash
echo -e "param_file: alexnet.yaml\ninput_topic: /object_analytics/rgb > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml"
launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/analytics_movidius_ncs.py
```

#### Step 3. *[In terminal 3]* Launch moving object node.
```bash
source </ros2/install/dir>/local_setup.bash
source </my/overlay_ws/dir>/install/local_setup.bash
ros2 run moving_object moving_object
```

### 3. Interfaces

ros2_moving_object package publishes some messages to indicate different status/data.
 - **/moving\_object/moving\_objects** merges info from the 3 input messages into one message and calculating (on demand) the velocity info of the tracked moving objects.

### 4. Known issues

--

### 5. TODO

--