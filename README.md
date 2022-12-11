# find-object

<table>
    <tbody>
        <tr>
           <td>Linux</td>
           <td><a href="https://github.com/introlab/find-object/actions/workflows/cmake.yml"><img src="https://github.com/introlab/find-object/actions/workflows/cmake.yml/badge.svg" alt="Build Status"/> <br> <a href="https://github.com/introlab/find-object/actions/workflows/ros1.yml"><img src="https://github.com/introlab/find-object/actions/workflows/ros1.yml/badge.svg" alt="Build Status"/> <br> <a href="https://github.com/introlab/find-object/actions/workflows/ros2.yml"><img src="https://github.com/introlab/find-object/actions/workflows/ros2.yml/badge.svg" alt="Build Status"/>
           </td>
        </tr>
        <tr>
           <td>Windows</td>
           <td><a href="https://ci.appveyor.com/project/matlabbe/find-object/branch/master"><img src="https://ci.appveyor.com/api/projects/status/hn51r6p5c0peqctb/branch/master?svg=true" alt="Build Status"/>
           </td>
        </tr>
     </tbody>
  </table>

## Standalone
Find-Object project, visit the [home page](http://introlab.github.io/find-object/) for more information.

## ROS1

### Install

Binaries:
```bash
sudo apt-get install ros-$ROS_DISTRO-find-object-2d
```

Source:

 * To include `xfeatures2d` and/or `nonfree` modules of OpenCV, to avoid conflicts with `cv_bridge`, build same OpenCV version that is used by `cv_bridge`. Install it in `/usr/local` (default).

```bash
cd ~/catkin_ws
git clone https://github.com/introlab/find-object.git src/find_object_2d
catkin_make
```

### Run
```bash
roscore
# Launch your preferred usb camera driver
rosrun uvc_camera uvc_camera_node
rosrun find_object_2d find_object_2d image:=image_raw
```
See [find_object_2d](http://wiki.ros.org/find_object_2d) for more information.

## ROS2

### Install

Binaries:
```bash
To come...
```

Source:

```bash
cd ~/ros2_ws
git clone https://github.com/introlab/find-object.git src/find_object_2d
colcon build
```

### Run
```bash
# Launch your preferred usb camera driver
ros2 launch realsense2_camera rs_launch.py
 
# Launch find_object_2d node:
ros2 launch find_object_2d find_object_2d.launch.py image:=/camera/color/image_raw
 
# Draw objects detected on an image:
ros2 run find_object_2d print_objects_detected --ros-args -r image:=/camera/color/image_raw
```
#### 3D Pose (TF)
A RGB-D camera is required. Example with Realsense D400 camera:
```bash
# Launch your preferred usb camera driver
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
 
# Launch find_object_2d node:
ros2 launch find_object_2d find_object_3d.launch.py \
   rgb_topic:=/camera/color/image_raw \
   depth_topic:=/camera/aligned_depth_to_color/image_raw \
   camera_info_topic:=/camera/color/camera_info
 
# Show 3D pose in camera frame:
ros2 run find_object_2d tf_example
```
See [find_object_2d](http://wiki.ros.org/find_object_2d) for more information (same parameters/topics are used between ROS1 and ROS2 versions).
