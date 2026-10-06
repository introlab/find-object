# find-object

#### CI Latest

| | Build |
|---|---|
| Desktop | [![Linux](https://github.com/introlab/find-object/actions/workflows/cmake.yml/badge.svg)](https://github.com/introlab/find-object/actions/workflows/cmake.yml) [![Windows](https://github.com/introlab/find-object/actions/workflows/cmake-windows.yml/badge.svg)](https://github.com/introlab/find-object/actions/workflows/cmake-windows.yml) [![macOS](https://github.com/introlab/find-object/actions/workflows/cmake-macos.yml/badge.svg)](https://github.com/introlab/find-object/actions/workflows/cmake-macos.yml) |
| ROS 2 | [![ROS 2](https://github.com/introlab/find-object/actions/workflows/ros2.yml/badge.svg)](https://github.com/introlab/find-object/actions/workflows/ros2.yml) |

#### ROS Binaries

| | Distro | Ubuntu | Released | In apt | Build |
|---|---|---|---|---|---|
| ROS 1 | Noetic (EOL) | 20.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Fnoetic%2Fdistribution.yaml&query=%24.repositories.find_object_2d.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/noetic/distribution.yaml) | [![apt](https://img.shields.io/ros/v/noetic/find_object_2d?label=%20)](https://index.ros.org/p/find_object_2d/#noetic) |  |
| ROS 2 | Humble | 22.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Fhumble%2Fdistribution.yaml&query=%24.repositories.find_object_2d.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml) | [![apt](https://img.shields.io/ros/v/humble/find_object_2d?label=%20)](https://index.ros.org/p/find_object_2d/#humble) | [![build](http://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__find_object_2d__ubuntu_jammy_amd64__binary)](http://build.ros2.org/job/Hbin_uJ64__find_object_2d__ubuntu_jammy_amd64__binary/) |
| ROS 2 | Jazzy | 24.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Fjazzy%2Fdistribution.yaml&query=%24.repositories.find_object_2d.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml) | [![apt](https://img.shields.io/ros/v/jazzy/find_object_2d?label=%20)](https://index.ros.org/p/find_object_2d/#jazzy) | [![build](http://build.ros2.org/buildStatus/icon?job=Jbin_uN64__find_object_2d__ubuntu_noble_amd64__binary)](http://build.ros2.org/job/Jbin_uN64__find_object_2d__ubuntu_noble_amd64__binary/) |
| ROS 2 | Kilted | 24.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Fkilted%2Fdistribution.yaml&query=%24.repositories.find_object_2d.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/kilted/distribution.yaml) | [![apt](https://img.shields.io/ros/v/kilted/find_object_2d?label=%20)](https://index.ros.org/p/find_object_2d/#kilted) | [![build](http://build.ros2.org/buildStatus/icon?job=Kbin_uN64__find_object_2d__ubuntu_noble_amd64__binary)](http://build.ros2.org/job/Kbin_uN64__find_object_2d__ubuntu_noble_amd64__binary/) |
| ROS 2 | Lyrical | 26.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Flyrical%2Fdistribution.yaml&query=%24.repositories.find_object_2d.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/lyrical/distribution.yaml) | [![apt](https://img.shields.io/ros/v/lyrical/find_object_2d?label=%20)](https://index.ros.org/p/find_object_2d/#lyrical) |  |
| ROS 2 | Rolling | 26.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Frolling%2Fdistribution.yaml&query=%24.repositories.find_object_2d.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/rolling/distribution.yaml) | [![apt](https://img.shields.io/ros/v/rolling/find_object_2d?label=%20)](https://index.ros.org/p/find_object_2d/#rolling) |  |

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
