---
layout: default
title: Find-Object
---

# Overview
<img src="https://raw.githubusercontent.com/introlab/find-object/master/src/resources/Find-Object.png"
 alt="Find-Object logo" title="Find-Object" width="120" />

Using a webcam:

1. Go "Edit" -> "Add object...",
2. Present an object,
3. Select the features extracted from the object, return to main screen,
4. Play ("Edit" -> "Start") and
5. See highlighted features corresponding to the object.

Features:

* You can change any parameters at runtime, make it easier to test feature detectors and descriptors without always recompiling.
* Detectors/descriptors supported (from OpenCV): BRIEF, Dense, FAST, GoodFeaturesToTrack, MSER, ORB, SIFT, STAR, SURF, FREAK and BRISK.
* Sample code with the OpenCV C++ interface [below](#code-example)...
* For an example of an application using SURF descriptors: see my project [RTAB-Map](http://introlab.github.io/rtabmap) (an appearance-based loop closure detector for SLAM).

# Screenshots

* Add object:
  <img src="https://raw.githubusercontent.com/wiki/introlab/find-object/doc/Add_object.png">

* Enjoy detections:
  <img src="https://raw.githubusercontent.com/wiki/introlab/find-object/doc/Find_object.png">

* Change algorithms' parameters (mapping OpenCV names), here using STAR detector and BRIEF descriptors:
  <img src="https://raw.githubusercontent.com/wiki/introlab/find-object/doc/Parameters.png">

# License

* If OpenCV is built **without the nonfree** module, Find-Object can be used under the permissive BSD License.
* If OpenCV is built **with the nonfree** module, Find-Object is free for research only because it depends on SURF and SIFT features. SIFT and SURF are not free for commercial use.
  * SURF noncommercial notice: <http://www.vision.ee.ethz.ch/~surf/download.html>
  * SIFT patent: <http://www.cs.ubc.ca/~lowe/keypoints/>

# Author

* [Mathieu Labbé](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php?title=Mathieu_Labbé&setlang=en)
  * Similar project: [RTAB-Map](http://introlab.github.io/rtabmap) (6-DOF RGB-D SLAM/scanning with a Kinect)

### Citing

{% raw %}
```
@misc{labbe11findobject,
   Author = {{Labb\'{e}, M.}},
   Howpublished = {\url{http://introlab.github.io/find-object}},
   Note = {accessed YYYY-MM-DD},
   Title = {{Find-Object}},
   Year = 2011
}
```
{% endraw %}

# News
October 2026

* Added Qt6 support
* Added OpenCV 5 support
* Added ros2 lyrical release

July 2015

* [New Version 0.6.0](https://github.com/introlab/find-object/releases/tag/0.6.0)
  * Added OpenCV 3 support (new feature detectors/descriptors are available)

May 2015

* Moving from [GoogleCode](http://find-object.googlecode.com) to GitHub!

February 2015

* New Version 0.5.1
  * Added Drag&Drop feature to drop image files into objects and scene panels.
  * Added Save/Load session in File menu. Everything (objects, features extracted, vocabulary and settings) of the current session is saved into a single file, and can be restored very fast.
  * Objects can be added/removed through TCP.
  * ROS: added `session_path` parameter to load a session on start.
  * Downloads:
    * [Find-Object-0.5.1-Source.zip](http://bit.ly/FindObject051zip) (Source)
    * [Find-Object-0.5.1-win64.exe](http://bit.ly/FindObject051win64exe) (Windows binaries, installer)
    * [Find-Object-0.5.1-Darwin.dmg](http://bit.ly/FindObject051dmg) (Mac OS X binaries)

August 2014

* New Version 0.5.0
  * Improved stability.
  * Console mode: see `$ find_object --help` on terminal.
  * Added **ASIFT**: check `Feature2D/Affine` parameter to activate. If you have many CPUs, you may want to increase `General/Threads` to 4.
  * Added library interface.
  * Added **JSON** output and [TCP request example](https://github.com/introlab/find-object/wiki/TcpRequest).
  * Downloads:
    * [Find-Object-0.5.0-Source.zip](http://bit.ly/1pOaPmR) (Source)
    * [Find-Object-0.5.0-Win64.exe](http://bit.ly/1wKTzUp) (Windows binaries, installer)
    * [Find-Object-0.5.0-Win64-cuda.exe](http://bit.ly/VRrzgY) (Windows binaries, installer, CUDA 6 required)
    * [Find-Object-0.5.0-Darwin.dmg](http://bit.ly/1tQlsFU) (Mac OS X binaries)
* I did a fork of the source code for GitHub and I will keep it synchronized with the svn here. It will be easier for you to customize the code with a fork on GitHub.
  * [GitHub for Find-Object](https://github.com/introlab/find-object)
* I posted a video on my another project [RTAB-Map](http://introlab.github.io/rtabmap) where Find-Object runs in parallel during the mapping (you can [try the demo here](http://wiki.ros.org/rtabmap_ros#Robot_mapping_with_Find-Object)). There are five books which can be detected, take a look:

  <a href="http://www.youtube.com/watch?v=o1GSQanY-Do" target="_blank"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/findobject.jpeg"
 alt="video" title="video"/></a>

[Older news](#older-news)

# Tutorials
### Find objects with a webcam
<a href="https://github.com/introlab/find-object/wiki/FindObjectsWithWebcam"><img src="https://raw.githubusercontent.com/wiki/introlab/find-object/doc/tutorial1/13.png" alt="Find objects with a webcam"></a>

### About changing parameters
<a href="https://github.com/introlab/find-object/wiki/ChangeParameters"><img src="https://raw.githubusercontent.com/wiki/introlab/find-object/doc/tutorial2/4.png" alt="About changing parameters"></a>

### Place recognition (likelihood computation)
<a href="https://github.com/introlab/find-object/wiki/InvertedSearch"><img src="https://raw.githubusercontent.com/wiki/introlab/find-object/doc/tutorial3/12.png" alt="Place recognition (likelihood computation)"></a>

### Detection of multiple occurrences of the same object
<a href="https://github.com/introlab/find-object/wiki/MultiDetection"><img src="https://raw.githubusercontent.com/wiki/introlab/find-object/doc/tutorial4/1.png" alt="Detection of multiple occurrences of the same object"></a>

### Beers wall (many objects detection)
<a href="https://github.com/introlab/find-object/wiki/BeersWall"><img src="https://raw.githubusercontent.com/wiki/introlab/find-object/doc/tutorial5/results1000.jpg" alt="Detection of multiple occurrences of the same object"></a>

### TCP request
<a href="https://github.com/introlab/find-object/wiki/TcpRequest"><img src="https://raw.githubusercontent.com/wiki/introlab/find-object/doc/tcp_request.png" alt="TCP request example"></a>

# Install

## Binaries (recommended)

* Windows, Mac OS X: See the [Releases](https://github.com/introlab/find-object/releases) page.
* Linux : See [From source](#from-source) below.

## ROS
 See [find_object_2d](https://github.com/introlab/find-object/tree/master#find_object_2d-ros-package) on GitHub.

## From source
### Requirements

* CMake
* Qt
* OpenCV
* Visual Studio or MinGW (only for Windows)

```bash
Ubuntu :
 $ sudo apt-get install cmake libqt4-dev libopencv2.4-dev
```

### Download

* Download ZIP file on top of this page or clone the latest version from GIT:

```bash
 $ git clone https://github.com/introlab/find-object.git
```

### Build

* Unix (Ubuntu, Mac OS X):

```bash
 $ cd find_object/build
 $ cmake -DCMAKE_BUILD_TYPE=Release ..
 $ make
 $ cd ../bin
 $ ./find_object
```
* Windows (Visual Studio):

```bash
 $ cd find_object/build

 (CMake 3)
 $ cmake -G"Visual Studio 10 2010 Win64" ..
 $ cmake --build . --config Release

 (CMake 2)
 $ cmake -G"NMake Makefiles" -DCMAKE_BUILD_TYPE=Release ..
 $ nmake

 $ cd ../bin
 $ Find-Object.exe
```
* Windows (MinGW):

```bash
 $ cd find_object/build
 $ cmake -G"MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release ..
 $ make
 $ cd ../bin
 $ Find-Object.exe
```

# Uninstall

* Windows:
  * Use the uninstaller.
  * For a full cleanup, you can remove the config file in `$USER_DIRECTORY$/Documents/FindObject`.
* Mac OS X:
  * Drag and drop the application in the trash.
  * For a full cleanup, you can remove the config file in `~/.find_object`.
* Ubuntu or Mac OS X "Unix" installation style:
  * Remove `$YOUR_INSTALL_PATH$/bin/find_object` (default path is `/usr/local/bin/find_object`).
  * For a full cleanup, you can remove the config file in `~/.find_object`.

```bash
 $ sudo rm /usr/local/bin/find_object
 $ rm -rf ~/.find_object
```

# Code example

* Here an example of features extraction (with OpenCV C++ interface) and nearest neighbor search with Flann library. This code can be found in this sample application [main.cpp](https://github.com/introlab/find-object/blob/master/example/main.cpp). Sample images from OpenCV : [box.png](https://raw.githubusercontent.com/introlab/find-object/master/bin/box.png) and [box_in_scene.png](https://raw.githubusercontent.com/introlab/find-object/master/bin/box_in_scene.png). Example results:

```bash
./find_object-example box.png box_in_scene.png
Loading images: 5 ms
Object: 198 keypoints detected in 12 ms
Scene: 355 keypoints detected in 27 ms
Object: 198 descriptors extracted in 20 ms
Scene: 355 descriptors extracted in 42 ms
Time creating FLANN index = 3 ms
Time nearest neighbor search = 3 ms
Time finding homography = 2 ms
Inliers=34 Outliers=12
```
<img src="https://raw.githubusercontent.com/wiki/introlab/find-object/doc/Box_example.png">

```cpp
// OpenCV stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography

//...

// LOAD IMAGES (as grayscale)
cv::Mat objectImg = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
cv::Mat sceneImg = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

std::vector<cv::KeyPoint> objectKeypoints;
std::vector<cv::KeyPoint> sceneKeypoints;
cv::Mat objectDescriptors;
cv::Mat sceneDescriptors;

////////////////////////////
// EXTRACT KEYPOINTS
////////////////////////////
// The detector can be any of (see OpenCV features2d.hpp):
// cv::FeatureDetector * detector = new cv::DenseFeatureDetector();
// cv::FeatureDetector * detector = new cv::FastFeatureDetector();
// cv::FeatureDetector * detector = new cv::GFTTDetector();
// cv::FeatureDetector * detector = new cv::MSER();
// cv::FeatureDetector * detector = new cv::ORB();
cv::FeatureDetector * detector = new cv::SIFT();
// cv::FeatureDetector * detector = new cv::StarFeatureDetector();
// cv::FeatureDetector * detector = new cv::SURF(600.0);
// cv::FeatureDetector * detector = new cv::BRISK();
detector->detect(objectImg, objectKeypoints);
detector->detect(sceneImg, sceneKeypoints);
delete detector;

////////////////////////////
// EXTRACT DESCRIPTORS
////////////////////////////
// The extractor can be any of (see OpenCV features2d.hpp):
// cv::DescriptorExtractor * extractor = new cv::BriefDescriptorExtractor();
// cv::DescriptorExtractor * extractor = new cv::ORB();
cv::DescriptorExtractor * extractor = new cv::SIFT();
// cv::DescriptorExtractor * extractor = new cv::SURF(600.0);
// cv::DescriptorExtractor * extractor = new cv::BRISK();
// cv::DescriptorExtractor * extractor = new cv::FREAK();
extractor->compute(objectImg, objectKeypoints, objectDescriptors);
extractor->compute(sceneImg, sceneKeypoints, sceneDescriptors);
delete extractor;

////////////////////////////
// NEAREST NEIGHBOR MATCHING USING FLANN LIBRARY (included in OpenCV)
////////////////////////////
cv::Mat results;
cv::Mat dists;
int k=2; // find the 2 nearest neighbors
if(objectDescriptors.type()==CV_8U)
{
	// Binary descriptors detected (from ORB or Brief)

	// Create Flann LSH index
	cv::flann::Index flannIndex(sceneDescriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);

	// search (nearest neighbor)
	flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
}
else
{
	// assume it is CV_32F
	// Create Flann KDTree index
	cv::flann::Index flannIndex(sceneDescriptors, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);

	// search (nearest neighbor)
	flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
}

// Conversion to CV_32F if needed
if(dists.type() == CV_32S)
{
	cv::Mat temp;
	dists.convertTo(temp, CV_32F);
	dists = temp;
}


////////////////////////////
// PROCESS NEAREST NEIGHBOR RESULTS
////////////////////////////
// Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
float nndrRatio = 0.8;
std::vector<cv::Point2f> mpts_1, mpts_2; // Used for homography
std::vector<int> indexes_1, indexes_2; // Used for homography
std::vector<uchar> outlier_mask;  // Used for homography
for(unsigned int i=0; i<objectData.rows; ++i)
{
	// Check if this descriptor matches with those of the objects
	// Apply NNDR
	if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 && dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
	{
		mpts_1.push_back(objectKeypoints.at(i).pt);
		indexes_1.push_back(i);

		mpts_2.push_back(sceneKeypoints.at(results.at<int>(i,0)).pt);
		indexes_2.push_back(results.at<int>(i,0));
	}
}

// FIND HOMOGRAPHY
int nbMatches = 8;
if(mpts_1.size() >= nbMatches)
{
	cv::Mat H = findHomography(mpts_1,
			mpts_2,
			cv::RANSAC,
			1.0,
			outlier_mask);
	// Do what you want with the homography (like showing a rectangle)
	// The "outlier_mask" contains a mask representing the inliers and outliers.
	// ...
}

```

# Older news
July 2014

* **ROS** new feature
  * When using Kinect-like sensors, you can set **find_object_2d** node in a mode that  publishes 3D positions of the objects over TF. See 3D position of objects on ROS [find_object_2d](http://wiki.ros.org/find_object_2d) page.
  <img src="https://raw.githubusercontent.com/wiki/introlab/find-object/doc/ros_find_object_3d.png" width="600">

June 2014

* New Version 0.4.6
  * Fixed a bug when changing from float descriptors to binary descriptors (issue 22).
  * Fixed slow BRISK issue... now running under 10 ms (issue 21).
  * Fixed missing libOpenNI.dylib on Mac (issue 25).
  * Downloads:
    * [Find-Object-0.4.6-Source.zip](http://bit.ly/1pnrmPW) (Source)
    * [Find-Object-0.4.6-Win64.exe](http://bit.ly/VrDu5B) (Windows binaries, installer)
    * [Find-Object-0.4.6-Win64-cuda6.exe](http://bit.ly/VrCZsc) (Windows binaries, installer, CUDA 6 required)
    * [Find-Object-0.4.6-Darwin.dmg](http://bit.ly/1wCc0Hi) (Mac OS X binaries)
* New Version 0.4.5
  * Added GPU-SURF support when OpenCV is built with CUDA. See parameter "Feature2D -> SURF -> SURF_gpu".
  * Downloads:
    * [Find-Object-0.4.5-Source.zip](http://bit.ly/1qpURA3) (Source)
    * [Find-Object-0.4.5-win64-cuda6.exe](http://bit.ly/1oCcBbL) (Windows 64 bits binaries, installer, [CUDA 6](https://developer.nvidia.com/cuda-downloads) driver required)
* New Version 0.4.4
  * Added bag-of-words (BOW) matching using an **incremental vocabulary**.
    * New tutorial [Beers wall](https://github.com/introlab/find-object/wiki/BeersWall) for detection of many objects.
    * Updated tutorial [Place recognition](https://github.com/introlab/find-object/wiki/InvertedSearch).
  * Downloads:
    * [Find-Object-0.4.4-Darwin.dmg](http://bit.ly/1n4p9mU) (Mac OS X binaries)
    * [Find-Object-0.4.4-Source.zip](http://bit.ly/1xFFOV3) (Source)
    * [Find-Object-0.4.4-Win32.exe](http://bit.ly/1nwa6E5) (Windows binaries, installer)
    * [Find-Object-0.4.4-Win32.zip](http://bit.ly/1oS4a87) (Windows binaries)

May 2014

* New Version 0.4.3
  * Added **TCP/IP interface** to publish information about detected objects on a port.
  * Updated tutorial [FindObjectsWithWebcam](https://github.com/introlab/find-object/wiki/FindObjectsWithWebcam#TCP_information) with TCP connection example.
  * Downloads:
    * [Find-Object-0.4.3-Darwin.dmg](http://bit.ly/1fN33Xe) (Mac OS X binaries)
    * [Find-Object-0.4.3-Source.zip](http://bit.ly/RkSGyY) (Source)
    * [Find-Object-0.4.3-Win32.exe](http://bit.ly/1kJ7ikd) (Windows binaries, installer)
    * [Find-Object-0.4.3-Win32.zip](http://bit.ly/1lVAiIj) (Windows binaries)

April 2014

* New Version 0.4.2
  * Updated ros-pkg for ROS Hydro (Catkin build).
  * Mouse over parameters' label to show a description.
  * **Multi-threading** : See `General/threads` parameter.
  * Detection of **multiple occurrences of the same object** : See `General/multiDetection` parameter and tutorial [MultiDetection](https://github.com/introlab/find-object/wiki/MultiDetection).
  * Added save/load settings in File menu.
  * Downloads:
    * [Find-Object-0.4.2-Darwin.dmg](http://bit.ly/1etchYj) (Mac OS X binaries)
    * [Find-Object-0.4.2-Source.zip](http://bit.ly/1etcxXl) (Source)
    * [Find-Object-0.4.2-Win32.exe](http://bit.ly/1ne1qkL) (Windows binaries, installer)
    * [Find-Object-0.4.2-Win32.zip](http://bit.ly/1n1CPlP) (Windows binaries)

February 2013

* New version 0.4.1
  * Updated binary/source releases with OpenCV 2.4.3. **FREAK** and **BRISK** descriptors are now available.

August 2012

* New version 0.4.0
  * Added camera controls to browse frames of a video file or a directory of images.
  * New **inverted search mode** for likelihood computation (see this [InvertedSearch tutorial](https://github.com/introlab/find-object/wiki/InvertedSearch)).
  * Added in the "Parameters" panel a new group for the choice of the FLANN nearest neighbor strategy (KD-Tree, K-Means, Linear, composite, auto-tuned). All FLANN parameters can be modified.
  * Added 3 tutorials [below](#tutorials).

May 2012

* New version 0.3.0.
  * Updated to use OpenCV 2.4 and ROS Fuerte. Some feature detectors/descriptors were updated with new/removed/modified parameters from OpenCV 2.4.
  * Updated example [below](#code-example) accordingly to changes from OpenCV 2.4.
  * If you upgrade from a previous version, it is recommended to execute "Edit->Restore all default settings" to clean the parameters.

February 2012

* New version 0.2.2
  * Support still images processing (png, jpg, bmp, tiff). Objects can be added from image files (Edit->Add objects from files...) and a scene can be loaded from an image file (Edit->Load scene from file...).
  * Objects can be updated more easily when parameters are changed (button "Update objects"). You can also set "true" parameter General/autoUpdateObjects to update objects for each change "on the fly".
  * Modified how objects are saved/loaded, now to/from a directory of images.
  * Camera can be also set to use a video file (avi), see "Edit->Camera from video file...".
  * ROS package's name is changed to find_object_2d, to differ from existing find_object package on the ROS's [wiki](http://www.ros.org/wiki/find_object).

January 2012

* New version 0.2.1
  * Fixed a crash on Windows when adding an object (r67).
  * New parameter `Homography/homographyComputed` to detect outliers using RANSAC (default true). It is the same behavior as before, but it can be now easily deactivated to show all matched features. With the ROS package, this parameter must be on to publish objects detected.
  * New parameters `NN/minDistanceUsed` and `NN/minDistance` can be used instead of/or with the NNDR criterion for nearest neighbor matching. To accept a match, the distance with the nearest neighbor must be < minDistance. New statistics are added in Statistics panel to show the min and max matched distances, to better set this parameter across the different descriptors. For info when using NNDR, a match is accepted if the distance with the nearest neighbor is < nndrRatio X distance with the second nearest neighbor (see [Lowe](http://www.google.com/url?sa=t&rct=j&q=distinctive%20image%20features%20from%20scale-invariant%20keypoints&source=web&cd=1&ved=0CCEQFjAA&url=http%3A%2F%2Fwww.cs.ubc.ca%2F~lowe%2Fpapers%2Fijcv04.pdf&ei=bvoET9jNLcne0QH336mHAg&usg=AFQjCNExaFk5YsadEaL01y1K6FRDDh5u3g) (figure 11) to know how to adjust this parameter).
  * ROS package: the full homography is sent with objects detected (a more general way than only sending the center position of the objects). See [find_object_2d](http://www.ros.org/wiki/find_object_2d) for more details.
