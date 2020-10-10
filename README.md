# Unofficial implemnetation of VITAMIN-E (WIP)
* VITAMIN-E : VIsual Tracking And MappINg with Extremely Dense Feature Points (CVPR2019)
	* [https://staff.aist.go.jp/shuji.oishi/assets/projects/VITAMIN-E/index.html](https://staff.aist.go.jp/shuji.oishi/assets/projects/VITAMIN-E/index.html)
* **NOTE**
	* This project is ongoing, not done yet.
	* Not optimized yet and runs slowly.

***

## Prerequisites

Please install the following tools and libraries:

* CMake 3.10+
* C++ Compiler (C++11 supports) such as:
	* Visual C++ 2015+ (if Windows)
	* GCC 7+ (if Linux)
* OpenCV 4+ with VTK
	* core
	* highgui
	* calib3d
	* imgproc
	* features2d
	* viz (VTK required)
	* video

This program is confirmed running successfully in the following environments:

* Windows
	* Windows 10 x64
	* CMake 3.17.1
	* Visual Studio 2019
	* OpenCV 4.3.0 with VTK 8.0

***

## How to build on Windows

* Open command prompt.
* Run the following commands.
```
> dir /b
README.md
sources

> mkdir build
> cd build
> cmake ..\sources
```
* Open the generated .sln file in "build" folder by VisualStudio.
* Build "ALL_BUILD" in Release mode.

***

## How to run on Windows

### Arguments

* --video="C:\path\to\video"
* --calib="C:\path\to\calib.yml"
* --param="TBD"

```
> slam.exe --video="C:\path\to\video" --calib="C:\path\to\calib.yml" --param="TBD"
```

### Camera calibration file

* Pinhole camera model is available.
* Please do a calibration using a checkerboards as described in the following page:
	* https://docs.opencv.org/master/d4/d94/tutorial_camera_calibration.html
* Calibration results must be saved like the following:
```
%YAML 1.1
---
calib:
  numOfPoints: 70
  reprojError: 0.3507153023198588
width: 1280
height: 720
fx: 968.1090867
fy: 979.82648
cx: 637.876864
cy: 354.645136
```
