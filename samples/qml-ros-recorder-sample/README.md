qml-ros-recorder-sample
===========================

Example use of qml-ros-recorder. Follow the build
instructions in [the qml-ros README](../../README.md) before trying to run this sample. It is tested with
Qt 5.10.1 on the following:

  - Android 6.0.1 (arm-v7) built with SDK API 18 and NDK r10e on ArchLinux host

It can be launched by loading the project into QtCreator.


Prerequisites
-------------

 - ROS libraries cross-compiled for arm-v7 and their headers. Follow the instructions [here](http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages) and adapt the `.pro` file using the `lib` and `include` directories found in `roscpp_android/output/target`.
 - The `rosbag_recorder` service. Clone [this](https://github.com/chili-epfl/rosbag-recorder) repo inside your catkin workspace's `src` directory and run `catkin_make` from the root. Then add the `devel/include` directory to your compiler's search path (or copy it's contents to your ROS `include` directory). Also, start the service on your ROS master: `rosrun rosbag_recorder rosbag_recorder_server.py`.

build
-----

Make sure to use binaries from the arm-v7 Qt installation.

```
    $ mkdir build && cd build
    $ qmake ..
    $ make
```

An APK can be created using `androiddeployqt` from your arm-v7 Qt installation.
