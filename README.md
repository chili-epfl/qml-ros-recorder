qml-ros-recorder
===================

QML plugin for querying the ROS master for the list of available topics, and start/stop recording a selection of topics. Tested with Qt 5.10.0 on
the following:

  - Android 6.0.1 (arm-v7) built with SDK API 18 and NDK r10e on ArchLinux host
  - ArchLinux (rolling)

Prerequisites
-------------

 - If using Android, ROS libraries cross-compiled for arm-v7 and their headers. Follow the instructions [here](http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages) and adapt the `.pro` file using the `lib` and `include` directories found in `roscpp_android/output/target`.
 - The `rosbag_recorder` service. Follow instructions in [this](https://github.com/chili-epfl/rosbag-recorder) repository. Then start the service on your ROS master: `rosrun rosbag_recorder rosbag_recorder_server.py`.

build
-----

```
    $ mkdir build && cd build
    $ qmake ..
    $ make install
```

For Android, make sure to use the `qmake` binary from the arm-v7 Qt installation.
