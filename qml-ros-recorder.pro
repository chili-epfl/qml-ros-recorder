QT += qml quick network gui

CONFIG += qt plugin c++11
CONFIG -= android_install

linux:!android{
    CONFIG += link_pkgconfig
    _ROSPATH = "/opt/ros/$$(ROS_DISTRO)"
    isEmpty(_ROSPATH){
        message("ROS DISTRO" not detected...)
    }else{
        message(" found /opt/ros/$(ROS_DISTRO)")
        message(" Proceeding with the config ...")
        INCLUDEPATH += "/opt/ros/$$(ROS_DISTRO)/include"
        INCLUDEPATH += /home/wafa/catkin_ws/install/include
        LIBS += -L"/opt/ros/$$(ROS_DISTRO)/lib"
        LIBS += -L"/home/wafa/catkin_ws/install/lib/"
        LIBS += -lroscpp -lboost_signals -lboost_filesystem -lrosconsole
        LIBS += -lrosconsole_backend_interface -lboost_regex -lxmlrpcpp -ldynamic_reconfigure_config_init_mutex -lroscpp_serialization -lrostime -lboost_date_time -lcpp_common -lboost_system -lboost_thread -lconsole_bridge -ltinyxml -lclass_loader -lPocoFoundation -lroslib -lboost_iostreams -lnodeletlib -lbondcpp -luuid -lrosbag -lrosbag_storage -lboost_program_options -lroslz4 -llz4 -ltopic_tools -lactionlib -lmessage_filters -lrosconsole_bridge -lrospack

    }
}
android {
    _ROS_ANDROID_NDK = "$$(ROS_ANDROID_NDK)"
    isEmpty(_ROS_ANDROID_NDK){
        message("ROS_ANDROID_NDK" not detected...)
    }else{
        message(" found $$(ROS_ANDROID_NDK)")
        INCLUDEPATH += "$$(ROS_ANDROID_NDK)/roscpp_android/output/target/include"
        LIBS += -L "$$(ROS_ANDROID_NDK)/roscpp_android/output/target/lib/"
        QT += androidextras
        LIBS += -lrosconsole_print
        LIBS += -lroscpp -lboost_signals -lboost_filesystem -lrosconsole
        LIBS += -lrosconsole_backend_interface -lboost_regex -lxmlrpcpp -ldynamic_reconfigure_config_init_mutex -lroscpp_serialization -lrostime -lboost_date_time -lcpp_common -lboost_system -lboost_thread -lconsole_bridge -ltinyxml -lclass_loader -lPocoFoundation -lroslib -lboost_iostreams -lnodeletlib -lbondcpp -luuid -lrosbag -lrosbag_storage -lboost_program_options -lroslz4 -llz4 -ltopic_tools -lactionlib -lmessage_filters -lrosconsole_bridge -lrospack
    }
}

HEADERS += \
    src/RosRecorderPlugin.h \
    src/RosRecorder.h

SOURCES += \
    src/RosRecorderPlugin.cpp \
    src/RosRecorder.cpp

TEMPLATE = lib
TARGET = rosrecorderplugin

QMAKE_CXXFLAGS -= -O2
QMAKE_CXXFLAGS_RELEASE -= -O2

QMAKE_CXXFLAGS += -O3
QMAKE_CXXFLAGS_RELEASE += -O3

TARGET = $$qtLibraryTarget($$TARGET)
uri = QtRosRecorder


#Install plugin library, qmldir
qmldir.files = qmldir
OTHER_FILES += qmldir
unix {
    installPath = $$[QT_INSTALL_QML]/$$replace(uri, \\., /)
    qmldir.path = $$installPath
    target.path = $$installPath
    INSTALLS += target qmldir
}
message("Config done.")
