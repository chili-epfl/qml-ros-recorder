TEMPLATE = lib
TARGET = rosrecorderplugin

CONFIG += qt plugin c++11
CONFIG -= android_install

QT += qml quick network gui

QMAKE_CXXFLAGS -= -O2
QMAKE_CXXFLAGS_RELEASE -= -O2

QMAKE_CXXFLAGS += -O3
QMAKE_CXXFLAGS_RELEASE += -O3

TARGET = $$qtLibraryTarget($$TARGET)
uri = ch.epfl.chili.ros.recorder

HEADERS += \
    src/RosRecorderPlugin.h \
    src/RosRecorder.h

SOURCES += \
    src/RosRecorderPlugin.cpp \
    src/RosRecorder.cpp

OTHER_FILES += qmldir

android {
	INCLUDEPATH += /home/florian/ros-android-ndk/roscpp_android/output/target/include
	LIBS += -L"/home/florian/ros-android-ndk/roscpp_android/output/target/lib/" 
} else {
	INCLUDEPATH += /opt/ros/kinetic/include
	LIBS += -L"/opt/ros/kinetic/lib/" 
}

LIBS += -lroscpp -lboost_signals -lboost_filesystem -lrosconsole

android {
    LIBS += -lrosconsole_print
}

LIBS += -lrosconsole_backend_interface -lboost_regex -lxmlrpcpp -ldynamic_reconfigure_config_init_mutex -lroscpp_serialization -lrostime -lboost_date_time -lcpp_common -lboost_system -lboost_thread -lconsole_bridge -ltinyxml -lclass_loader -lPocoFoundation -lroslib -lboost_iostreams -lnodeletlib -lbondcpp -luuid -lrosbag -lrosbag_storage -lboost_program_options -lroslz4 -llz4 -ltopic_tools -lactionlib -lmessage_filters -lrosconsole_bridge -lrospack

#Install plugin library, qmldir
qmldir.files = qmldir
unix {
    installPath = $$[QT_INSTALL_QML]/$$replace(uri, \\., /)
    qmldir.path = $$installPath
    target.path = $$installPath
    INSTALLS += target qmldir
}
