TEMPLATE = lib
TARGET = qmlrosrecorderplugin

CONFIG += qt plugin c++11
CONFIG -= android_install

QT += qml quick network gui

QMAKE_CXXFLAGS -= -O2
QMAKE_CXXFLAGS_RELEASE -= -O2

QMAKE_CXXFLAGS += -O3
QMAKE_CXXFLAGS_RELEASE += -O3

TARGET = $$qtLibraryTarget($$TARGET)
uri = QMLRosRecorder

HEADERS += \
    src/QMLRosRecorderPlugin.h \
    src/RosRecorder.h

SOURCES += \
    src/QMLRosRecorderPlugin.cpp \
    src/RosRecorder.cpp

OTHER_FILES += qmldir

android {
	INCLUDEPATH += /home/florian/ros-android-ndk/roscpp_android/output/target/include
	LIBS += -L"/home/florian/ros-android-ndk/roscpp_android/output/target/lib/" 
} else {
	INCLUDEPATH += /opt/ros/kinetic/include
	LIBS += -L"/opt/ros/kinetic/lib/" 
}

LIBS += -L"/home/florian/ros-android-ndk/roscpp_android/output/target/lib/" -lroscpp -lboost_signals -lboost_filesystem -lrosconsole -lrosconsole_print -lrosconsole_backend_interface -lboost_regex -lxmlrpcpp -ldynamic_reconfigure_config_init_mutex -lroscpp_serialization -lrostime -lboost_date_time -lcpp_common -lboost_system -lboost_thread -lconsole_bridge -lrotate_recovery -ltinyxml -lclass_loader -lPocoFoundation -lroslib -lglobal_planner -lnavfn -lcostmap_2d -llayers -llaser_geometry -lpcl_ros_filters -lpcl_ros_io -lpcl_ros_tf -lpcl_common -lpcl_octree -lpcl_kdtree -lpcl_search -lpcl_sample_consensus -lpcl_filters -lpcl_io -lpcl_features -lpcl_registration -lpcl_surface -lpcl_tracking -lpcl_ml -lpcl_keypoints -lpcl_segmentation -lpcl_stereo -lpcl_recognition -lboost_iostreams -lqhullstatic -lflann_cpp_s -lflann_cpp_s-gd -lnodeletlib -lbondcpp -luuid -lrosbag -lrosbag_storage -lboost_program_options -lroslz4 -llz4 -ltopic_tools -lvoxel_grid -ltf -ltf2_ros -lactionlib -lmessage_filters -ltf2 -lmove_slow_and_clear -ldwa_local_planner -lclear_costmap_recovery -lcarrot_planner -lbase_local_planner -ltrajectory_planner_ros -lrobot_state_publisher_solver -ltf_conversions -lkdl_conversions -lkdl_parser -lorocos-kdl -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world -lrosconsole_bridge -lmoveit_exceptions -lmoveit_background_processing -lmoveit_kinematics_base -lmoveit_robot_model -lmoveit_transforms -lmoveit_robot_state -lmoveit_robot_trajectory -lmoveit_planning_interface -lmoveit_collision_detection -lmoveit_collision_detection_fcl -lmoveit_kinematic_constraints -lmoveit_planning_scene -lmoveit_constraint_samplers -lmoveit_planning_request_adapter -lmoveit_profiler -lmoveit_trajectory_processing -lmoveit_distance_field -lmoveit_kinematics_metrics -lmoveit_dynamics_solver -lgeometric_shapes -lshape_tools -leigen_conversions -lrandom_numbers -lsrdfdom -lpointcloud_filters -llaser_scan_filters -lmean -lparams -lincrement -lmedian -ltransfer_function -linteractive_markers -lcompressed_image_transport -lcv_bridge -lopencv_core -lopencv_androidcamera -lopencv_flann -lopencv_imgproc -lopencv_highgui -lopencv_features2d -lopencv_calib3d -lopencv_ml -lopencv_video -lopencv_legacy -lopencv_objdetect -lopencv_photo -lopencv_gpu -lopencv_videostab -lopencv_ocl -lopencv_superres -lopencv_nonfree -lopencv_stitching -lopencv_contrib -limage_transport -lcompressed_depth_image_transport -lamcl_sensors -lamcl_map -lamcl_pf -ldepth_image_proc -lstereo_image_proc -limage_proc -limage_geometry -lpolled_camera -lcamera_info_manager -lpluginlib_tutorials -lnodelet_math -lcollada_parser -lcamera_calibration_parsers -lresource_retriever -lrospack

#Install plugin library, qmldir
qmldir.files = qmldir
unix {
    installPath = $$[QT_INSTALL_QML]/$$replace(uri, \\., /)
    qmldir.path = $$installPath
    target.path = $$installPath
    INSTALLS += target qmldir
}
