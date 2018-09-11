TEMPLATE = app

CONFIG += qt c++11

QT += qml quick

SOURCES += src/main.cpp

RESOURCES += qml.qrc

android {
        INCLUDEPATH += /home/florian/ros-android-ndk/roscpp_android/output/target/include
        LIBS += -L"/home/florian/ros-android-ndk/roscpp_android/output/target/lib/"
} else {
        INCLUDEPATH += /opt/ros/melodic/include
        INCLUDEPATH += /home/wafa/catkin_ws/install/include
        LIBS += -L"/opt/ros/melodic/lib/"
        LIBS += -L"/home/wafa/catkin_ws/install/lib/"
}

android {
    target.path = /libs/armeabi-v7a
    export(target.path)
    INSTALLS += target
    export(INSTALLS)


    DISTFILES += \
        android/AndroidManifest.xml \
        android/gradle/wrapper/gradle-wrapper.jar \
        android/gradlew \
        android/res/values/libs.xml \
        android/build.gradle \
        android/gradle/wrapper/gradle-wrapper.properties \
        android/gradlew.bat

    ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android
}
