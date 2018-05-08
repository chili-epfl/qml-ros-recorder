/*
 * Copyright (C) 2018 EPFL
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/.
 */

/**
 * @file QMLRos.cpp
 * @brief QML wrapper source for RosRecorder
 * @author Florian Zimmermann
 * @date 2018-03-26
 */

#include "RosRecorder.h"

#ifdef Q_OS_ANDROID
#include <android/log.h>
#else
#include <cstdio>
#endif
#include <stdarg.h>

#include <ros/ros.h>
#include <rosbag_recorder/RecordTopics.h>
#include <rosbag_recorder/StopRecording.h>

#include <QNetworkInterface>

#include <algorithm>

static void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
#ifdef Q_OS_ANDROID
    __android_log_vprint(ANDROID_LOG_INFO, "RosRecorder", msg, args);
#else
    vprintf(msg, args);
    printf("\n");
#endif
    va_end(args);
}

static QString getDeviceIpAddress() {
    QList<QHostAddress> list = QNetworkInterface::allAddresses();

    for(int i = 0; i < list.count(); ++i) {
      if(!list[i].isLoopback()) {
          if (list[i].protocol() == QAbstractSocket::IPv4Protocol)
            return list[i].toString();
      }
    }

    return "";
}

RosRecorder::RosRecorder(QQuickItem* parent)
: QQuickItem(parent) {
    status = "Idle";
    masterIp = "192.168.1.100";
}

RosRecorder::~RosRecorder() {
    stopNode();
}

void RosRecorder::startNode() {
    QString nodeIp = getDeviceIpAddress();
    QString sanitizedNodeIp = QString(nodeIp).replace('.', '_');
    QByteArray tmp = nodeIp.toUtf8();
    log("Node IP: %s", tmp.data());

    int argc = 3;
    QByteArray master = QString("__master:=http://" + masterIp + ":11311").toUtf8();
    QByteArray ip = QString("__ip:=" + nodeIp).toUtf8();
    char *argv[argc] = { "qml_ros_recorder", master.data(), ip.data() };

    log("Initializing ROS");
    for (int i = 0; i < argc; ++i) {
        log("Argument %i: %s", i, argv[i]);
    }

    QString nodeName("ros_recorder_" + sanitizedNodeIp);
    ros::init(argc, &argv[0], nodeName.toStdString());

    log("Looking for ROS master...");

    if (ros::master::check()) {
        log("ROS master found");
    } else {
        log("No ROS master");
    }

    log(ros::master::getURI().c_str());

    nodeHandle.reset(new ros::NodeHandle());

    status = "Running";
    emit RosRecorder::statusChanged();

    log("Node started");

    refreshTopics();
}

void RosRecorder::stopNode() {
    stopAll();
    availableTopics.clear();
    delete nodeHandle.release();
    ros::shutdown();

    status = "Idle";

    emit availableTopicsChanged();
    emit statusChanged();
}

void RosRecorder::refreshTopics() {
    std::vector<ros::master::TopicInfo> topics;
    if (ros::master::getTopics(topics)) {
        log("Refreshing available topics");

        availableTopics.clear();
        for (const auto &topic : topics) {
            availableTopics.append(QString(topic.name.c_str()));
        }

        std::sort(availableTopics.begin(), availableTopics.end());

        emit availableTopicsChanged();
    }
}

void RosRecorder::startRecording(const QString &name) {
    if (!topicsToRecord.length()) {
        return;
    }

    if (currentlyRecording.contains(name)) {
        stopRecording(name);
    }

    log("Starting recording to bag %s. Topics:", name.toStdString().c_str());
    std::vector<std::string> topics;
    for (int i = 0; i < topicsToRecord.length(); ++i) {
        topics.push_back(topicsToRecord[i].toString().toStdString());
        log(topics[i].c_str());
    }

    ros::ServiceClient client = nodeHandle->serviceClient<rosbag_recorder::RecordTopics>("record_topics");
    rosbag_recorder::RecordTopics srv;
    srv.request.name = name.toStdString();
    srv.request.topics = topics;

    if (client.call(srv)) {
        log("Response: %s", srv.response.success ? "true" : "false");
        currentlyRecording.insert(name, QVariant(static_cast<qulonglong>(ros::Time::now().toNSec())));
        emit currentlyRecordingChanged();
    }
    else {
        log("Failed to call service \"record_topics\"");
    }
}

void RosRecorder::stopRecording(const QString &name) {
    if (!currentlyRecording.contains(name)) {
        return;
    }

    log("Stopping recording to bag %s.", name.toStdString().c_str());
    ros::ServiceClient client = nodeHandle->serviceClient<rosbag_recorder::StopRecording>("stop_recording");
    rosbag_recorder::StopRecording srv;
    srv.request.name = name.toStdString();

    if (client.call(srv)) {
        log("Response: %s", srv.response.success ? "true" : "false");
        currentlyRecording.remove(name);
        emit currentlyRecordingChanged();
    }
    else {
        log("Failed to call service \"stop_recording\"");
    }
}

void RosRecorder::stopAll() {
    if (!currentlyRecording.empty()) {
        return;
    }

    ros::ServiceClient client = nodeHandle->serviceClient<rosbag_recorder::StopRecording>("stop_recording");
    rosbag_recorder::StopRecording srv;

    for (auto it = currentlyRecording.begin(); it != currentlyRecording.end(); ++it) {
        srv.request.name = it.key().toStdString();
        if (client.call(srv)) {
            log("Response: %s", srv.response.success ? "true" : "false");
        }
        else {
            log("Failed to call service \"stop_recording\"");
        }
    }

    currentlyRecording.clear();

    emit currentlyRecordingChanged();
}
