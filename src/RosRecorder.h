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
 * @file QMLRos.h
 * @brief QML wrapper header for RosRecorder
 * @author Florian Zimmermann
 * @date 2018-03-26
 */

#ifndef ROSNODE_H
#define ROSNODE_H

#include <QQuickItem>

#include <memory>

namespace ros {
    class NodeHandle;
}

class RosRecorder : public QQuickItem {
    /* *INDENT-OFF* */
    Q_OBJECT
    /* *INDENT-ON* */

    Q_PROPERTY(QString status READ getStatus NOTIFY statusChanged)
    Q_PROPERTY(QString masterIp READ getMasterIp WRITE setMasterIp NOTIFY masterIpChanged)
    Q_PROPERTY(QVariantList availableTopics READ getAvailableTopics NOTIFY availableTopicsChanged)
    Q_PROPERTY(QVariantList topicsToRecord READ getTopicsToRecord WRITE setTopicsToRecord NOTIFY topicsToRecordChanged)

public:
    /**
     * @brief Creates a new RosRecorder with the given QML parent
     *
     * @param parent The QML parent
     */
    RosRecorder(QQuickItem* parent = 0);

    /**
     * @brief Destroys this RosRecorder
     */
    ~RosRecorder();

    /**
     * @brief Gets this ROS node's status
     *
     * @return This ROS node's status
     */
    const QString &getStatus() const { return status; }

    /**
     * @brief Gets the ROS master's IP address
     *
     * @return The ROS master's IP address
     */
    const QString &getMasterIp() const { return masterIp; }

    const QVariantList &getAvailableTopics() const { return availableTopics; }
    const QVariantList &getTopicsToRecord() const { return topicsToRecord; }

    /**
     * @brief Sets the ROS master's IP address
     *
     * @param The ROS master's IP address
     */
    void setMasterIp(const QString &masterIp) { this->masterIp = masterIp; }

    void setTopicsToRecord(const QVariantList &topicsToRecord) { this->topicsToRecord = topicsToRecord; }

public slots:
    void refreshTopics();
    void startRecording(const QString &name);
    void stopRecording(const QString &name);

signals:
    /**
     * @brief Emitted when this ROS node's status changes
     */
    void statusChanged();

    /**
     * @brief Emitted when the ROS master's IP address changes
     */
    void masterIpChanged();

    void availableTopicsChanged();

    void topicsToRecordChanged();

private:
    void stopAll();
    void startNode();
    void stopNode();

    QString status;                   ///< Status of this ROS node
    QString masterIp;                 ///< IP address of ROS master

    std::unique_ptr<ros::NodeHandle> nodeHandle;

    QVariantList availableTopics;
    QVariantList topicsToRecord;

    QHash<QString, uint64_t> currentlyRecording; ///< name -> timestamp when started
};

#endif /* ROSNODE_H */
