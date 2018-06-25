import QtQuick 2.7
import QtQuick.Window 2.2
import QtQuick.Controls 2.3
import QtQml.Models 2.2
import QtQuick.Dialogs 1.3
import QtQuick.Extras 1.4
import QtQuick.Layouts 1.3

import "./pages"

ApplicationWindow {
    id: root
    visible: true

    property bool mobile: Qt.platform.os === "android"
    width: mobile ? Screen.width : 1080
    height: mobile ? Screen.height : 720

    TopicSelection {
        id: topicSelection
    }

    CurrentlyRecording {
        id: currentlyRecording
    }
    
    header: ToolBar {
        contentHeight: toolButton.implicitHeight

        ToolButton {
            id: toolButton
            text: "\u2630"
            font.pixelSize: Qt.application.font.pixelSize * 1.6
            onClicked: {
                stackView.pop()
                drawer.open()
            }
        }

        Label {
            text: stackView.currentItem.title
            anchors.centerIn: parent
        }
    }

    Drawer {
        id: drawer
        width: root.width * 0.4
        height: root.height

        Column {
            id: mainColumn
            anchors.fill: parent
            padding: 5

            ItemDelegate {
                id: startItem
                text: qsTr("Select topics to record")
                width: parent.width
                onClicked: {
                    stackView.push(topicSelection)
                    drawer.close()
                }
            }

            ItemDelegate {
                text: qsTr("Currently recording")
                width: parent.width
                onClicked: {
                    var recording = []
                    for (var name in topicSelection.recorder.currentlyRecording) {
                        recording.push([name, topicSelection.recorder.currentlyRecording[name]])
                    }

                    currentlyRecording.recorder = topicSelection.recorder
                    currentlyRecording.recording = recording
                    stackView.push(currentlyRecording)
                    drawer.close()
                }
            }
        }
    }

    StackView {
        id: stackView
        initialItem: topicSelection
        anchors.fill: parent
        onCurrentItemChanged: console.log(currentItem)
    }
}
