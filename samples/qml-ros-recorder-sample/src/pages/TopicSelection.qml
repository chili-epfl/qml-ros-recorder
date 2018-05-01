import QtQuick 2.7
import QtQuick.Window 2.2
import QtQuick.Controls 2.3
import QtQml.Models 2.2
import QtQuick.Dialogs 1.3
import QtQuick.Extras 1.4
import QtQuick.Layouts 1.3

import QMLRosRecorder 1.0

Page {
    id: root
    padding: 5
    spacing: 5

    title: qsTr("Select topics for recording")

    RosRecorder {
        id: rosRecorder

        onStatusChanged: {
            console.log("Status: " + rosRecorder.status)
        }
    }

    GridLayout {
        anchors.centerIn: parent
        columns: 1
        columnSpacing: 8
        rowSpacing: 12

        Repeater {
        	id: topicRepeater
        	Layout.bottomMargin: 24

            model: rosRecorder.availableTopics
            CheckBox {
            	id: topicCheckBox
                checked: false
                text: modelData

                contentItem: Text {
                	text: topicCheckBox.text
                	color: topicCheckBox.checkState == Qt.Checked ? "#66EE88" : "#888888"
                	font.pointSize: 24
                	leftPadding: topicCheckBox.indicator.width + topicCheckBox.spacing
                }
            }
        }

        RowLayout {
        	spacing: 32

	        Button {
	        	id: refreshButton
	        	text: "Refresh topics"
	        	onClicked: rosRecorder.refreshTopics()

	        	contentItem: Text {
			        text: refreshButton.text
			        font.pointSize: 28
			        horizontalAlignment: Text.AlignHCenter
			        verticalAlignment: Text.AlignVCenter
			        elide: Text.ElideRight
			    }
	        }

	        Button {
	        	id: recordButton
	        	property bool recording: false

	        	text: recording ? "Stop recording" : "Start recording"
	        	onClicked: {
	        		if (!recordButton.recording) {
		        		var topics = []
		        		for (var i = 0; i < topicRepeater.count; ++i) {
		        			var checkBox = topicRepeater.itemAt(i)
		        			if (checkBox.checked) {
		        				topics.push(checkBox.text)
		        			}
		        		}

		        		if (topics.length) {
		        			rosRecorder.topicsToRecord = topics
		        			rosRecorder.startRecording("test")
		        			recordButton.recording = true
		        		}
		        	}
		        	else {
		        		rosRecorder.stopRecording("test")
		        		recordButton.recording = false
		        	}
	        	}

	        	contentItem: Text {
			        text: recordButton.text
			        font.pointSize: 28
			        horizontalAlignment: Text.AlignHCenter
			        verticalAlignment: Text.AlignVCenter
			        elide: Text.ElideRight
			    }
	        }
	    }
    }
}
