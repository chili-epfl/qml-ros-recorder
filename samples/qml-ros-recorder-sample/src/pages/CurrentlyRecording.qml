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

    title: qsTr("Currently recording topics:")

    property var recorder
    property var recording: []

	Flickable {
	    anchors.fill: parent
        anchors.centerIn: parent
	    contentHeight: mainLayout.height
	    contentWidth: mainLayout.width

	    GridLayout {
	    	id: mainLayout
	        anchors.centerIn: parent
	        columns: 1
	        columnSpacing: 8
	        rowSpacing: 12

	        Repeater {
	        	id: topicRepeater
	        	Layout.bottomMargin: 24

	            model: recording
	            RowLayout {
	            	Text {
	                	text: modelData[0] + " (started at time " + modelData[1] + ")"
	                }

	                Button {
	                	text: "Stop recording"
	                	onClicked: {
	                		recorder.stopRecording(modelData[0])

		                    recording = []
		                    for (var name in recorder.currentlyRecording) {
		                        recording.push([name, recorder.currentlyRecording[name]])
		                    }
	                	}
	                }
	            }
	        }
	    }
	}
}
