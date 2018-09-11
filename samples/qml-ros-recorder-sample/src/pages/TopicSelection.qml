import QtQuick 2.7
import QtQuick.Window 2.2
import QtQuick.Controls 2.3
import QtQml.Models 2.2
import QtQuick.Dialogs 1.3
import QtQuick.Extras 1.4
import QtQuick.Layouts 1.3

import ch.epfl.chili.ros.recorder 1.0

Page {
    id: root
    padding: 5
    spacing: 5

    title: qsTr("Select topics for recording")

    property var recorder




    RosRecorder {
        id: rosRecorder

        onStatusChanged: {
            console.log("Status: " + rosRecorder.status)
        }
        masterIp:"127.0.0.1"


        Component.onCompleted: {
        	recorder = rosRecorder
        	startNode()
        }
    }

	Flickable {
	    anchors.fill: parent
        anchors.centerIn: parent
	    contentHeight: mainLayout.height
	    contentWidth: mainLayout.width

        ColumnLayout{
            spacing: 20
            Row{

        TextField{
            id: masterIPtx
            text: "127.0.0.1"

        }
            Button{
                id:resetMasterIpBtn
                text:"Reset Master IP"
                onClicked: rosRecorder.masterIp = masterIPtx.text

            }
            }

	    GridLayout {
	    	id: mainLayout
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

		      //   	contentItem: Text {
				    //     text: refreshButton.text
				    //     font.pointSize: 28
				    //     horizontalAlignment: Text.AlignHCenter
				    //     verticalAlignment: Text.AlignVCenter
				    //     elide: Text.ElideRight
				    // }
		        }

		        Button {
		        	id: recordButton

		        	text: "Start recording"
		        	onClicked: {
		        		if (bagNameText.text != "") {
			        		var topics = []
			        		for (var i = 0; i < topicRepeater.count; ++i) {
			        			var checkBox = topicRepeater.itemAt(i)
			        			if (checkBox.checked) {
			        				topics.push(checkBox.text)
			        			}
			        		}

			        		if (topics.length) {
			        			rosRecorder.topicsToRecord = topics
			        			rosRecorder.startRecording(bagNameText.text)
			        		}
			        	}
		        	}

		      //   	contentItem: Text {
				    //     text: recordButton.text
				    //     font.pointSize: 28
				    //     horizontalAlignment: Text.AlignHCenter
				    //     verticalAlignment: Text.AlignVCenter
				    //     elide: Text.ElideRight
				    // }
		        }

		        TextField {
		        	id: bagNameText
		        	text: "Test"
		        }
		    }
		}
    }
    }


}
