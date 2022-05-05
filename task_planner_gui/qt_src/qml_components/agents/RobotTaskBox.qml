import QtQuick 2.5
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.1

Rectangle{

    id: userTaskBox
    property var taskID: "Id"
    property var taskName: "Task"
    property var startName: "Start"
    property var goalName: "Goal"
    property string countourColor: "#2d325a"

    height: 90
    width: 235

    color: "#00000000"
    border.width: (width - (width-2*(1.0-0.7071067811)*radius))/1.5
    border.color: countourColor
    radius: 20
    opacity: 1
    clip: true

    Rectangle{
        id: box_text_task
        color: "#3a3f6b"
        width: parent.width - 2*parent.radius*(1.0-0.70710678118)
        height: parent.height - 2*parent.radius*(1.0-0.70710678118)
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        opacity: 1
        z: -1
    }

    Row{
        anchors.verticalCenter: box_text_task.verticalCenter
        anchors.left: box_text_task.left
        anchors.leftMargin: 15
        Column{
            id: column1

            Row{
                spacing: 2
                Image {
                    id: image_name
                    width: 20
                    height: 20
                    source: "../../assets/ic_speaker_notes_48px.svg"
                    ColorOverlay {
                        anchors.fill: image_name
                        source: image_name
                        color: "#ffffff"  // make image like it lays under red glass
                    }
                }
                Text{
                    text: userTaskBox.taskName
                    color: "#ffffff"
                }
            }

            Text{
                id: text_task_description
                text: userTaskBox.taskID
                color: "#8f92a3"
                font.pixelSize: 12
            }

            Row{
                spacing: 2
                Image {
                    id: image_from
                    width: 20
                    height: 20
                    source: "../../assets/ic_call_received_48px.svg"
                    ColorOverlay {
                        anchors.fill: image_from
                        source: image_from
                        color: "#ffffff"  // make image like it lays under red glass
                    }
                }
                Text{
                    text: userTaskBox.startName
                    color: "#ffffff"
                }
                Image {
                    id: image_to
                    width: 20
                    height: 20
                    source: "../../assets/ic_call_made_48px.svg"
                    ColorOverlay {
                        anchors.fill: image_to
                        source: image_to
                        color: "#ffffff"  // make image like it lays under red glass
                    }
                }
                Text{
                    text: userTaskBox.goalName
                    color: "#ffffff"
                }
            }
        }
    }





}
