import QtQuick 2.0
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Styles 1.4
import QtCharts 2.1
import QtGraphicalEffects 1.0
import QtQuick.Window 2.2

import "../agents"

Rectangle{
    id: popupPoints
    property var taskID
    property var taskName
    property var taskStart
    property var taskGoal
    property int point_idx
    property real popup_x: popup.x
    property real popup_y: popup.y
    property var sca
    width: 12
    height: 12
    radius: 6
    color: "black"
    scale: ma.containsMouse ? 2.4 : 1.6

    Rectangle{
        width: 8
        height: 8
        radius: 5
        color: "white"
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
    }


    ToolTip {
        id: tooltTip1
        parent: popupPoints
        visible: ma.containsMouse ? true : false
        x: 15
        y: 15
        background: UserTaskBox{
            countourColor: "#d3d3ed"
            taskID: popupPoints.taskID
            taskName: popupPoints.taskName
            startName: popupPoints.taskStart
            goalName: popupPoints.taskGoal
        }


    }

    MouseArea {
        id: ma
        anchors.fill: parent
        hoverEnabled: true
//        onHoveredChanged: {
//            containsMouse ? sca.markerSize=20 : sca.markerSize=15
//        }
    }


    Popup {
        id: popup
        x: 0
        y: 0
        modal: true
        focus: true
        UserTaskBox{
            countourColor: "#3a3f6b"
            taskID: popupPoints.taskID
            taskName: popupPoints.taskName
            startName: popupPoints.taskStart
            goalName: popupPoints.taskGoal
        }
    }
}



