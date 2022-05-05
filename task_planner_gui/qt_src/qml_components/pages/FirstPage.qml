import QtQuick 2.5
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.1

import "../agents"

Item {
    width: 1560
    height: 750
    Rectangle {
        id: rectangle1
        anchors.fill: parent
        color: "#25294a"
        radius: 50
        anchors.bottomMargin: 140
        border.width: 0
        Material.elevation: 102

        Text {
            id: text1
            y: -67
            width: 262
            height: 47
            text: qsTr("Task Manager")
            anchors.bottom: parent.top
            anchors.bottomMargin: 15
            anchors.left: parent.left
            anchors.leftMargin: 40
            font.bold: true
            font.pixelSize: 40
            color: "#ffffff"
        }

        Row{
            id: row
            anchors.rightMargin: 70
            anchors.leftMargin: 70
            anchors.bottomMargin: 50
            anchors.topMargin: 80
            anchors.fill: parent
            spacing: 30


            //Robot 1
            RobotListView{
                id: robot1
                lv_mod: backend
                robot_name: backend.agentName
                usr: user1
            }


            //User 1
            UserListview{
                id: user1
                lv_mod: backend2
                user_name: backend2.agentName
                lv_mod_robot: backend
                lv_backend_number: 2
            }




//            //User 2
//            UserListview{
//                id: user2
//                lv_mod: backend3
//                user_name: backend3.agentName
//                lv_mod_robot: backend
//                lv_backend_number: 3
//                start_visible: false
//                buttons_clickable: false
//            }

//            //Robot 2
//            RobotListView{
//                id: robot2
//                lv_mod: backend4
//                robot_name: backend4.agentName
//                start_visible: false
//            }



        }


        Rectangle {
            id: top_separator
            height: 5
            color: "#363840"
            anchors.top: parent.top
            anchors.topMargin: 80
            anchors.right: parent.right
            anchors.rightMargin: 50
            anchors.left: parent.left
            anchors.leftMargin: 50
        }

        Rectangle {
            id: bottom_separator
            height: 5
            color: "#363840"
            anchors.top: row.top
            anchors.topMargin: 400
            anchors.right: parent.right
            anchors.rightMargin: 50
            anchors.left: parent.left
            anchors.leftMargin: 50
        }

        Rectangle {
            id: rectangle5
            x: 0
            y: 675
            width: 511
            height: 57
            color: "#25294a"
            radius: 50
            border.width: 0

            Text {
                id: text3
                x: 24
                y: -43
                width: 361
                height: 32
                color: "#868aa6"
                text: qsTr("Human-Robot Proximity Level")
                font.pixelSize: 25
            }

            Text {
                id: text4
                x: 67
                y: 17
                text: qsTr("Close")
                font.pixelSize: 20
                color: "#ffffff"
            }

            Rectangle {
                id: rectangle8
                x: 31
                y: 14
                width: 30
                height: 30
                color: "#f26868"
                radius: 10
            }


            Text {
                id: text5
                x: 186
                y: 17
                text: qsTr("Medium")
                font.pixelSize: 20
                color: "#ffffff"
            }

            Rectangle {
                id: rectangle9
                x: 150
                y: 14
                width: 30
                height: 30
                color: "#f2f269"
                radius: 10
            }

            Text {
                id: text6
                x: 325
                y: 17
                text: qsTr("Weak")
                font.pixelSize: 20
                color: "#ffffff"
            }

            Rectangle {
                id: rectangle10
                x: 289
                y: 14
                width: 30
                height: 30
                color: "#68f2ad"
                radius: 10
            }

        }





    }

}
