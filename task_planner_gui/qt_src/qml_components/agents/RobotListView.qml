import QtQuick 2.5
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.1



Item{
    id: item1
    property var lv_mod
    property var usr
    property string robot_name
    property bool start_visible: true
    anchors.top: parent.top
    anchors.topMargin: 0
    width: 300
    height: 400

    Rectangle{


        clip: true
        id: recta_main1
        width: parent.width
        height: parent.height
        color: "#2d325a"

        Text {
            id: text_rectan_title
            x: 85
            y: -48
            width: 131
            height: 32
            color: "#868aa6"
            text: qsTr(robot_name)
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: 25
        }
        RoundButton {
            id: bttn_add
            x: 0
            y: 325
            text: "+"
            anchors.horizontalCenterOffset: 0
            anchors.horizontalCenter: parent.horizontalCenter
            visible: if (start_visible) return false
            onClicked: {
                visible = false
                listView.visible = true;
                animation_visible.start()
            }
        }
        ListView {
            id: listView
            visible: start_visible
            verticalLayoutDirection: ItemView.BottomToTop
//                keyNavigationWraps: true
            orientation: ListView.Vertical
//                boundsBehavior: Flickable.DragOverBounds
            flickableDirection: Flickable.AutoFlickDirection
//                        clip: true
            model: lv_mod.model
            property var mo: lv_mod.model
            z: 1
            anchors.bottomMargin: 83
            anchors.fill: parent
            delegate:
                Rectangle{
                    color: "#2d325a"
                    id: delegateRect1
                    height: edit.height+55
                    width: listView.width
                RobotDelegate{
                    mod: lv_mod
                    idx: index
                    edt: edit
                    lv: listView
//                    remove_time: removeAn.duration
                }

                ListView.onAdd: SequentialAnimation{
                    NumberAnimation {
                        target: delegateRect1; property: "opacity"; from: 0.0; to: 1.0; duration: 500;
                    }
                }

                ListView.onRemove: SequentialAnimation {
                    id: seqAnim1
                    PropertyAction {id:act1; target: delegateRect1; property: "ListView.delayRemove"; value: true}
                    NumberAnimation {
                        id: removeAn
                        target: delegateRect1; property: "height"; from: edit.height+35; to: 1; duration: 1000; //edit.expectedTime*1000;
                    }
                    onStopped: {
                        applicationWindow.endRemoveItemAnimationSig()
                        console.log("Task correctly removed from the visualizer")
                    }
                }
                Connections{
                    target: lv_mod
                    onForceRemove: {
                        delegateRect1.ListView.delayRemove = false
                        console.log("ListView.delayRemove (ForceRemove) = "+delegateRect1.ListView.delayRemove)
                    }
                }
                Connections{
                    target: lv_mod
                    onForceRemoveAnim: {
                        if (index == -1)
                            applicationWindow.endRemoveItemAnimationSig()
                    }
                }

            }
        }

        Connections{
            target: lv_mod
            onStartRemoveSig: removeAn_transition.start()
        }
        SequentialAnimation{
            id: removeAn_transition
            NumberAnimation {target: lv_mod.delegate; property: "opacity"; from: 1.0; to: 0.0; duration: 500;}
            onStopped: {
                applicationWindow.removeItemSig1(0,"SUCCESS");
            }
        }

        Connections {
            target: lv_mod
            onNewDataCallbackSig: {
                applicationWindow.addNewTask()
                console.log("Signal onNewDataCallbackSig received in QML");
            }
        }



        Rectangle {
            id: rect_zero_tick
            y: 331
            width: 3
            height: 81
            color: "#ffffff"
            radius: 1
            visible: true
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 0
            anchors.left: parent.left
            anchors.leftMargin: 30
            z: 1
            Rectangle{
                id: rect_inner_zero_tick
                width: 20
                height: rect_inner_zero_tick.width
                color: "#d24444"
                radius: rect_inner_zero_tick.width
                anchors.horizontalCenter: parent.horizontalCenter
                Rectangle{
                    id: rect_2inner_zero_tick
                    color: "#ffffff"
                    radius: width
                    anchors.rightMargin: 5
                    anchors.leftMargin: 5
                    anchors.bottomMargin: 5
                    anchors.topMargin: 5
                    anchors.fill: parent
                }
            }
        }

    }

    Rectangle {
        id: rectan_button_robot
        x: 0
        width: 300
        height: 72
        color: "#2d325a"
        radius: 30
        anchors.top: parent.bottom
        anchors.topMargin: 20

        RoundButton {
            id: stop_btn
            enabled: true
            y: 515
            width: 120
            height: 45
            text: qsTr("Stop")
            anchors.left: parent.left
            anchors.leftMargin: 30
            anchors.verticalCenter: parent.verticalCenter
            Material.background: Material.Red
            onClicked: {
                enabled = false
                resume_btn.enabled = true
                applicationWindow.overrideSig("STOP")
            }
        }

        RoundButton {
            id: resume_btn
            enabled: false
            x: 238
            y: 515
            width: 120
            height: 45
            text: qsTr("Resume")
            anchors.right: parent.right
            anchors.rightMargin: 30
            anchors.verticalCenter: parent.verticalCenter
            Material.background: Material.Green
            onClicked: {
                enabled = false
                stop_btn.enabled = true
                applicationWindow.overrideSig("RESUME")
            }
        }
    }

}
