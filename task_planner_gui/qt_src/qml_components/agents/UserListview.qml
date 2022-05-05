import QtQuick 2.5
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.1


//User1
Item{



    property var lv_mod    //backend
    property var lv_mod_robot    //backend
    property int lv_backend_number
    property string user_name
    property bool start_visible: true
    property bool buttons_clickable: true

    id: item1
    width: 300
    anchors.bottom: parent.bottom
    anchors.bottomMargin: 0
    anchors.top: parent.top
    anchors.topMargin: 0
    Rectangle{
        id: recta_main
        width: parent.width
        height: 400
        color: "#2d325a"
        anchors.top: parent.top
        anchors.topMargin: 0

        RoundButton {
            id: roundButton
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
            anchors.fill: parent
            verticalLayoutDirection: ItemView.BottomToTop
            keyNavigationWraps: true
            orientation: ListView.Vertical
            boundsBehavior: Flickable.DragOverBounds
            flickableDirection: Flickable.AutoFlickDirection
            clip: true
            model: lv_mod.model
            Connections {
                target: lv_mod
                onModelChanged: {
                    console.log("Changing data")
                }
            }
            NumberAnimation {
                id: animation_visible
                target: listView; property: "opacity"; from: 0; to: 1.0; duration: 1000;
            }


            delegate:
                Rectangle{
                color: "#2d325a"
                id: delegateRect
                height: edit.height+35
                width: listView.width
                opacity: 1.0
                ListView.onRemove: SequentialAnimation {
                    id: seqAnim
                    PropertyAction {id:act1; target: delegateRect; property: "ListView.delayRemove"; value: true }
                    ParallelAnimation{
                        NumberAnimation {
                            id: removeAn
                            target: delegateRect; property: "height"; from: edit.height+35; to: 0; duration: 1000;
                        }
                        NumberAnimation {
                            id: removeAn2
                            target: delegateRect; property: "x";  to: -2*listView.width; duration: 1000;
                        }
                        NumberAnimation {
                            id: removeAn3
                            target: delegateRect; property: "opacity"; from: 1.0; to: 0; duration: 250;
                        }
                    }
                    PropertyAction {id:act2; target: delegateRect; property: "ListView.delayRemove"; value: false }
                }
                ListView.onAdd: ParallelAnimation{
                    NumberAnimation {
                        target: delegateRect; property: "opacity"; from: 0; to: 1.0; duration: 500;
                    }
                    NumberAnimation {
                        target: delegateRect; property: "scale"; from: 0; to: 1.0; duration: 250;
                    }
                }
                UserDelegate{
                    mod: lv_mod
                    idx: index
                    edt: edit
                    lv: listView
                    anim1: removeAn2
                    anim2: removeAn3
                    backend_number: lv_backend_number
                    bttns_clickable: buttons_clickable
                }

            }


        }

        Text {
            id: text_rectan_title
            x: 85
            y: -48
            width: 131
            height: 32
            color: "#868aa6"
            text: qsTr(user_name)
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: 25
        }

        Connections {
            target: lv_mod
            onNewDataCallbackSig: {
                lv_mod.newTask
//                lv_mod.newRows
                console.log("New callback data signal received");
            }
        }
    }



    Rectangle{
        id: rect_alert_text_1
        x: 76
        width: parent.width
        height: 70
        color: "#2d325a"
        radius: 30
        anchors.top: recta_main.bottom
        anchors.topMargin: 20
        anchors.horizontalCenterOffset: 0
        anchors.horizontalCenter: parent.horizontalCenter
        Text{
            id: alert_text_1
            visible: start_visible
            width: parent.width-15
            text: ""
            font.bold: true
            color: "#ffffff"
            wrapMode: Text.WordWrap
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            horizontalAlignment: Text.AlignHCenter
            Connections{
                target: lv_mod_robot
                onHighProximitySig: {
                    alert_text_1.text = "<font color=\"#fc6262\">Warning!</font> High proximity task incoming. Attention to the surroundings"
                }
            }
            Connections{
                target: lv_mod_robot
                onStopHighProximitySig: {
                    alert_text_1.text = ""
                }
            }
        }
    }

    DropShadow {
        id: drop_shadow
        visible: false
        x: 0
        y: 0
        anchors.fill: recta_main
        horizontalOffset: 0
        verticalOffset: 0
        radius: 20
        anchors.topMargin: 0
        samples: 25
        color: "#ff4d4d"
        source: recta_main
        NumberAnimation {
            id: shadow_anim_start; target: drop_shadow; property: "radius"; from: 0; to: 25; duration: 500;
            onStarted: {
                drop_shadow.visible = true
            }
        }
        NumberAnimation {
            id: shadow_anim_stop; target: drop_shadow; property: "radius"; from: 25; to: 0; duration: 500;
            onStopped: {
                drop_shadow.visible = false
            }
        }
    }
    //Arrivo segnale di alta prossimità
    Connections{
        target: lv_mod_robot
        onHighProximitySig: {
            shadow_anim_start.start()
        }
    }
    Connections{
        target: lv_mod_robot
        onStopHighProximitySig: {
            shadow_anim_stop.start()
        }
    }


}
