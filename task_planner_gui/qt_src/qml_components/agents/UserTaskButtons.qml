import QtQuick 2.5
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.1


Item{
    id: row_action_buttons
    width: 50
    height: parent.height
    anchors.top: parent.top
    anchors.bottom: parent.bottom
    anchors.right: parent.right
    anchors.rightMargin: 7
    Button {
        id: button_accept
        visible: true
        enabled: if (idx === 0) {
                     button_accept.enabled = true
                 }
                 else {
                     button_accept.enabled = false
                 }
        Connections {
            target: mod
            onActionButtonsActive: {
                button_accept.enabled = false
                if (idx === 0) {
                     button_accept.enabled = true
                 }
                 else {
                     button_accept.enabled = false
                 }
            }
        }

        width: parent.width
        height: parent.height/2+5
        anchors.right: parent.right
        anchors.rightMargin: 0
        anchors.top: parent.top
        anchors.topMargin: 0
        Material.background: Material.Green
        property int double_click: 0
        x: 31

        Image {
            id: image_play
            opacity: 0.5
            width: 30
            height: 30
            anchors.horizontalCenterOffset: -2
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            source: "../../assets/ic_done_48px.svg"
        }
        onClicked:
        {
//            switch (button_accept.p) {
//            case 0:
//                image_play.source = "../../assets/ic_done_all_48px.svg"
//                button_accept.Material.background = Material.LightGreen
//                button_accept.double_click = button_accept.double_click + 1
//                selectacceptedUserTaskSig(backend_number)
//                break

//            case 1:
                anim1.to = 0
                anim2.duration = 500
                selectRemoveItemSig(backend_number,"SUCCESS")
                button_cancel.enabled = false
                button_accept.enabled = false
//                break

//            }
        }

    }

    Button {
        id: button_cancel
        x: 0
        y: 44
        width:parent.width
        height: parent.height/2+5
        anchors.right: parent.right
        anchors.rightMargin: 0
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 0
        visible: true
        Material.background: Material.Red
        enabled: if (idx === 0) {
                     button_cancel.enabled = true
                 }
                 else {
                     button_cancel.enabled = false
                 }
        Connections {
            target: mod
            onActionButtonsActive: {
                if (idx === 0) {
                     button_cancel.enabled = true
                 }
                 else {
                     button_cancel.enabled = false
                 }
            }
        }

        Image {
            id: image_stop
            opacity: 0.5
            width: 30
            height: 30
            anchors.horizontalCenterOffset: -2
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            source: "../../assets/ic_clear_48px.svg"
        }
        onClicked: {
            selectRemoveItemSig(backend_number,"REJECTED")
            button_cancel.enabled = false
            button_accept.enabled = false
        }
    }
}
