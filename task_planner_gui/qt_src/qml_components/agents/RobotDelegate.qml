import QtQuick 2.5
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.1

Item {

    property var mod
    property int idx
    property var edt
    property var lv
    property var anim1
    property var anim2
    property int backend_number
    property bool bttns_clickable



    //Specify here new removeItemSig
    function selectRemoveItemSig(bkc_num, result) {
        switch (bkc_num) {
        case 1:
            applicationWindow.removeItemSig1(idx,result)
            break;
        case 2:
            applicationWindow.removeItemSig2(idx,result)
            break;
        case 3:
            applicationWindow.removeItemSig3(idx,result)
            break;
        case 4:
            applicationWindow.removeItemSig4(idx,result)
            break;

        }
    }

    function selectacceptedUserTaskSig(bkc_num) {
        switch (bkc_num) {
        case 2:
            applicationWindow.acceptedUserTaskSig1(idx)
            break;
        case 3:
            applicationWindow.acceptedUserTaskSig2(idx)
            break;
       }
    }

    Item {
        id: delegateItem
        width: lv.width
        height: edt.height
        Rectangle {
            id: rectan
            width: 3
            height: edt.height+55
            radius: 60
            anchors.leftMargin: 30
            anchors.left: parent.left
            Rectangle{
                id: inner_rect
                width: 20
                height: inner_rect.width
                color: "#2f5bcc"
                radius: inner_rect.width
                anchors.horizontalCenter: parent.horizontalCenter
                Rectangle{
                    id: inner_inner_rect
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

        UserTaskBox{
            taskID: edt.id
            taskName: edt.name
            startName:edt.startName
            goalName: edt.goalName
            anchors.left: rectan.right
            anchors.leftMargin: 20
//            UserTaskButtons{
//                z: -1
//            }
        }

    }

}
