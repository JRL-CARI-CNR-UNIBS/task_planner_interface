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
    property int remove_time
    property string de: "#ffffff"

        Item {
            id: delegateItem
            width: lv.width
            height: edt.height
            visible: true
            Rectangle {
                id: rectan
                width: 3
                height: edt.height+35
                color: {
                    if (idx === 0) {
                        if (edt.proximity <= 0.33){return de = "#40f5a3"}
                        else if (edt.proximity > 0.33 && edt.proximity <= 0.66){return de = "#fcba03"}
                        else {return de = "#f54040"}
                    }
                    else {return de}
                }
                radius: 35
                anchors.leftMargin: 30;
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

//            Column{
//                id: column_right
//                anchors.right: rectan.left
//                anchors.rightMargin: 20
//                Text{
//                    id: text_time
//                    visible: true
//                    anchors.right: parent.right
//                    property int time_display_task: edt.expectedTime
//                    text: time_display_task + " s"
//                    color: "#ffffff"
//                    font.bold: true
//                    NumberAnimation {
//                        id: anim_time; target: text_time; property: "time_display_task"; from: edt.expectedTime; to: 0; duration: edt.expectedTime*1000;
//                    }
//                    Connections{
//                        target: backend
//                        onRemItemSig: {
//                            if (idx >= 0) {anim_time.to = edt.expectedTime}
//                            else {anim_time.to = 0
//                                anim_time.start()
//                            }
////                            console.log("remove w: "+idx)
//                        }
//                    }
//                }

//            }

            Rectangle{
                id: box_text_task
                property string countourColor: "#2d325a"
                anchors.left: rectan.left
                anchors.leftMargin: 20
//                anchors.bottomMargin: 20
                height: 90
                width: 235
                radius: 20
                color: "#ffffff"
                border.width: (width - (width-2*(1.0-0.7071067811)*radius))/1.5
                border.color: countourColor

                opacity: 0.1
                clip: true
            }

            Rectangle{
                id: box_text_task_contour
                color: "#00000000"
                radius: 20
                border.width: 10
                border.color: "#2d325a"
                height: box_text_task.height+10;
                width: box_text_task.width+10;
                anchors.horizontalCenter: box_text_task.horizontalCenter
                anchors.verticalCenter: box_text_task.verticalCenter
                opacity: 1
                clip: true
                z:1
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
                            text: edt.name
                            color: "#ffffff"
                        }
                    }


                    Text{
                        id: text_task_description
                        text: edt.id
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
                            text: edt.startName
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
                            text: edt.goalName
                            color: "#ffffff"
                        }
                    }
                }
            }
        }

}
