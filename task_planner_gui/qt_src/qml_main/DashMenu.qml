import QtQuick 2.5
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.1


Rectangle {
    width: 191
    color: "#2d325a"
    radius: 40
    anchors.left: parent.left
    anchors.leftMargin: 50
    anchors.bottom: parent.bottom
    anchors.bottomMargin: 54
    anchors.top: parent.top
    anchors.topMargin: 50

    ListModel {
        id: dash_model
        ListElement {
            name: qsTr("Task Manager")
            source_image: "../assets/ic_dashboard_48px.svg"
        }
        ListElement {
            name: qsTr("Data Analysis")
            source_image: "../assets/ic_assessment_48px.svg"
        }
        ListElement {
            name: qsTr("Settings")
            source_image: "../assets/ic_settings_48px.svg"
        }
    }

    ListView {
        id: dash_menu_lv
        y: 123
        height: 299
        anchors.right: parent.right
        anchors.rightMargin: 0
        anchors.left: parent.left
        anchors.leftMargin: 0
        spacing: 20
        model: dash_model
        highlightMoveDuration: 100
        property string curr_page: "Page 1" //Starting page name

        highlight: Rectangle {
            z: 2
            color: "white"
            opacity: 0.2
        }
        delegate: Rectangle {
            id: item_menu1
            y: 134
            height: 54
            color: "#454867"
            opacity: ListView.isCurrentItem ? 1 : 0.3
            anchors.right: parent.right
            anchors.rightMargin: 0
            anchors.left: parent.left
            anchors.leftMargin: 0
            MouseArea {
                anchors.fill: parent
                onClicked: {
                    dash_menu_lv.currentIndex = index
                    switch (dash_menu_lv.currentIndex) {
                    case 0:
                        if (dash_menu_lv.curr_page !== "Page 1")
                            load_page("Page 1")
                        dash_menu_lv.curr_page = "Page 1"
                        break
                    case 1:
                        if (dash_menu_lv.curr_page !== "Page 2")
                            load_page("Page 2")
                        dash_menu_lv.curr_page = "Page 2"
                        break
                    case 2:
                        if (dash_menu_lv.curr_page !== "Page 3")
                            load_page("Page 3")
                        dash_menu_lv.curr_page = "Page 3"
                        break
                    }
                }

            }
            RowLayout {
                id: row1
                x: 0
                y: 0
                anchors.fill: parent

                Text {
                    id: item_menu_text1
                    width: 100
                    height: 26
                    color: "#ffffff"
                    text: name
                    font.bold: true
                    anchors.left: parent.left
                    anchors.leftMargin: 60
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    anchors.verticalCenter: parent.verticalCenter
                    font.pixelSize: 17
                }

                Rectangle {
                    id: item_menu_rect1
                    width: 10
                    color: "#31e445"
                    anchors.right: parent.right
                    anchors.rightMargin: 0
                    anchors.top: parent.top
                    anchors.topMargin: 0
                    anchors.bottom: parent.bottom
                    anchors.bottomMargin: 0
                }

                Image {
                    id: item_menu_image1
                    width: 40
                    height: 40
                    scale: 0.7
                    opacity: 1
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.left: parent.left
                    anchors.leftMargin: 5
                    source: source_image
                    ColorOverlay {
                        anchors.fill: parent
                        source: item_menu_image1
                        color: "#ffffff"  // make image like it lays under red glass
                    }
                }
            }
        }
    }


}

