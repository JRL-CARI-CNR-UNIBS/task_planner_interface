import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Styles 1.4
import QtCharts 2.1
import QtGraphicalEffects 1.0
import QtQuick.Layouts 1.3


import "../agents"
import "../plots"


Item {
    width: 1560
    height: 830


    Rectangle {
        id: rectangle2
        anchors.fill: parent
        color: "#25294a"
        radius: 50
        anchors.bottomMargin: 0
        border.width: 0
        Material.elevation: 102

        Text {
            id: text1
            y: -67
            width: 262
            height: 47
            text: qsTr("Settings")
            anchors.bottom: parent.top
            anchors.bottomMargin: 15
            anchors.left: parent.left
            anchors.leftMargin: 40
            font.bold: true
            font.pixelSize: 40
            color: "#ffffff"
        }

    }


}
