import QtQuick 2.0
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Styles 1.4
import QtCharts 2.1
import QtGraphicalEffects 1.0
import QtQuick.Window 2.2

import "../agents"

Item{
    id: popupPoints
    property var taskID
    width: 12
    height: 12
    opacity: 1

    Text{
        text: taskID
        color: "white"
    }
}



