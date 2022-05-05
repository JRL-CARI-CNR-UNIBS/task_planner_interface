import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Styles 1.4
import QtCharts 2.1

ChartView {
    property var axX: axisX
    property var axY: axisY
    antialiasing: true
    backgroundColor: "transparent"
    titleColor: "white"
    backgroundRoundness: 25
    Component.onCompleted: {
        margins.top = 20
        margins.bottom = 20
        legend.visible = false
    }


    ValueAxis {
        id: axisX
        labelsVisible: false
        gridVisible: false
        min: 0
        max: 100
        tickCount: 5
        color: "white"
        labelsColor: "white"
//        titleText: "<font color='white'>Tasks</font>"
    }
    ValueAxis {
        id: axisY
        min: 0
        max: 100
        tickCount: 5
        color: "white"
        labelsColor: "white"
        titleText: "<font color='white'>Time [s]</font>"

    }
    ValueAxis {
        id: axisY2
        min: 0
        max: 0
        tickCount: 5
        color: "white"
        labelsColor: "white"
        titleText: "<font color='white'>Time [s]</font>"

    }

}
