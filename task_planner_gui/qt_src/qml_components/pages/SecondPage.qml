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
            text: qsTr("Data Analysis")
            anchors.bottom: parent.top
            anchors.bottomMargin: 15
            anchors.left: parent.left
            anchors.leftMargin: 40
            font.bold: true
            font.pixelSize: 40
            color: "#ffffff"
        }

        ComboBox {
            id: userComboBox
            y: 88
            currentIndex: 2
            model: ListModel {
                id: cbItems
                ListElement { text: "User 1"; color: "Yellow" }
            }
            height: 35
            anchors.right: rectCalendar.right
            anchors.rightMargin: 0
            anchors.left: rectCalendar.left
            anchors.leftMargin: 0
            onCurrentIndexChanged: console.debug(cbItems.get(currentIndex).text + ", " + cbItems.get(currentIndex).color)
        }

        Rectangle {
            id: rectCalendar
            y: 137
            height: 300
            anchors.right: parent.right
            anchors.rightMargin: 50
            radius: 2
            anchors.left: rectTaskSessions.left
            anchors.leftMargin: 0
            color: "#2d325a"
            Calendar {
                property var previous_date: Date()
                anchors.right: parent.right
                anchors.rightMargin: 15
                anchors.left: parent.left
                anchors.leftMargin: 15
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 15
                anchors.top: parent.top
                anchors.topMargin: 15
                frameVisible: false
                visibleMonth: Date().getMonth()
                minimumDate: new Date(2018, 0, 1)
                maximumDate: {
                    //Modify the date to add current day to maximumDate
                    var new_date = new Date()
                    new_date.setDate(new_date.getDate())
                    new_date.setFullYear(new_date.getFullYear())
                    new_date.setMonth(new_date.getMonth())
                    return new_date
                }
                onClicked: {
                    if (selectedDate.toString() !== previous_date.toString()){
                        applicationWindow.selectDateSig(selectedDate.toLocaleDateString(locale,'ddd MMM d yyyy'))
                    }
                    previous_date = selectedDate
                }
                Component.onCompleted: {
                    applicationWindow.selectDateSig(selectedDate.toLocaleDateString(locale,'ddd MMM d yyyy'))
                    previous_date = selectedDate
                }

                style: CalendarStyle {
                    gridVisible: false
                    background: Rectangle{
                        color: "#2d325a"
                    }
                    dayOfWeekDelegate: Rectangle{
                        color: "#2d325a"
                        height: 30
                        Label {
                            text: locale.dayName(styleData.index, Locale.ShortFormat)
                            anchors.centerIn: parent
                            color: "white"
                        }
                    }

                    dayDelegate: Rectangle {
                        color: styleData.selected ? "#646b9e" : (styleData.visibleMonth && styleData.valid ? "#2d325a" : "#25294a");
                        Label {
                            text: styleData.date.getDate()
                            anchors.centerIn: parent
                            color: styleData.valid ? "white" : "grey"
                        }

                    }
                }
            }

        }










        Rectangle {
            id: rectLinePlots
            width: 600
            radius: 30
            color: "#2d325a"
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 50
            anchors.top: parent.top
            anchors.topMargin: 50
            anchors.left: parent.left
            anchors.leftMargin: 50
            Column{
                id: column
                height: 699
                anchors.verticalCenter: parent.verticalCenter
                anchors.right: parent.right
                anchors.left: parent.left
                spacing: 5

                DefaultChart {
                    id: inner_chart_view1
                    height: 230
                    anchors.right: parent.right
                    anchors.rightMargin: 0
                    anchors.left: parent.left
                    anchors.leftMargin: 0
//                    animationOptions: ChartView.SeriesAnimations
                    Component.onCompleted: {
                        inner_chart_view1.axY.titleText = "<font color='white'>Task Time [s]</font>"
                    }

                    Item{
                        id:it05
                    }

                    LineSeries {
                        id: inner_line_series1
                        name: "LineSeries"
                        axisX: inner_chart_view1.axX
                        axisY: inner_chart_view1.axY
                        color: "white"
                        pointsVisible: true
                        width: 5

                        NumberAnimation {
                            id: startAnim5; target: inner_chart_view1; property: "opacity"; from: 0; to: 1.0; duration: 500;
                        }

                        Connections{
                            target: vs_us_backend
                            onUpdatePlotDataSig: {
                                inner_line_series1.clear()
                                startAnim5.start()
                                inner_chart_view1.axY.min = 0
                                inner_chart_view1.axX.min = 0
                                inner_chart_view1.axX.max = vs_us_backend.plotCount()-1
                                inner_chart_view1.axX.tickCount = vs_us_backend.plotCount()

                                for(var i = 0; i < it05.children.length; i++)
                                    it05.children[i].destroy()

                                var time_tot = 0.0
                                for(i = 0; i < vs_us_backend.plotCount(); i++)
                                    time_tot = time_tot + vs_us_backend.getTime(i)
                                inner_chart_view1.axY.max = time_tot*1.1

                                time_tot = 0.0
                                for(i = 0; i < vs_us_backend.plotCount(); i++) {
                                    time_tot = time_tot + vs_us_backend.getTime(i)
                                    inner_line_series1.append(i, time_tot) // point by point append - slow

                                    var p = inner_chart_view1.mapToPosition(inner_line_series1.at(i), inner_line_series1);
                                    var taskID = vs_us_backend.getTaskID(i)
                                    var taskName = vs_us_backend.getTaskName(i)
                                    var taskStart = vs_us_backend.getStart(i)
                                    var taskGoal = vs_us_backend.getGoal(i)

                                    var component = Qt.createComponent("../plots/TextPoints.qml");
                                    if (component.status == Component.Ready)
                                        component.createObject(it05, {x: p.x-10, y: p.y-25, popup_x: p.x, popup_y: p.y, point_idx:i, taskID: taskID, taskName: taskName, taskStart: taskStart, taskGoal: taskGoal});

                                }



                            }
                        }

                    }

                    Text {
                        id: text4
                        x: 221
                        width: 194
                        height: 32
                        color: "#868aa6"
                        text: qsTr("Robot Task Time")
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.top: parent.top
                        anchors.topMargin: -10
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignHCenter
                        font.pixelSize: 20
                    }
                }



                DefaultChart{
                    id: inner_chart_view2
                    height: 230
                    anchors.right: parent.right
                    anchors.left: parent.left
                    anchors.leftMargin: 0
                    anchors.rightMargin: 0
//                    animationOptions: ChartView.SeriesAnimations
                    Component.onCompleted: {
                        inner_chart_view2.axY.titleText = "<font color='white'>DistanceHR [m]</font>"
                    }

                    Item{
                        id:it04
                    }

                    LineSeries {
                        id: inner_line_series2
                        name: "LineSeries"
                        axisX: inner_chart_view2.axX
                        axisY: inner_chart_view2.axY
                        color: "white"
                        pointsVisible: true
                        width: 5

                        NumberAnimation {
                            id: startAnim4; target: inner_chart_view2; property: "opacity"; from: 0; to: 1.0; duration: 500;
                        }

                        Connections{
                            target: vs_us_backend
                            onUpdatePlotDataSig: {
                                inner_line_series2.clear()
                                startAnim4.start()
                                inner_chart_view2.axY.min = 0.0
                                inner_chart_view2.axY.max = vs_us_backend.getMaxDistanceHR()
                                inner_chart_view2.axX.min = 0
                                inner_chart_view2.axX.max = vs_us_backend.plotCount()-1
                                inner_chart_view2.axX.tickCount = vs_us_backend.plotCount()

                                for(var i = 0; i < it04.children.length; i++)
                                    it04.children[i].destroy()

                                for(var i = 0; i < vs_us_backend.plotCount(); i++) {
                                    inner_line_series2.append(i, vs_us_backend.getDistanceHR(i)) // point by point append - slow

                                    var p = inner_chart_view2.mapToPosition(inner_line_series2.at(i), inner_line_series2);
                                    var taskID = vs_us_backend.getTaskID(i)
                                    var taskName = vs_us_backend.getTaskName(i)
                                    var taskStart = vs_us_backend.getStart(i)
                                    var taskGoal = vs_us_backend.getGoal(i)

                                    var component = Qt.createComponent("../plots/TextPoints.qml");
                                    if (component.status == Component.Ready)
                                        component.createObject(it04, {x: p.x-10, y: p.y-25, popup_x: p.x, popup_y: p.y, point_idx:i, taskID: taskID, taskName: taskName, taskStart: taskStart, taskGoal: taskGoal});

                                }
                            }
                        }

                    }

                    Text {
                        id: text3
                        x: 221
                        width: 194
                        height: 32
                        color: "#868aa6"
                        text: qsTr("DistanceHR")
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.top: parent.top
                        anchors.topMargin: -10
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignHCenter
                        font.pixelSize: 20
                    }
                }

                DefaultChart {
                    id: inner_chart_view3
                    y: 1
                    height: 230
                    anchors.right: parent.right
                    anchors.left: parent.left
//                    animationOptions: ChartView.SeriesAnimations
                    Component.onCompleted: {
                        inner_chart_view3.axY.titleText = "<font color='white'>Robot Override</font>"
                    }

                    Item{
                        id:it03
                    }

                    LineSeries {
                        id: inner_line_series3
                        name: "LineSeries"
                        width: 5
                        color: "#ffffff"

                        NumberAnimation {
                            id: startAnim3; target: inner_chart_view3; property: "opacity"; from: 0; to: 1.0; duration: 500;
                        }

                        Connections{
                            target: vs_us_backend
                            onUpdatePlotDataSig: {
                                inner_line_series3.clear()
                                startAnim3.start()

                                console.log("quaa")

                                inner_chart_view3.axY.min = 0.0
                                inner_chart_view3.axY.max = vs_us_backend.getMaxSpeedOvr()
                                inner_chart_view3.axX.min = 0
                                inner_chart_view3.axX.max = vs_us_backend.plotCount()-1
                                inner_chart_view3.axX.tickCount = vs_us_backend.plotCount()

                                console.log("quaas")

                                for(var i = 0; i < it03.children.length; i++)
                                    it03.children[i].destroy()

                                for(var i = 0; i < vs_us_backend.plotCount(); i++) {
                                    inner_line_series3.append(i, vs_us_backend.getSpeedOvr(i)) // point by point append - slow

                                    var p = inner_chart_view3.mapToPosition(inner_line_series3.at(i), inner_line_series3);
                                    var taskID = vs_us_backend.getTaskID(i)
                                    var taskName = vs_us_backend.getTaskName(i)
                                    var taskStart = vs_us_backend.getStart(i)
                                    var taskGoal = vs_us_backend.getGoal(i)

                                    var component = Qt.createComponent("../plots/TextPoints.qml");
                                    if (component.status == Component.Ready)
                                        component.createObject(it03, {x: p.x-10, y: p.y-25, popup_x: p.x, popup_y: p.y, point_idx:i, taskID: taskID, taskName: taskName, taskStart: taskStart, taskGoal: taskGoal});

                                }

                                console.log("quae")
                            }
                        }
                        pointsVisible: true
                        axisX: inner_chart_view3.axX
                        axisY: inner_chart_view3.axY
                    }

                    Text {
                        id: text5
                        x: 203
                        width: 194
                        height: 32
                        color: "#868aa6"
                        text: qsTr("Override Robot")
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.top: parent.top
                        anchors.topMargin: -10
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignHCenter
                        font.pixelSize: 20
                    }
                    anchors.topMargin: 562
                    anchors.rightMargin: 0
                    anchors.bottomMargin: 47
                    anchors.leftMargin: 0
                }

            }
        }









        Rectangle {
            id: rectScatterPlots
            x: -4
            y: -7
            width: 440
            color: "#2d325a"
            radius: 30
            anchors.topMargin: 50
            anchors.bottom: parent.bottom
            anchors.leftMargin: 50
            Column {
                id: column1
                height: 722
                spacing: 5
                anchors.right: parent.right
                anchors.left: parent.left
                anchors.verticalCenter: parent.verticalCenter

                DefaultChart {
                    id: chart_view5
                    width: 431
                    height: 360
                    anchors.horizontalCenter: parent.horizontalCenter
//                    antialiasing: true
                    z: 2
//                    animationDuration: 2000
//                    animationOptions: ChartView.SeriesAnimations
                    Component.onCompleted: {
                        chart_view5.axX.labelsVisible = true
                        chart_view5.axX.gridVisible = true
                        chart_view5.axY.max = 100
                        chart_view5.axX.titleText = "<font color='white'>Task Time [s]</font>"
                        chart_view5.axY.titleText = "<font color='white'>Robot Override</font>"


                    }


                    Item{
                        id:it01
                    }

                    ScatterSeries {
                        id: scatter2
                        name: "Scatter1"
                        axisX: chart_view5.axX
                        axisY: chart_view5.axY
                        color: "white"
                        borderColor: "#000000"
                        borderWidth: 3
                        markerShape: ScatterSeries.MarkerShapeCircle
                        markerSize: 15

                        NumberAnimation {
                            id: startAnim; target: chart_view5; property: "opacity"; from: 0; to: 1.0; duration: 500;
                        }

                        Connections{
                            target: vs_us_backend
                            onUpdatePlotDataSig: {

                                chart_view5.axX.max = 100.0
                                chart_view5.axY.max = 100.0

                                startAnim.start()
                                scatter2.removePoints(0,scatter2.count)

                                for(var i = 0; i < it01.children.length; i++)
                                    it01.children[i].destroy()

                                for(var i = 0; i < vs_us_backend.plotCount(); i++) {
                                    scatter2.append(vs_us_backend.getTime(i),vs_us_backend.getSpeedOvr(i))
                                }

//                                chart_view5.axX.max = vs_us_backend.getMaxTime()
                                for(var i = 0; i < vs_us_backend.plotCount(); i++) {
                                    var p = chart_view5.mapToPosition(scatter2.at(i), scatter2);
                                    var taskID = vs_us_backend.getTaskID(i)
                                    var taskName = vs_us_backend.getTaskName(i)
                                    var taskStart = vs_us_backend.getStart(i)
                                    var taskGoal = vs_us_backend.getGoal(i)

                                    var component = Qt.createComponent("../plots/LabelPoints.qml");
                                    if (component.status == Component.Ready)
                                        component.createObject(it01, {x: p.x-scatter2.markerSize/2, y: p.y-scatter2.markerSize/2, popup_x: p.x, popup_y: p.y, point_idx:i, taskID: taskID, taskName: taskName, taskStart: taskStart, taskGoal: taskGoal});
//                                    marker_size:scatter2.markerSize,
                                }

                            }
                        }
                    }
                    Item {
                        id: gradient_background2
                        anchors.bottomMargin: 72
                        anchors.leftMargin: 93
                        anchors.rightMargin: 41
                        anchors.topMargin: 36
                        anchors.fill: parent
                        opacity: 1
                        z: -1
                        RadialGradient {
                            anchors.fill: parent
                            horizontalOffset: -parent.width*0.5
                            verticalOffset: parent.height*0.5
                            gradient: Gradient{
                                GradientStop { position: 0.40; color: "#81c784" }
                                GradientStop { position: 0.80; color: "#ffb74d" }
                                GradientStop { position: 1.00; color: "#ef5350" }
                            }
                        }
                    }
                }

                DefaultChart {
                    id: chart_view4
                    width: 431
                    height: 360
                    anchors.horizontalCenter: parent.horizontalCenter
                    antialiasing: true
                    z: 2
                    Component.onCompleted: {
                        chart_view4.axX.labelsVisible = true
                        chart_view4.axX.gridVisible = true
                        chart_view4.axY.max = 5
                        chart_view4.axX.titleText = "<font color='white'>Task Time [s]</font>"
                        chart_view4.axY.titleText = "<font color='white'>DistanceHR [m]</font>"
                    }

                    Item{
                        id:it02
                    }

                    ScatterSeries {
                        id: scatter1
                        name: "Scatter1"
                        axisX: chart_view4.axX
                        axisY: chart_view4.axY
                        color: "white"
                        borderColor: "#000000"
                        borderWidth: 3

                        NumberAnimation {
                            id: startAnim2; target: chart_view4; property: "opacity"; from: 0; to: 1.0; duration: 500;
                        }

                        Connections{
                            target: vs_us_backend
                            onUpdatePlotDataSig: {

                                chart_view4.axX.max = 100.0
                                chart_view4.axY.max = 5.0

                                startAnim2.start()
                                scatter1.removePoints(0,scatter1.count)

                                for(var i = 0; i < it02.children.length; i++)
                                    it02.children[i].destroy()

                                for(var i = 0; i < vs_us_backend.plotCount(); i++) {
                                    scatter1.append(vs_us_backend.getTime(i),vs_us_backend.getDistanceHR(i))
                                }

                                for(var i = 0; i < vs_us_backend.plotCount(); i++) {
                                    var p = chart_view4.mapToPosition(scatter1.at(i), scatter1);
                                    var taskID = vs_us_backend.getTaskID(i)
                                    var taskName = vs_us_backend.getTaskName(i)
                                    var taskStart = vs_us_backend.getStart(i)
                                    var taskGoal = vs_us_backend.getGoal(i)

                                    var component = Qt.createComponent("../plots/LabelPoints.qml");
                                    if (component.status == Component.Ready)
                                        component.createObject(it02, {x: p.x-scatter1.markerSize/2, y: p.y-scatter1.markerSize/2, popup_x: p.x, popup_y: p.y, point_idx:i, taskID: taskID, taskName: taskName, taskStart: taskStart, taskGoal: taskGoal});

                                }
                            }
                        }
                    }
                    Item {
                        id: gradient_background
                        width: 280
                        height: 230
                        anchors.verticalCenterOffset: 5
                        anchors.horizontalCenterOffset: 0
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
//                        anchors.fill: parent
                        opacity: 1
                        z: -1
                        RadialGradient {
                            anchors.rightMargin: -29
                            anchors.bottomMargin: 13
                            anchors.leftMargin: 0
                            anchors.topMargin: -35
                            anchors.fill: parent
                            horizontalOffset: -parent.width*0.5
                            verticalOffset: -parent.height*0.5
                            gradient: Gradient{
                                GradientStop { position: 0.40; color: "#81c784" }
                                GradientStop { position: 0.80; color: "#ffb74d" }
                                GradientStop { position: 1.00; color: "#ef5350" }
                            }
                        }
                    }
                }

            }
            anchors.left: rectLinePlots.right
            anchors.bottomMargin: 50
            anchors.top: parent.top
        }

        Rectangle {
            id: rectTaskSessions
            x: 0
            y: -11
            color: "#2d325a"
            radius: 30
            anchors.right: rectCalendar.right
            anchors.rightMargin: 0
            anchors.topMargin: 491
            anchors.bottom: parent.bottom
            anchors.leftMargin: 50
            anchors.left: rectScatterPlots.right
            anchors.bottomMargin: 50
            anchors.top: parent.top

            ListView {
                id: list_sessions
                anchors.rightMargin: 0
                anchors.leftMargin: 20
                anchors.bottomMargin: 33
                anchors.topMargin: 51
                anchors.fill: parent
                clip: true
                spacing: 7
                model: vs_us_backend.model
                focus: true

                highlight: Rectangle {
                    color: 'grey'
                    opacity: 0.3
                    z: 2
                }

                highlightMoveDuration: 100
                ScrollBar.vertical: ScrollBar {
                    id: scrollBarLv
                    active: true

                }
                add: Transition {
                        NumberAnimation { property: "opacity"; from: 0; to: 1.0; duration: 200 }
                        NumberAnimation { property: "x"; from: -500; to: 0; duration: 200 }
                    }
                remove: Transition {
                        NumberAnimation { property: "opacity"; from: 1.0; to: 0; duration: 400 }
                        NumberAnimation { property: "x"; to: -500.0; duration: 400 }
                    }
                displaced: Transition {
                    NumberAnimation { properties: "x,y"; duration: 800; easing.type: Easing.OutBounce }
                }
                delegate: Rectangle{
                    id: rectMainListView
                    height: 30
                    width: list_sessions.width-1.1*scrollBarLv.width
                    color: "#2d325a"

                    Component.onCompleted: {
//                        console.log("curr index     ",list_sessions.currentIndex)
                        //TODO Meglio non ripeterno n volte (per ogni delegate), riduce prestazioni
//                        changeIdxAnim.start()
                    }
                    NumberAnimation {
                        id: changeIdxAnim
                        target: list_sessions; property: "currentIndex";  to: list_sessions.count-1;
                    }

                    RowLayout{
                        anchors.fill: parent

                        Rectangle {
                            id: component_bttn_add
                            anchors.verticalCenter: parent.verticalCenter
                            anchors.left: parent.left
                            width: rectMainListView.height
                            height: rectMainListView.height
                            Button {
                                id: bttn_add
                                anchors.fill: parent
                                background: Rectangle {
                                    color: "#f55353"
                                    opacity: 1
                                }
                                Image {
                                    id: image_stop
                                    anchors.fill: parent
                                    opacity: 0.5
                                    scale: 0.5
                                    source: "../../assets/ic_clear_48px.svg"
                                }
                                onClicked: {
                                    applicationWindow.removeSessionSig(edit.id,index);
                                }
                            }
                        }

                        Item{
                            id: rectInnerListView
                            Layout.fillWidth: true
                            anchors.top: parent.top
                            anchors.bottom: parent.bottom
                            MouseArea {
                                id: mouseAreaLvItem
                                anchors.fill: parent
                                onClicked: {
                                    list_sessions.currentIndex = index
                                    applicationWindow.selectSessionSig(edit.id)
                                }
                            }

                            RowLayout{
                                anchors.fill: parent
                                spacing: 1
                                Item{
                                    id: rectLeftLv
                                    width: rectInnerListView.width*0.5
                                    anchors.top: parent.top
                                    anchors.bottom: parent.bottom
                                    Text {
                                        anchors.verticalCenter: parent.verticalCenter
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        color: "white"
                                        text: "Session " + edit.id
                                    }
                                }
                                Item{
                                    width: rectInnerListView.width*0.5
                                    anchors.top: parent.top
                                    anchors.bottom: parent.bottom
                                    anchors.left: rectLeftLv.right
                                    Text {
                                        anchors.verticalCenter: parent.verticalCenter
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        color: "white"
                                        text: edit.clock
                                    }
                                }
                            }

                        }

                    }
                }





            }

            Text {
                id: titleLvRect
                x: 203
                width: 194
                height: 32
                color: "#868aa6"
                text: qsTr("Available Sessions")
                anchors.bottom: parent.top
                anchors.bottomMargin: 5
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 20
            }

            Rectangle {
                id: rectLimitBotLine
                width: 300
                height: 2
                color: "#4d4f64"
                anchors.right: parent.right
                anchors.rightMargin: 10
                anchors.left: parent.left
                anchors.leftMargin: 10
                anchors.top: list_sessions.bottom
                anchors.topMargin: 0
            }

            Text {
                id: titleLvRect1
                x: 206
                y: 13
                width: 32
                height: 32
                color: "#868aa6"
                text: qsTr("Id")
                anchors.horizontalCenterOffset: -54
                anchors.bottom: parent.top
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                horizontalAlignment: Text.AlignHCenter
                anchors.bottomMargin: -45
                font.pixelSize: 15
            }

            Text {
                id: titleLvRect2
                x: 213
                y: 13
                width: 39
                height: 32
                color: "#868aa6"
                text: qsTr("Time")
                anchors.bottom: parent.top
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenterOffset: 79
                anchors.horizontalCenter: parent.horizontalCenter
                horizontalAlignment: Text.AlignHCenter
                anchors.bottomMargin: -45
                font.pixelSize: 15
            }
        }

        Text {
            id: titleLvRect3
            x: 203
            y: 50
            width: 194
            height: 32
            color: "#868aa6"
            text: qsTr("Calendar Sessions")
            anchors.horizontalCenterOffset: 0
            anchors.bottom: userComboBox.top
            verticalAlignment: Text.AlignVCenter
            anchors.horizontalCenter: rectCalendar.horizontalCenter
            horizontalAlignment: Text.AlignHCenter
            anchors.bottomMargin: 5
            font.pixelSize: 20
        }







    }





}
