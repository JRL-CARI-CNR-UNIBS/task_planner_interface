import QtQuick 2.5
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.1



import "../qml_components/pages"
import "../qml_components/agents"

ApplicationWindow
{
    id: applicationWindow
    visible: true
    width: 1920
    height: 1080
    color: "#202442"
    title: qsTr("Minimal Qml")



    signal addNewTask();
    signal removeItemSig1(string index_sig,string result_sig)
    signal removeItemSig2(string index_sig,string result_sig)
    signal removeItemSig3(string index_sig,string result_sig)
    signal removeItemSig4(string index_sig,string result_sig)
    signal acceptedUserTaskSig1(string index_sig);
    signal acceptedUserTaskSig2(string index_sig);
    signal endRemoveItemAnimationSig()
    signal overrideSig(string status);
    signal selectDateSig(string date);
    signal selectSessionSig(string session);
    signal removeSessionSig(int session_id, int idx_lv);


    //    Material.theme: Material.Dark

    StackView{
        id: myStackView
        anchors.bottomMargin: 54
        anchors.topMargin: 138
        anchors.leftMargin: 60
        anchors.rightMargin: 54
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.top: parent.top
        anchors.left: dash_menu.right

        replaceExit: Transition {
            ParallelAnimation{
                PropertyAnimation {
                    property: "opacity"
                    to: 0
                    duration: 50
                }
            }
        }

        initialItem: first_page
    }


    //Main Rectangle
    Component{
        id: first_page
        FirstPage{
        }
    }

    Component{
        id: second_page
        SecondPage{
        }
    }

    Component{
        id: third_page
        ThirdPage{
        }
    }

    function load_page(page){
        switch (page) {
            case 'Page 1':
                myStackView.replace(first_page);
                break;
            case 'Page 2':
                myStackView.replace(second_page);
                break;
            case 'Page 3':
                myStackView.replace(third_page);
                break;
        }
    }


    
    DashMenu{
        id: dash_menu
    }


    Text {
        id: text2
        y: 36
        width: 300
        color: "#646a7c"
        text: qsTr("pHRI Visual Interface")
        anchors.bottom: myStackView.top
        anchors.bottomMargin: 64
        anchors.left: myStackView.left
        anchors.leftMargin: 42
        font.pixelSize: 16
    }




















}





















