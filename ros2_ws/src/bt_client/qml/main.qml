import QtQml
import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Dialogs
import QtQuick.Layouts
import QtQuick.Shapes  // for dynamic arrows
import QtLocation
import QtPositioning

import "BtUtils.js" as BtUtils
import "LayoutUtils.js" as LayoutUtils
import "TranslationUtils.js" as TranslationUtils

import "."


ApplicationWindow {
    id: window
    width: 960
    height: 640
    visible: true
    title: "Mission Client"

    property string currentLanguage: "zh"
    property color backgroundColor: "#080808"
    property var nodeMap: [] 
    property int cliHeaderHeight: 49
    Shortcut {
        sequence: "Esc"
        onActivated: commandLineWindow.toggleWindow()
    }

    // Bt state parser
    Connections {
        target: rosBridge
        function onBtXmlReceived(xmlText) {
            let btRoot = btParser.parseFromXml(xmlText)
            if (btRoot)
                missionProgressVisualizer.updateBtTree(btRoot) 
        }
    }

    // Background
    Rectangle {
        id: background
        anchors.fill: parent
        color: backgroundColor
        z: 0
    }

    // ToolBar at the top
    header: ToolBar {
        height: 40
        Material.background: "#D0D0D0"

        Flow {
            anchors.fill: parent
            anchors.rightMargin: logo.width + 10

            ToolButton {
                id: menuButton
                text: "☰"  // Hamburger menu symbol
                font.pixelSize: 16
                height: parent.height

                contentItem: Text {
                        text: parent.text
                        font: parent.font
                        color: "#000000"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                }

                onClicked: {
                    contextMenu.popup()
                }

                Menu {
                    id: contextMenu
                    y: parent.height

                    Action {
                        text: qsTr("匯出任務")  // "Export Mission"
                        icon.name: "document-save"
                        onTriggered: exportBT()
                    }
            
                    MenuSeparator {}
            
                    Action {
                        text: qsTr("語言切換")  // "Switch Language"
                        icon.name: "preferences-desktop-locale"
                        onTriggered: switchLanguage()
                    }
            
                    MenuSeparator {}
            
                    Action {
                        text: qsTr("結束程式")  // "Exit"
                        icon.name: "application-exit"
                        onTriggered: Qt.quit()
                    }
                }
            }

            Label {
                anchors.verticalCenter: parent.verticalCenter
                anchors.left: menuButton.right
                anchors.leftMargin: 5
                text: "USV Mission Client"
                font.pixelSize: 16
                font.bold: true
                color: "#303030"
                elide: Label.ElideRight
            }
        }

        Image {
            id: logo
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.right: parent.right
            source: "qrc:/resources/logo.jpg"
            height: parent.height
            width: height * 3.5
            // fillMode: Image.PreserveAspectFit
            smooth: true
        }
    }

    // All panels container
    ColumnLayout {
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.bottomMargin: cliHeaderHeight

        spacing: 0

        // Top panel with tabs
        TabBar {
            id: tabBar
            Layout.fillWidth: true
            currentIndex: 0 

            // Define the custom TabButton component inline
            component StyledTabButton: TabButton {
                property string buttonText: ""
                implicitHeight: 35
        
                text: buttonText
        
                contentItem: Text {
                    text: parent.text
                    font.pixelSize: 16
                    font.bold: parent.checked
                    color: {
                        if (!parent.enabled) return "#aaaaaa"
                        if (parent.checked) return "#ffffff"
                        if (parent.hovered) return "#eeeeee"
                        return "#999999"
                    }
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    elide: Text.ElideRight
                }

                background: Rectangle {
                    color: {
                        if (!parent.enabled) return "#0a0a0a"
                        if (parent.checked) return "#790707"
                        if (parent.hovered) return "#808080"
                        if (parent.pressed) return "#500303"
                        return "#2a2a2a"
                    }
                    border.color: {
                        if (parent.checked) return "#240000"
                        return "#555555"
                    }
                    border.width: 1
                    radius: 0
    
                    Behavior on color {
                        ColorAnimation { duration: 150 }
                    }
                }
            }

            StyledTabButton {
                buttonText: qsTr("任務進度")
            }
            StyledTabButton {
                buttonText: qsTr("編輯任務")
            }
            StyledTabButton {
                buttonText: qsTr("航線規劃")
            }
            // StyledTabButton {
            //     buttonText: qsTr("雲台控制")
            // }
            StyledTabButton {
                buttonText: qsTr("飛控參數")
            }
        }

        // Tab contents
        StackLayout {
            currentIndex: tabBar.currentIndex
            Layout.fillWidth: true
            Layout.fillHeight: true

            // Tab:  Mission Progress
            BtVisualizer {
                id: missionProgressVisualizer
                btParserRef: btParser
                currentLanguage: currentLanguage
                showRealTimeUpdates: true
                showStatusColors: true
            }

            // Tab: Edit Mission
            MissionEditPanel{
                id: missionEditPanel
                btParserRef: btParser
                treeNodesModelParserRef: treeNodesModelParser
            }

            // Tab: GPS Navigation
            NavigationPanel {
                id: navigationPanel
                anchors.fill: parent
                vehicleTrackerRef: vehicleTracker
                fileManagerRef: fileManager
                center: QtPositioning.coordinate(24.54178, 121.86804)
            }

            // Tab: Gimbal Control
            // GimbalControlPanel {
            //     id: gimbalControlPanel
            //     gimbalStreamerRef: gimbalStreamer
            //     gimbalControllerRef: gimbalController
            //     ros2TopicSubscriberRef: ros2TopicSubscriber
            // } 

            // Tab: MAVLink Panel
            MavlinkPanel {
                id: mavlinkPanel
                anchors.fill: parent
                mavlinkClientRef: mavlinkClient
            }
        }
    }

    // Command line window
    CommandLineWindow {
        id: commandLineWindow
        anchors.left: parent.left
        anchors.right:parent.right
        qmlCliRef: qmlCli
        z: 2000  // High z-index to appear above everything
    }

    // -----------------------------------------
    //             Helper functions
    // -----------------------------------------
    function switchLanguage() {
        currentLanguage = TranslationUtils.switchLanguage(currentLanguage)
        missionProgressVisualizer.currentLanguage = currentLanguage
    }
}
