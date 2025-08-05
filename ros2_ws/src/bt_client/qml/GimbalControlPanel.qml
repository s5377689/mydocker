import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window

Item {
    id: gimbalControlPanel
    property var gimbalStreamer
    property var gimbalController
    property var ros2TopicSubscriber
    property bool streamerConnected: gimbalStreamer.isConnected()
    property bool controllerConnected: gimbalController.isConnected()
    property string imageSource: "gimbal"  // or "ros2"

    property color themeColor: "#2b2b2b"
    property color accentColor: "#AFAFFF"
    property color bgColor: "#1e1e1e"
    property color textColor: "#1f1f1f"

    property string targetYaw: ""
    property string targetPitch: ""
    property string velocity: "10" // Default velocity for gimbal yaw/pitch rotation
    property string zoom_int: "" // Zoom level integer part
    property string zoom_frac: "" // Zoom level fractional part

    Rectangle {
        anchors.fill: parent
        color: bgColor

        RowLayout {
            anchors.fill: parent
            anchors.margins: 16
            spacing: 16

            // =======================
            // === Live Camera Feed ===
            // =======================
            Rectangle {
                id: videoPanel
                Layout.preferredWidth: parent.width * 0.6
                Layout.fillHeight: true
                color: themeColor
                border.color: Qt.darker(themeColor)
                radius: 12

                Image {
                    id: gimbalFrame
                    anchors.fill: parent
                    anchors.margins: 10
                    source: {
                        if (imageSource === "gimbal") {
                            return "image://gimbal/latest?" + Date.now()
                        } else if (imageSource === "ros2") {
                            return "image://ros2/latest?" + Date.now()
                        } else {
                            return ""
                        }
                    }
                    fillMode: Image.Stretch
                    asynchronous: true

                    MouseArea {
                        anchors.fill: gimbalFrame
                        onClicked: function(mouse) {
                            if (mouse.button === Qt.LeftButton) {
                                gimbalController.onImageLeftClicked(
                                    mouse.x, mouse.y, gimbalFrame.width, gimbalFrame.height
                                )
                            }
                        }
                    }

                    Timer {
                        interval: 1000 / 30 // 30 FPS
                        running: true
                        repeat: true
                        onTriggered: {
                            if (imageSource === "gimbal") {
                                gimbalFrame.source = "image://gimbal/latest?" + Date.now()
                            } else if (imageSource === "ros2") {
                                gimbalFrame.source = "image://ros2/latest?" + Date.now()
                            } else {
                                gimbalFrame.source = ""
                            }
                        }
                    }
                }
            }

            // =======================
            // === Control Panel ===
            // =======================
            Rectangle {
                id: controlPanel
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: themeColor
                border.color: Qt.darker(themeColor)
                radius: 12

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 16 
                    spacing: 20

                    // === Target Yaw/Pitch Control ===
                    // RowLayout {
                    //     Layout.alignment: Qt.AlignHCenter
                    //     spacing: 12

                    //     TextField {
                    //         placeholderText: "Yaw"
                    //         text: targetYaw
                    //         onTextChanged: targetYaw = text
                    //         inputMethodHints: Qt.ImhFormattedNumbersOnly
                    //         Layout.preferredWidth: 80
                    //     }
                    //     TextField {
                    //         placeholderText: "Pitch"
                    //         text: targetPitch
                    //         onTextChanged: targetPitch = text
                    //         inputMethodHints: Qt.ImhFormattedNumbersOnly
                    //         Layout.preferredWidth: 80
                    //     }
                    //     Button {
                    //         text: "Rotate To"
                    //         onClicked: {
                    //             // Convert to float or int as needed
                    //             gimbalController.rotateTo(Number(targetYaw), Number(targetPitch))
                    //         }
                    //     }
                    // }
                    
                    // === Zoom Control ===
                    RowLayout {
                        Layout.alignment: Qt.AlignHCenter
                        spacing: 12

                        Button {
                            text: "-"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalController.zoomOut()
                        }
                        Button {
                            text: "x"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalController.stopZoom()
                        }
                        Button {
                            text: "+"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalController.zoomIn()
                        }
                    }

                    // === Arrow Pad Control ===
                    GridLayout {
                        columns: 3
                        rowSpacing: 10
                        columnSpacing: 10
                        Layout.alignment: Qt.AlignHCenter

                        // Row 1
                        Item {}
                        Button {
                            text: "↑"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalController.up(Number(velocity))
                        }
                        Item {}

                        // Row 2
                        Button {
                            text: "←"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalController.left(Number(velocity))
                        }
                        Button {
                            text: "x"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalController.stopRotation()
                        }
                        Button {
                            text: "→"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalController.right(Number(velocity))
                        }

                        // Row 3
                        Item {}
                        Button {
                            text: "↓"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalController.down(Number(velocity))
                        }
                        Button {
                            text: "Reset"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalController.resetPose()
                        } 
                    }

                    TextField {
                        placeholderText: "Vel.(°/s)"
                        text: velocity
                        onTextChanged: velocity = text
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        Layout.fillWidth: ture
                        Layout.fillHeight: true
                    }                    
                    TextField {
                        placeholderText: "Zoom level int"
                        text: zoom_int
                        onTextChanged: zoom_int = text
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        Layout.fillWidth: ture
                        Layout.fillHeight: true
                    }
                    TextField {
                        placeholderText: "Zoom level frac"
                        text: zoom_frac
                        onTextChanged: zoom_frac = text
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        Layout.fillWidth: ture
                        Layout.fillHeight: true
                    }
                    Button {
                        text: "Set Zoom"
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        onClicked: gimbalController.setZoom(Number(zoom_int), Number(zoom_frac))
                    }
                    // === Gimbal Status ===
                    RowLayout {
                        Rectangle {
                            Layout.minimumWidth: 140 
                            Layout.minimumHeight: 80
                            color: accentColor
                            radius: 3

                            Text {
                                anchors.top: parent.top
                                anchors.topMargin: 3 
                                anchors.left: parent.left
                                anchors.leftMargin: 5 
                                text: "Yaw: " + gimbalController.yaw.toFixed(1) +
                                    "°\nPitch: " + gimbalController.pitch.toFixed(1) +
                                    "°\nYaw Velocity: " + gimbalController.yaw_velocity.toFixed(1) +
                                    "°/s\nPitch Velocity: " + gimbalController.pitch_velocity.toFixed(1) +
                                    "°/s\nZoom Level: " + gimbalController.zoom_level.toFixed(1) + "x"
                                font.pixelSize: 13
                                font.bold: true
                                color: textColor
                            }
                        }
                        ColumnLayout {
                            Button {
                                text: streamerConnected ? "Stream ON" : "Stream OFF"
                                Layout.alignment: Qt.AlignRight
                                Layout.preferredWidth: 65
                                Layout.preferredHeight: 25
                                font.bold: true
                                font.pixelSize: 9
                                background: Rectangle {
                                    implicitWidth: 65
                                    implicitHeight: 25
                                    radius: 15 
                                    border.color: streamerConnected ? "#5cafff" : "#ff5c5c"
                                    border.width: 2
                                    gradient: Gradient {
                                        GradientStop { position: 0.0; color: streamerConnected ? "#7bbfff" : "#ff7b7b" }
                                        GradientStop { position: 1.0; color: streamerConnected ? "#3c7bff" : "#ff3c3c" }
                                    }
                                    // Optional: subtle shadow
                                    Rectangle {
                                        anchors.fill: parent
                                        color: "black"
                                        opacity: 0.08
                                        radius: 20
                                        anchors.topMargin: 3
                                        anchors.leftMargin: 3
                                        z: -1
                                    }
                                }
                                onClicked: {
                                    if (gimbalStreamer.isConnected()) {
                                        gimbalStreamer.stop()
                                        streamerConnected = false
                                        imageSource = "none"
                                        console.log("Gimbal Streamer stopped")
                                    } else {
                                        gimbalStreamer.start()
                                        streamerConnected = true
                                        imageSource = "gimbal"
                                        console.log("Gimbal Streamer started")
                                    }
                                }
                            }
                            Button {
                                text: controllerConnected ? "Control ON" : "Control OFF"
                                Layout.alignment: Qt.AlignRight
                                Layout.preferredWidth: 65
                                Layout.preferredHeight: 25
                                font.bold: true
                                font.pixelSize: 9
                                background: Rectangle {
                                    implicitWidth: 65
                                    implicitHeight: 25
                                    radius: 15 
                                    border.color: controllerConnected ? "#5cafff" : "#ff5c5c"
                                    border.width: 2
                                    gradient: Gradient {
                                        GradientStop { position: 0.0; color: controllerConnected ? "#7bbfff" : "#ff7b7b" }
                                        GradientStop { position: 1.0; color: controllerConnected ? "#3c7bff" : "#ff3c3c" }
                                    }
                                    // Optional: subtle shadow
                                    Rectangle {
                                        anchors.fill: parent
                                        color: "black"
                                        opacity: 0.08
                                        radius: 20
                                        anchors.topMargin: 3
                                        anchors.leftMargin: 3
                                        z: -1
                                    }
                                }
                                onClicked: {
                                    if (gimbalController.isConnected()) {
                                        gimbalController.stop()
                                        controllerConnected = false
                                    } else {
                                        gimbalController.start()
                                        controllerConnected = true
                                    }
                                }
                            }
                            Button {
                                text: ros2TopicSubscriber.running ? "ROS2 ON" : "ROS2 OFF"
                                Layout.alignment: Qt.AlignRight
                                Layout.preferredWidth: 65
                                Layout.preferredHeight: 25
                                font.bold: true
                                font.pixelSize: 9
                                background: Rectangle {
                                    implicitWidth: 65
                                    implicitHeight: 25
                                    radius: 15 
                                    border.color: ros2TopicSubscriber.running ? "#5cafff" : "#ff5c5c"
                                    border.width: 2
                                    gradient: Gradient {
                                        GradientStop { position: 0.0; color: ros2TopicSubscriber.running ? "#7bbfff" : "#ff7b7b" }
                                        GradientStop { position: 1.0; color: ros2TopicSubscriber.running ? "#3c7bff" : "#ff3c3c" }
                                    }
                                    // Optional: subtle shadow
                                    Rectangle {
                                        anchors.fill: parent
                                        color: "black"
                                        opacity: 0.08
                                        radius: 20
                                        anchors.topMargin: 3
                                        anchors.leftMargin: 3
                                        z: -1
                                    }
                                }
                                onClicked: {
                                    if (ros2TopicSubscriber.running) {
                                        ros2TopicSubscriber.stop()
                                        imageSource = "none"
                                        console.log("ROS2 topic subscriber stopped")
                                    } else {
                                        ros2TopicSubscriber.start()
                                        imageSource = "ros2"
                                        console.log("ROS2 topic subscriber started")
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
