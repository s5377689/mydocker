import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window

Item {
    id: gimbalControlPanel
    property var gimbalStreamerRef: null
    property var gimbalControllerRef: null
    property var ros2TopicSubscriberRef: null
    property bool controllerConnected: gimbalControllerRef.connected
    property bool streamerConnected: gimbalStreamerRef.connected
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
                                gimbalControllerRef.onImageLeftClicked(
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
                    //             gimbalControllerRef.rotateTo(Number(targetYaw), Number(targetPitch))
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
                            onClicked: gimbalControllerRef.zoomOut()
                        }
                        Button {
                            text: "x"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalControllerRef.stopZoom()
                        }
                        Button {
                            text: "+"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalControllerRef.zoomIn()
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
                            onClicked: gimbalControllerRef.up(Number(velocity))
                        }
                        Item {}

                        // Row 2
                        Button {
                            text: "←"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalControllerRef.left(Number(velocity))
                        }
                        Button {
                            text: "x"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalControllerRef.stopRotation()
                        }
                        Button {
                            text: "→"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalControllerRef.right(Number(velocity))
                        }

                        // Row 3
                        Item {}
                        Button {
                            text: "↓"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalControllerRef.down(Number(velocity))
                        }
                        Button {
                            text: "Reset"
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onClicked: gimbalControllerRef.resetPose()
                        } 
                    }

                    TextField {
                        placeholderText: "Vel.(°/s)"
                        text: velocity
                        onTextChanged: velocity = text
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                    }                    
                    TextField {
                        placeholderText: "Zoom level int"
                        text: zoom_int
                        onTextChanged: zoom_int = text
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                    }
                    TextField {
                        placeholderText: "Zoom level frac"
                        text: zoom_frac
                        onTextChanged: zoom_frac = text
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                    }
                    Button {
                        text: "Set Zoom"
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        onClicked: gimbalControllerRef.setZoom(Number(zoom_int), Number(zoom_frac))
                    }
                    // === Gimbal Status ===
                    RowLayout {
                        Rectangle {
                            Layout.preferredWidth: statusText.implicitWidth + 20
                            Layout.preferredHeight: statusText.implicitHeight + 10
                            color: accentColor
                            radius: 3

                            Text {
                                id: statusText
                                anchors.top: parent.top
                                anchors.topMargin: 3 
                                anchors.left: parent.left
                                anchors.leftMargin: 5 
                                text: "Yaw: " + gimbalControllerRef.yaw.toFixed(1) +
                                    "°\nPitch: " + gimbalControllerRef.pitch.toFixed(1) +
                                    "°\nYaw Velocity: " + gimbalControllerRef.yaw_velocity.toFixed(1) +
                                    "°/s\nPitch Velocity: " + gimbalControllerRef.pitch_velocity.toFixed(1) +
                                    "°/s\nZoom Level: " + gimbalControllerRef.zoom_level.toFixed(1) + "x"
                                font.pixelSize: 16
                                font.bold: true
                                color: textColor
                                wrapMode: Text.Wrap
                            }
                        }
                        ColumnLayout {
                            Layout.alignment: Qt.AlignTop
                            Layout.leftMargin: 8

                            Button {
                                text: streamerConnected ? "Stream ON" : "Stream OFF"
                                Layout.alignment: Qt.AlignRight
                                Layout.preferredWidth: 85
                                Layout.preferredHeight: 30
                                font.bold: true
                                font.pixelSize: 12
                                background: Rectangle {
                                    implicitWidth: 85
                                    implicitHeight: 30
                                    radius: 10
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
                                    if (gimbalStreamerRef.connected) {
                                        gimbalStreamerRef.stop()
                                        imageSource = "none"
                                        console.log("Gimbal Streamer stopped")
                                    } else {
                                        gimbalStreamerRef.start()
                                        imageSource = "gimbal"
                                        console.log("Gimbal Streamer started")
                                    }
                                }
                            }
                            Button {
                                text: controllerConnected ? "Control ON" : "Control OFF"
                                Layout.alignment: Qt.AlignRight
                                Layout.preferredWidth: 85
                                Layout.preferredHeight: 30
                                font.bold: true
                                font.pixelSize: 12
                                background: Rectangle {
                                    implicitWidth: 85
                                    implicitHeight: 30
                                    radius: 10
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
                                    if (gimbalControllerRef.connected) {
                                        gimbalControllerRef.stop()
                                    } else {
                                        gimbalControllerRef.start()
                                    }
                                }
                            }
                            Button {
                                text: ros2TopicSubscriber.running ? "ROS2 ON" : "ROS2 OFF"
                                Layout.alignment: Qt.AlignRight
                                Layout.preferredWidth: 85
                                Layout.preferredHeight: 30
                                font.bold: true
                                font.pixelSize: 12
                                background: Rectangle {
                                    implicitWidth: 85
                                    implicitHeight: 30
                                    radius: 10
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
