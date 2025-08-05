import QtQuick

Rectangle {
    id: root
    width: nodeText.implicitWidth + 22
    height: nodeText.implicitHeight + 14

    property color color1
    property color color2
    property bool isHovered: false
    property bool isPressed: false
    property int styleIndex: 0

    function getBorderColor(index) {
        switch (index) {
            case 0: return "black";  // IDLE
            case 1: return "green";  // RUNNING
            case 2: return "cyan";   // SUCCESS
            case 3: return "red";   // FAILURE
            default: return "red"; // IDLE
        }
    }

    gradient: Gradient {
        GradientStop { position: 0.0; color: color1 }
        GradientStop { position: 1.0; color: color2 }
    }

    radius: 10

    border.color: getBorderColor(styleIndex)
    border.width: isHovered ? 3 : 2
    z: isPressed ? 3: 2

    property int nodeId
    property string label
    property Canvas arrowsCanvas

    Text {
        id: nodeText
        anchors.centerIn: parent
        color: "white"
        text: label
        wrapMode: Text.NoWrap
        font.pointSize: 12
        font.family: "FiraCode"
    }

    MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        drag.target: root

        onEntered: root.isHovered = true
        onExited: root.isHovered = false

        onPressed: root.isPressed = true
        onReleased: root.isPressed = false

        onPositionChanged: {
            arrowsCanvas.requestPaint()
        }
    }

    Rectangle {
        id: glowOverlay
        anchors.fill: parent
        color: "cyan"
        opacity: 0
        radius: root.radius
        z: 3

        Behavior on opacity {
            NumberAnimation {
                duration: 300
            }
        }
    }
}
