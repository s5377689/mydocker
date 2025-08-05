import QtQuick
import QtQuick.Controls
import QtQuick.Effects

import "BtUtils.js" as BtUtils

Rectangle {
    id: root

    // node's position determines its number of connection ports
    enum NodePosition {
        Root,
        Internal,
        Leaf
    }
    property int nodePosition: {
        if (nodeType === "Root")
            return MovableNode.NodePosition.Root;
        else if (nodeType === "Control" || nodeType === "Condition")
            return MovableNode.NodePosition.Internal;
        else
            return MovableNode.NodePosition.Leaf;
    }

    property var btVisualizer: null  // Reference to the BtVisualizer to inform drag action

    property string label
    property Canvas arrowsCanvas

    property color colorFrom
    property color colorTo
    property bool isHovered: false
    property bool isPressed: false
    property int styleIndex: 0

    // BT Node properties
    property int nodeId
    property var btNode: null  // Store the BtNode object
    property string nodeType: "Unknown"  // Node type assigned by bt parser

    // Layout properties
    property bool showAttributes: true
    property int nodeTypeIconWidth
    property int nodeWidthMargin
    property int nodeHeightMargin

    // Font properties
    property int fontSize: 12
    property string fontFamily: "FiraCode"
    property bool fontBold: true

    width: Math.max(nodeTypeIconWidth + nodeText.implicitWidth + nodeWidthMargin, showAttributes ? attributesColumn.implicitWidth + nodeWidthMargin : 0)
    height: Math.max(nodeText.implicitHeight, nodeTypeIconWidth) + (showAttributes ? attributesColumn.implicitHeight : 0) + nodeHeightMargin

    // Repaint when resized
    onWidthChanged: {
        if (arrowsCanvas) {
            arrowsCanvas.requestPaint();
        }
    }
    onHeightChanged: {
        if (arrowsCanvas) {
            arrowsCanvas.requestPaint();
        }
    }

    // Smooth transitions for size changes
    Behavior on width {
        NumberAnimation {
            duration: 200
            easing.type: Easing.OutQuad
        }
    }
    Behavior on height {
        NumberAnimation {
            duration: 200
            easing.type: Easing.OutQuad
        }
    }

    function getBorderColor(index) {
        switch (index) {
        case 0:
            return "black";  // IDLE
        case 1:
            return "#00ff00";  // RUNNING
        case 2:
            return "cyan";   // SUCCESS
        case 3:
            return "red";   // FAILURE
        default:
            return "red"; // IDLE
        }
    }

    layer.enabled: true
    layer.effect: MultiEffect {
        shadowEnabled: true
        shadowHorizontalOffset: 0
        shadowVerticalOffset: 8
        shadowBlur: 0.3
        shadowScale: 1.0
        shadowColor: "#80000000"
    }

    // Glow effect when hovered
    Rectangle {
        id: glowLayer
        anchors.fill: parent
        anchors.margins: -4
        radius: parent.radius + 4
        color: isHovered ? "#efefef" : "transparent"
        z: -4

        Behavior on color {
            ColorAnimation {
                duration: 200
                easing.type: Easing.OutQuad
            }
        }
    }

    // Animated gradient
    gradient: Gradient {
        GradientStop {
            position: 0.0
            color: isHovered ? Qt.lighter(colorFrom, 1.15) : colorFrom
        }
        GradientStop {
            position: 1.0
            color: isHovered ? Qt.lighter(colorTo, 1.15) : colorTo
        }
    }

    radius: 10

    // Enhanced border with smooth transitions
    border.color: getBorderColor(styleIndex)
    border.width: isHovered ? 3 : 2

    Behavior on border.width {
        NumberAnimation {
            duration: 150
            easing.type: Easing.OutQuad
        }
    }

    Behavior on border.color {
        ColorAnimation {
            duration: 150
            easing.type: Easing.OutQuad
        }
    }

    // Scale effect on hover
    scale: isHovered ? 1.02 : 1.0
    Behavior on scale {
        NumberAnimation {
            duration: 200
            easing.type: Easing.OutBack
        }
    }

    // Title area with drag functionality
    MouseArea {
        id: titleMouseArea
        anchors.fill: parent
        hoverEnabled: true
        drag.target: root
        acceptedButtons: Qt.LeftButton

        onEntered: {
            root.isHovered = true;
        }
        onExited: {
            root.isHovered = false;
        }
        onPressed: mouse => {
            root.isPressed = true;
        }
        onReleased: {
            root.isPressed = false;
        }
        onPositionChanged: {
            if (arrowsCanvas) {
                arrowsCanvas.requestPaint();
            }
        }
    }

    // Node text and attributes
    Column {
        id: mainColumn
        anchors.centerIn: parent
        spacing: 5

        // Title area with drag functionality
        Rectangle {
            id: titleArea
            anchors.horizontalCenter: parent.horizontalCenter
            width: nodeText.implicitWidth + 10
            height: nodeText.implicitHeight + 12
            color: "transparent"
            radius: 5

            Row {
                anchors.centerIn: parent  // Centers the entire Row in the parent
                spacing: 8  // Space between icon and text

                Image {
                    id: nodeTypeIcon
                    width: nodeTypeIconWidth
                    height: nodeTypeIconWidth
                    source: BtUtils.getNodeTypeIcon(nodeType)
                    anchors.verticalCenter: parent.verticalCenter
                }

                Text {
                    id: nodeText
                    color: "white"
                    text: label
                    wrapMode: Text.NoWrap
                    font.pointSize: fontSize
                    font.family: fontFamily
                    font.bold: fontBold
                    anchors.verticalCenter: parent.verticalCenter
                }
            }

            // Underline appears on hover
            Rectangle {
                id: underline
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.bottom: parent.bottom
                // anchors.bottomMargin: 1
                width: titleMouseArea.containsMouse ? nodeTypeIcon.width + nodeText.implicitWidth + 12 : 0
                height: 5
                radius: 1

                gradient: Gradient {
                    GradientStop {
                        position: 0.0
                        color: "transparent"
                    }
                    GradientStop {
                        position: 0.5
                        color: titleMouseArea.containsMouse ? "#9c9c9c" : "transparent"
                    }
                    GradientStop {
                        position: 1.0
                        color: "transparent"
                    }
                }

                Behavior on width {
                    NumberAnimation {
                        duration: 200
                        easing.type: Easing.OutQuad
                    }
                }

                Behavior on color {
                    ColorAnimation {
                        duration: 200
                        easing.type: Easing.OutQuad
                    }
                }
            }
        }

        Column {
            id: attributesColumn
            anchors.horizontalCenter: parent.horizontalCenter
            spacing: 3
            visible: showAttributes && btNode && Object.keys(btNode.attributes).length > 0

            // Smooth opacity transition
            opacity: showAttributes ? 1.0 : 0.0
            Behavior on opacity {
                NumberAnimation {
                    duration: 150
                    easing.type: Easing.OutQuad
                }
            }

            Repeater {
                model: btNode ? Object.keys(btNode.attributes) : []
                delegate: Row {
                    spacing: 5
                    anchors.horizontalCenter: parent.horizontalCenter

                    Text {
                        color: "#CCCCCC"
                        text: modelData + ":"
                        font.pointSize: 9
                        font.family: "FiraCode"
                        anchors.verticalCenter: parent.verticalCenter
                    }

                    TextField {
                        id: attributeField
                        text: btNode ? (btNode.attributes[modelData] || "") : ""
                        font.pointSize: 9
                        font.family: "FiraCode"
                        color: "white"

                        background: Rectangle {
                            color: "#333333"
                            border.color: attributeField.activeFocus ? "#00AAFF" : "#555555"
                            border.width: 1
                            radius: 3

                            // Subtle glow when focused
                            Rectangle {
                                anchors.fill: parent
                                anchors.margins: -2
                                color: "transparent"
                                border.color: attributeField.activeFocus ? "#00AAFF" : "transparent"
                                border.width: 1
                                radius: 5
                                opacity: 0.3
                                z: -1
                            }
                        }

                        width: Math.max(120, implicitWidth + 15)
                        height: 25

                        onTextChanged: {
                            if (btNode) {
                                let newAttrs = Object.assign({}, btNode.attributes);
                                newAttrs[modelData] = text;
                                btNode.attributes = newAttrs;
                            }
                        }
                    }
                }
            }
        }
    }

    // Mouse area for right-click context menu
    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.RightButton

        onPressed: mouse => {
            if (mouse.button === Qt.RightButton) {
                contextMenu.popup();
            }
        }
    }

    // Context menu for showing/hiding attributes
    Menu {
        id: contextMenu
        MenuItem {
            text: showAttributes ? "Hide Attributes" : "Show Attributes"
            onTriggered: {
                showAttributes = !showAttributes;
                if (arrowsCanvas) {
                    arrowsCanvas.requestPaint();
                }
            }
        }
    }

    component ConnectionPort: Item {
        property bool isTopPort: true  // Top or bottom port
        property var missionEditPanel: null
        width: 18
        height: 18
        z: 10

        // Top circle (for Internal, Leaf nodes)
        Rectangle {
            id: portCircle
            visible: parent.visible
            width: 18
            height: 18
            radius: 9
            color: "#5588FF"
            border.color: "#000000"
            border.width: 2
            anchors.centerIn: parent

            Behavior on color {
                ColorAnimation {
                    duration: 100
                }
            }
        }

        Rectangle {
            visible: parent.visible
            width: 20
            height: 20
            radius: 10
            color: "#000000"
            opacity: 0.2
            anchors.centerIn: parent
            z: -1  // Behind the top circle
        }

        // Invisible item for drag and drop
        Item {
            anchors.fill: parent
            Drag.active: dragArea.drag.active
            Drag.dragType: Drag.Automatic
            Drag.supportedActions: Qt.CopyAction
            Drag.mimeData: {
                "isTopPort": isTopPort.toString(),
                "nodeId": btNode.id.toString()
            }

            MouseArea {
                id: dragArea
                anchors.fill: parent
                acceptedButtons: Qt.LeftButton
                drag.target: parent
            }
        }

        DropArea {
            anchors.fill: parent
            onDropped: (drop) => {
                // Handle connection logic here
                let sourceNodeId = parseInt(drop.getDataAsString("nodeId"));
                let sourceIsTopPort = drop.getDataAsString("isTopPort") === "true";

                if (btNode.id === sourceNodeId) {
                    console.warn("Cannot connect a port to itself.");
                    return;
                }
                if (isTopPort === sourceIsTopPort)
                {
                    console.warn("Cannot connect ports of the same type.");
                    return;
                }
                else {
                    // Inform the BtVisualizer about the connection
                    console.log("Connecting ports:", sourceNodeId, btNode.id, isTopPort);
                    if (root.btVisualizer) {
                        // if isTopPort, this node is added as a child to the source node
                        // otherwise, the source node is added as a child to this node
                        root.btVisualizer.connectNodes(sourceNodeId, btNode.id, isTopPort);
                    }
                }
            }
        }
    }

    // Top circle (for Internal, Leaf nodes)
    ConnectionPort {
        isTopPort: true
        missionEditPanel: root.missionEditPanel
        visible: root.nodePosition === MovableNode.NodePosition.Internal || root.nodePosition === MovableNode.NodePosition.Leaf
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: -9  // Half outside the node
    }

    // Bottom circle (for Internal, Root nodes)
    ConnectionPort {
        isTopPort: false
        missionEditPanel: root.missionEditPanel
        visible: root.nodePosition === MovableNode.NodePosition.Internal || root.nodePosition === MovableNode.NodePosition.Root
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: -9  // Half outside the node
    }
}
