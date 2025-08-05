import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import "BtUtils.js" as BtUtils

Rectangle {
    id: root
    color: "#2C2C2C"
    border.color: "#555555"
    border.width: 1

    property var availableNodes: []
    readonly property color nodeColor: "#1a1d2c"
    property var globalDragProxy: null  // Reference to the global drag proxy
    property var componentToDrop: null
    property var editMissionPage: null
    property bool dragActive: false
    property var activeDragItem: null

    function startDrag(dragItem) {
        dragActive = true
        activeDragItem = dragItem
    }

    function endDrag() {
        dragActive = false
        activeDragItem = null
    }

    // Main layout
    Column {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        // Header
        Text {
            width: parent.width
            text: "Available Nodes"
            color: "white"
            font.pixelSize: 16
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
        }
        
        // Seperator line
        Rectangle {
            width: parent.width
            height: 1
            color: "#555555"
        }
        
        // Scrollable node list
        ScrollView {
            width: parent.width
            height: parent.height - y
            clip: true

            ListView {
                id: nodeListView
                width: parent.width
                model: availableNodes
                spacing: 12
                interactive: !root.dragActive

                delegate: Rectangle {
                    id: nodeItem
                    width: nodeListView.width - 10
                    height: 55
                    radius: 17

                    property var btNode: modelData
                    property string nodeType: modelData.nodeType || "Unknown"
                    property color nodeColor: "#1a1d2cff" 
                    property bool isDragging: false

                    // Drag state visual feedback
                    opacity: isDragging ? 0.8 : 1.0
                    scale: isDragging ? 0.95 : 1.0

                    Behavior on opacity {
                        NumberAnimation { duration: 150 }
                    }
                    Behavior on scale {
                        NumberAnimation { duration: 150 }
                    }

                    // Replace solid color with gradient
                    gradient: Gradient {
                        GradientStop { 
                            position: 0.0; 
                            color: nodeMouseArea.containsMouse ? Qt.lighter(nodeColor, 1.4) : nodeColor 
                        }
                        GradientStop { 
                            position: 1.0; 
                            color: nodeMouseArea.containsMouse ? Qt.darker(nodeColor, 1.2) : Qt.darker(nodeColor, 1.3) 
                        }
                    }

                    border.color: Qt.lighter(nodeColor, 1.5)
                    border.width: 2

                    Behavior on color {
                        ColorAnimation {
                            duration: 200
                            easing.type: Easing.OutQuad
                        }
                    }

                    // Icon + Text Layout
                    Item { 
                        anchors.fill: parent
                        anchors.margins: 10

                        // Node type icon
                        Image {
                            id: nodeTypeIcon
                            width: 25
                            height: 25
                            source: BtUtils.getNodeTypeIcon(nodeType)
                            anchors.left: parent.left
                            anchors.verticalCenter: parent.verticalCenter
                            anchors.leftMargin: 10
                        }

                        Text {
                            text: btNode ? btNode.name : "Unknown"
                            color: "white"
                            font.pixelSize: 14
                            font.bold: true
                            anchors.left: nodeTypeIcon.right
                            anchors.leftMargin: 10
                            anchors.verticalCenter: parent.verticalCenter
                            elide: Text.ElideRight
                            width: parent.width - nodeTypeIcon.width - 30
                        }
                    }  // End of ListView

                    // Support for drag and drop to create a node
                    MouseArea {
                        id: nodeMouseArea
                        anchors.fill: parent
                        hoverEnabled: true
                        cursorShape: Qt.PointingHandCursor
                        enabled: !root.dragActive || root.activeDragItem === nodeItem

                        property bool isDragging: false
                        property point dragStartPoint: Qt.point(0, 0)

                        onPressed: (mouse) => {
                            dragStartPoint = Qt.point(mouse.x, mouse.y)
                            isDragging = false
                       }

                        onPositionChanged: (mouse) => {
                            if (pressed && !isDragging) {

                                // Check if the mouse has moved significantly
                                let deltaX = Math.abs(mouse.x - dragStartPoint.x)
                                let deltaY = Math.abs(mouse.y - dragStartPoint.y)

                                if (deltaX > 5 || deltaY > 5) {
                                    isDragging = true
                                    root.startDrag(nodeItem)

                                    // Start global drag
                                    if (root.globalDragProxy) {
                                        let nodeData = {
                                            name: btNode ? btNode.name : "Unknown",
                                            nodeType: nodeType,
                                            attributes: btNode ? btNode.attributes : {}
                                        }

                                        // Convert local mouse position to position inside the tab page
                                        let pagePoint = nodeItem.mapToItem(editMissionPage, mouse.x, mouse.y)
                                        let sourceSize = Qt.size(nodeItem.width, nodeItem.height)

                                        root.globalDragProxy.startDrag(nodeData, pagePoint, sourceSize);
                                    }
                                }
                            }

                            if (isDragging && root.globalDragProxy) {
                                // Update drag position
                                let pagePoint = nodeItem.mapToItem(editMissionPage, mouse.x, mouse.y);
                                root.globalDragProxy.updatePosition(pagePoint);
                            }
                        }

                        onReleased: (mouse) => {
                            if (isDragging) {
                                if (root.globalDragProxy && root.componentToDrop) {
                                    let targetPoint = nodeItem.mapToItem(componentToDrop, mouse.x, mouse.y);

                                    if (root.componentToDrop.handleGlobalDrop) {
                                        let success = root.componentToDrop.handleGlobalDrop(
                                            root.globalDragProxy.dragData,
                                            targetPoint.x,
                                            targetPoint.y
                                        )
                                    } else {
                                        console.warn("No handleGlobalDrop method on componentToDrop")
                                    }

                                    root.globalDragProxy.endDrag();
                                }

                                isDragging = false
                                root.endDrag();
                            }
                        }

                        onCanceled: {
                            if (isDragging) {
                                if (root.globalDragProxy)
                                    root.globalDragProxy.endDrag()
                                root.endDrag();
                                isDragging = false
                            }
                        }
                    }

                    // Subtle shadow effect
                    Rectangle {
                        anchors.fill: parent
                        anchors.margins: 2
                        radius: parent.radius - 1
                        color: "transparent"
                        border.color: Qt.rgba(0, 0, 0, 0.3)
                        border.width: 2
                        z: -1
                    }
                } // End of delegate
            } // End of ListView
        } // End of ScrollView
    }  // End of Column
}