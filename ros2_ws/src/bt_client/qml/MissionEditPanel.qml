import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs

import "."


Item {
    id: root
    anchors.fill: parent
    property var btParserRef: null
    property var treeNodesModelParserRef: null

    // File dialog for loading BT XML
    FileDialog {
        id: fileDialog
        title: "Select a BT xml file"
        nameFilters: ["XML files (*.xml)"]
        currentFolder: "file:///home/robuff/data/bt"
        onAccepted: {
            // Parse the selected file
            console.log("selectedFile: " + new URL(selectedFile).pathname)
            let btRoot = btParserRef.parseFromFile(new URL(selectedFile).pathname)
            if (btRoot)
                editMissionVisualizer.updateBtTree(btRoot)
        }
    }

    // Context Menu
    Menu {
        id: contextMenu

        MenuItem {
            text: "Load Tree from File"
            onTriggered: {
                fileDialog.open()
            } 
        }

        MenuSeparator {}

        MenuItem {
            text: "Save Tree to File"
            enabled: editMissionVisualizer.btRoot !== null
            onTriggered: {
                console.log("TO-DO: saving to file")
            }
        }

        MenuSeparator {}

        MenuItem {
            text: "Clear All"
            enabled: editMissionVisualizer.btRoot !== null
            onTriggered: {
                editMissionVisualizer.clearBtTree()
            }
        }
    }

    // Global drag proxy for cross-component dragging
    Item {
        id: globalDragProxy
        width: dragVisual.width
        height: dragVisual.height
        z: 10000   // on top of everything
        visible: false

        property var dragData: null
        property point hotSpot: Qt.point(0, 0)

        function startDrag(nodeData, startPoint, sourceSize) {
            dragData = nodeData
            hotSpot = Qt.point(sourceSize.width / 2, sourceSize.height / 2)

            // Position the proxy at the start point minus hotspot
            x = startPoint.x - hotSpot.x
            y = startPoint.y - hotSpot.y

            visible = true
        }

        function updatePosition(windowPoint) {
            x = windowPoint.x - hotSpot.x
            y = windowPoint.y - hotSpot.y
        }

        function endDrag() {
            visible = false
            dragData = null
        }

        // Visual representation of the dragged item
        Rectangle {
            id: dragVisual
            width: Math.max(contentRow.implicitWidth + 16, 120)
            height: Math.max(contentRow.implicitHeight + 16, 40)

            radius: 12
            color: "#1a1d2c"
            border.color: "#ffffff"
            border.width: 2
            opacity: 0.9

            // Enhanced shadow for depth
            Rectangle {
                anchors.fill: parent
                anchors.topMargin: 3
                anchors.leftMargin: 3
                radius: parent.radius
                color: "#40000000"
                z: -1
            }

            // Content
            RowLayout {
                id: contentRow
                anchors.centerIn: parent
                anchors.margins: 8
                spacing: 8

                // Node name
                Text {
                    text: globalDragProxy.dragData ? globalDragProxy.dragData.name : ""
                    color: "white"
                    font.pixelSize: 13
                    font.bold: true
                    elide: Text.ElideRight
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignVCenter
                }
            }

            // Pulsing animation to show it's being dragged
            SequentialAnimation on scale {
                running: globalDragProxy.visible
                loops: Animation.Infinite
                NumberAnimation { from: 1.0; to: 1.05; duration: 600 }
                NumberAnimation { from: 1.05; to: 1.0; duration: 600 }
            }
        }
    } // End of globalDragProxy

    RowLayout {
        anchors.fill: parent

        // Left Panel
        NodePalette {
            id: nodePalette
            Layout.preferredWidth: 200
            Layout.fillHeight: true
            globalDragProxy: globalDragProxy
            componentToDrop: editMissionVisualizer
            editMissionPage: root
        }

        // Right Panel: BtVisualizer for editing
        BtVisualizer {
            id: editMissionVisualizer
            Layout.fillWidth: true
            Layout.fillHeight: true

            btParserRef: btParserRef
            currentLanguage: currentLanguage
            showRealTimeUpdates: true
            showStatusColors: true
            globalDragProxy: globalDragProxy
        }
    }

    // Background mouse area for context menu
    MouseArea {
        id: backgroundMouseArea
        anchors.fill: parent
        z: 0  // Behind other components but above file drop area
        acceptedButtons: Qt.RightButton

        onClicked: (mouse) => {
            contextMenu.x = mouse.x
            contextMenu.y = mouse.y
            contextMenu.open()
        }
    }

    // File drop area - only accepts files, positioned behind the visualizer
    DropArea {
        id: fileDropArea
        anchors.fill: parent
        z: -1

        // Only accept file drops
        keys: ["text/uri-list"]
                    
        onEntered: (drag) => {
            if (drag.hasUrls)
                background.color = Qt.lighter(backgroundColor, 3)
        }
        onDropped: {
            background.color = backgroundColor

            if (drop.hasUrls) {
                console.log("Parsing file: " + new URL(drop.urls).pathname)
                let btRoot = btParserRef.parseFromFile(new URL(drop.urls).pathname)
                if (btRoot)
                    editMissionVisualizer.updateBtTree(btRoot)
            }
        }
        onExited: {
            background.color = backgroundColor
        }
    }

    Component.onCompleted: {
        let nodesModelPath = "/home/robuff/data/bt/tree_nodes_model.xml"
        let availableNodes = treeNodesModelParserRef.parseFromFile(nodesModelPath)

        // Add predefined control nodes and root node to the palette
        let rootNode = btParserRef.createBtNode("Root")
        rootNode.isRoot = true  // Mark as root node
        let sequenceNode = btParserRef.createBtNode("Sequence")
        let fallbackNode = btParserRef.createBtNode("Fallback")

        availableNodes.unshift(fallbackNode)
        availableNodes.unshift(sequenceNode)
        availableNodes.unshift(rootNode)

        nodePalette.availableNodes = availableNodes

        console.log("Available nodes loaded: " + availableNodes.length)
    }
}   