import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import "BtUtils.js" as BtUtils
import "LayoutUtils.js" as LayoutUtils
import "TranslationUtils.js" as TranslationUtils

Item {
    id: root

    // Public properties
    property var btRoot: null
    property var btParserRef: null   // To get node types from their names
    property string currentLanguage: "zh"
    property bool showRealTimeUpdates: true
    property bool showStatusColors: true
    property var globalDragProxy: null  // Reference to the global drag proxy
    
    // Internal properties
    property var objList: []
    property var id2NodeMap: ({})
    property real canvasScale: 1.0

    // Font
    property int fontSize: 12
    property string fontFamily: "FiraCode"
    property bool fontBold: true

    // Drawing properties
    property color colorFrom: "#2e346e"
    property color colorTo: "#171a38"

    // Manage connection editing
    property bool dragConnectionActive: false
    property string dragPortType: ""  // "top" or "bottom"
    property string dropPortType: ""  // "top" or "bottom"
    property var dragSourceNode: null
    property var dropTargetNode: null

    property bool editable: false  // Whether the tree is editable (true for MissionEditPanel)


    function clearBtTree() {
        btRoot = null
        clearAllObjects()
        id2NodeMap = {}
        arrowsCanvas.requestPaint()
    }

    function renderNewBtTree(newBtRoot) {
        clearAllObjects()
        btRoot = newBtRoot
        btRoot.isRoot = true  // Ensure we know which one is the root node for editing the connection
        if (btRoot) {
            buildTree(btRoot, canvas.width/2, 50, 0)
            arrowsCanvas.requestPaint()
            centerView()
        }
    }

    function updateBtTree(newBtRoot) {
        if (newBtRoot && BtUtils.sameTree(btRoot, newBtRoot)) {
            // Same tree structure, just update status
            updateNodeStatus(newBtRoot)
            updateNodeColor() 
            arrowsCanvas.requestPaint()
        }
        else {
            // Different structure, rebuild the tree
            renderNewBtTree(newBtRoot)
        }
    }

    // Recursively update the status data in object list based on the new btRoot
    function updateNodeStatus(btRoot, fromIdx = 0) {
        // Assume the same tree structure but node statuses may have changed
        if (!btRoot)
            return 0

        // Update this node's status
        objList[fromIdx].btNode.status = btRoot.status
        let startIdx = fromIdx
        fromIdx += 1
        
        // Recursively update children
        for (let i = 0; i < btRoot.children.length; ++i)
        {   
            let child = btRoot.children[i]
            let num_updated = updateNodeStatus(child, fromIdx)
            fromIdx += num_updated
        }

        return fromIdx - startIdx
    }

    // Node Color Management
    function updateNodeColor() {
        for (let obj of objList) {
            let node = obj.btNode

            if (node.status === "IDLE") {
                obj.styleIndex = 0
            }
            else if (node.status === "RUNNING") {
                obj.styleIndex = 1
            }
            else if (node.status === "SUCCESS") {
                obj.styleIndex = 2
            }
            else if (node.status === "FAILURE") {
                obj.styleIndex = 3
            }
        }
    }
    
    function clearAllObjects() {
        for (let obj of objList) {
            obj.destroy()
        }
        objList = []
    }
    
    function centerView() {
        Qt.callLater(() => {
            const centerX = (canvasScale * canvas.width) / 2
            const visibleWidth = flickable.width
            flickable.contentX = Math.max(0, centerX - visibleWidth / 2)
            flickable.contentY = 0
        })
    }
    
    function exportBT() {
        return BtUtils.exportBT(btRoot)
    }
    
    function getLabel(name) {
        return TranslationUtils.getLabel(name, currentLanguage)
    }

    function connectNodes(sourceNodeId, targetNodeId, addAsChild) {
        // if addAsChild is true, add targetNode as a child of sourceNode
        // otherwise, add sourceNode as a child of targetNode
        let sourceNode = root.id2NodeMap[sourceNodeId]
        let targetNode = root.id2NodeMap[targetNodeId]

        if (!sourceNode || !targetNode) {
            console.error("Invalid node IDs for connection:", sourceNodeId, targetNodeId)
            return
        }

        if (addAsChild) {
            sourceNode.addChild(targetNode)
        } else {
            targetNode.addChild(sourceNode)
        }

        arrowsCanvas.requestPaint()
    }

    function handleGlobalDrop(nodeData, localX, localY) {
        console.log("Creating node from drop:", nodeData.name, "at local position:", localX, localY)

        // Convert local coordinates to canvas coordinates
        let canvasX = (localX + flickable.contentX) / canvasScale
        let canvasY = (localY + flickable.contentY) / canvasScale

        console.log("Canvas coordinates:", canvasX, canvasY)

        let comp = Qt.createComponent("MovableNode.qml")
        if (comp.status !== Component.Ready) {
            console.error("Failed to load MovableNode.qml:", comp.errorString())
            return null
        }

        let label = getLabel(nodeData.name)
        let btNodeObj = Qt.createQmlObject('import Bt 1.0; BtNode {}', canvas)
        btNodeObj.status = "IDLE"  // Default status
        btNodeObj.attributes = nodeData.attributes || {}
        btNodeObj.id = objList.length  // Assign a unique ID based on current object count

        // Create MovableNode object
        let obj = comp.createObject(canvas, {
            x: canvasX - 75,  // Center the node on drop position
            y: canvasY - 30,
            colorFrom: colorFrom,
            colorTo: colorTo,
            label: label,
            btVisualizer: root,
            arrowsCanvas: arrowsCanvas,
            btNode: btNodeObj,
            nodeType: nodeData.nodeType,
            styleIndex: 0,  // IDLE by default

            // Layout properties
            nodeTypeIconWidth: 25,
            nodeWidthMargin: 36,
            nodeHeightMargin: 36,
            showAttributes: true,

            // Font properties
            fontSize: fontSize,
            fontFamily: fontFamily,
            fontBold: fontBold
        })

        if (obj) {
            objList.push(obj)
            root.id2NodeMap[obj.btNode.id] = obj.btNode
            return obj
        } else {
            console.error("Failed to create MovableNode object")
            return null
        }
    }

    function buildTree(btNode, x, y, depth) {
        createAllNodes(btNode, x, y, depth)
        arrangeNodePositions(btNode, x, y, depth)
    }

    function rearrangeCurrentTree() {
        if (!btRoot || objList.length === 0) {
            return
        }
        arrangeNodePositions(btRoot, canvas.width / 2, 50, 0)
        arrowsCanvas.requestPaint()
        centerView()
    }

    function createAllNodes(btNode, x, y, depth) {
        const nodeTypeIconWidth = 25
        const nodeWidthMargin = 36
        const nodeHeightMargin = 36
        const showAttributes = true

        let comp = Qt.createComponent("MovableNode.qml")
        if (comp.status !== Component.Ready) {
            console.error("Failed to load MovableNode.qml:", comp.errorString())
            return
        }

        let label = getLabel(btNode.name)

        // Create the node at a temporary position
        let obj = comp.createObject(canvas, {
            x: x,
            y: y,
            label: label,
            arrowsCanvas: arrowsCanvas,
            colorFrom: colorFrom,
            colorTo: colorTo,
        
            // BT Node properties
            btVisualizer: root,
            btNode: btNode,
            nodeType: btNode.nodeType,

            // Layout properties
            nodeTypeIconWidth: nodeTypeIconWidth,
            nodeWidthMargin: nodeWidthMargin,
            nodeHeightMargin: nodeHeightMargin,
            showAttributes: showAttributes,
        
            // Font properties
            fontSize: fontSize,
            fontFamily: fontFamily,
            fontBold: fontBold
        })

        // Assign a unique ID to the BtNode (used to find the corresponding MovableNode object)
        btNode.id = objList.length

        // Set initial status
        if (btNode.status === "IDLE") obj.styleIndex = 0
        else if (btNode.status === "RUNNING") obj.styleIndex = 1
        else if (btNode.status === "SUCCESS") obj.styleIndex = 2
        else if (btNode.status === "FAILURE") obj.styleIndex = 3

        objList.push(obj)

        // Map the BtNode ID to the MovableNode object
        root.id2NodeMap[btNode.id] = obj.btNode

        // Recursively create all child nodes
        for (let child of btNode.children) {
            createAllNodes(child, x, y + 150, depth + 1)  // Temporary Y position
        }
    }

    function arrangeNodePositions(btNode, x, y, depth) {
        // Find the node object for this btNode
        let obj = objList.find(o => o.btNode.id === btNode.id)
        if (!obj) {
            console.log(`Node ${btNode.name} with ID:${btNode.id} not found`)
            return
        }

        const spacingX = 35
        const spacingY = 150

        // Position the current node
        obj.x = x - obj.width / 2
        obj.y = y

        let childCount = btNode.children.length
        if (childCount === 0) return

        // Calculate total width of children using ACTUAL widths
        let totalChildrenWidth = 0
        let childObjects = []
    
        for (let child of btNode.children) {
            let childObj = objList.find(o => o.btNode === child)
            if (childObj) {
                totalChildrenWidth += childObj.width  // Use actual width!
                childObjects.push(childObj)
            }
        }

        let totalSpacing = (childCount - 1) * spacingX
        let totalWidth = totalChildrenWidth + totalSpacing
        let startX = x - totalWidth / 2

        // Position children using their actual widths
        let currentX = startX
        for (let i = 0; i < btNode.children.length; i++) {
            let child = btNode.children[i]
            let childObj = childObjects[i]
        
            if (childObj) {
                let childCenterX = currentX + childObj.width / 2
                let childY = y + spacingY

                // Recursively arrange child subtrees
                arrangeNodePositions(child, childCenterX, childY, depth + 1)
            
                currentX += childObj.width + spacingX
            }
        }
    }
    
    // ===============================================
    //                 Main UI Layout
    // ===============================================
    Flickable {
        id: flickable
        anchors.fill: parent
        contentWidth: canvas.width * canvasScale
        contentHeight: canvas.height * canvasScale
        clip: true
        interactive: !dragConnectionActive  // Disable interaction during drag connections

        Component.onCompleted: {
            flickable.contentX = (canvas.width * canvasScale - flickable.width) / 2
            flickable.contentY = 0
        }

        ScrollBar.vertical: ScrollBar {}
        ScrollBar.horizontal: ScrollBar {}

        Item {
            id: canvas
            width: 3000
            height: 1000
            scale: canvasScale
            transformOrigin: Item.TopLeft
            z: 2
        }

        WheelHandler {
            id: zoomHandler
            target: null
            onWheel: (event) => {
                if (event.modifiers & Qt.ControlModifier) {
                    const factor = event.angleDelta.y > 0 ? 1.1 : 1 / 1.1
                    canvasScale = Math.max(0.2, Math.min(4.0, canvasScale * factor))
                    event.accepted = true
                }
            }
        }

        Item {
            id: arrowsContainer
            width: canvas.width
            height: canvas.height
            scale: canvasScale
            transformOrigin: Item.TopLeft

            property real dashOffset: 0

            NumberAnimation on dashOffset {
                from: 20
                to: 0
                duration: 1000
                loops: Animation.Infinite
                easing.type: Easing.Linear
                running: showRealTimeUpdates
            }

            Timer {
                id: paintTimer
                interval: 33
                repeat: true
                running: showRealTimeUpdates
                onTriggered: {
                    arrowsCanvas.requestPaint()
                }
            }

            // Draw arrows between nodes
            Canvas {
                id: arrowsCanvas
                anchors.fill: parent
                z: 1

                onPaint: {
                    let ctx = getContext("2d")
                    ctx.clearRect(0, 0, width, height)
                    ctx.lineWidth = 3

                    for (let obj of objList) {
                        let btNode = obj.btNode

                        // For each child of the parent node, draw an arrow
                        for (let childBtNode of btNode.children) {
                            let childObj = objList.find(e => e.btNode === childBtNode)
                            if (!childObj)
                                continue

                            let x1 = obj.x + obj.width / 2
                            let y1 = obj.y + obj.height
                            let x2 = childObj.x + childObj.width / 2
                            let y2 = childObj.y

                            ctx.setLineDash([]) 
                            ctx.lineDashOffset = 0
                            
                            if (showStatusColors) {
                                let status = childBtNode.status
                                if (status === "RUNNING") {
                                    ctx.strokeStyle = "#00FF00"
                                    ctx.setLineDash([5, 5]) 
                                    ctx.lineDashOffset = arrowsContainer.dashOffset
                                } else if (status === "SUCCESS") {
                                    ctx.strokeStyle = "#00FFFF"
                                } else if (status === "FAILURE") {
                                    ctx.strokeStyle = "#FF0000"
                                } else {
                                    ctx.lineWidth = 4
                                    ctx.strokeStyle = "#212121"
                                }
                            } else {
                                ctx.lineWidth = 4
                                ctx.strokeStyle = "#212121"
                            }

                            ctx.beginPath()
                            ctx.moveTo(x1, y1)
                            ctx.bezierCurveTo(x1, (y1 + y2)/2, x2, (y1 + y2)/2, x2, y2)
                            ctx.stroke()
                        }
                    }
                }
            }
        }
    }

    // Rearrange Button
    Button {
        id: rearrangeButton
        width: 40
        height: 40

        anchors.top: parent.top
        anchors.right: parent.right
        anchors.topMargin: 15
        anchors.rightMargin: 15
        z: 1001

        // Modern background with gradient and shadow
        background: Rectangle {
            id: buttonBg
            radius: 30  // Circular button
        
            gradient: Gradient {
                GradientStop { 
                    position: 0.0
                    color: rearrangeButton.pressed ? "#0052A3" : 
                        rearrangeButton.hovered ? "#0078D4" : "#0099FF"
                }
                GradientStop { 
                    position: 1.0
                    color: rearrangeButton.pressed ? "#003D7A" : 
                        rearrangeButton.hovered ? "#106EBE" : "#0078D4"
                }
            }
        
            border.color: rearrangeButton.hovered ? "#FFFFFF" : "#005599"
            border.width: rearrangeButton.hovered ? 2 : 1
        
            // Smooth transitions
            Behavior on border.width {
                NumberAnimation { duration: 150 }
            }
        
            // Drop shadow effect
            Rectangle {
                anchors.fill: parent
                anchors.topMargin: 3
                anchors.leftMargin: 3
                radius: parent.radius
                color: "#40000000"  // Semi-transparent black
                z: -1
            }
        }

        // Image icon content
        contentItem: Item {
            anchors.fill: parent
        
            Image {
                id: iconImage
                anchors.centerIn: parent
                width: parent.width * 0.7
                height: parent.height * 0.7
                source: "qrc:/icons/rearrange.png"
                fillMode: Image.PreserveAspectFit
                smooth: true
            
                // Scale animation on press
                scale: rearrangeButton.pressed ? 0.9 : 1.0
                Behavior on scale {
                    NumberAnimation { duration: 100 }
                }
            }
        
            // Fallback text if image fails to load
            Text {
                anchors.centerIn: parent
                text: "⚡"
                color: "white"
                font.pixelSize: 24
                font.bold: true
                visible: iconImage.status !== Image.Ready
            }
        }

        // Hover animation
        PropertyAnimation {
            id: hoverAnimation
            target: rearrangeButton
            property: "scale"
            to: 1.1
            duration: 200
            easing.type: Easing.OutCubic
        }
    
        PropertyAnimation {
            id: unhoverAnimation
            target: rearrangeButton
            property: "scale"
            to: 1.0
            duration: 200
            easing.type: Easing.OutCubic
        }

        onHoveredChanged: {
            if (hovered) {
                hoverAnimation.start()
            } else {
                unhoverAnimation.start()
            }
        }

        onClicked: {
            rearrangeCurrentTree()
        
            // Click feedback animation
            clickAnimation.start()
        }
    
        // Click animation
        SequentialAnimation {
            id: clickAnimation
            PropertyAnimation {
                target: rearrangeButton
                property: "scale"
                to: 0.95
                duration: 50
            }
            PropertyAnimation {
                target: rearrangeButton
                property: "scale"
                to: 1.1
                duration: 100
            }
        }

        // Tooltip
        ToolTip.visible: hovered
        ToolTip.text: qsTr("重新排列節點")
        ToolTip.delay: 500
    }
}