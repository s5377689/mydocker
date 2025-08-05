import QtQml
import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts
import QtQuick.Shapes  // for dynamic arrows

import "."


ApplicationWindow {
    id: window
    width: 640
    height: 480
    visible: true
    title: "Mission Client"

    property var gimbalStreamerCpp: gimbalStreamer
    property var gimbalControllerCpp: gimbalController
    property var ros2TopicSubscriberCpp: ros2TopicSubscriber

    // ===========================
    //          BT Parser 
    // ===========================
    property var btRoot
    property var nodeMap: [] 

    Connections {
        target: rosBridge
        function onBtXmlReceived(xmlText) {
            // console.log("Received BT XML from ROS2 topic")

            // Clear previous nodes
            for (let entry of nodeMap)
            {
                entry.item.destroy()
            }
            nodeMap = []

            btRoot = btParser.parseFromXml(xmlText)
            if (btRoot) {
                renderTree(
                    btRoot, canvas.width/2, 50, 0
                )
                arrowsCanvas.requestPaint()
                updateNodeColor()
            } else {
                console.error("Failed to load BT file.")
            }
        }
    }

    Rectangle {
        id: background
        anchors.fill: parent
        color: "#121212"
        z: 0
    }

    Rectangle {
        id: menuBarRegion
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: 35
        color: "#C8C8F8"
        border.color: Qt.darker(color)
        border.width: 1

        MenuBar {
            id: menuBar
            anchors.top: parent.top
            height: parent.height
            font.pixelSize: 15

            Menu {
                // title: "File"
                title: qsTr("檔案")
                Action {
                    // text: "Load BT XML"
                    text: "載入任務"
                    onTriggered: fileDialog.open()
                }
                Action {
                    // text: "Exit"
                    text: "結束程式"
                    onTriggered: Qt.quit()
                }
            }
        }

        Image {
            id: logo
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.right: parent.right
            source: "qrc:/resources/logo.jpg"
            height: menuBar.height
            width: height * 3.5
            // fillMode: Image.PreserveAspectFit
            smooth: true
        }
    }

    FileDialog {
        id: fileDialog
        title: "Select a BT xml file"
        nameFilters: ["XML files (*.xml)"]
        currentFolder: "file:///home/robuff/data/bt"
        onAccepted: {
            // Clear previous nodes
            for (let entry of nodeMap)
            {
                entry.item.destroy()
            }
            nodeMap = []
            console.log("selectedFile: " + new URL(selectedFile).pathname)
            btRoot = btParser.parseFromFile(new URL(selectedFile).pathname)
            if (btRoot) {
                renderTree(
                    btRoot, canvas.width/2, 50, 0
                )
                arrowsCanvas.requestPaint()

                Qt.callLater(() => {
                    const centerX = (viewport.canvasScale * canvas.width) / 2
                    const visibleWidth = flickable.width
                    flickable.contentX = Math.max(0, centerX - visibleWidth / 2)
                    flickable.contentY = 0
                })
            } else {
                console.error("Failed to load BT file.")
            }
        }
    }

    TabBar {
        id: tabBar
        anchors.top: menuBarRegion.bottom
        width: 300
        TabButton {
            // text: qsTr("Mission Progress")
            text: qsTr("任務進度")
        }
        TabButton {
            // text: qsTr("Edit Mission")
            text: qsTr("編輯任務")
        }
        TabButton {
            // text: qsTr("Gimbal Control")
            text: qsTr("雲台控制")
        }
    }

    StackLayout {
        width: parent.width
        currentIndex: tabBar.currentIndex
        anchors.top: tabBar.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom

        Item {
            id: viewport
            z: 1
            property real canvasScale: 1.0

            // ===========================
            //        BT Visualizer
            // ===========================
            Flickable {
                id: flickable
                anchors.fill: parent
                contentWidth: canvas.width * viewport.canvasScale
                contentHeight: canvas.height * viewport.canvasScale
                clip: true

                Component.onCompleted: {
                    flickable.contentX = (canvas.width * viewport.canvasScale - flickable.width) / 2
                    flickable.contentY = 0
                }

                ScrollBar.vertical: ScrollBar {}
                ScrollBar.horizontal: ScrollBar {}

                Item {
                    id: canvas
                    width: 3000
                    height: 1000
                    scale: viewport.canvasScale
                    transformOrigin: Item.TopLeft
                    z: 2
                }

                WheelHandler {
                    id: zoomHandler
                    target: null
                    onWheel: (event) => {
                        if (event.modifiers & Qt.ControlModifier)
                        {
                            const factor = event.angleDelta.y > 0 ? 1.1 : 1 / 1.1
                            viewport.canvasScale = Math.max(0.2, Math.min(4.0, viewport.canvasScale * factor))
                            event.accepted = true
                        }
                    }
                }

                Item {
                    id: arrowsContainer
                    width: canvas.width
                    height: canvas.height
                    scale: viewport.canvasScale
                    transformOrigin: Item.TopLeft

                    property real dashOffset: 0

                    NumberAnimation on dashOffset {
                        from: 20
                        to: 0
                        duration: 1000
                        loops: Animation.Infinite
                        easing.type: Easing.Linear
                        running: true
                    }

                    Timer {
                        id: paintTimer
                        interval: 33
                        repeat: true
                        running: true
                        onTriggered: {
                            arrowsCanvas.requestPaint()
                        }
                    }

                    Canvas {
                        id: arrowsCanvas
                        anchors.fill: parent
                        z: 1

                        onPaint: {
                            let ctx = getContext("2d")
                            ctx.clearRect(0, 0, width, height)
                            ctx.lineWidth = 3

                            for (let entry of nodeMap)
                            {
                                let parentItem = entry.item
                                let parentData = entry.data

                                // For each child of the parent node, draw an arrow
                                for (let child of parentData.children)
                                {
                                    let childEntry = nodeMap.find(e => e.data === child)
                                    if (!childEntry)
                                        continue
                                    let childItem = childEntry.item

                                    let x1 = parentItem.x + parentItem.width / 2
                                    let y1 = parentItem.y + parentItem.height
                                    let x2 = childItem.x + childItem.width / 2
                                    let y2 = childItem.y

                                    ctx.setLineDash([]) 
                                    ctx.lineDashOffset = 0
                                    let status = child.status
                                    if (status === "RUNNING")
                                    {
                                        ctx.strokeStyle = "#00FF00"
                                        ctx.setLineDash([5, 5]) 
                                        ctx.lineDashOffset = arrowsContainer.dashOffset
                                    }
                                    else if (status === "SUCCESS")
                                    {
                                        ctx.strokeStyle = "#00FFFF"
                                    }
                                    else if (status === "FAILURE")
                                    {
                                        ctx.strokeStyle = "#FF0000"
                                    }
                                    else
                                    {
                                        ctx.strokeStyle = "#090909"
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
        }
        Item {
            id: editMission
            // to-do
        }   
        // -----------------------------------------
        //            Gimbal Control Tab
        // -----------------------------------------
        GimbalControlPanel {
            id: gimbalControlPanel
            anchors.fill: parent
            anchors.leftMargin: 10
            anchors.rightMargin: 10
            anchors.topMargin: 10
            anchors.bottomMargin: 10
            gimbalStreamer: gimbalStreamerCpp
            gimbalController: gimbalControllerCpp
            ros2TopicSubscriber: ros2TopicSubscriberCpp
        } 
    }

    property string currentLanguage: "zh"

    property var translations: {
        "zh": {
            "sequence": " → ",
            "fallback": " ? ",
            "arm": "解鎖",
            "disarm": "上鎖",
            "guided": "導航模式",
            "hold": "停止",
            "registertarget": "註冊目標",
            "navigate": "自主導航",
            "search": "自主搜索",
            "navigatewhilesearch": "自主導航搜索",
            "zigzag": "Z字衝刺",
            "traceeight": "八字機動",
            "stopgimbalcontrol": "停止雲台控制",
            "resumegimbalcontrol": "恢復雲台控制",
            "stopgimbalstreamer": "停止雲台視訊",
            "resumegimbalstreamer": "恢復雲台視訊",
            "takephoto": "拍照",
        },
        "en": {
            "sequence": "→",
            "fallback": "?",
        }
    }

    function switchLanguage() {
        if (currentLanguage === "zh") {
            currentLanguage = "en";
        } else {
            currentLanguage = "zh";
        }
        arrowsCanvas.requestPaint();
    }

    function getLabel(name) {
        let lowerName = name.toLowerCase();
        return translations[currentLanguage][lowerName] || name;
    }

    function renderTree(node, x, y, depth)
    {
        let colors = ["#0D0D0D", "#1A1A1A", "#202020", "#2A2A2A", "#323232", "#404040", "#4A4A4A",
            "#545454", "#5E5E5E", "#686868"]

        const spacingX = 150
        const spacingY = 150

        let comp = Qt.createComponent("MovableNode.qml")
        if (comp.status !== Component.Ready) {
            console.error("Failed to load MovableNode.qml:", comp.errorString())
            return
        }

        let label = getLabel(node.name)

        let obj = comp.createObject(canvas, {
            x: x,
            y: y,
            color1: colors[depth % colors.length],
            color2: colors[(depth + 1) % colors.length],
            nodeId: nodeMap.length,
            label: label,
            arrowsCanvas: arrowsCanvas
        })
        nodeMap.push({ item: obj, data: node })

        let childCount = node.children.length
        let baseX = x - ((childCount - 1) * spacingX) / 2

        for (let i = 0; i < childCount; ++i)
        {
            let child = node.children[i]
            let childX = baseX + i * spacingX
            let childY = y + spacingY
            renderTree(child, childX, childY, depth + 1)
        }
    }

    function updateNodeColor()
    {
        for (let entry of nodeMap)
        {
            let item = entry.item
            let data = entry.data

            if (data.status === "IDLE")
            {
                item.styleIndex = 0
            }
            else if (data.status === "RUNNING")
            {
                item.styleIndex = 1
            }
            else if (data.status === "SUCCESS")
            {
                item.styleIndex = 2
            }
            else if (data.status === "FAILURE")
            {
                item.styleIndex = 3
            }
        }
    }
}
