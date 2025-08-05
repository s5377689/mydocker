import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Effects

import "BtUtils.js" as BtUtils
import "LayoutUtils.js" as LayoutUtils
import "TranslationUtils.js" as TranslationUtils

Item {
    id: commandWindow
    
    // Properties
    property bool isOpen: false
    property alias commandText: commandInput.text
    property int resizeHandleHeight: 4
    property int headerHeight: 45
    property int windowHeight: 320
    property int minWindowHeight: 200
    property int maxWindowHeight: 700
    property real animationDuration: 250

    property var qmlCliRef: null

    // Resize properties
    property bool isResizing: false
    property int resizeStartY: 0
    property int resizeStartHeight: 0

    // Theme colors
    readonly property color primaryBlue: "#0366d6"
    readonly property color primaryBlueDark: "#0253c4"
    readonly property color successGreen: "#28a745"
    readonly property color dangerRed: "#dc3545"
    readonly property color warningOrange: "#fd7e14"
    readonly property color darkBg: "#0d1117"
    readonly property color cardBg: "#161b22"
    readonly property color borderColor: "#30363d"
    readonly property color textPrimary: "#f0f6fc"
    readonly property color textSecondary: "#8b949e"
    readonly property color textMuted: "#6e7681"

    height: resizeHandleHeight + headerHeight + windowHeight
    y: parent.height - resizeHandleHeight - headerHeight  // Make the header always visible

    // Handle messages from C++ running a command
    Connections {
        target: qmlCliRef

        function onCommandRunning(msg) {
            printRunning(msg);
        }

        function onCommandError(errorMsg) {
            printError(errorMsg);
        }
        
        function onCommandWarning(warningMsg) {
            printWarning(warningMsg);
        }

        function onCommandInfo(infoMsg) {
            printInfo(infoMsg);
        }

        function onCommandSuccess(msg) {
            printHighLight(msg);
        }

        function onCommandFailure(msg) {
            printError(msg);
        }
    }

    // Handle parent resize events
    Connections {
        target: parent

        function onHeightChanged() {
            if (isOpen) {
                y = parent.height - resizeHandleHeight - headerHeight - windowHeight

                // Ensure we don't go off-screen
                if (y < 0) {
                    y = 0
                }

                // Ensure we stay within bounds
                if (y + windowHeight + headerHeight + resizeHandleHeight > parent.height) {
                    windowHeight = Math.max(
                        minWindowHeight,
                        parent.height - headerHeight - resizeHandleHeight
                    )
                }
            }
            else {
                y = parent.height - headerHeight - resizeHandleHeight
            }
        }
    }
    
    // Animations
    PropertyAnimation {
        id: slideUpAnimation
        target: commandWindow
        property: "y"
        to: commandWindow.parent.height - resizeHandleHeight - headerHeight - windowHeight
        duration: animationDuration
        easing.type: Easing.OutCubic
    }
    
    PropertyAnimation {
        id: slideDownAnimation
        target: commandWindow
        property: "y"
        to: commandWindow.parent.height - headerHeight - resizeHandleHeight
        duration: animationDuration
        easing.type: Easing.InCubic
    }
    
    // Models
    ListModel {
        id: historyModel
    }
    ListModel {
        id: messagesModel
    }
    
    // Resize handle area at the top
    Rectangle {
        id: resizeHandle
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: resizeHandleHeight

        gradient: Gradient {
            orientation: Gradient.Horizontal
            GradientStop { 
                position: 0.0; 
                color: "transparent" 
            }
            GradientStop { 
                position: 0.1; 
                color: isOpen && (resizeMouseArea.containsMouse || isResizing) ? 
                    Qt.rgba(borderColor.r, borderColor.g, borderColor.b, 0.3) : "transparent"
            }
            GradientStop { 
                position: 0.3; 
                color: isOpen && (resizeMouseArea.containsMouse || isResizing) ? borderColor : "transparent"
            }
            GradientStop { 
                position: 0.7; 
                color: isOpen && (resizeMouseArea.containsMouse || isResizing) ? borderColor : "transparent"
            }
            GradientStop { 
                position: 0.9; 
                color: isOpen && (resizeMouseArea.containsMouse || isResizing) ? 
                    Qt.rgba(borderColor.r, borderColor.g, borderColor.b, 0.3) : "transparent"
            }
            GradientStop { 
                position: 1.0; 
                color: "transparent" 
            }
        }

        Behavior on color {
            ColorAnimation { duration: 200 }
        }

        MouseArea {
            id: resizeMouseArea
            anchors.fill: parent
            cursorShape: isOpen ? Qt.SizeVerCursor : Qt.ArrowCursor
            hoverEnabled: isOpen

            onPressed: {
                isResizing = true
                resizeStartY = mouseY
                resizeStartHeight = windowHeight
            }

            onPositionChanged: {
                if (!isOpen || !isResizing)
                    return;

                let deltaY = mouseY - resizeStartY
                let newHeight = resizeStartHeight - deltaY

                windowHeight = Math.max(
                    minWindowHeight,
                    Math.min(maxWindowHeight, newHeight)
                )

                // Prevent window from going off-screen
                windowHeight = Math.min(
                    windowHeight,
                    commandWindow.parent.height
                )

                commandWindow.y = commandWindow.parent.height - windowHeight - headerHeight - resizeHandleHeight
            }

            onReleased: {
                isResizing = false
            }
        }
    }
        
    // Header area (always visible)
    Rectangle {
        id: header
        anchors.top: resizeHandle.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        height: headerHeight
        color: cardBg
        border.color: borderColor
        border.width: 1

        // Only round the top corners
        topLeftRadius: 12
        topRightRadius: 12
        bottomLeftRadius: 0
        bottomRightRadius: 0
            
        // Header gradient
        gradient: Gradient {
            GradientStop { position: 0.0; color: "#21262d" }
            GradientStop { position: 0.5; color: cardBg }
            GradientStop { position: 1.0; color: "#1c2128" }
        }
            
        MouseArea {
            anchors.fill: parent
            onClicked: toggleWindow()
            cursorShape: Qt.PointingHandCursor
        }
            
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 15
            anchors.rightMargin: 15
            spacing: 15

            // Terminal icon
            Rectangle {
                width: 28
                height: 28
                color: successGreen
                radius: 8

                // Icon glow
                layer.enabled: true
                layer.effect: MultiEffect {
                    shadowEnabled: true
                    shadowColor: successGreen
                    shadowBlur: 0.15
                    shadowScale: 1.1
                }

                Text {
                    anchors.centerIn: parent
                    text: "⌘"
                    color: "white"
                    font.pixelSize: 16
                    font.weight: Font.Bold
                }
            }

            Column {
                Layout.fillWidth: true
                spacing: 3
                    
                Text {
                    text: "Command Terminal"
                    color: textPrimary
                    font.pixelSize: 14
                    font.weight: Font.DemiBold
                    font.family: "SF Pro Display, Segoe UI, system-ui"
                }
                    
                Text {
                    text: isOpen ? "Click to minimize" : "Click to expand"
                    color: textSecondary
                    font.pixelSize: 11
                    font.family: "SF Pro Text, Segoe UI, system-ui"
                }
            }

            Text {
                Layout.alignment: Qt.AlignRight
                text: isOpen ? "▼" : "▲"
                color: "white"
                font.pixelSize: 12
                font.family: "SF Pro Text, Segoe UI, system-ui"
            }
        }
    }
        
    // Command window content
    Rectangle {
        id: windowContent
        anchors.top: header.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        color: "#1e1e1e"
        visible: commandWindow.y < commandWindow.parent.height - headerHeight - resizeHandleHeight - 1

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 8
            spacing: 8
                
            // Command history area
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#0d1117"
                border.color: "#30363d"
                border.width: 1
                radius: 4
                clip: true

                ScrollView {
                    anchors.fill: parent
                    anchors.margins: 4
                    clip: true
                        
                    ListView {
                        id: historyListView
                        model: ListModel {
                            id: displayModel
                        }
                        spacing: 4
                            
                        // delegate: Item {
                        //     width: historyListView.width
                        //     height: 25
                        //     clip: true  // Prevent overflow

                        //     // Indicator background for command type
                        //     Rectangle {
                        //         anchors.fill: parent
                        //         color: model.messageType === "command" ? "#1a1f24" : "transparent"
                        //         radius: 3
                        //         opacity: 0.5
                        //     }
                                
                        //     Row {
                        //         id: contentRow
                        //         anchors.left: parent.left
                        //         anchors.right: parent.right
                        //         anchors.top: parent.top
                        //         anchors.margins: 4
                        //         spacing: 8
                                    
                        //         Text {
                        //             id: timestampText
                        //             text: model.timestamp
                        //             color: "#666666"
                        //             font.pixelSize: 10
                        //             font.family: "Consolas, Monaco, monospace"
                        //             width: 65
                        //             elide: Text.ElideRight
                        //             anchors.top: parent.top
                        //         }
                                    
                        //         Text {
                        //             id: msgTypeText
                        //             text: model.messageType === "command" ? ">" : 
                        //                 model.messageType === "running" ? "▶" :
                        //                 model.messageType === "highlight" ? "★" :
                        //                 model.messageType === "error" ? "✗" :
                        //                 model.messageType === "warning" ? "⚠" : "•"
                        //             color: model.messageType === "command" ? "#1c83c7" :
                        //                 model.messageType === "running" ? "#00ff00" :
                        //                 model.messageType === "highlight" ? "#1bf5d1" :
                        //                 model.messageType === "error" ? "#ff4444" :
                        //                 model.messageType === "warning" ? "#ffaa00" : "#888888"
                        //             font.pixelSize: 12
                        //             font.bold: true
                        //             font.family: "Consolas, Monaco, monospace"
                        //             width: 20
                        //             anchors.top: parent.top
                        //         }
            
                        //         // Multi-line message text
                        //         Text {
                        //             text: model.text
                        //             color: model.messageType === "command" ? "#ffffff" :
                        //                 model.messageType === "running" ? "#00ff00" :
                        //                 model.messageType === "highlight" ? "#1bf5d1" :
                        //                 model.messageType === "error" ? "#ff4444" :
                        //                 model.messageType === "warning" ? "#ffaa00" : "#cccccc"
                        //             font.pixelSize: 12
                        //             font.family: "Consolas, Monaco, monospace"
                        //             width: parent.width - msgTypeText.width - timestampText.width - 10 
                        //             wrapMode: Text.Wrap
                        //             maximumLineCount: 10
                        //             elide: Text.ElideRight
                        //             textFormat: Text.PlainText
                        //         }
                        //     }
                        // }

                        // delegate: Item {
                        //     width: historyListView.width
                        //     height: messageColumn.height + 8
                        //     clip: true
    
                        //     Rectangle {
                        //         anchors.fill: parent
                        //         color: model.messageType === "command" ? "#1a1f24" : "transparent"
                        //         radius: 3
                        //         opacity: 0.3
                        //     }
    
                        //     Column {
                        //         id: messageColumn
                        //         anchors.left: parent.left
                        //         anchors.right: parent.right
                        //         anchors.top: parent.top
                        //         anchors.margins: 4
                        //         spacing: 2
        
                        //         // Header row with metadata
                        //         Row {
                        //             spacing: 8
                        //             width: parent.width
            
                        //             Text {
                        //                 text: model.timestamp
                        //                 color: "#666666"
                        //                 font.pixelSize: 10
                        //                 font.family: "Consolas, Monaco, monospace"
                        //                 width: 65
                        //             }
            
                        //             Text {
                        //                 text: model.messageType === "command" ? ">" : 
                        //                     model.messageType === "running" ? "▶" :
                        //                     model.messageType === "highlight" ? "★" :
                        //                     model.messageType === "error" ? "✗" :
                        //                     model.messageType === "warning" ? "⚠" : "•"
                        //                 color: model.messageType === "command" ? "#1c83c7" :
                        //                     model.messageType === "running" ? "#00ff00" :
                        //                     model.messageType === "highlight" ? "#1bf5d1" :
                        //                     model.messageType === "error" ? "#ff4444" :
                        //                     model.messageType === "warning" ? "#ffaa00" : "#888888"
                        //                 font.pixelSize: 12
                        //                 font.bold: true
                        //                 font.family: "Consolas, Monaco, monospace"
                        //                 width: 20
                        //             }
                        //         }
        
                        //         // Message content (full width, multi-line)
                        //         Text {
                        //             text: model.text
                        //             color: model.messageType === "command" ? "#ffffff" :
                        //                 model.messageType === "running" ? "#00ff00" :
                        //                 model.messageType === "highlight" ? "#1bf5d1" :
                        //                 model.messageType === "error" ? "#ff4444" :
                        //                 model.messageType === "warning" ? "#ffaa00" : "#cccccc"
                        //             font.pixelSize: 12
                        //             font.family: "Consolas, Monaco, monospace"
            
                        //             width: parent.width - 8
                        //             wrapMode: Text.Wrap
                        //             maximumLineCount: 15
                        //             elide: Text.ElideRight
                        //             textFormat: Text.PlainText
                        //         }
                        //     }
                        // }

                        delegate: Item {
                            width: historyListView.width
                            height: Math.max(25, contentLayout.height + 8)
                            clip: true

                            Rectangle {
                                anchors.fill: parent
                                color: model.messageType === "command" ? "#1a1f24" : "transparent"
                                radius: 3
                                opacity: 0.3
                            }

                            RowLayout {
                                id: contentLayout
                                anchors.left: parent.left
                                anchors.right: parent.right
                                anchors.top: parent.top
                                anchors.margins: 4
                                spacing: 8
        
                                // Fixed-width timestamp
                                Text {
                                    text: model.timestamp
                                    color: "#666666"
                                    font.pixelSize: 10
                                    font.family: "Consolas, Monaco, monospace"
                                    Layout.preferredWidth: 65
                                    Layout.alignment: Qt.AlignTop
                                    elide: Text.ElideRight
                                }
        
                                // Fixed-width icon
                                Text {
                                    text: model.messageType === "command" ? ">" : 
                                        model.messageType === "running" ? "▶" :
                                        model.messageType === "highlight" ? "★" :
                                        model.messageType === "error" ? "✗" :
                                        model.messageType === "warning" ? "⚠" : "•"
                                    color: model.messageType === "command" ? "#1c83c7" :
                                        model.messageType === "running" ? "#00ff00" :
                                        model.messageType === "highlight" ? "#1bf5d1" :
                                        model.messageType === "error" ? "#ff4444" :
                                        model.messageType === "warning" ? "#ffaa00" : "#888888"
                                    font.pixelSize: 12
                                    font.bold: true
                                    font.family: "Consolas, Monaco, monospace"
                                    Layout.preferredWidth: 20
                                    Layout.alignment: Qt.AlignTop
                                }

                                // Message text (fills remaining space)
                                Text {
                                    text: model.text.trim()
                                    color: model.messageType === "command" ? "#ffffff" :
                                        model.messageType === "running" ? "#00ff00" :
                                        model.messageType === "highlight" ? "#1bf5d1" :
                                        model.messageType === "error" ? "#ff4444" :
                                        model.messageType === "warning" ? "#ffaa00" : "#cccccc"
                                    font.pixelSize: 12
                                    // lineHeight: 25
                                    // lineHeightMode: Text.FixedHeight
                                    font.family: "Consolas, Monaco, monospace"
            
                                    Layout.fillWidth: true  // Takes all remaining space
                                    Layout.alignment: Qt.AlignTop
                                    wrapMode: Text.Wrap
                                    maximumLineCount: 10
                                    elide: Text.ElideRight
                                    textFormat: Text.PlainText
                                }
                            }
                        }
                    }
                }
            }
                
            // Command input area
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 40
                color: "#21262d"
                border.color: commandInput.activeFocus ? "#58a6ff" : "#30363d"
                border.width: 2
                radius: 6
                    
                Behavior on border.color {
                    ColorAnimation { duration: 200 }
                }
                    
                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 8
                        
                    // Prompt
                    Text {
                        text: ">"
                        color: "#00ff00"
                        font.pixelSize: 14
                        font.bold: true
                        font.family: "Consolas, Monaco, monospace"
                    }
                        
                    // Command input field
                    TextField {
                        id: commandInput
                        Layout.fillWidth: true
                            
                        placeholderText: "Enter command..."
                        font.pixelSize: 14
                        font.family: "Consolas, Monaco, monospace"
                        color: "#ffffff"
                        selectionColor: "#58a6ff"
                            
                        background: Rectangle {
                            color: "transparent"
                        }
                            
                        onAccepted: executeCommand()
                            
                        // Command history navigation
                        Keys.onUpPressed: {
                            if (historyModel.count > 0) {
                                text = historyModel.get(historyModel.count - 1).text
                            }
                        }
                    }
                        
                    // Execute button
                    Button {
                        text: "Execute"
                            
                        background: Rectangle {
                            color: parent.hovered ? "#259c3d" : "#1a5c1a"
                            border.color: "#30363d"
                            border.width: 1
                            radius: 4
                                
                            Behavior on color {
                                ColorAnimation { duration: 150 }
                            }
                        }
                            
                        contentItem: Text {
                            text: parent.text
                            color: "#ffffff"
                            font.pixelSize: 12
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                            
                        onClicked: executeCommand()
                    }
                }
            }
                
            // Help text
            Text {
                Layout.fillWidth: true
                text: "Press Enter to execute • Up arrow for history • Esc to close"
                color: "#666666"
                font.pixelSize: 10
                horizontalAlignment: Text.AlignHCenter
            }
        }
    }
    
    Component.onCompleted: {
        // Add some example commands to history
        // addToHistory("help")
        // addToHistory("status")
        // addToHistory("reload")
    }

    // Functions
    function toggleWindow() {
        isOpen = !isOpen
        
        if (isOpen) {
            slideUpAnimation.start()
            commandInput.forceActiveFocus()
        } else {
            slideDownAnimation.start()
        }
    }
    
    function executeCommand() {
        let cmd = commandInput.text.trim()
        if (cmd !== "") {
            addToHistory(cmd)
            qmlCliRef.executeCommand(cmd)
            commandInput.text = ""
        }
    }
    
    function addToHistory(command) {
        let timestamp = new Date().toLocaleTimeString()

        // Add to command history
        historyModel.append({
            "text": command,
            "timestamp": timestamp
        })

        // Add to display model
        displayModel.append({
            "text": command,
            "timestamp": timestamp,
            "messageType": "command"
        })

        historyListView.positionViewAtEnd()
    }

    function printRunning(msg) {
        let timestamp = new Date().toLocaleTimeString()

        displayModel.append({
            "text": msg,
            "timestamp": timestamp,
            "messageType": "running"
        })

        historyListView.positionViewAtEnd()
    }

    function printError(msg) {
        let timestamp = new Date().toLocaleTimeString()

        displayModel.append({
            "text": msg,
            "timestamp": timestamp,
            "messageType": "error"
        })

        historyListView.positionViewAtEnd()
    }

    function printWarning(msg) {
        let timestamp = new Date().toLocaleTimeString()

        displayModel.append({
            "text": msg,
            "timestamp": timestamp,
            "messageType": "warning"
        })

        historyListView.positionViewAtEnd()
    }

    function printInfo(msg) {
        let timestamp = new Date().toLocaleTimeString()

        displayModel.append({
            "text": msg,
            "timestamp": timestamp,
            "messageType": "info"
        })

        historyListView.positionViewAtEnd()
    }

    function printHighLight(msg) {
        let timestamp = new Date().toLocaleTimeString()

        displayModel.append({
            "text": msg,
            "timestamp": timestamp,
            "messageType": "highlight"
        })

        historyListView.positionViewAtEnd()
    }
    
    function closeWindow() {
        if (isOpen) {
            toggleWindow()
        }
    }
}