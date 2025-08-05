import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    property var mavlinkClientRef
    property bool ardupilotConnected: mavlinkClientRef && mavlinkClientRef.connected
    property string filterText: ""
    property string editingParameter: ""
    property int listHeaderHeight: 35
    property int listItemHeight: 35
    property int editRowHeight: 35
    property int textFontSize: 12

    Rectangle {
        anchors.fill: parent
        color: "#f5f5f5"  // Better light gray color

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 20
            spacing: 15

            // Header + connection status
            RowLayout {
                Layout.fillWidth: true
                spacing: 10

                Item {
                    Layout.preferredWidth: parent.width * 0.3
                }

                Text {
                    text: "ArduPilot Parameters Editor"
                    font.pixelSize: 18
                    font.bold: true
                    color: "#333"
                    Layout.alignment: Qt.AlignCenter
                }

                Item {
                    Layout.preferredWidth: 30
                }

                // Connect Button + Status Indicator
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    color: ardupilotConnected ? "#e8f5e8" : "#ffeaea"
                    border.color: ardupilotConnected ? "#4caf50" : "#f44336"
                    border.width: 1
                    radius: 4

                    // Make it clickable
                    MouseArea {
                        anchors.fill: parent
                        hoverEnabled: true
                        cursorShape: mavlinkClientRef ? Qt.PointingHandCursor : Qt.ArrowCursor

                        // Visual feedback on hover
                        onEntered: {
                            if (mavlinkClientRef) {
                                parent.color = ardupilotConnected ? "#c8e6c9" : "#ffcdd2";
                            }
                        }

                        onExited: {
                            parent.color = ardupilotConnected ? "#e8f5e8" : "#ffeaea";
                        }

                        // Handle click
                        onClicked: {
                            if (mavlinkClientRef) {
                                if (mavlinkClientRef.connected) {
                                    mavlinkClientRef.disconnect();
                                } else {
                                    mavlinkClientRef.connectToSITL();
                                }
                            }
                        }
                    }

                    Text {
                        anchors.centerIn: parent
                        text: {
                            if (!mavlinkClientRef) {
                                return "MAVLink client not available";
                            } else if (ardupilotConnected) {
                                return `✓ Connected - ${paramModel.count} parameters loaded`;
                            } else {
                                return "⚡ Click to Connect to ArduPilot";
                            }
                        }
                        color: ardupilotConnected ? "#2e7d32" : "#c62828"
                        font.pixelSize: 14
                        font.weight: Font.Medium
                    }
                }
            }

            // Control panel
            RowLayout {
                spacing: 10

                // Actions Menu - Only visible when connected
                Rectangle {
                    Layout.preferredWidth: 150
                    Layout.preferredHeight: 40
                    color: "#ffffff"
                    border.color: "#ddd"
                    border.width: 1
                    radius: 4
                    opacity: ardupilotConnected ? 1 : 0

                    Behavior on opacity {
                        NumberAnimation {
                            duration: 100
                        }
                    }

                    Behavior on color {
                        ColorAnimation {
                            duration: 100
                        }
                    }

                    Behavior on border.color {
                        ColorAnimation {
                            duration: 100
                        }
                    }

                    // Make entire rectangle clickable
                    MouseArea {
                        anchors.fill: parent
                        hoverEnabled: true
                        cursorShape: Qt.PointingHandCursor

                        // Visual feedback on hover
                        onEntered: {
                            if (!actionsMenu.opened) {
                                parent.color = "#ffffff";
                                parent.border.color = "#6f63da";
                            }
                        }

                        onExited: {
                            if (!actionsMenu.opened) {
                                parent.color = "#ffffff";
                                parent.border.color = "#ddd";
                            }
                        }

                        // Handle click - open menu
                        onClicked: {
                            if (!actionsMenu.opened) {
                                actionsMenu.popup(parent, 0, parent.height + 1);
                            }
                        }
                    }

                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 8
                        spacing: 6

                        Text {
                            text: "⚙️"
                            font.pixelSize: textFontSize + 2
                            Layout.alignment: Qt.AlignVCenter
                        }

                        Text {
                            text: "Actions"
                            font.pixelSize: textFontSize + 2
                            font.bold: true
                            color: "#555"
                            Layout.fillWidth: true
                            Layout.alignment: Qt.AlignVCenter
                        }
                    }

                    Menu {
                        id: actionsMenu
                        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

                        // Menu background
                        background: Rectangle {
                            color: "#ffffff"
                            border.color: "#4caf50"
                            border.width: 2
                            radius: 10
                            implicitWidth: 270

                            // Drop shadow for depth
                            Rectangle {
                                anchors.fill: parent
                                anchors.margins: -2
                                color: "#18000000"
                                radius: parent.radius
                                z: -1
                            }

                            // Inner highlight border
                            Rectangle {
                                anchors.fill: parent
                                anchors.margins: 2
                                color: "transparent"
                                border.color: "#557ec07e"
                                border.width: 2
                                radius: parent.radius - 1
                            }
                        }

                        onOpened: {
                            let button = parent;
                            button.color = "#e8f5e8";
                            button.border.color = "#4caf50";
                        }

                        onClosed: {
                            let button = parent;
                            button.color = "#ffffff";
                            button.border.color = "#ddd";
                        }

                        // Menu items with type configuration
                        StyledMenuItem {
                            text: "Reset User Parameters"
                            itemType: "config"
                            onTriggered: resetConfigDialogLoader.open()
                        }

                        StyledMenuItem {
                            text: "Reboot ArduPilot"
                            itemType: "reboot"
                            onTriggered: rebootDialogLoader.open()
                        }

                        StyledMenuItem {
                            text: "Refresh Parameters"
                            itemType: "refresh"
                            onTriggered: {
                                if (mavlinkClientRef) {
                                    mavlinkClientRef.requestParameters();
                                    statusPopup.message = "Refreshing parameters from ArduPilot...";
                                    statusPopup.isSuccess = true;
                                    statusPopup.open();
                                }
                            }
                        }

                        // Beautiful separator with gradient
                        MenuSeparator {
                            background: Rectangle {
                                implicitHeight: 12
                                color: "transparent"

                                Rectangle {
                                    anchors.centerIn: parent
                                    width: parent.width * 0.85
                                    height: 2
                                    radius: 1
                                    gradient: Gradient {
                                        orientation: Gradient.Horizontal
                                        GradientStop {
                                            position: 0.0
                                            color: "transparent"
                                        }
                                        GradientStop {
                                            position: 0.2
                                            color: "#4caf50"
                                        }
                                        GradientStop {
                                            position: 0.5
                                            color: "#2196f3"
                                        }
                                        GradientStop {
                                            position: 0.8
                                            color: "#ff9800"
                                        }
                                        GradientStop {
                                            position: 1.0
                                            color: "transparent"
                                        }
                                    }

                                    // Glow effect
                                    Rectangle {
                                        anchors.centerIn: parent
                                        width: parent.width + 2
                                        height: parent.height + 1
                                        radius: parent.radius
                                        color: "transparent"
                                        border.color: "#ffffff"
                                        border.width: 1
                                        opacity: 0.5
                                    }
                                }
                            }
                        }

                        StyledMenuItem {
                            text: "⚠️ Reset ALL Parameters"
                            itemType: "danger"
                            onTriggered: resetAllDialogLoader.open()
                        }

                        // Add missing sensor reset item
                        StyledMenuItem {
                            text: "Reset Sensor Parameters"
                            itemType: "sensor"
                            onTriggered: resetSensorDialogLoader.open()
                        }
                    }
                }

                // Search/Filter Section
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 50
                    color: "#ffffff"
                    border.color: "#ddd"
                    border.width: 1
                    radius: 4
                    opacity: ardupilotConnected ? 1 : 0

                    Behavior on opacity {
                        NumberAnimation {
                            duration: 100
                        }
                    }

                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 8
                        spacing: 8

                        Text {
                            text: "Filter:"
                            font.pixelSize: textFontSize + 2
                            color: "#555"
                            Layout.alignment: Qt.AlignVCenter
                        }

                        TextField {
                            id: filterField
                            Layout.fillWidth: true
                            Layout.preferredHeight: 32
                            placeholderText: "Enter parameter name to filter..."
                            font.pixelSize: textFontSize
                            color: "#101010"

                            onTextChanged: {
                                filterText = text.trim();
                                filterTimer.restart();
                            }

                            background: Rectangle {
                                color: parent.focus ? "#f0f8ff" : "#ffffff"
                                border.color: parent.focus ? "#4caf50" : "#ddd"
                                border.width: 1
                                radius: 3
                            }
                        }

                        Button {
                            text: "Clear"
                            Layout.preferredWidth: 60
                            Layout.preferredHeight: 32
                            enabled: filterText.length > 0

                            onClicked: {
                                filterField.clear();
                                filterField.focus = true;
                            }
                        }

                        // Filter states
                        Text {
                            text: filteredModel.count + "/" + paramModel.count
                            font.pixelSize: 12
                            color: "#666"
                            Layout.alignment: Qt.AlignVCenter
                        }
                    }
                }
            }

            // Parameters list container
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                border.color: "#ddd"
                border.width: 1
                radius: 8
                color: "#ffffff"

                // Header for the list
                Rectangle {
                    id: listHeader
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.right: parent.right
                    height: listHeaderHeight
                    color: "#f0f0f0"
                    border.color: "#ddd"
                    border.width: 1

                    Row {
                        anchors.fill: parent
                        anchors.margins: 8

                        Text {
                            text: "Parameter Name"
                            width: parent.width * 0.5
                            height: parent.height
                            font.bold: true
                            font.pixelSize: textFontSize + 1
                            color: "#555"
                            verticalAlignment: Text.AlignVCenter
                        }

                        Text {
                            text: "Value"
                            width: parent.width * 0.5
                            height: parent.height
                            font.bold: true
                            font.pixelSize: textFontSize + 1
                            color: "#555"
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }

                // Parameters ListView
                ListView {
                    id: paramListView
                    anchors.top: listHeader.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.bottom: parent.bottom
                    anchors.margins: 1
                    clip: true

                    model: filteredModel

                    delegate: Rectangle {
                        width: paramListView.width
                        height: editingParameter === model.paramName ? listItemHeight + editRowHeight : listItemHeight
                        color: index % 2 === 0 ? "#ffffff" : "#f9f9f9"

                        Behavior on height {
                            NumberAnimation {
                                duration: 200
                            }
                        }

                        Column {
                            anchors.fill: parent
                            // anchors.margins: 8

                            Row {
                                width: parent.width
                                height: listItemHeight
                                spacing: 0
                                anchors.left: parent.left
                                anchors.leftMargin: 8

                                Text {
                                    id: nameText
                                    text: {
                                        let paramName = model.paramName || "";
                                        let filter = filterText.trim();

                                        if (!filter || !paramName)
                                            return paramName;

                                        let matchIndex = paramName.toLowerCase().indexOf(filter.toLowerCase());

                                        if (matchIndex === -1)
                                            return paramName;

                                        // Split and highlight the matched portion
                                        let beforeMatch = paramName.substring(0, matchIndex);
                                        let matchedText = paramName.substring(matchIndex, matchIndex + filter.length);
                                        let afterMatch = paramName.substring(matchIndex + filter.length);

                                        return beforeMatch + `<span style="color: #1976d2; font-weight: bold; background-color: #e3f2fd;">${matchedText}</span>` + afterMatch;
                                    }
                                    width: parent.width * 0.4
                                    height: parent.height
                                    font.family: "Consolas, Monaco, monospace"
                                    font.pixelSize: textFontSize
                                    color: "#333"
                                    verticalAlignment: Text.AlignVCenter
                                    elide: Text.ElideRight
                                    textFormat: Text.RichText
                                }

                                Text {
                                    id: valueText
                                    text: model.paramValue ? model.paramValue.toString() : ""
                                    width: parent.width * 0.5
                                    height: parent.height
                                    font.family: "Consolas, Monaco, monospace"
                                    font.pixelSize: textFontSize
                                    color: "#666"
                                    verticalAlignment: Text.AlignVCenter
                                    elide: Text.ElideRight
                                }
                            }

                            // Edit row shown when editing
                            RowLayout {
                                width: parent.width
                                height: editingParameter === model.paramName ? editRowHeight : 0
                                visible: editingParameter === model.paramName
                                spacing: 10

                                Item {
                                    Layout.preferredWidth: nameText.width - 5
                                }

                                TextField {
                                    id: editField
                                    Layout.alignment: Qt.AlignVCenter
                                    Layout.preferredWidth: parent.width * 0.25
                                    Layout.preferredHeight: parent.height - 6
                                    text: model.paramValue ? model.paramValue.toString() : ""
                                    color: "#101010"
                                    font.pixelSize: textFontSize
                                    validator: DoubleValidator {}

                                    background: Rectangle {
                                        color: "#ffffff"
                                        border.color: parent.focus ? "#4caf50" : "#ddd"
                                        border.width: 1
                                        radius: 2
                                    }

                                    onAccepted: {
                                        updateParameter(model.paramName, text);
                                    }
                                }

                                Button {
                                    text: "Update"
                                    Layout.preferredWidth: 60
                                    Layout.preferredHeight: parent.height - 6
                                    font.pixelSize: 10
                                    enabled: editField.text !== model.paramValue.toString()
                                    // ToolTip.text: "Update parameter value"
                                    ToolTip.text: "覆寫參數"
                                    ToolTip.visible: hovered

                                    onClicked: {
                                        updateParameter(model.paramName, editField.text);
                                    }
                                }

                                Button {
                                    text: "↻"
                                    Layout.preferredWidth: 30
                                    Layout.preferredHeight: parent.height - 6
                                    font.pixelSize: 12
                                    enabled: ardupilotConnected
                                    // ToolTip.text: "Refresh parameter"
                                    ToolTip.text: "重載參數"
                                    ToolTip.visible: hovered

                                    onClicked: {
                                        if (mavlinkClientRef) {
                                            mavlinkClientRef.refreshParameter(model.paramName);
                                        }
                                    }
                                }

                                Item {
                                    Layout.fillWidth: true
                                }
                            }
                        }

                        // Hover effect
                        MouseArea {
                            anchors.fill: parent
                            anchors.rightMargin: parent.width * 0.2  // Exclude button area
                            anchors.bottomMargin: editingParameter === model.paramName ? editRowHeight : 0
                            hoverEnabled: true
                            onEntered: parent.color = "#e3f2fd"
                            onExited: parent.color = index % 2 === 0 ? "#ffffff" : "#f9f9f9"

                            // Double-click to edit
                            onDoubleClicked: {
                                if (ardupilotConnected) {
                                    if (editingParameter === model.paramName) {
                                        editingParameter = "";
                                    } else {
                                        editingParameter = model.paramName;
                                    }
                                }
                            }
                        }
                    }

                    // Empty state message
                    Text {
                        anchors.centerIn: parent
                        text: {
                            if (!ardupilotConnected) {
                                return "Connect to ArduPilot to view parameters";
                            } else if (paramModel.count === 0) {
                                return "No parameters received yet...\nThis may take a moment.";
                            } else if (filteredModel.count === 0 && filterText) {
                                return `No parameters match "${filterText}"\nTry a different search term.`;
                            } else {
                                return "";
                            }
                        }
                        color: "#888"
                        font.pixelSize: textFontSize + 2
                        horizontalAlignment: Text.AlignHCenter
                        visible: paramModel.count === 0
                    }

                    // Scrollbar
                    ScrollBar.vertical: ScrollBar {
                        active: true
                        policy: ScrollBar.AsNeeded
                    }
                }
            }
        }
    }

    // Enhanced MenuItem with bright colors and icons
    component StyledMenuItem: MenuItem {
        id: menuItem
        property string itemType: "default"
        property string iconText: "•"

        // Color configuration for different item types
        readonly property var itemConfig: ({
                "config": {
                    icon: "⚙️",
                    bgColor: "#fff3e0",
                    hoverColor: "#ffe0b2",
                    borderColor: "#ff9800",
                    textColor: "#f57c00"
                },
                "reboot": {
                    icon: "🔌",
                    bgColor: "#e3f2fd",
                    hoverColor: "#bbdefb",
                    borderColor: "#2196f3",
                    textColor: "#1976d2"
                },
                "refresh": {
                    icon: "🔄",
                    bgColor: "#e8f5e8",
                    hoverColor: "#c8e6c9",
                    borderColor: "#4caf50",
                    textColor: "#2e7d32"
                },
                "sensor": {
                    icon: "📡",
                    bgColor: "#e8f5e8",
                    hoverColor: "#c8e6c9",
                    borderColor: "#4caf50",
                    textColor: "#2e7d32"
                },
                "danger": {
                    icon: "⚠️",
                    bgColor: "#ffebee",
                    hoverColor: "#ffcdd2",
                    borderColor: "#f44336",
                    textColor: "#d32f2f"
                }
            })[itemType] || {
            icon: "•",
            bgColor: "#f8f9fa",
            hoverColor: "#e9ecef",
            borderColor: "#6c757d",
            textColor: "#495057"
        }

        enabled: ardupilotConnected
        implicitWidth: 300
        implicitHeight: 45

        background: Rectangle {
            anchors.fill: parent
            anchors.margins: 2
            radius: 8
            color: menuItem.hovered ? itemConfig.hoverColor : "transparent"
            border.color: menuItem.hovered ? itemConfig.borderColor : "transparent"
            border.width: menuItem.hovered ? 2 : 0

            // Smooth animations
            Behavior on color {
                ColorAnimation {
                    duration: 200
                }
            }
            Behavior on border.color {
                ColorAnimation {
                    duration: 200
                }
            }
            Behavior on border.width {
                NumberAnimation {
                    duration: 200
                }
            }

            // Subtle background when not hovered
            Rectangle {
                anchors.fill: parent
                anchors.margins: menuItem.hovered ? 0 : 2
                radius: parent.radius - 2
                color: menuItem.hovered ? "transparent" : itemConfig.bgColor
                opacity: menuItem.hovered ? 0 : 0.3

                Behavior on opacity {
                    NumberAnimation {
                        duration: 200
                    }
                }
            }
        }

        contentItem: RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 12 
            anchors.rightMargin: 12
            spacing: 12

            // Icon with background circle
            Rectangle {
                Layout.preferredWidth: 32
                Layout.preferredHeight: 32
                Layout.alignment: Qt.AlignVCenter
                radius: 16
                color: itemConfig.bgColor
                border.color: itemConfig.borderColor
                border.width: menuItem.hovered ? 2 : 1

                Behavior on border.width {
                    NumberAnimation {
                        duration: 200
                    }
                }

                Text {
                    anchors.centerIn: parent
                    text: iconText || itemConfig.icon
                    font.pixelSize: 16
                }

                // Glow effect on hover
                Rectangle {
                    anchors.centerIn: parent
                    width: parent.width + 6
                    height: parent.height + 6
                    radius: (parent.width + 6) / 2
                    color: "transparent"
                    border.color: itemConfig.borderColor
                    border.width: 1
                    opacity: menuItem.hovered ? 0.3 : 0

                    Behavior on opacity {
                        NumberAnimation {
                            duration: 200
                        }
                    }
                }
            }

            // Text content
            Text {
                text: parent.parent.text.replace("⚠️ ", "")
                color: itemConfig.textColor
                font.pixelSize: textFontSize + 2
                font.bold: true
                Layout.fillWidth: true
                Layout.alignment: Qt.AlignVCenter
                verticalAlignment: Text.AlignVCenter

                Behavior on color {
                    ColorAnimation {
                        duration: 200
                    }
                }
            }

            // Status indicator
            Rectangle {
                Layout.preferredWidth: 8
                Layout.preferredHeight: 8
                Layout.alignment: Qt.AlignVCenter
                radius: 4
                color: itemConfig.borderColor
                visible: menuItem.enabled
                opacity: menuItem.hovered ? 1 : 0.6

                Behavior on opacity {
                    NumberAnimation {
                        duration: 200
                    }
                }

                // Pulse effect
                SequentialAnimation on scale {
                    running: menuItem.hovered
                    loops: Animation.Infinite
                    NumberAnimation {
                        from: 1.0
                        to: 1.3
                        duration: 800
                    }
                    NumberAnimation {
                        from: 1.3
                        to: 1.0
                        duration: 800
                    }
                }
            }
        }
    }

    // Unfiltered model for parameters
    ListModel {
        id: paramModel
    }

    // Filtered model for display
    ListModel {
        id: filteredModel
    }

    // Timer for filtering parameters after typing
    Timer {
        id: filterTimer
        interval: 300  // Delay in milliseconds
        repeat: false
        onTriggered: applyFilter()
    }

    Popup {
        id: statusPopup

        // Position at top-center - keep this fixed
        x: (parent.width - width) / 2
        y: 30

        width: Math.max(250, Math.min(parent.width * 0.8, contentRow.implicitWidth + 40))
        height: Math.max(70, contentRow.implicitHeight + 30)

        modal: false

        property alias message: statusText.text
        property alias isSuccess: statusRect.success

        background: Rectangle {
            id: statusRect
            property bool success: true

            // Modern card design
            color: "#ffffff"
            border.color: success ? "#28a745" : "#dc3545"
            border.width: 3
            radius: 12

            // Enhanced shadow with multiple layers
            Rectangle {
                anchors.fill: parent
                anchors.topMargin: 6
                anchors.leftMargin: 6
                color: "#00000030"
                radius: parent.radius
                z: -2
            }

            Rectangle {
                anchors.fill: parent
                anchors.topMargin: 3
                anchors.leftMargin: 3
                color: "#00000015"
                radius: parent.radius
                z: -1
            }
        }

        contentItem: RowLayout {
            id: contentRow
            anchors.centerIn: parent
            anchors.leftMargin: 15
            spacing: 12

            // Animated status icon
            Rectangle {
                Layout.preferredWidth: 32
                Layout.preferredHeight: 32
                radius: 16
                color: statusRect.success ? "#e8f5e8" : "#ffebee"
                border.color: statusRect.success ? "#28a745" : "#dc3545"
                border.width: 2

                Text {
                    anchors.centerIn: parent
                    text: statusRect.success ? "✅" : "❌"
                    font.pixelSize: 16
                }

                // Pulsing animation for the icon background
                SequentialAnimation on scale {
                    running: statusPopup.visible
                    loops: Animation.Infinite
                    NumberAnimation {
                        from: 1.0
                        to: 1.1
                        duration: 1000
                    }
                    NumberAnimation {
                        from: 1.1
                        to: 1.0
                        duration: 1000
                    }
                }
            }

            // Message text
            Text {
                id: statusText
                font.pixelSize: textFontSize + 1
                font.bold: true
                color: "#333"

                horizontalAlignment: Text.AlignLeft
                verticalAlignment: Text.AlignVCenter
                wrapMode: Text.WordWrap

                Layout.fillWidth: true
                Layout.alignment: Qt.AlignVCenter
                Layout.maximumWidth: parent.parent.width * 0.7
            }

            // Close button
            Button {
                Layout.preferredWidth: 24
                Layout.preferredHeight: 24
                text: "×"

                background: Rectangle {
                    color: parent.hovered ? "#f0f0f0" : "transparent"
                    radius: 12

                    Behavior on color {
                        ColorAnimation {
                            duration: 150
                        }
                    }
                }

                contentItem: Text {
                    text: parent.text
                    color: "#666"
                    font.pixelSize: 14
                    font.bold: true
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }

                onClicked: statusPopup.close()
            }
        }

        Timer {
            id: statusTimer
            interval: 4000
            onTriggered: statusPopup.close()
        }

        onOpened: statusTimer.start()
        onClosed: statusTimer.stop()

        enter: Transition {
            SequentialAnimation {
                // Scale bounce effect
                ParallelAnimation {
                    NumberAnimation {
                        property: "scale"
                        from: 0.5
                        to: 1.1
                        duration: 250
                        easing.type: Easing.OutBack
                    }
                    NumberAnimation {
                        property: "opacity"
                        from: 0.0
                        to: 1.0
                        duration: 200
                    }
                }

                // Settle to normal size
                NumberAnimation {
                    property: "scale"
                    from: 1.1
                    to: 1.0
                    duration: 150
                    easing.type: Easing.OutBack
                }
            }
        }

        exit: Transition {
            ParallelAnimation {
                NumberAnimation {
                    property: "scale"
                    from: 1.0
                    to: 0.8
                    duration: 150
                    easing.type: Easing.InBack
                }
                NumberAnimation {
                    property: "opacity"
                    from: 1.0
                    to: 0.0
                    duration: 150
                }
            }
        }
    }

    // Handle parameter updates properly
    Connections {
        target: mavlinkClientRef

        function onParametersChanged() {
            updateParameterModel();
        }

        function onConnectionChanged() {
            if (mavlinkClientRef && !mavlinkClientRef.connected) {
                paramModel.clear();
                filteredModel.clear();
            }
        }

        function onParameterUpdateResult(paramName, success, message) {
            console.log("Parameter update result:", paramName, success, message);
            statusPopup.message = `${paramName}: ${message}`;
            statusPopup.isSuccess = success;
            statusPopup.open();

            if (success)
                editingParameter = ""; // Clear editing state on success
        }
    }

    // Helper function to update parameter model
    function updateParameterModel() {
        paramModel.clear();
        if (mavlinkClientRef && mavlinkClientRef.parameters) {
            // Convert parameters object to sorted array
            let paramKeys = Object.keys(mavlinkClientRef.parameters).sort();
            for (let i = 0; i < paramKeys.length; i++) {
                let key = paramKeys[i];
                paramModel.append({
                    paramName: key,
                    paramValue: mavlinkClientRef.parameters[key]
                });
            }
        }
        applyFilter();
    }

    function applyFilter() {
        filteredModel.clear();

        let filter = filterText.toLowerCase().trim();

        for (let i = 0; i < paramModel.count; ++i) {
            let item = paramModel.get(i);
            let paramName = item.paramName.toString().toLowerCase();

            // Show item if no filter or if it contains the filter
            if (!filter || paramName.includes(filter)) {
                filteredModel.append({
                    paramName: item.paramName,
                    paramValue: item.paramValue
                });
            }
        }
    }

    function updateParameter(paramName, newValue) {
        if (!mavlinkClientRef || !mavlinkClientRef.connected) {
            statusPopup.message = "Not connected to ArduPilot";
            statusPopup.isSuccess = false;
            statusPopup.open();
            return;
        }

        console.log("Updating parameter:", paramName, "to", newValue);
        mavlinkClientRef.setParameter(paramName, newValue);
    }

    // Initialize on component completion
    Component.onCompleted: {
        if (mavlinkClientRef && mavlinkClientRef.parameters) {
            updateParameterModel();
        }
    }

    // ---------------------------------------------------------
    //                          Dialogs
    // ---------------------------------------------------------
    // Dialog Template
    Component {
        id: uniformDialogComponent

        Dialog {
            id: dialog
            modal: true
            parent: Overlay.overlay
            x: (parent.width - width) / 2
            y: (parent.height - height) / 2

            property string dialogTitle: ""
            property string dialogText: ""
            property string iconText: "ℹ️"
            property string primaryColor: "#2196f3"
            property string secondaryColor: "#e3f2fd"
            property string dangerColor: "#f44336"
            property bool isDangerous: false
            property string acceptText: "Yes"
            property string rejectText: "Cancel"

            signal dialogAccepted
            signal dialogRejected

            // Fixed sizing - don't rely on automatic sizing
            width: 450
            height: 350

            // Remove the automatic height calculation that was causing issues
            // height: contentHeight + headerHeight + footerHeight + 40

            // Custom header with proper sizing
            header: Rectangle {
                width: dialog.width
                height: 60
                color: isDangerous ? dangerColor : primaryColor

                // Proper rounded corners for header only
                Rectangle {
                    anchors.fill: parent
                    color: parent.color
                    radius: 8

                    // Cut off bottom rounded corners
                    Rectangle {
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.bottom: parent.bottom
                        height: 8
                        color: parent.color
                    }
                }

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 16
                    spacing: 12

                    Text {
                        text: iconText
                        font.pixelSize: 20
                        color: "white"
                        Layout.alignment: Qt.AlignVCenter
                    }

                    Text {
                        text: dialogTitle
                        font.pixelSize: 16
                        font.bold: true
                        color: "white"
                        Layout.fillWidth: true
                        Layout.alignment: Qt.AlignVCenter
                        wrapMode: Text.WordWrap
                    }
                }
            }

            // Custom background
            background: Rectangle {
                color: "#ffffff"
                radius: 8
                border.color: isDangerous ? dangerColor : primaryColor
                border.width: 2

                // Drop shadow effect
                Rectangle {
                    anchors.fill: parent
                    anchors.topMargin: 3
                    anchors.leftMargin: 3
                    color: "#00000020"
                    radius: parent.radius
                    z: -1
                }
            }

            // Fixed content area
            contentItem: Rectangle {
                color: "transparent"

                ScrollView {
                    anchors.fill: parent
                    anchors.margins: 20
                    clip: true

                    Text {
                        text: dialogText
                        font.pixelSize: textFontSize + 2
                        color: "#333"
                        wrapMode: Text.WordWrap
                        width: parent.width
                        lineHeight: 1.4
                        textFormat: Text.PlainText
                    }
                }
            }

            // Custom footer with proper sizing
            footer: Rectangle {
                width: dialog.width
                height: 70
                color: isDangerous ? "#ffebee" : secondaryColor

                // Proper rounded corners for footer only
                Rectangle {
                    anchors.fill: parent
                    color: parent.color
                    radius: 8

                    // Cut off top rounded corners
                    Rectangle {
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        height: 8
                        color: parent.color
                    }
                }

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 15
                    spacing: 12

                    Item {
                        Layout.fillWidth: true
                    }

                    // Cancel Button
                    Button {
                        Layout.preferredWidth: 100
                        Layout.preferredHeight: 36
                        text: rejectText

                        background: Rectangle {
                            color: parent.pressed ? "#f5f5f5" : "#ffffff"
                            border.color: "#bdbdbd"
                            border.width: 1
                            radius: 6

                            // Subtle shadow
                            Rectangle {
                                anchors.fill: parent
                                anchors.topMargin: 2
                                anchors.leftMargin: 2
                                color: "#00000015"
                                radius: parent.radius
                                z: -1
                            }
                        }

                        contentItem: Text {
                            text: parent.text
                            color: "#555"
                            font.pixelSize: 12
                            font.bold: true
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }

                        onClicked: {
                            dialog.dialogRejected();
                            dialog.close();
                        }
                    }

                    // Accept Button
                    Button {
                        Layout.preferredWidth: Math.max(140, acceptButtonText.contentWidth + 30)
                        Layout.preferredHeight: 36
                        text: acceptText

                        Text {
                            id: acceptButtonText
                            text: parent.text
                            visible: false
                            font.pixelSize: 12
                            font.bold: true
                        }

                        background: Rectangle {
                            color: {
                                if (isDangerous) {
                                    return parent.pressed ? "#c62828" : "#f44336";
                                } else {
                                    return parent.pressed ? "#1976d2" : primaryColor;
                                }
                            }
                            border.color: {
                                if (isDangerous) {
                                    return "#b71c1c";
                                } else {
                                    return "#1565c0";
                                }
                            }
                            border.width: 2
                            radius: 6

                            // Subtle shadow
                            Rectangle {
                                anchors.fill: parent
                                anchors.topMargin: 2
                                anchors.leftMargin: 2
                                color: "#00000020"
                                radius: parent.radius
                                z: -1
                            }
                        }

                        contentItem: Text {
                            text: parent.text
                            color: "white"
                            font.pixelSize: 12
                            font.bold: true
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }

                        onClicked: {
                            dialog.dialogAccepted();
                            dialog.close();
                        }
                    }
                }
            }

            // Smooth opening animation
            enter: Transition {
                NumberAnimation {
                    property: "scale"
                    from: 0.8
                    to: 1.0
                    duration: 200
                    easing.type: Easing.OutBack
                }
                NumberAnimation {
                    property: "opacity"
                    from: 0.0
                    to: 1.0
                    duration: 200
                }
            }

            exit: Transition {
                NumberAnimation {
                    property: "scale"
                    from: 1.0
                    to: 0.8
                    duration: 150
                }
                NumberAnimation {
                    property: "opacity"
                    from: 1.0
                    to: 0.0
                    duration: 150
                }
            }
        }
    }

    // Reset Sensor Dialog
    Loader {
        id: resetSensorDialogLoader
        sourceComponent: uniformDialogComponent

        function open() {
            if (item) {
                item.dialogTitle = "Reset Sensor Parameters";
                item.dialogText = "This will reset only sensor calibration parameters to factory defaults.\n\n" + "✓ Accelerometer calibration\n" + "✓ Compass calibration\n" + "✓ Gyroscope calibration\n\n" + "User configuration and flight data will be preserved.\n\n" + "Continue with sensor reset?";
                item.iconText = "🔄";
                item.primaryColor = "#4caf50";
                item.secondaryColor = "#e8f5e8";
                item.acceptText = "Yes, Reset Sensors";
                item.rejectText = "Cancel";
                item.isDangerous = false;

                item.dialogAccepted.connect(function () {
                    if (mavlinkClientRef) {
                        mavlinkClientRef.resetSensorParametersToDefault();
                    }
                });

                item.open();
            }
        }
    }

    // Reset Config Dialog
    Loader {
        id: resetConfigDialogLoader
        sourceComponent: uniformDialogComponent

        function open() {
            if (item) {
                item.dialogTitle = "Reset Configuration Parameters";
                item.dialogText = "⚠️ WARNING ⚠️\n\n" + "This will reset user configurable parameters to defaults:\n\n" + "• Flight modes\n" + "• Safety settings\n" + "• Tuning parameters\n" + "• Airframe configuration\n\n" + "Operation counters and vehicle statistics will be preserved.\n" + "ArduPilot will reboot automatically.\n\n" + "Are you sure you want to continue?";
                item.iconText = "⚙️";
                item.primaryColor = "#ff9800";
                item.secondaryColor = "#fff3e0";
                item.acceptText = "Yes, Reset Config";
                item.rejectText = "Cancel";
                item.isDangerous = false;

                item.dialogAccepted.connect(function () {
                    if (mavlinkClientRef) {
                        mavlinkClientRef.resetConfigParametersToDefault();
                        statusPopup.message = "Configuration parameters reset. ArduPilot will reboot.";
                        statusPopup.isSuccess = true;
                        statusPopup.open();
                        mavlinkClientRef.rebootArduPilot();
                        reconnectTimer.start();
                    }
                });

                item.open();
            }
        }
    }

    // Reset ALL Dialog (Dangerous)
    Loader {
        id: resetAllDialogLoader
        sourceComponent: uniformDialogComponent

        function open() {
            if (item) {
                item.dialogTitle = "DANGER: Reset ALL Parameters";
                item.dialogText = "🚨 EXTREME WARNING 🚨\n\n" + "This will reset EVERYTHING to factory defaults:\n\n" + "• ALL configuration parameters\n" + "• ALL sensor calibrations\n" + "• ALL safety settings\n" + "• ALL flight data and counters\n" + "• ALL tuning parameters\n\n" + "This is equivalent to a complete factory reset!\n\n" + "ArduPilot will reboot and you'll need to:\n" + "• Recalibrate all sensors\n" + "• Reconfigure all settings\n" + "• Redo safety checks\n\n" + "ARE YOU ABSOLUTELY SURE?";
                item.iconText = "⚠️";
                item.primaryColor = "#f44336";
                item.secondaryColor = "#ffebee";
                item.acceptText = "Yes, I understand the risks";
                item.rejectText = "Cancel";
                item.isDangerous = true;

                item.dialogAccepted.connect(function () {
                    if (mavlinkClientRef) {
                        mavlinkClientRef.resetAllParametersToDefault();
                    }
                });

                item.open();
            }
        }
    }

    // Reboot Dialog
    Loader {
        id: rebootDialogLoader
        sourceComponent: uniformDialogComponent

        function open() {
            if (item) {
                item.dialogTitle = "Reboot ArduPilot";
                item.dialogText = "This will restart the ArduPilot flight controller.\n\n" + "• Current connection will be lost\n" + "• ArduPilot will reboot (takes ~5-10 seconds)\n" + "• Will automatically reconnect after reboot\n\n" + "Continue with reboot?";
                item.iconText = "🔌";
                item.primaryColor = "#2196f3";
                item.secondaryColor = "#e3f2fd";
                item.acceptText = "Yes, Reboot";
                item.rejectText = "Cancel";
                item.isDangerous = false;

                item.dialogAccepted.connect(function () {
                    if (mavlinkClientRef) {
                        mavlinkClientRef.rebootArduPilot();
                        statusPopup.message = "Rebooting ArduPilot...";
                        statusPopup.isSuccess = true;
                        statusPopup.open();
                        reconnectTimer.start();
                    }
                });

                item.open();
            }
        }
    }

    // ---------------------------------------------------------
    //                          Timers
    // ---------------------------------------------------------
    Timer {
        id: reconnectTimer
        interval: 2000
        repeat: false

        onTriggered: {
            console.log("Reconnecting to ArduPilot...");

            if (mavlinkClientRef) {
                statusPopup.message = "Reconnecting to ArduPilot...";
                statusPopup.isSuccess = true;
                statusPopup.open();

                mavlinkClientRef.connectToSITL();
                reconnectMonitorTimer.start();
            }
        }
    }

    Timer {
        id: reconnectMonitorTimer
        interval: 1000
        repeat: true
        property int attempts: 0
        property int maxAttemps: 5

        onTriggered: {
            attempts += 1;

            if (mavlinkClientRef && mavlinkClientRef.connected) {
                stop();
                attempts = 0;
                statusPopup.message = "Successfully reconnected to ArduPilot.";
                statusPopup.isSuccess = true;
                statusPopup.open();
            } else if (attempts >= maxAttemps) {
                stop();
                attempts = 0;
                statusPopup.message = `Failed to reconnect to ArduPilot after ${attempts} attempts.`;
                statusPopup.isSuccess = false;
                statusPopup.open();
            } else {
                statusPopup.message = `Reconnecting ... (${attempts}/${maxAttempts})`;
                statusPopup.isSuccess = true;
                statusPopup.open();

                if (mavlinkClientRef) {
                    mavlinkClientRef.connectToSITL();
                }
            }
        }
    }
}
