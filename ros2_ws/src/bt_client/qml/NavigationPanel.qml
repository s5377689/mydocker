import QtQuick 
import QtQuick.Controls
import QtLocation 
import QtPositioning 
import QtQuick.Dialogs

Item {
    id: root
    property var vehicleMarker: null
    property var vehicleTrackerRef: null
    property var fileManagerRef: null
    property var waypoints: []
    property var center: QtPositioning.coordinate(24.54178, 121.86804)

    // Path recording
    property var navigationPath: []
    property bool isRecording: false
    property int maxPathPoints: 1000
    property real minDistanceThreshold: 0.00001 // Minimum distance to record new point (in degrees)

    Component.onCompleted: {
        if (vehicleTrackerRef) {
            center = QtPositioning.coordinate(vehicleTrackerRef.latitude, vehicleTrackerRef.longitude)
            console.log("Vehicle Tracker Initialized at: " + center.latitude + ", " + center.longitude)
            startRecording()
        }
    }

    Connections {
        target: vehicleTrackerRef

        function onPositionChanged() {
            if (vehicleTrackerRef) {
                updateVehiclePosition(vehicleTrackerRef.latitude, vehicleTrackerRef.longitude)

                if (isRecording) {
                    recordPathPoint(vehicleTrackerRef.latitude, vehicleTrackerRef.longitude)
                }
            }
        }
    }

    // File dialogs for loading and saving mission plans
    FileDialog {
        id: loadFileDialog
        title: "Load Mission Plan"
        nameFilters: ["Plan files (*.plan)", "All files (*)"]
        currentFolder: "file:///home/robuff/data/plans"

        onAccepted: {
            var filePath = selectedFile.toString();
            var fullPath = filePath;

            // Remove "file://" prefix if present
            if (filePath.startsWith("file://")) {
                fullPath = filePath.substring(7);
            }

            console.log("Loading plan from: " + fullPath);
            var loadedWaypoints = fileManagerRef.loadWaypoints(fullPath)

            if (loadedWaypoints.lenth <= 0) {
                console.error("Failed to load waypoints from: " + fullPath);
                return;
            }

            clearAllWaypoints();

            for (var i = 0; i < loadedWaypoints.length; ++i) {
                var wp = loadedWaypoints[i];
                addWaypoint(wp.lat, wp.lon);
            }

            // Center map on the first waypoint
            map.center = QtPositioning.coordinate(
                loadedWaypoints[0].lat, loadedWaypoints[0].lon
            );
            console.log("Loaded " + loadedWaypoints.length + " waypoints from: " + fullPath);
        }

        onRejected: {
            console.log("Load dialog cancelled.");
        }
    }

    FileDialog {
        id: saveFileDialog
        title: "Save Mission Plan"
        fileMode: FileDialog.SaveFile
        nameFilters: ["Plan files (*.plan)", "All files (*)"]
        defaultSuffix: "plan"
        currentFolder: "file:///home/robuff/data/plans"

        onAccepted: {
            var planString = waypointsToPlanString();
            if (planString.length < 0) {
                console.warn("No waypoints to save.");
                return;
            }

            var filePath = selectedFile.toString();                
            var fullPath = filePath;

            // Remove "file:///" prefix if present
            if (filePath.startsWith("file://")) {
                fullPath = filePath.substring(7);
            }

            // Split path and filename
            var lastSlashIndex = fullPath.lastIndexOf("/");
            var directory = fullPath.substring(0, lastSlashIndex + 1);
            var filename = fullPath.substring(lastSlashIndex + 1);

            console.log("Saving plan to: " + directory + filename);

            var success = fileManagerRef.saveTextFile(planString, directory, filename);
            if (success) {
                console.log("Plan saved successfully to: " + directory + filename);
            } else {
                console.error("Failed to save plan to: " + directory + filename);
            }
        }

        onRejected: {
            console.log("Save dialog cancelled.");
        }
    }

    // Save button
    Rectangle {
        anchors.bottom: parent.bottom
        anchors.right: parent.right
        anchors.margins: 10
        width: 120
        height: 40
        radius: 5
        color: "#1a1d2c"
        border.color: "#ffffff"
        border.width: 2
        z: 1000

        Text {
            anchors.centerIn: parent
            text: "Save Plan"
            font.bold: true
        }

        MouseArea {
            anchors.fill: parent
            onClicked: {
                if (waypointModel.count === 0) {
                    console.warn("No waypoints to save.");
                    return;
                }

                var timestamp = new Date().toISOString().replace(/[:.]/g, "-");
                var defaultName = "mission-" + timestamp + ".plan";
                saveFileDialog.currentFile = "file:///home/robuff/data/plans/" + defaultName;

                saveFileDialog.open();
            }
        }
    }

    // Add Load button next to Save button
    Rectangle {
        anchors.bottom: parent.bottom
        anchors.right: parent.right
        anchors.rightMargin: 140  // Position next to Save button
        anchors.bottomMargin: 10
        width: 120
        height: 40
        radius: 5
        color: "#2c1a1d"
        border.color: "#ffffff"
        border.width: 2
        z: 1000

        Text {
            anchors.centerIn: parent
            text: "Load Plan"
            color: "#ffffff"
            font.bold: true
        }

        MouseArea {
            anchors.fill: parent
            onClicked: {
                loadFileDialog.open();
            }
        }
    }

    // For online map
    Plugin {
        id: mapPlugin
        name: "osm"
    }

    // Plugin {
    //     id: mapPlugin
    //     name: "osm"
    //     PluginParameter {
    //         name: "map.file"
    //         value: "/home/bill/data/maps/osm-2020-0210-taiwan.mbtiles"
    //     }
    // }

    Map {
        id: map
        anchors.fill: parent
        plugin: mapPlugin
        center: root.center
        zoomLevel: 14

        // To draw paths between waypoints
        ListModel {
            id: waypointModel
        }

        Component {
            id: waypointMarkerComponent

            MapQuickItem {
                id: marker
                anchorPoint.x: waypointIcon.width / 2
                anchorPoint.y: waypointIcon.height

                property int waypointNum: -1
                property var lat: 0
                property var lon: 0
                coordinate: QtPositioning.coordinate(lat, lon)

                sourceItem: Column {
                    spacing: 2

                    Image {
                        id: waypointIcon
                        source: "qrc:/icons/marker.png"
                        width: 40
                        height: 40
                        fillMode: Image.PreserveAspectFit
                        smooth: true

                        Text {
                            x: -3 
                            y: waypointIcon.height / 2
                            text: waypointNum
                            font.pixelSize: 15
                            font.bold: true
                            color: "red"
                            horizontalAlignment: Text.AlignHCenter
                            wrapMode: Text.WordWrap
                            elide: Text.ElideRight
                        }
                    }

                    Text {
                        text: lat.toFixed(5) + ", " + lon.toFixed(5)
                        font.pixelSize: 12
                        color: "black"
                        horizontalAlignment: Text.AlignHCenter
                        wrapMode: Text.WordWrap
                        elide: Text.ElideRight
                        width: 100
                    }
                }
            }
        }

        // For navigation path visualization
        MapPolyline {
            id: navigationPathLine
            line.width: 5
            line.color: "orange"
            visible: false

            opacity: 0.8 
        }

        MapPolyline {
            id: waypointPath
            line.width: 3
            line.color: "blue"
            path: {
                var coords = []
                for (var i = 0; i < waypointModel.count; i++) {
                    var wp = waypointModel.get(i)
                    coords.push(QtPositioning.coordinate(wp.lat, wp.lon))
                }
                return coords
            }
        }

        MouseArea {
            id: mapMouseArea
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton | Qt.RightButton
            property point lastMousePos
            drag.target: null

            onPressed: function(mouse) {
                lastMousePos = Qt.point(mouse.x, mouse.y)
            }

            onPositionChanged: function(mouse) {
                if (mouse.buttons & Qt.LeftButton) {
                    var oldCenter = map.toCoordinate(lastMousePos)
                    var newCenter = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                    var offsetLat = newCenter.latitude - oldCenter.latitude
                    var offsetLon = newCenter.longitude - oldCenter.longitude
                    map.center.latitude -= offsetLat
                    map.center.longitude -= offsetLon
                    lastMousePos = Qt.point(mouse.x, mouse.y)
                }
            }

            onClicked: function(mouse) {
                if ((mouse.button === Qt.RightButton) && !(mouse.modifiers & Qt.ShiftModifier)) {
                    var coord = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                    addWaypoint(coord.latitude, coord.longitude)
                } else if ((mouse.button === Qt.RightButton)
                           && (waypointModel.count > 0)) {
                    waypointModel.remove(waypointModel.count - 1)
                    var marker = waypoints.pop()
                    if (marker) {
                        map.removeMapItem(marker)
                        marker.destroy()
                    }
                }
            }

            WheelHandler {
                id: zoomHandler
                target: map
                onWheel: function(event) {
                    if (event.angleDelta.y > 0)
                        map.zoomLevel = Math.min(map.maximumZoomLevel, map.zoomLevel + 1)
                    else if (event.angleDelta.y < 0)
                        map.zoomLevel = Math.max(map.minimumZoomLevel, map.zoomLevel - 1)
                }
            }
        }
    }

    Component {
    id: vehicleMarkerComponent
        MapQuickItem {
            id: vehicleMarker
            anchorPoint.x: vehicleIcon.width / 2
            anchorPoint.y: vehicleIcon.height

            property var lat: 0
            property var lon: 0
            coordinate: QtPositioning.coordinate(lat, lon)

            sourceItem: Column {
                spacing: 2
                Image {
                    id: vehicleIcon
                    source: "qrc:/icons/fune.png"
                    width: 70
                    height: 70
                    fillMode: Image.PreserveAspectFit
                    smooth: true
                }
                Text {
                    text: "Vehicle\n" + lat.toFixed(5) + ", " + lon.toFixed(5)
                    font.pixelSize: 12
                    color: "red"
                    font.bold: true
                    horizontalAlignment: Text.AlignHCenter
                    wrapMode: Text.WordWrap
                    width: 80
                }
            }
        }
    }

    function updateVehiclePosition(lat, lon) {
        if (vehicleMarker)
        {
            map.removeMapItem(vehicleMarker)
            vehicleMarker.destroy()
        }

        vehicleMarker = vehicleMarkerComponent.createObject(
            map,
            {
                lat: lat,
                lon: lon
            }
        )
        map.addMapItem(vehicleMarker)
    }

    function addWaypoint(lat, lon) {
        waypointModel.append({ lat: lat, lon: lon })

        var new_marker = waypointMarkerComponent.createObject(
            map,
            {
                waypointNum: waypointModel.count,  // Number starts from 1
                lat: lat,
                lon: lon
            }
        )
        map.addMapItem(new_marker)
        waypoints.push(new_marker)
    }

    function waypointsToPlanString() {
        if (waypointModel.count === 0) {
            console.warn("No waypoints to convert to plan string.")
            return ""
        }

        // Create the plan data object
        var planData = {
            "fileType": "Plan",
            "geoFence": {
                "circles": [],
                "polygons": [],
                "version": 2
            },
            "groundStation": "QGroundControl",
            "mission": {
                "cruiseSpeed": 10,
                "firmwareType": 3,
                "globalPlanAltitudeMode": 1,
                "hoverSpeed": 5,
                "items": [],
                "plannedHomePosition": [
                    waypointModel.get(0).lat,
                    waypointModel.get(0).lon,
                    0
                ],
                "vehicleType": 10,
                "version": 2
            },
            "rallyPoints": {
                "points": [],
                "version": 2
            },
            "version": 1
        }

        // Convert waypoints to QGC mission items
        for (var i = 0; i < waypointModel.count; ++i) {
            var waypoint = waypointModel.get(i);
            var missionItem = {
                "AMSLAltAboveTerrain": null,
                "Altitude": 5,
                "AltitudeMode": 1,
                "autoContinue": true,
                "command": 16,  // MAV_CMD_NAV_WAYPOINT
                "doJumpId": i + 1,
                "frame": 3,  // MAV_FRAME_GLOBAL_RELATIVE_ALT
                "params": [
                    0,
                    0,
                    0,
                    0,
                    waypoint.lat,
                    waypoint.lon,
                    5
                ],
                "type": "SimpleItem"
            }

            planData.mission.items.push(missionItem);
        }

        var jsonString = JSON.stringify(planData, null, 4);
        return jsonString;
    }

    function clearAllWaypoints() {
        waypointModel.clear()

        // Remove all existing markers from the map
        for (var i = 0; i < waypoints.length; ++i) {
            var marker = waypoints[i];
            if (marker) {
                map.removeMapItem(marker)
                marker.destroy()
            }
        }

        waypoints = []
    }

    function startRecording() {
        isRecording = true
        navigationPathLine.visible = true
        var initPoint = {
            lat: vehicleTrackerRef ? vehicleTrackerRef.latitude : 0,
            lon: vehicleTrackerRef ? vehicleTrackerRef.longitude : 0,
            timestamp: new Date()
        }
        navigationPath = [initPoint]
    }

    function stopRecording() {
        isRecording = false
        navigationPathLine.visible = false
    }

    function clearPath() {
        navigationPath = []
    }

    function recordPathPoint(lat, lon) { 
        // Check distance threshold
        if (navigationPath.length > 0) {
            var lastPoint = navigationPath[navigationPath.length - 1]
            var distance = Math.sqrt(
                Math.pow(lastPoint.lat - lat, 2) +
                Math.pow(lastPoint.lon - lon, 2)
            )
            if (distance < minDistanceThreshold) {
                return  // Ignore point if too close
            }
        }

        var pathPoint = {
            lat: lat,
            lon: lon,
            timestamp: new Date()
        }

        navigationPath.push(pathPoint)

        // Limit array size
        if (navigationPath.length > maxPathPoints) {
            navigationPath.shift()  // Remove oldest point
        }

        // Update the path line on the map
        var coords = getPathCoordinates()
        navigationPathLine.path = coords
    }

    function getPathCoordinates() {
        var coords = []
        for (var i = 0; i < navigationPath.length; ++i) {
            var coord = QtPositioning.coordinate(
                navigationPath[i].lat,
                navigationPath[i].lon
            )
            
            if (coord.isValid) {
                coords.push(coord)
            } else {
                console.warn("Invalid coordinate at index " + i + ": " + navigationPath[i].lat + ", " + navigationPath[i].lon);
            }
        }

        return coords
    }
}