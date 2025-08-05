#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include "rclcpp/rclcpp.hpp"

#include "gimbal/gcs/gimbal_image_provider.hpp"
#include "gimbal/gcs/gimbal_streamer.hpp"
#include "gimbal/gcs/gimbal_controller.hpp"
#include "bt_parser.hpp"
#include "tree_nodes_model_parser.hpp"
#include "bt_node.hpp"
#include "ros_bridge.hpp"
#include "ros2_topic_subscriber.hpp"
#include "ros2_image_provider.hpp"
#include "qml_cli.hpp"
#include "mavlink_client.hpp"
#include "vehicle_tracker.hpp"
#include "file_manager.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // QQuickStyle::setStyle("Material");   // Google Material Design style
    QQuickStyle::setStyle("Fusion");  // Modern, cross-platform style
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    // Read and write text files
    FileManager fileManager;
    engine.rootContext()->setContextProperty("fileManager", &fileManager);

    // GPS Navigation Panel
    VehicleTracker vehicle_tracker;
    engine.rootContext()->setContextProperty("vehicleTracker", &vehicle_tracker);

    // BtNode to be created dynamically in QML
    qmlRegisterType<BtNode>("Bt", 1, 0, "BtNode");

    // MAVLink client for communication with Ardupilot
    MAVLinkClient mavlink_client;
    engine.rootContext()->setContextProperty("mavlinkClient", &mavlink_client);

    // QML command line interface
    QmlCommandLine qmlCli;
    engine.rootContext()->setContextProperty("qmlCli", &qmlCli);

    // Gimbal Streamer
    gimbal::GimbalImageProvider img_provider;
    engine.addImageProvider("gimbal", &img_provider);

    const char * eo_rtsp_url = "rtsp://192.168.144.25:8554/main.264";
    gimbal::gcs::GimbalStreamer streamer(nullptr, eo_rtsp_url);
    streamer.disablePublish();  // Disable publishing to ROS2 topic
    streamer.setImageProvider(&img_provider);
    engine.rootContext()->setContextProperty("gimbalStreamer", &streamer);

    // Gimbal Controller
    const char * control_ip = "192.168.144.25";
    uint16_t control_port = 37260;

    gimbal::gcs::GimbalController controller(
        nullptr, control_ip, control_port
    );
    engine.rootContext()->setContextProperty("gimbalController", &controller);

    // Receive images from ROS2 topic "/camera/compressed/image_search"
    Ros2TopicSubscriber ros2TopicSubscriber;
    engine.rootContext()->setContextProperty("ros2TopicSubscriber", &ros2TopicSubscriber);

    Ros2ImageProvider ros2ImageProvider(&ros2TopicSubscriber);
    engine.addImageProvider("ros2", &ros2ImageProvider);
    engine.rootContext()->setContextProperty("ros2ImageProvider", &ros2ImageProvider);
    
    // Ros2 bridge to recieve behavior tree XML
    RosBridge rosBridge;
    engine.rootContext()->setContextProperty("rosBridge", &rosBridge);

    // Parser for behavior tree XML
    BtParser parser;
    engine.rootContext()->setContextProperty("btParser", &parser);

    // Parser for tree nodes model
    TreeNodesModelParser treeNodesModelParser;
    engine.rootContext()->setContextProperty("treeNodesModelParser", &treeNodesModelParser);

    engine.load(QUrl(QStringLiteral(
        "qrc:/qml/main.qml"
    )));

    if (engine.rootObjects().isEmpty()) {
        return -1;
    }

    return app.exec(); 
}