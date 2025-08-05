#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "rclcpp/rclcpp.hpp"

#include "gimbal/gcs/gimbal_image_provider.hpp"
#include "gimbal/gcs/gimbal_streamer.hpp"
#include "gimbal/gcs/gimbal_controller.hpp"
#include "parser.h"
#include "btnode.h"
#include "ros_bridge.h"
#include "ros2_topic_subscriber.hpp"
#include "ros2_image_provider.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

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

    engine.load(QUrl(QStringLiteral(
        "qrc:/qml/main.qml"
    )));

    if (engine.rootObjects().isEmpty()) {
        return -1;
    }

    return app.exec(); 
}